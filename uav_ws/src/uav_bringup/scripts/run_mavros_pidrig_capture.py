#!/usr/bin/env python3
"""Start MAVROS/VIO and record one manually armed rig session; never command PX4."""
import copy
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

import run_mavros_vio_validation as base


class RigMonitor(base.EvidenceMonitor):
    def __init__(self):
        super().__init__()
        self.require_disarmed = True

    def check_inputs(self):
        now = time.monotonic()
        state = self.latest.get('/mavros/state')
        if state is None or not state.connected or now - self.last_receive.get('/mavros/state', 0) > 3:
            raise RuntimeError('飞控连接/状态未就绪。')
        if self.require_disarmed and state.armed:
            raise RuntimeError('预热尚未结束，请先手动上锁。')
        if now - self.last_receive.get('/mavros/imu/data_raw', 0) > 1 or self.imu_stamp <= 0 or now - self.imu_advanced_at > 1:
            raise RuntimeError('IMU 未就绪或时间戳停滞。')
        if now - self.last_receive.get('/mavros/timesync_status', 0) > 3:
            raise RuntimeError('时间同步数据未就绪。')

    def freshly_disarmed(self):
        state = self.latest.get('/mavros/state')
        return state is not None and state.connected and not state.armed and time.monotonic() - self.last_receive.get('/mavros/state', 0) < 3


stop_requested = False


def request_stop(*_):
    global stop_requested
    stop_requested = True


def run():
    global stop_requested
    os.chdir(base.WS)
    existing = subprocess.run(['pgrep', '-af', '[m]avros_node|[r]un_subscribe_msckf|[r]ealsense2_camera_node|[o]dom_bridge'], capture_output=True, text=True)
    if existing.returncode == 0:
        raise RuntimeError('请先退出已有 MAVROS/相机/VIO/桥接，再执行此入口：\n' + existing.stdout)
    get_package_share_directory('mavros_extras')
    share = Path(get_package_share_directory('mavros'))
    config = yaml.safe_load((base.WS / 'src/uav_bringup/config/uav.yaml').read_text())
    if config['uav_control']['backend'] != 'mavros' or config['uav_bridge']['enabled'] is not False:
        raise RuntimeError('要求基础配置 backend=mavros、uav_bridge.enabled=false。')
    session = base.WS / 'log' / datetime.now().strftime('vio_ekf_pidrig_%Y%m%d_%H%M%S_%f')
    session.mkdir()
    print(f'日志目录：{session}\n请保持未解锁，等待“可以手动解锁”提示。', flush=True)
    source_config = session / 'uav_source.yaml'
    source_config.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    allowed = [item.attrib['name'] for item in ET.parse(share / 'mavros_plugins.xml').getroot().findall('class')]
    params = session / 'mavros_params.yaml'
    params.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'plugin_allowlist': allowed + ['odometry'], 'plugin_denylist': ['image_pub', 'vibration', 'distance_sensor', 'rangefinder', 'wheel_odometry']}},
        '/mavros_node': {'ros__parameters': {'fcu_url': 'serial:///dev/ttyS6:921600', 'gcs_url': '', 'tgt_system': 1, 'tgt_component': 1, 'fcu_protocol': 'v2.0'}},
    }))
    rclpy.init()
    monitor = RigMonitor()
    base.monitor = monitor
    events = (session / 'events.jsonl').open('w', buffering=1)
    memory = (session / 'memory.log').open('w', buffering=1)

    def event(name, **fields):
        record = {'time_local': datetime.now().astimezone().isoformat(), 'monotonic_s': time.monotonic(), 'event': name, **fields}
        events.write(json.dumps(record, ensure_ascii=False) + '\n')
        print(f'{record["time_local"]} {name} {fields}', flush=True)

    try:
        topics = list(dict.fromkeys(base.TOPICS + ['/mavros/extended_state', '/mavros/rc/in', '/mavros/rc/out', '/mavros/imu/data', '/mavros/imu/static_pressure']))
        bag = base.start(['ros2', 'bag', 'record', '--storage', 'sqlite3', '--max-cache-size', '33554432', '-o', str(session / 'bag'), *topics], session / 'recorder.log', session / 'recorder_ros')
        mavros_log = session / 'mavros.log'
        mavros = base.start(['ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '--params-file', str(share / 'launch/px4_config.yaml'), '--params-file', str(params)], mavros_log, session / 'mavros_ros')
        base.warmup('MAVROS', [mavros, bag], [mavros_log], session / 'mavros_warmup_memory.log')
        event('等待 MAVROS IMU 时基切换与稳定', timeout_s=120, stable_s=5,
              reason='等待首次 TIMESYNC 收敛后 IMU 头时间戳改用飞控时间加同步偏移')
        deadline = time.monotonic() + 120
        last_backwards = monitor.imu_backwards
        stable_since = None
        while True:
            monitor.spin(0.2)
            try:
                monitor.check_inputs()
                if not monitor.odometry_subscribers() or 'Plugin odometry initialized' not in mavros_log.read_text():
                    raise RuntimeError('等待 odometry 插件加载。')
            except RuntimeError:
                stable_since = None
                if mavros.poll() is not None or time.monotonic() > deadline or stop_requested:
                    raise
                continue
            if monitor.imu_backwards != last_backwards:
                last_backwards = monitor.imu_backwards
                stable_since = None
                event('观测到 IMU 时间戳回退', count=last_backwards,
                      note='与 MAVROS 首次收敛切换消息头时基一致；继续等待稳定窗口。')
            if last_backwards > 0:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 5:
                    event('时间同步启动门控通过', observed_imu_backwards=last_backwards,
                          stable_s=5, note='IMU 时间戳连续递增且 TIMESYNC 状态持续更新。')
                    break
            if mavros.poll() is not None or time.monotonic() > deadline or stop_requested:
                raise RuntimeError('等待 MAVROS IMU 时基切换/稳定超时（120 秒）；未启动 VIO，也未解锁。')
        (session / 'px4_parameters_before.json').write_text(json.dumps(monitor.read_px4_parameters(), indent=2))
        vio_log = session / 'vio.log'
        vio = base.start(['ros2', 'launch', 'uav_bringup', 'uav.launch.py', f'config_file:={source_config}', 'start_control:=false', 'start_mission:=false', 'start_mavros:=false', 'start_camera:=true', 'start_openvins:=true', 'start_bridge:=false', 'vio_respawn:=false', 'vio_verbosity:=INFO'], vio_log, session / 'vio_ros')
        bridge_config = copy.deepcopy(config['uav_bridge'])
        bridge_config.update(backend='mavros', enabled=False)
        bridge_file = session / 'bridge_disabled.yaml'
        bridge_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': bridge_config}}))
        bridge = base.start(['ros2', 'run', 'uav_bridge', 'odom_bridge', '--ros-args', '--params-file', str(bridge_file)], session / 'bridge_warmup.log', session / 'bridge_warmup_ros')
        base.warmup('VIO', [mavros, vio, bridge, bag], [mavros_log, vio_log, session / 'bridge_warmup.log'], session / 'vio_warmup_memory.log')
        monitor.check_vio(config['uav_bridge'])
        if stop_requested:
            return
        base.stop(bridge)
        bridge_config.update(backend='mavros', enabled=True)
        bridge_file = session / 'bridge_enabled.yaml'
        bridge_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': bridge_config}}))
        bridge = base.start(['ros2', 'run', 'uav_bridge', 'odom_bridge', '--ros-args', '--params-file', str(bridge_file)], session / 'bridge.log', session / 'bridge_ros')
        event('视觉回传开启')
        tx_before = monitor.tx_odometry
        monitor.spin(3)
        monitor.check_vio(config['uav_bridge'])
        if monitor.tx_odometry <= tx_before or any(job.poll() is not None for job in (mavros, vio, bridge, bag)):
            raise RuntimeError('视觉 MAVLink 输出或运行进程未就绪。')
        monitor.require_disarmed = False
        event('可以手动解锁', note='推荐 Stabilized 自稳模式；PX4 参数未写入，脚本不发送控制命令。')
        ever_armed, disarmed_since, last_state, last_issue = False, None, None, None
        started, next_sample = time.monotonic(), 0
        while rclpy.ok():
            monitor.spin(0.1)
            state = monitor.latest.get('/mavros/state')
            fresh = state is not None and state.connected and time.monotonic() - monitor.last_receive.get('/mavros/state', 0) < 3
            current = (bool(state.armed), state.mode) if fresh else None
            if current != last_state:
                event('飞控状态变化', armed=current[0] if current else None, mode=current[1] if current else None)
                last_state = current
            if fresh and state.armed:
                ever_armed, disarmed_since = True, None
            elif fresh and ever_armed:
                disarmed_since = disarmed_since or time.monotonic()
            else:
                disarmed_since = None
            issue = None
            try:
                monitor.check_inputs()
            except RuntimeError as error:
                issue = str(error)
            failed = [str(job.args) for job in (mavros, vio, bridge, bag) if job.poll() is not None]
            if failed:
                issue = '进程退出：' + '; '.join(failed)
            if issue != last_issue:
                event('输入或进程状态', issue=issue)
                last_issue = issue
            if time.monotonic() - started >= next_sample:
                try:
                    base.sample_memory(memory, time.monotonic() - started)
                except RuntimeError as error:
                    event('资源告警', message=str(error))
                for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
                    try:
                        memory.write(f'{path}: {path.read_text().strip()}\n')
                    except OSError:
                        pass
                (session / 'live_evidence.json').write_text(json.dumps(monitor.snapshot(), indent=2))
                next_sample += 5
            if disarmed_since is not None and time.monotonic() - disarmed_since >= 10:
                event('上锁后记录完成')
                break
            if stop_requested:
                if monitor.freshly_disarmed():
                    event('未解锁状态下结束记录')
                    break
                event('等待手动上锁', note='保持录制与运行链路；Ctrl-C 不会停止电机。')
                stop_requested = False
        (session / 'px4_parameters_after.json').write_text(json.dumps(monitor.read_px4_parameters(), indent=2))
        print(f'采集完成：{session}\n请通过 QGC 下载对应 PX4 ULog；ULog 保存在飞控 SD 卡，不在 ROS bag 内。', flush=True)
    finally:
        # Fresh disarmed evidence is required before tearing down a started rig link.
        if '/mavros/state' in monitor.latest:
            warned = False
            while rclpy.ok() and not monitor.freshly_disarmed():
                if not warned:
                    event('等待手动上锁或恢复状态连接', note='未确认上锁，保留全部运行链路。')
                    warned = True
                monitor.spin(0.2)
        for job in reversed(base.jobs[:]):
            base.stop(job)
        for stream in base.streams:
            stream.close()
        events.close()
        memory.close()
        monitor.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        run()
    except Exception as error:
        print(f'采集未完成：{error}', flush=True)
        raise SystemExit(1)
