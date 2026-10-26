# SD 启动配置核对（2026-10-21）

用户指定优先通过 SD 卡启动配置修改频率，不刷写固件。停止将刷写作为当前默认下一步；已生成固件仅保留为候选产物，未刷写。

核对源码基线 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`：

- rcS 会在模块启动前执行 `/fs/microsd/etc/config.txt`，后期执行 `/fs/microsd/etc/extras.txt`。SD 卡脚本可以设置参数和调用已有模块命令，不是替换整个 PX4 固件。
- `IMU_INTEG_RATE` 可以通过参数/启动脚本配置；切换 DDS 前已从实机读取到 200 Hz，不需要为此刷固件。
- MAVLink 支持 `mavlink stream -d <飞控设备> -s HIGHRES_IMU -r 200`，可放到 extras.txt，在对应 MAVLink 实例启动后执行。它设置 MAVLink 输出频率，不作用于 DDS；不能直接在目前被 DDS 占用的 GPS1 上另开 MAVLink。
- 当前 DDS client 的 CLI 只提供传输、设备、波特率、IP、端口、命名空间等启动选项，没有动态话题速率或同步周期选项；模块参数也未提供同步周期设置。DDS 的 SensorCombined 默认限速来自编译生成的 dds_topics.h。不能承诺写一个 SD 脚本就能把当前 DDS 输出从 100 Hz 改为 200 Hz，或修复本次同步周期问题。

## 用户提供的实机 SD 内容（2026-10-22）

用户确认 `etc/config.txt` 不存在，`etc/extras.txt` 存在，内容为：

```sh
mavlink stream -d /dev/ttyS2 -s HIGHRES_IMU -r 200
```

这是对飞控 GPS1/UART3 对应 MAVLink 实例设置 HIGHRES_IMU 的 200 Hz 输出请求，不是 DDS 配置，也不能单凭请求值认定实际交付频率。结合此前读取的 IMU_INTEG_RATE=200，应区分内部积分率、MAVLink 请求率和当前 DDS 实测约 100 Hz 三件事。

已授权切换后，MAV_2_CONFIG=0、UXRCE_DDS_CFG=201，GPS1 上运行 DDS。上述命令只查找已有 MAVLink 实例，不会启动新实例、抢占 DDS 串口或修改其限速。若该设备未运行 MAVLink，已核对源码会警告 `mavlink for device /dev/ttyS2 is not running` 并返回 1；这不等于它导致本次 DDS 同步误差。

本次仅记录用户提供的文件内容，没有覆盖 SD 文件、修改参数、切换协议或刷写。按参考工程保留 100 Hz DDS 基线；下一步需要实际板端版本和同步模块状态，优先只读查看 `ver all`、`uxrce_dds_client status`、`param show UXRCE_DDS_SYNCT`、`param show UXRCE_DDS_SYNCC`。当前 DDS 通道未提供这类通用终端命令接口，需要已有 QGC 控制台或其他维护通道返回输出。

来源：`firmware/ROMFS/px4fmu_common/init.d/rcS`、`firmware/src/modules/mavlink/mavlink_main.cpp`、`upstream/uxrce_dds_client.cpp`、`firmware/src/modules/uxrce_dds_client/module.yaml`、`upstream/dds_topics.h.em`。实机当前固件 SHA 仍未重新读取，上述为已核对基线，不替代实际板端 help 和文件检查。
