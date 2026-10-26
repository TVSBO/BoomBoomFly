# 本轮 Kalibr 离线执行工具

使用现有标定录包，不要求重新录制。原始输入是
`../vio_calib_20261018_112654_iZIADC/calibration_full.bag`（ROS 1），
本轮独立输出在 `../vio_calib_run_20261018/`。

用户已确认 AprilGrid：6×6，标签黑色外边长25mm，间距7.5mm，刚性安装未改变。
`target_confirmation.json`保存确认信息。只选原录包20–144秒，避开末段共同断档；
所选区间状态均连接、未解锁。标定期间不启动实机定位、桥接或任务。

## 环境修复

原`kalibr:latest`需source ROS与`/kalibr_ws/devel/setup.bash`，且缺少wx。
`Dockerfile.kalibr-vio`记录依赖修复。此次legacy Docker build在保存阶段长时间停留；
依赖安装成功的中间层已保存为`kalibr:vio-20261018`，
ID为`sha256:157fadb4cd060973ccc5241fe97e22ded075a06691af3c3664c596266e29bf4b`。
MPLBACKEND由脚本设为Agg，原镜像保留。
双目PDF导出另需python3-cairo：当前容器已补齐，Dockerfile和bootstrap已更新；
上述中间镜像本身尚不含这次补装，使用时需运行bootstrap或按Dockerfile重建。

本轮容器名`vio-kalibr-20261018`，输出目录挂载到`/data`，
原始ROS1包只读挂载到`/data/calibration_full.bag`，本目录工具只读挂载到`/tools`。
**宿主输出目录里同名的0字节calibration_full.bag只是嵌套挂载占位，
真正的输入始终是上面的3.5GB源文件；不要把占位文件复制到Windows当作数据。**

`start_kalibr_session.sh`补齐依赖并自检，然后保持容器运行。
该Kalibr版本将`--help`的正常SystemExit捕获后返回2，脚本验证帮助内容而不只看退出码；
不需要为“rosdep view is empty”提示联网更新ROS依赖数据库。

## 执行顺序

1. `prepare_continuous_bag.py`生成双目标定输入：465对严格配对图像（约3.75Hz，
   设置上限4Hz），全部22022条IMU保留。原bag不修改；输出存在时不要直接覆盖重跑。
2. 容器中运行`bash /tools/run_stereo.sh`，结果文件前缀为
   `/data/continuous_20_144_4hz-*`，终端完整日志保存到`stereo.log`。
3. 检查双目结果的重投影误差、基线、覆盖和参数不确定性，再运行
   `bash /tools/run_imu_camera.sh`。联合标定使用原始高帧率包、相同20–144秒区间，
   图像提取上限20Hz（原30Hz数据实际通常选到约15Hz），没有降低IMU采样率。
4. `imu.yaml`沿用项目已有IMU噪声和200Hz名义采样率，不是本轮重新完成Allan噪声标定。
   联合标定使用`calibrated` IMU模型，估计相机—IMU外参及时间偏移，保留相机链几何。
5. 新标定写独立OpenVINS测试目录，关闭在线标定，用原始两份独立验证bag做对照。
   不直接覆盖正式配置，不开启PX4回传，不放宽门限。

## 在 Windows 的既有计算环境继续

如果使用Windows/WSL上的既有Kalibr环境，可复制**真正的ROS1源包**、
`continuous_20_144_4hz.bag`、`target.yaml`、`imu.yaml`及本目录两个执行脚本中的命令。
把`/data/...`替换成本地路径即可；无需重新采集或再次从ROS2转换。
相机入口参数为`--models pinhole-radtan pinhole-radtan`；
本机IMU入口使用`--cams`（以Windows所用Kalibr的`--help`为准）。
应保持相同区间、模型、目标尺寸和图像频率，以便对照。

本机镜像为ARM64，Windows/WSL若为x86_64应使用其已有匹配架构环境，
不能把ARM编译的ROS工作区当作Windows本地安装包。

参考：[Kalibr相机标定](https://github.com/ethz-asl/kalibr/wiki/Multiple-camera-calibration)、
[相机—IMU标定](https://github.com/ethz-asl/kalibr/wiki/Camera-IMU-calibration)。
