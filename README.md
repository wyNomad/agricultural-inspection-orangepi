# Agricultural Inspection Robot · Orange Pi Workspace

**农业巡检机器人项目的香橙派侧 ROS 2 工作空间，中期成果。**

本仓库保存农业巡检机器人项目部署在香橙派主控上的部分 ROS 2 包，包含底盘、IMU、RGB-D 相机、雷达、建图定位与导航配置。该项目采用香橙派与 Jetson Nano 双主控，当前仓库对应香橙派侧的中期工程快照，最终版本暂未公开。

同一项目的早期工程结构见 [`agricultural-inspection-robot`](https://github.com/wyNomad/agricultural-inspection-robot)。当前代码仍保留历史设备路径和调试配置。

## 工作空间组成

| 包 / 目录 | 用途 |
| --- | --- |
| [`chassis_driver`](src/chassis_driver) | 订阅速度指令、串口通信、轮式里程计 |
| [`imu_odometry`](src/imu_odometry) | IMU 串口读取与消息发布 |
| [`my_robot_launch`](src/my_robot_launch) | 底盘、IMU、雷达、相机与图像压缩的组合启动 |
| [`my_robot_description`](src/my_robot_description) | 底盘描述与 TF |
| [`my_robot_slam`](src/my_robot_slam) | RGB-D / ICP 里程计、RTAB-Map 建图与 EKF |
| [`my_robot_navi`](src/my_robot_navi) | RTAB-Map 定位与 Nav2 |
| [`my_robot_version`](src/my_robot_version) | YOLO 与 RGB-D 三维目标感知 |
| [`LSLIDAR_X_ROS2-20240228`](src/LSLIDAR_X_ROS2-20240228) | 镭神雷达驱动与消息包 |
| [`ros2_astra_camera`](src/ros2_astra_camera) | Astra RGB-D 相机驱动与消息包 |

## 环境与构建

需要 Linux、ROS 2、colcon 和 rosdep。以下采用 ROS 2 Humble 路径举例，历史设备镜像与完整版本组合尚未记录为验证矩阵。

```bash
git clone https://github.com/wyNomad/agricultural-inspection-orangepi.git
cd agricultural-inspection-orangepi
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --symlink-install
source install/setup.bash
```

除包清单声明的依赖外，还需按模块准备 PySerial、NumPy、`tf_transformations`、RTAB-Map、Nav2、`robot_localization`、图像传输插件。视觉检测额外使用 Ultralytics。部分依赖尚未完整写入 `package.xml`。

硬件相关依赖见 [Astra 驱动说明](src/ros2_astra_camera/README.MD)和[雷达驱动说明](src/LSLIDAR_X_ROS2-20240228/src/README.md)。

## 启动入口

每个终端先加载 ROS 2 与本工作空间。首次接入时先单独验证传感器和设备参数，再使用组合启动。

| 任务 | 命令 |
| --- | --- |
| 底盘描述与 TF | `ros2 launch my_robot_description rsp_launch.py` |
| 整机传感器与底盘 | `ros2 launch my_robot_launch robot_launch.py` |
| 仅启用组合启动中的感知组 | `ros2 launch my_robot_launch robot_launch.py move:=false see:=true` |
| 底盘串口节点 | `ros2 run chassis_driver serial_bridge` |
| IMU 节点 | `ros2 run imu_odometry imu_node` |
| 三维目标检测 | `ros2 run my_robot_version object_detector_3d` |
| 建图 | `ros2 launch my_robot_slam slam_launch.py` |
| 定位与导航 | `ros2 launch my_robot_navi navi_launch.py` |

`robot_launch.py` 通过 `move` 控制底盘和 IMU，通过 `see` 控制雷达、相机和图像压缩；两个参数默认均为 `true`。组合启动本身不包含建图、Nav2 或机器人描述发布，需根据实际联调方案另行启动。

底盘串口参数可独立覆盖：

```bash
ros2 run chassis_driver serial_bridge --ros-args \
  -p port:=/dev/ttyACM0 -p baudrate:=115200
```

## 配置核对

| 配置项 | 位置与说明 |
| --- | --- |
| 底盘串口与几何参数 | `serial_bridge.py` 的 ROS 参数；默认端口 `/dev/ttyACM0` |
| IMU 串口 | `imu_node.py` 顶部常量；当前为 `/dev/ttyUSB1`、115200，尚非 ROS 参数 |
| 雷达型号 | `robot_launch.py` 当前包含 `lsn10p_launch.py`；按实机型号核对 |
| 建图数据库 | `my_robot_slam/launch/slam_launch.py`，仍为原设备绝对路径 |
| 定位数据库 | `my_robot_navi/launch/navi_launch.py`，仍为原设备绝对路径 |
| 导航与融合参数 | `my_robot_navi/config/nav2_params.yaml`、`my_robot_slam/config/efk.yaml` |
| 图像链路 | 感知组发布压缩彩色图；建图、定位所需话题与解压节点需要按配置对齐 |

仓库在建图包和导航包中分别带有 `rtabmap.db`。定位前需确认使用哪份地图，并替换源码中的旧路径。**建图启动文件带有 `--delete_db_on_start`，会清空所指向的数据库，应使用独立的建图输出路径。**

可先检查输入是否存在：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/odom
ros2 topic hz /wheel/odom
```

还应核对相机内参、RGB/深度话题以及 `map`、`odom`、`base_link`、传感器坐标之间的 TF。仓库中的示例配置不能替代本机标定。

## 与其他仓库的关系

本仓库与 [`agricultural-inspection-robot`](https://github.com/wyNomad/agricultural-inspection-robot) 属于同一个农业巡检机器人项目：后者保留早期工程结构，本仓库保留香橙派侧中期包。两者阶段和板端配置不同，目前均为阶段性成果，最终版本暂未公开。

存在同名包的工作空间应分别构建、分别加载，避免叠加环境后加载到另一份配置。

## 验证与后续维护

本轮依据源码核对包、启动入口和配置差异，尚未进行全量构建或整机运行验证。

当前仓库作为香橙派侧中期成果保留；环境、地图及配置应按对应阶段查阅，最终版本暂未公开。

## 代码来源与许可证

工作空间包含外部设备驱动、地图与模型资源。请遵循各子目录原有的许可证和资源授权；根目录尚未声明统一许可证。
