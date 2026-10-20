# bringup_full_mission

面向 PX4 DDS 的多任务航点控制器。Point-LIO 为 PX4 EKF2 提供外部视觉里程计；下视 D435i 可修正作业点，上视相机配置用于圆环横向校正。任务执行和执行器命令由 `full_mission_controller.py` 管理。

## 构建与启动

需要 ROS 2 Humble、PX4 1.14 对应的 `px4_msgs`、Point-LIO 和连接好的相机。

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_full_mission
source install/setup.bash
ros2 launch bringup_full_mission full_mission_hw.launch.py \
  use_pointlio:=true use_pointlio_px4_visual_odom:=true \
  use_px4_control_watchdog:=true auto_arm:=false \
  selected_tasks:=1,3,5,7 \
  drop_port:=/dev/ttyUSB0 approach_speed:=0.25
~~~

默认由遥控器解锁。执行器命令 `drop_command` 默认为空；确认串口、脚本和无桨测试后，才显式传入命令路径。

## 任务与校正

航点坐标使用 `(右, 前, 上)`，原点与航向在启动时捕获。任务 1–5 可使用下视相机进行黑框水平校正；圆环阶段可使用前视相机校正横向位置。相机 namespace、序列号和修正阈值通过 launch 参数设置。

PX4 local position 是控制反馈。检查 `/fmu/out/vehicle_local_position` 的有效标志以及视觉里程计、相机话题和 setpoint 频率。执行前核对 `selected_tasks`、相机序列号、`drop_command` 和串口设备；完整参数见 `launch/full_mission_hw.launch.py`。
