# bringup_square_mission

使用 Point-LIO 视觉里程计和 PX4 EKF2 的正方形航点任务。位置反馈来自 PX4 local position；本包负责唯一的 Offboard 控制输出。

## 构建与启动

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_square_mission
source install/setup.bash
ros2 launch bringup_square_mission square_mission_hw.launch.py \
  use_pointlio:=true use_pointlio_px4_visual_odom:=true \
  use_px4_control_watchdog:=true auto_arm:=false \
  takeoff_altitude:=0.40 square_side_length:=2.0 approach_speed:=0.25
~~~

首次运行时确认传感器、PX4 Agent 和 local position 均正常。默认需要遥控器手动解锁；控制器在 PX4 armed 后请求 Offboard。

## 路线与坐标

路线以启动时捕获的 PX4 local position 为原点，任务坐标为 `(右, 前, 上)`。默认航线起飞至 0.30 m，依次前进、右移、后退并返回原点上方，最后下降。yaw 锁定为起始航向。

Point-LIO `base_link` 表示 MID-360 IMU 原点，桥接器用 launch 中的 `base_link_to_base` 外参补偿到机体中心。不要再叠加相同的 PX4 外部视觉偏置。

## 飞前检查

~~~bash
ros2 topic hz /odom
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic hz /fmu/in/trajectory_setpoint
~~~

确认 `xy_valid`、`z_valid` 为 `true`，并用 `ros2 topic info -v /fmu/in/trajectory_setpoint` 检查只有一个控制发布者。完整参数见 `launch/square_mission_hw.launch.py`。
