# bringup_pointlio_hover

Point-LIO 到 PX4 EKF2 的定点悬停 bringup。Point-LIO 仅提供定位；本包是唯一的 Offboard setpoint 发布者。

~~~text
MID-360 -> /odom -> 机体中心补偿 -> /fmu/in/vehicle_visual_odometry
                                      -> PX4 EKF2 -> /fmu/out/vehicle_local_position
                                      -> fixed_point_hover.py
                                      -> /fmu/in/trajectory_setpoint
~~~

## 构建与启动

在 Ubuntu 22.04、ROS 2 Humble 工作空间中：

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_pointlio_hover
source install/setup.bash
~~~

分别在终端启动 Livox 驱动、Micro XRCE-DDS Agent 和任务：

~~~bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
ros2 launch bringup_pointlio_hover hover_hw.launch.py \
  use_pointlio:=true use_pointlio_px4_visual_odom:=true \
  use_hover_control:=true takeoff_altitude:=0.2 \
  reference_capture_delay_sec:=3.0 hover_auto_arm:=false
~~~

默认不自动解锁。`takeoff_altitude` 是捕获位置上方的高度。机体中心外参在 launch 中定义；修改后同步检查 Point-LIO、静态 TF 和 PX4 视觉里程计配置，避免重复补偿。

## 检查

~~~bash
ros2 topic hz /odom
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic info -v /fmu/in/trajectory_setpoint
~~~

起飞前确认 PX4 的 `xy_valid` 与 `z_valid` 均为 `true`，且只有一个节点发布 trajectory setpoint。更多 launch 参数见 `launch/hover_hw.launch.py`。
