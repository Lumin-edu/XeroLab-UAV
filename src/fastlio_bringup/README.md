# fastlio_bringup

FAST-LIO + PX4 DDS 定点悬停包。Livox 驱动和 Micro XRCE-DDS Agent 由单独终端启动。

~~~text
/livox/lidar + /livox/imu -> FAST-LIO /Odometry
                           -> 机体中心补偿 -> PX4 visual odometry
                           -> PX4 EKF2 -> fixed_point_hover.py
~~~

## 构建与启动

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fastlio_bringup
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
ros2 launch fastlio_bringup fastlio_hover.launch.py \
  use_fastlio:=true use_visual_odom:=true use_hover_control:=true \
  takeoff_altitude:=0.2 hover_auto_arm:=false
~~~

默认不自动解锁。FAST-LIO `body` 到机体中心的外参、输入话题和跳变限制在 launch 参数中配置；外参修改后检查 TF 与 PX4 视觉定位，避免重复补偿。

## 检查

~~~bash
ros2 topic hz /Odometry
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic info -v /fmu/in/vehicle_visual_odometry
~~~

解锁前确认 PX4 的 `xy_valid`、`z_valid` 为 `true`，且只有一个节点发布 setpoint。详见 `launch/fastlio_hover.launch.py`。
