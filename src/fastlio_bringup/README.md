# fastlio_bringup

Position-hold bringup using FAST-LIO and PX4 DDS. Start the Livox driver and Micro XRCE-DDS Agent separately.

```text
/livox/lidar + /livox/imu -> FAST-LIO /Odometry
                           -> body-center compensation -> PX4 visual odometry
                           -> PX4 EKF2 -> fixed_point_hover.py
```

## Build and launch

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fastlio_bringup
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
ros2 launch fastlio_bringup fastlio_hover.launch.py \
  use_fastlio:=true use_visual_odom:=true use_hover_control:=true \
  takeoff_altitude:=0.2 hover_auto_arm:=false
```

Automatic arming is disabled by default. Configure the FAST-LIO `body` to vehicle-center extrinsic, input topics and jump limits through launch arguments. After changing extrinsics, check TF and PX4 visual odometry to avoid applying the same offset twice.

## Checks

```bash
ros2 topic hz /Odometry
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic info -v /fmu/in/vehicle_visual_odometry
```

Before arming, confirm PX4 reports `xy_valid` and `z_valid` as `true` and that only one node publishes setpoints. See `launch/fastlio_hover.launch.py` for the full parameter list.
