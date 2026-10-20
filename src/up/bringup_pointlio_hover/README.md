# bringup_pointlio_hover

Position-hold bringup using Point-LIO odometry and PX4 EKF2. Point-LIO provides localization; this package is the only Offboard setpoint publisher.

```text
MID-360 -> /odom -> body-center compensation -> /fmu/in/vehicle_visual_odometry
                                             -> PX4 EKF2 -> /fmu/out/vehicle_local_position
                                             -> fixed_point_hover.py
                                             -> /fmu/in/trajectory_setpoint
```

## Build and launch

For an Ubuntu 22.04 and ROS 2 Humble workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_pointlio_hover
source install/setup.bash
```

Start the Livox driver, Micro XRCE-DDS Agent and mission in separate terminals:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
ros2 launch bringup_pointlio_hover hover_hw.launch.py \
  use_pointlio:=true use_pointlio_px4_visual_odom:=true \
  use_hover_control:=true takeoff_altitude:=0.2 \
  reference_capture_delay_sec:=3.0 hover_auto_arm:=false
```

Automatic arming is disabled by default. `takeoff_altitude` is relative to the captured position. The body-center extrinsic is defined in the launch file; after changing it, check Point-LIO, static TF and PX4 visual odometry settings to avoid duplicate compensation.

## Checks

```bash
ros2 topic hz /odom
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic info -v /fmu/in/trajectory_setpoint
```

Before takeoff, confirm PX4 reports `xy_valid` and `z_valid` as `true` and that only one node publishes trajectory setpoints. See `launch/hover_hw.launch.py` for all parameters.
