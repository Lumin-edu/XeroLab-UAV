# bringup_square_mission

Square waypoint mission using Point-LIO visual odometry and PX4 EKF2. PX4 local position supplies feedback; this package is the sole Offboard control publisher.

## Build and launch

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_square_mission
source install/setup.bash
ros2 launch bringup_square_mission square_mission_hw.launch.py \
  use_pointlio:=true use_pointlio_px4_visual_odom:=true \
  use_px4_control_watchdog:=true auto_arm:=false \
  takeoff_altitude:=0.40 square_side_length:=2.0 approach_speed:=0.25
```

On the first run, confirm sensors, PX4 Agent and local position are working. Manual arming is the default; the controller requests Offboard after PX4 is armed.

## Route and coordinates

The route origin is the PX4 local position captured at startup. Mission coordinates are `(right, forward, up)`. The default route climbs to 0.30 m, moves forward, right and backward, returns above the origin, then descends. Yaw is locked to the initial heading.

Point-LIO `base_link` represents the MID-360 IMU origin. The bridge applies the `base_link_to_base` extrinsic from the launch file to compensate to the vehicle center. Do not apply the same PX4 external-vision offset a second time.

## Preflight checks

```bash
ros2 topic hz /odom
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic hz /fmu/in/trajectory_setpoint
```

Confirm `xy_valid` and `z_valid` are `true`. Use `ros2 topic info -v /fmu/in/trajectory_setpoint` to verify there is only one control publisher. See `launch/square_mission_hw.launch.py` for all parameters.
