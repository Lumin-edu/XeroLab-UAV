# bringup_ego

Hardware bringup for MID-360, Point-LIO, EGO Planner and PX4 Offboard. The planner subscribes to obstacle point clouds and body-center-compensated odometry; the PX4 bridge publishes the final setpoints.

```text
/livox/lidar + /livox/imu -> Point-LIO -> /odom + /cloud_registered
                                      -> EGO -> /ego/position_cmd
                                      -> ego_px4_bridge -> PX4 DDS
```

## Build and inspect planning

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_ego
source install/setup.bash
ros2 launch bringup_ego ego_avoidance_hw.launch.py \
  use_livox_driver:=false use_pointlio:=true \
  use_px4_control_watchdog:=true output_enabled:=false
```

Start the Livox driver and Micro XRCE-DDS Agent for the hardware setup. PX4 control output is disabled by default so you can inspect point clouds, odometry, planned trajectories and diagnostics first. Mission coordinates are `(right, forward, up)`. See `config/ego_planner_hw.yaml` for configuration, extrinsics and obstacle inflation parameters.

## Enable PX4 output

Enable both output gates only after a propeller-off check and verification of PX4 local position, visual odometry, the EGO trajectory and exclusive setpoint publishing:

```bash
ros2 launch bringup_ego ego_avoidance_hw.launch.py \
  output_enabled:=true hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false auto_offboard:=true
```

Run only one Offboard controller at a time. Automatic landing is disabled by default. See `launch/ego_avoidance_hw.launch.py` for additional parameters and diagnostic topics.
