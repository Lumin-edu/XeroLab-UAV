# bringup_ego_multi_mission

Hardware multi-waypoint mission using Point-LIO and EGO Planner. The route is defined in `config/multi_waypoints.yaml`. The runner publishes each goal in sequence and advances only after reaching and settling at the waypoint. It stops the route on timeout or odometry loss.

## Build and inspect

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_ego_multi_mission
source install/setup.bash
ros2 launch bringup_ego_multi_mission ego_multi_mission_hw.launch.py \
  use_livox_driver:=false use_pointlio:=true output_enabled:=false
```

Use this mode to check sensors, fused odometry, point clouds and planning results. Route YAML uses `(right, forward, up)` coordinates; the node converts them to the FLU coordinates used by Point-LIO and EGO.

## Control output

The launch defaults are `output_enabled:=false` and `auto_arm:=false`. After propeller-off verification, check `/ego/odom_fused`, `/ego/cloud_registered_fused`, `/ego/position_cmd` and PX4 status before setting `output_enabled:=true` and `hardware_confirmation:=ENABLE_PX4_OUTPUT`.

Review a copy of the route before running it. Check the flight area, speed limits, waypoint tolerance and timeout. Run only one Offboard controller at a time. See `launch/ego_multi_mission_hw.launch.py` for parameters.
