# bringup_ego_mpc

Converts EGO B-spline trajectories into PX4 NED acceleration setpoints using linear MPC. The MPC constrains velocity, acceleration and jerk over the prediction horizon. It falls back to position hold if solving fails, the trajectory expires or localization becomes unhealthy.

## Build and inspect

OSQP is required. Install a compatible version if needed:

```bash
python3 -m pip install --user 'osqp>=0.6.3,<2'
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_ego_mpc
source install/setup.bash
ros2 launch bringup_ego_mpc ego_mpc_hw.launch.py \
  use_px4_control_watchdog:=true output_enabled:=false
```

By default, the package does not publish PX4 control output, arm automatically or send an initial goal. Inspect `/ego/odom_base`, `/ego/planning/bspline`, `/ego_hw/diagnostics` and `/ego_mpc/solver_diagnostics` before propeller-off testing.

## Enable output

Hardware output requires both explicit gates; both are disabled by default:

```bash
ros2 launch bringup_ego_mpc ego_mpc_hw.launch.py \
  output_enabled:=true hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false auto_offboard:=false
```

Point-LIO and EGO use ROS FLU coordinates; the controller converts them to PX4 NED. When tuning speed, acceleration or solver time, check both `config/mpc_params.yaml` and the EGO planner configuration. Confirm worst-case solver time stays below the control period. Main implementation files are `scripts/ego_mpc_controller.py`, `scripts/constrained_mpc.py` and `scripts/reference_sampler.py`.
