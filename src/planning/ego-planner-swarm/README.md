# EGO Planner integration

This directory retains the EGO Planner ROS 2 packages used by the hardware bringups in `src/up/`. The planner consumes odometry and point clouds, publishes B-spline trajectories, and `traj_server` samples them as `quadrotor_msgs/PositionCommand` messages.

## Build

From the workspace root, build the planner packages together with the retained trajectory message package:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src/control/quadrotor_msgs src/planning/ego-planner-swarm/src/planner \
  --packages-up-to ego_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Use the hardware launch files in `src/up/bringup_ego`, `src/up/bringup_ego_multi_mission` or `src/up/bringup_ego_mpc`. Simulator nodes, simulator support packages and simulation-only launch files have been removed from this checkout.

## Upstream

The planner is based on [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm), distributed under GPL-3.0. The retained `quadrotor_msgs/PositionCommand` interface comes from that project and is maintained in `src/control/quadrotor_msgs`. See the repository root README for the other integrated projects.
