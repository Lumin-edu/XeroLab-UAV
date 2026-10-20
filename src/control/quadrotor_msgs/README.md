# quadrotor_msgs

Minimal ROS 2 message package for the `PositionCommand` interface consumed by EGO Planner trajectory management and the PX4 hardware bridges.

Build it together with the planner:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src/control/quadrotor_msgs src/planning/ego-planner-swarm/src/planner \
  --packages-up-to ego_planner
```

The message is retained from [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) under GPL-3.0. The unused simulator messages and helper libraries were removed with the simulation packages.
