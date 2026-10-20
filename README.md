# UAV Mission Stack · MAVROS

ROS 2 Humble packages for connecting PX4 through MAVROS. This branch contains the MAVROS variants; PX4 uXRCE-DDS tasks are maintained on `main`, and ROS 1 Noetic tasks on `ros1`. Simulation has been removed.

## Packages

| Path | Function |
| --- | --- |
| `fastlio_up/` | FAST-LIO localization with MAVROS fixed-point control |
| `src/mavros_up/` | Point-LIO localization with MAVROS fixed-point control |
| `action/` | FAST-LIO + SUPER trajectory planning with MAVROS setpoints |
| `src/driver/`, `src/perception/` | Livox driver and localization sources |

The two fixed-point packages are alternatives. `action` is a separate controller; run only one package that publishes flight setpoints.

## Build

On Ubuntu 22.04 with ROS 2 Humble and MAVROS installed:

~~~bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths \
  src/driver/livox_ros_driver2 src/perception/FAST_LIO \
  src/perception/Point-LIO fastlio_up src/mavros_up \
  --ignore-src --rosdistro humble -r -y
colcon build --symlink-install \
  --packages-up-to fastlio_up bringup_ego_mavros
source install/setup.bash
~~~

The `action` package also requires `super_planner` and `mars_quadrotor_msgs` from an external ROS 2 overlay. Those packages are not included here. The SUPER planner source is available at [hku-mars/SUPER](https://github.com/hku-mars/SUPER); the message package must be supplied by the SUPER workspace or the deployment overlay used by your team.

## Start

Start MAVROS in its own terminal, then start one selected bringup. See each package README for its launch command, topic checks and parameters.

- [FAST-LIO + MAVROS](fastlio_up/README.md)
- [Point-LIO + MAVROS](src/mavros_up/README.md)
- [SUPER trajectory integration](action/README.md)

## Acknowledgements

Vendored localization and driver source retain their original licenses and documentation.

| Project | Use | Source |
| --- | --- | --- |
| Livox ROS Driver 2 | MID-360 driver | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) |
| FAST-LIO | LiDAR-inertial localization | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) |
| Point-LIO | LiDAR-inertial localization | [HY-LiYihan/Point-LIO](https://github.com/HY-LiYihan/Point-LIO), originally [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) |
| MAVROS | PX4/MAVLink ROS bridge | [mavlink/mavros](https://github.com/mavlink/mavros) |
| SUPER | Polynomial trajectory planner | [hku-mars/SUPER](https://github.com/hku-mars/SUPER) |

`action` depends on external `super_planner` and `mars_quadrotor_msgs` packages; their source is not vendored in this repository. Keep their upstream attribution and license notices with the external overlay. The `mars_quadrotor_msgs` repository was not identifiable from the public GitHub repository search at the time of writing, so the package name is documented without an invented URL.
