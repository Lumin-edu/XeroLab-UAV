# UAV Mission Stack

ROS workspace for UAV missions and navigation. Flight interfaces and ROS versions are maintained on separate branches:

| Branch | Environment | Scope |
| --- | --- | --- |
| [`main`](https://github.com/Lumin-edu/uav-mission-stack/tree/main) | ROS 2 Humble | PX4 uXRCE-DDS hardware missions and EGO Planner |
| [`mavros`](https://github.com/Lumin-edu/uav-mission-stack/tree/mavros) | ROS 2 Humble | FAST-LIO, Point-LIO and SUPER integrations through MAVROS |
| [`ros1`](https://github.com/Lumin-edu/uav-mission-stack/tree/ros1) | ROS 1 Noetic | FAST-LIO/MAVROS position control, visual-tag missions and servo interface |

This repository does not include PX4 firmware, ROS distributions or flight hardware. Switch to the relevant branch and follow its README before building. Simulation packages are not included.

## `main` workspace

This branch targets Ubuntu 22.04, ROS 2 Humble, PX4 1.14 and Livox MID-360. The primary localization path is:

```text
MID-360 -> livox_ros_driver2 -> Point-LIO -> PX4 EKF2
                                      |           |
                                      +-> EGO     +-> Offboard mission
```

Key directories:

```text
src/driver/          Livox driver
src/perception/      Point-LIO and FAST-LIO
src/control/         px4_msgs for PX4 1.14
src/planning/        EGO Planner
src/up/              Hardware hover, waypoint, EGO and MPC bringups
src/fastlio_bringup/ FAST-LIO + PX4 DDS position hold
```

## Build hardware packages

With ROS 2 Humble, `rosdep` and `colcon` installed on Ubuntu 22.04:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths \
  src/driver/livox_ros_driver2 \
  src/perception/Point-LIO \
  src/control/px4_msgs \
  src/planning/ego-planner-swarm/src/planner \
  src/up src/fastlio_bringup \
  --ignore-src --rosdistro humble -r -y

colcon build --symlink-install \
  --base-paths \
    src/driver/livox_ros_driver2 \
    src/perception/Point-LIO \
    src/control/px4_msgs \
    src/planning/ego-planner-swarm/src/planner \
    src/up src/fastlio_bringup \
  --packages-up-to \
    bringup_pointlio_hover bringup_square_mission bringup_full_mission \
    bringup_ego bringup_ego_multi_mission bringup_ego_mpc fastlio_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The explicit base paths limit discovery to the hardware packages in this branch.

## Hardware entry points

| Mission | Launch command |
| --- | --- |
| Point-LIO position hold | `ros2 launch bringup_pointlio_hover hover_hw.launch.py` |
| Square waypoint route | `ros2 launch bringup_square_mission square_mission_hw.launch.py` |
| EGO obstacle avoidance | `ros2 launch bringup_ego ego_avoidance_hw.launch.py` |
| EGO multi-waypoint mission | `ros2 launch bringup_ego_multi_mission ego_multi_mission_hw.launch.py` |
| EGO + MPC | `ros2 launch bringup_ego_mpc ego_mpc_hw.launch.py` |
| FAST-LIO position hold | `ros2 launch fastlio_bringup fastlio_hover.launch.py` |

Read the package README and launch arguments before use. PX4 output is disabled by default or requires manual arming. Run one Offboard controller at a time. Before flight, check localization validity, setpoint publishers, sensor extrinsics, the flight area and manual takeover. Match the `px4_msgs` branch to the PX4 firmware version.

## Upstream acknowledgements

This workspace incorporates or adapts the open-source projects below. Refer to each project's included license for its terms. Mission logic not identified as upstream is maintained in this repository.

| Project | Use | Source |
| --- | --- | --- |
| Livox ROS Driver 2 | MID-360 driver | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) |
| Point-LIO | LiDAR-inertial localization | [HY-LiYihan/Point-LIO](https://github.com/HY-LiYihan/Point-LIO), originally [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) |
| FAST-LIO | LiDAR-inertial localization | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) |
| `px4_msgs` | PX4 uORB ROS 2 messages | [PX4/px4_msgs](https://github.com/PX4/px4_msgs) |
| EGO Planner Swarm | Local trajectory planning | [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) |

Upstream source keeps its original license and README. This project README follows GitHub's guidance for project overview, setup, usage and attribution: [About READMEs](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-readmes).
