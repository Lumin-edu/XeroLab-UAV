# UAV Mission Stack · ROS 1

ROS 1 Noetic workspace for FAST-LIO localization, MAVROS flight control and camera-guided missions. ROS 2 Humble packages live on `main` and `mavros`; this branch contains no simulation code.

## Contents

~~~text
src/bringup/                  catkin mission and MAVROS integration
src/perception/FAST_LIO/       FAST-LIO ROS 1 localization
src/driver/livox_ros_driver2/  Livox driver source and ROS 1 launch files
paotou.py                      standalone serial servo utility
~~~

## Requirements and build

Install Ubuntu 20.04, ROS Noetic, MAVROS, the Livox ROS 1 driver, and FAST-LIO dependencies first. Build the workspace package:

~~~bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src/bringup src/perception/FAST_LIO \
  --ignore-src --rosdistro noetic -r -y
catkin_make --pkg bringup
source devel/setup.bash
~~~

The Livox driver is distributed as upstream source in this tree; follow its ROS 1 build instructions in [its README](src/driver/livox_ros_driver2/README.md).

## Flight control

Start MAVROS in a separate terminal, then launch one bringup from [src/bringup](src/bringup/README.md). Mission targets use the captured body frame `(right, forward, up)`. Keep `hover_auto_arm:=false` until MAVROS state and local position are valid. Do not run multiple setpoint publishers at the same time.

## Third-party acknowledgements

Vendored source retains its upstream license and README.

| Project | Use | Source |
| --- | --- | --- |
| FAST-LIO | LiDAR-inertial localization | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) |
| Livox ROS Driver 2 | MID-360 driver | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) |
| MAVROS | PX4/MAVLink ROS bridge | [mavlink/mavros](https://github.com/mavlink/mavros) |

This README follows GitHub's project overview, setup, use and attribution guidance: [About READMEs](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-readmes).
