# drone_detect

Optional ROS 2 package for detecting other aircraft in a depth image and estimating their pose error in the observing aircraft's world frame. It is not started by the hardware mission bringups.

## Build and launch

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to drone_detect
source install/setup.bash
ros2 launch drone_detect drone_detect.launch.py my_id:=0 \
  odom_topic:=/mavros/local_position/odom
```

The launch file loads `config/camera.yaml` and `config/default.yaml`. Update the camera intrinsics and topic remappings for the installed sensor and aircraft IDs before use.

## Interfaces

The node subscribes to `odometry`, `depth` and `/others_odom`. It publishes `new_depth_image`, `debug_depth_image`, `/debug_info` and per-aircraft pose-error topics named `drone<ID>to<MY_ID>_pose_err`. Its camera model, pixel threshold and expected aircraft dimensions are configured through ROS parameters.

This package is adapted from [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm), distributed under GPL-3.0.
