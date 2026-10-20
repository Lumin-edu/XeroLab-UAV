# bringup_ego_mavros

Point-LIO and MAVROS fixed-point control for PX4. This package is independent of the PX4 uXRCE-DDS bringups on `main`.

~~~text
Point-LIO /odom -> /mavros/vision_pose/pose -> PX4 EKF
/mavros/local_position/odom -> fixed_point_mavros.py
                            -> /mavros/setpoint_raw/local
~~~

## Build and launch

Start MAVROS separately with the connection string for the flight controller.

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_ego_mavros
source install/setup.bash
ros2 launch bringup_ego_mavros fixed_point_mavros.launch.py \
  target_x:=1.0 target_y:=2.0 target_z:=0.5 \
  auto_arm:=false auto_offboard:=true
~~~

The launch starts Point-LIO and the vision bridge by default. Set `use_pointlio:=false` when Point-LIO is already running. Targets are offsets from the captured pose in `(right, forward, up)`; MAVROS performs ENU/NED conversion.

## Check

~~~bash
ros2 topic hz /odom
ros2 topic hz /mavros/vision_pose/pose
ros2 topic echo /mavros/local_position/odom --once
ros2 topic echo /mavros/state --once
~~~

Keep `auto_arm:=false` until local odometry and PX4 state are valid. Only one node should publish a setpoint. Parameters are declared in `launch/fixed_point_mavros.launch.py`.
