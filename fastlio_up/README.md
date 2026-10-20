# fastlio_up

Standalone ROS 2 fixed-point control using FAST-LIO, MAVROS and PX4. The package bridges FAST-LIO odometry to MAVROS external vision, then sends a position target through MAVROS.

~~~text
FAST-LIO /Odometry -> /mavros/vision_pose/pose -> PX4 EKF
/mavros/local_position/odom -> fixed_point_mavros.py
                            -> /mavros/setpoint_position/local
~~~

## Build and launch

Start MAVROS separately with the serial or UDP URL for the flight controller.

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fastlio_up
source install/setup.bash
ros2 launch fastlio_up fixed_point_mavros.launch.py \
  target_x:=0.0 target_y:=0.0 target_z:=0.4 \
  auto_arm:=false auto_offboard:=true
~~~

The launch starts FAST-LIO and the pose bridge by default. Use `use_fastlio:=false` if an existing FAST-LIO node publishes `/Odometry`.

Targets use the captured body frame: `(x, y, z) = (right, forward, up)`. The controller captures a local position and heading after `reference_capture_delay_sec`, then converts the offset once into the fixed local target. MAVROS handles ENU/NED conversion.

## Check

~~~bash
ros2 topic hz /Odometry
ros2 topic hz /mavros/vision_pose/pose
ros2 topic echo /mavros/local_position/odom --once
ros2 topic echo /mavros/state --once
~~~

Keep `auto_arm:=false` until the estimator and MAVROS state are valid. Do not run another Offboard controller at the same time. See `launch/fixed_point_mavros.launch.py` and `config/` for parameters.

