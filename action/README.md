# action

FAST-LIO, SUPER and MAVROS integration for point-to-point obstacle avoidance. FAST-LIO provides odometry and registered clouds; SUPER plans a polynomial trajectory; `action_controller` streams sampled position, velocity and acceleration setpoints through MAVROS.

~~~text
FAST-LIO -> SUPER -> /planning_cmd/poly_traj
                         -> action_controller -> /mavros/setpoint_raw/local
~~~

This package requires ROS 2 Humble, MAVROS, Livox ROS Driver 2, FAST-LIO, and the external ROS packages `super_planner` and `mars_quadrotor_msgs`. The latter two are not included in this repository and must be provided by a sourced overlay.

## Build and launch

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to action
source install/setup.bash
ros2 launch action action.launch.py
~~~

Start MAVROS separately. The launch starts Livox, FAST-LIO, SUPER and the controller by default. Disable `start_livox_driver` or `start_fastlio` if those processes are already running. Update the MID-360 network addresses in the driver's configuration before connecting hardware.

## Send a goal

After checking localization and MAVROS state, publish a target in the shared ENU world frame:

~~~bash
ros2 topic pub --once /action/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 1.0, y: -1.0, z: 1.0}, orientation: {w: 1.0}}}"
~~~

The controller waits for MAVROS to report armed and `OFFBOARD` before passing the goal to SUPER. Arming is manual by default. Check `/Odometry`, `/cloud_registered`, `/planning_cmd/poly_traj`, `/mavros/state` and `/mavros/setpoint_raw/local`.

Only this controller should publish MAVROS setpoints while it is active. Validate the sensor frame, local origin, acceleration support and failsafe behavior in SITL or a propeller-off test before flight.

## Credits

FAST-LIO, Livox ROS Driver 2 and MAVROS retain their upstream source, license and documentation. SUPER and `mars_quadrotor_msgs` are external dependencies; their source repository and license notices are not part of this checkout. See the branch README for the dependency list.

