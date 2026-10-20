# bringup_full_mission

Multi-task waypoint controller for PX4 DDS. Point-LIO provides external visual odometry to PX4 EKF2. A downward-facing D435i can correct work points; the upward-facing camera configuration supports lateral correction during ring traversal. `full_mission_controller.py` manages mission execution and actuator commands.

## Build and launch

Requires ROS 2 Humble, `px4_msgs` matching PX4 1.14, Point-LIO and connected cameras.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_full_mission
source install/setup.bash
ros2 launch bringup_full_mission full_mission_hw.launch.py \
  use_pointlio:=true use_pointlio_px4_visual_odom:=true \
  use_px4_control_watchdog:=true auto_arm:=false \
  selected_tasks:=1,3,5,7 \
  drop_port:=/dev/ttyUSB0 approach_speed:=0.25
```

Manual arming is the default. The actuator command `drop_command` is empty by default; verify the serial port, command script and propeller-off behavior before setting it.

## Tasks and corrections

Waypoint coordinates use `(right, forward, up)`, relative to the position and heading captured at startup. Tasks 1-5 can use the downward-facing camera for horizontal correction around black frames. The ring phase can use the forward-facing camera for lateral correction. Set camera namespaces, serial numbers and correction thresholds through launch arguments.

PX4 local position provides control feedback. Check the validity flags in `/fmu/out/vehicle_local_position`, along with visual odometry, camera topics and setpoint rate. Before running, verify `selected_tasks`, camera serial numbers, `drop_command` and the serial port. See `launch/full_mission_hw.launch.py` for all parameters.
