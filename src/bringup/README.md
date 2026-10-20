# bringup

ROS 1 Noetic mission package for MID-360, FAST-LIO and MAVROS. MAVROS is started separately. The package includes fixed-point control, AprilTag-assisted tasks, camera/servo nodes and monitoring tools.

## Build

From the workspace root:

~~~bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src/bringup src/perception/FAST_LIO \
  --ignore-src --rosdistro noetic -r -y
catkin_make --pkg bringup
source devel/setup.bash
~~~

Build/install the Livox ROS 1 driver and MAVROS separately using their upstream instructions.

## Fixed-point flight

Start MAVROS first, then launch the mission package:

~~~bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
roslaunch bringup hover_hw.launch \
  use_fastlio:=true use_fastlio_mavros_visual_odom:=true \
  use_hover_control:=true target_x:=0.0 target_y:=0.0 target_z:=0.4 \
  lock_yaw:=true hover_auto_arm:=false
~~~

Targets are offsets from the captured local position, in `(right, forward, up)`. The visual odometry bridge publishes ROS ENU to MAVROS; MAVROS handles PX4 NED conversion. The default lever arm is zero. Keep manual arming enabled until `/mavros/state` and `/mavros/local_position/pose` are valid.

## Tag mission and servo

The AprilTag mission visits configured targets, applies bounded horizontal corrections and runs the configured drop/landing action. Configure route and camera IDs in `config/tag_drop_targets.yaml` and `config/apriltag_settings.yaml`, then launch:

~~~bash
roslaunch bringup tag_drop_mission.launch \
  video_device:=/dev/video0 servo_port:=/dev/ttyUSB0 \
  drop_tag_id:=0 landing_tag_id:=0
~~~

Test the actuator without connecting to hardware:

~~~bash
python3 paotou.py --port /dev/ttyUSB0 --angle 90 --dry-run
~~~

For hardware testing, confirm servo model, port and angle range before removing `--dry-run`. Never run `hover_hw.launch`, `up.launch` and `tag_drop_mission.launch` together; they can publish competing setpoints.

## Check

~~~bash
rostopic echo /mavros/state
rostopic echo /mavros/local_position/pose
rostopic echo /tag_detections
rostopic echo /tag_drop_mission/mission_phase
~~~

FAST-LIO, Livox and MAVROS source/license details are linked from the branch README.

