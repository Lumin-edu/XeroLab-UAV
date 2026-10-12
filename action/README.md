# action

`action` 是一个独立的 ROS 2 集成包，用于结合 FAST-LIO 外部定位和 SUPER
避障规划，实现无人机定点飞行：

```text
FAST-LIO /Odometry ──┬── action_controller ── /mavros/vision_pose/pose
                     └── SUPER（输入 /cloud_registered）
SUPER /planning_cmd/poly_traj ── action_controller ── /mavros/setpoint_raw/local
                       └──────── 完整多项式 ── /action/super_polynomial_trajectory
```

本包不导入或链接 `fastlio_up`。`scripts/` 中的坐标和轨迹辅助代码独立实现。
本包只有 `action_controller.py` 会发布 MAVROS setpoint，因此不要同时运行
`fixed_point_mavros.py`。

## 编译

请在工作区中构建 `action` 及其依赖，包括 `mars_quadrotor_msgs`、
`super_planner` 和 FAST-LIO：

```bash
cd /home/duanyh/sysu/dyh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to action
source install/setup.bash
```

## 启动

启动文件会依次启动 Livox MID360 驱动、FAST-LIO、SUPER 和 MAVROS 桥接控制节点：

```bash
ros2 launch action action.launch.py
```

默认配置与 `fast_lio/config/mid360.yaml` 匹配：Livox 驱动在 `/livox/lidar` 和
`/livox/imu` 发布自定义消息，FAST-LIO 订阅这两个话题。驱动从
`livox_ros_driver2/config/MID360_config.json` 读取网络配置。当前配置中的主机 IP
为 `192.168.1.50`，MID360 IP 为 `192.168.1.182`。启动前请按实际雷达网口和设备地址
修改该配置文件。

如果 Livox 驱动已在其他终端运行，可关闭启动文件中的驱动。使用 rosbag 或已有
FAST-LIO 实例时，也可以单独关闭 FAST-LIO：

```bash
ros2 launch action action.launch.py start_livox_driver:=false
ros2 launch action action.launch.py start_livox_driver:=false start_fastlio:=false
```

可以通过 `livox_driver_launch_file` 更换驱动启动文件，并选择与传感器匹配的 FAST-LIO
配置。不要在另一个终端重复启动同一台 MID360 的驱动。

默认的 `super_real.yaml` 基于 ROS 2 SUPER 示例配置，已修改以下集成项：

- 里程计话题：`/Odometry`
- 注册点云话题：`/cloud_registered`
- 目标话题：`/goal_pose`
- `click_height: -10.0`，表示保留目标中的 z 值
- 使用适合真实无人机的较保守速度和加速度限制
- `rog_map.virtual_ground_height: -0.7`，按当前栅格量化后将走廊底界降到约 `-0.1 m`
- 备份轨迹使用 `penna_thr: 1.0e+5`，并通过 `backup_stop_time_scale: 1.5` 保留制动时间余量

`super_planner.backup_traj_en` 目前只是 SUPER 的历史配置项，源码不会用它跳过备份轨迹规划。
备份轨迹仍会参与整条轨迹的提交；如果备份优化失败，SUPER 不会发布主轨迹，控制器会继续保持当前位置。

## 发送目标点

在 FAST-LIO/SUPER 世界坐标系中向 `/action/goal` 发布目标点。控制器会先缓存目标；
等 MAVROS 报告无人机已解锁并进入 `OFFBOARD` 模式后，再将目标转发给 SUPER 的
`/goal_pose`。目标四元数用于设置 yaw；单位四元数表示 yaw 为零。

```bash
ros2 topic pub --once /action/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world},
     pose: {position: {x: 1.0, y: -1.0, z: 1.0},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

无人机需连接 MAVROS。默认需要手动解锁；只有在
`config/action_controller.yaml` 中启用 `auto_arm` 后，控制器才会自动请求解锁。目标
缓存后，控制器会先持续发送 raw setpoint，完成预流后请求进入 `OFFBOARD`。MAVROS
确认无人机已解锁并处于 `OFFBOARD` 后，控制器才会把目标发送给 SUPER，避免 SUPER
轨迹时钟在等待进入 `OFFBOARD` 时提前运行。

## 平滑轨迹执行

SUPER 使用分段七阶位置和 yaw 多项式进行规划。规划轨迹提交后，SUPER 会在
`/planning_cmd/poly_traj` 发布完整系数，随后在同一话题发布不含系数的心跳。控制器会
缓存完整多项式，并依据 ROS 时钟、SUPER 提供的绝对轨迹起始时间和每段持续时间进行
采样。完整多项式消息也会转发到 `/action/super_polynomial_trajectory`，方便录制和检查。

控制器每个周期将采样得到的位置、速度、加速度、yaw 和 yaw rate 填入
`mavros_msgs/PositionTarget`，发布到 `/mavros/setpoint_raw/local`。这样飞控能使用
SUPER 多项式的速度和加速度前馈，不会只跟踪一串离散位置点。ROS 侧数值仍使用 ENU；
MAVROS 会负责转换到 NED。规划坐标到 MAVROS 本地坐标的固定原点偏移只应用于位置。
`PositionTarget` 没有 jerk 字段，因此 jerk 保留在完整七阶多项式表示中，但不会单独
发送给飞控。

`config/action_controller.yaml` 中的 `max_velocity_mps` 默认设为 `1.0`。控制器会为
整条多项式计算保守的速度上界，并按统一时间比例放慢轨迹；位置沿原规划路径推进，速度、
加速度和 yaw rate 按时间缩放关系同步调整。这样发布给 MAVROS 的速度向量模长不超过
配置值，同时保持位置、速度和加速度之间的轨迹关系。SUPER 的
`traj_opt.boundary.max_vel` 也设为 `1.0`，但它是优化器的软惩罚参数，不能单独作为硬限速。
这个设置约束 action 发布的轨迹指令；飞机实际速度还受飞控响应和惯性影响，若需要硬件侧
的物理限速，还需在飞控中设置对应速度限制。

默认启用加速度前馈：`acceleration_feedforward: true`。如果当前飞控固件不支持本地
加速度 setpoint，可在 `config/action_controller.yaml` 中关闭。飞行前请在 SITL 或受控
台架上确认所用 PX4/飞控固件支持该功能。预流和安全保持时使用最新本地位置与 yaw，
速度、加速度和 yaw rate 均设为零；SUPER 心跳超时后，目标会冻结在当前测得的本地位置。

## 坐标和标定约定

- FAST-LIO 的 `/Odometry`、`/cloud_registered`、目标点和 SUPER 的
  `/planning_cmd/poly_traj` 必须使用相同数值的 ENU 世界坐标。SUPER 不会根据输入消息
  的 `frame_id` 执行坐标变换。
- MAVROS 执行常规 ENU/NED 转换。`action` 不会手动交换坐标轴或翻转 z。
- FAST-LIO 首个有效位姿作为外部视觉位姿流的本地原点。已验证配置中的
  `planner_to_mavros_offset_xyz` 为零；只有在测得 MAVROS 本地 ENU 原点存在固定偏移时才
  设置该参数。
- 默认机体外参为单位变换，与已验证的 FAST-LIO/MAVROS 标定一致。如果存在非零杆臂，
  需要同时针对 SUPER 使用的物理参考点和视觉位姿进行标定。

## 监控话题

```bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/odom
ros2 topic echo /planning_cmd/poly_traj
ros2 topic echo /mavros/setpoint_raw/local
```
