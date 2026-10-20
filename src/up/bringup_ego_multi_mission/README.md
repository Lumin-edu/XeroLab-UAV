# bringup_ego_multi_mission

基于 Point-LIO 与 EGO Planner 的实机多航点任务。路线由 `config/multi_waypoints.yaml` 定义；运行器逐点发布目标，到点并稳定后才继续。超时或里程计失效时停止路线。

## 构建与检查

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_ego_multi_mission
source install/setup.bash
ros2 launch bringup_ego_multi_mission ego_multi_mission_hw.launch.py \
  use_livox_driver:=false use_pointlio:=true output_enabled:=false
~~~

此模式用于确认传感器、融合里程计、点云和规划结果。路线 YAML 使用 `(右, 前, 上)` 坐标；节点发布前转换到 Point-LIO/EGO 的 FLU 坐标。

## 控制输出

launch 默认 `output_enabled:=false`、`auto_arm:=false`。只有完成无桨验证并检查 `/ego/odom_fused`、`/ego/cloud_registered_fused`、`/ego/position_cmd` 和 PX4 状态后，才显式打开 `output_enabled:=true` 并传入 `hardware_confirmation:=ENABLE_PX4_OUTPUT`。

运行前编辑路线副本，核对区域、限速、到点容差和超时。一次只运行一个 Offboard 控制器。参数见 `launch/ego_multi_mission_hw.launch.py`。
