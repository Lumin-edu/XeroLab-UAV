# bringup_ego

MID-360、Point-LIO、EGO Planner 与 PX4 Offboard 实机 bringup。规划器订阅原始障碍点云和经过机体中心补偿的里程计；PX4 桥接节点负责最终 setpoint。

~~~text
/livox/lidar + /livox/imu -> Point-LIO -> /odom + /cloud_registered
                                      -> EGO -> /ego/position_cmd
                                      -> ego_px4_bridge -> PX4 DDS
~~~

## 构建与规划检查

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_ego
source install/setup.bash
ros2 launch bringup_ego ego_avoidance_hw.launch.py \
  use_livox_driver:=false use_pointlio:=true \
  use_px4_control_watchdog:=true output_enabled:=false
~~~

Livox 驱动和 Micro XRCE-DDS Agent 需按硬件环境启动。默认关闭 PX4 控制输出，适合先检查点云、里程计、规划轨迹和诊断。当前任务坐标为 `(右, 前, 上)`；配置、外参和障碍膨胀参数见 `config/ego_planner_hw.yaml`。

## 开启 PX4 输出

完成无桨检查并确认 PX4 local position、视觉里程计、EGO 轨迹及 setpoint 独占正常后，才打开双重输出门：

~~~bash
ros2 launch bringup_ego ego_avoidance_hw.launch.py \
  output_enabled:=true hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false auto_offboard:=true
~~~

仅运行一个 Offboard 控制器。自动降落默认关闭；更多参数和检查话题见 `launch/ego_avoidance_hw.launch.py`。
