# bringup_ego_mpc

EGO B-spline 轨迹经线性 MPC 转为 PX4 NED 加速度 setpoint。MPC 约束预测时域内的速度、加速度与 jerk；求解失败、轨迹过期或定位不健康时回到位置保持。

## 构建与检查

依赖 OSQP。首次使用时安装兼容版本：

~~~bash
python3 -m pip install --user 'osqp>=0.6.3,<2'
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup_ego_mpc
source install/setup.bash
ros2 launch bringup_ego_mpc ego_mpc_hw.launch.py \
  use_px4_control_watchdog:=true output_enabled:=false
~~~

默认不发布 PX4 控制输出、不自动解锁，也不自动发送起始目标。检查 `/ego/odom_base`、`/ego/planning/bspline`、`/ego_hw/diagnostics` 与 `/ego_mpc/solver_diagnostics` 后，再进行拆桨测试。

## 开启输出

硬件输出需要显式通过两个门控参数；默认值保持关闭：

~~~bash
ros2 launch bringup_ego_mpc ego_mpc_hw.launch.py \
  output_enabled:=true hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false auto_offboard:=false
~~~

Point-LIO/EGO 使用 ROS FLU，控制器转换为 PX4 NED。调整速度、加速度或 MPC 求解时间时，需同时检查 `config/mpc_params.yaml` 与 EGO planner 配置，并确认求解最坏时间低于控制周期。主要入口：`scripts/ego_mpc_controller.py`、`scripts/constrained_mpc.py`、`scripts/reference_sampler.py`。
