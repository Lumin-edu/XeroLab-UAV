# UAV Mission Stack

无人机任务与导航 ROS 工作空间。不同飞控接口和 ROS 版本放在独立分支：

| 分支 | 环境 | 内容 |
| --- | --- | --- |
| [`main`](https://github.com/Lumin-edu/uav-mission-stack/tree/main) | ROS 2 Humble | PX4 uXRCE-DDS 实机任务与 EGO Planner |
| [`mavros`](https://github.com/Lumin-edu/uav-mission-stack/tree/mavros) | ROS 2 Humble | 通过 MAVROS 连接 PX4 的 FAST-LIO、Point-LIO 与 SUPER 集成 |
| [`ros1`](https://github.com/Lumin-edu/uav-mission-stack/tree/ros1) | ROS 1 Noetic | FAST-LIO/MAVROS 定点控制、视觉标签任务和舵机接口 |

本仓库不提供 PX4 固件、ROS 发行版或飞行硬件驱动安装包。请先切换到对应分支，再按该分支 README 部署。

## `main` 工作空间

当前分支面向 Ubuntu 22.04、ROS 2 Humble、PX4 1.14 和 Livox MID-360。主定位链路为：

```text
MID-360 -> livox_ros_driver2 -> Point-LIO -> PX4 EKF2
                                      |           |
                                      +-> EGO     +-> Offboard mission
```

主要目录：

```text
src/driver/       Livox 驱动
src/perception/   Point-LIO 与 FAST-LIO
src/control/      PX4 1.14 的 px4_msgs
src/planning/     EGO Planner
src/up/           实机悬停、航点、EGO 与 MPC bringup
src/fastlio_bringup/ FAST-LIO + PX4 DDS 定点悬停
```

## 构建实机任务

在 Ubuntu 22.04 已安装 ROS 2 Humble、`rosdep` 和 `colcon` 后：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths \
  src/driver/livox_ros_driver2 \
  src/perception/Point-LIO \
  src/control/px4_msgs \
  src/planning/ego-planner-swarm/src/planner \
  src/up src/fastlio_bringup \
  --ignore-src --rosdistro humble -r -y

colcon build --symlink-install \
  --base-paths \
    src/driver/livox_ros_driver2 \
    src/perception/Point-LIO \
    src/control/px4_msgs \
    src/planning/ego-planner-swarm/src/planner \
    src/up src/fastlio_bringup \
  --packages-up-to \
    bringup_pointlio_hover bringup_square_mission bringup_full_mission \
    bringup_ego bringup_ego_multi_mission bringup_ego_mpc fastlio_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

此命令仅构建当前分支包含的实机 ROS 包。

## 实机入口

| 任务 | 启动命令 |
| --- | --- |
| Point-LIO 定点悬停 | `ros2 launch bringup_pointlio_hover hover_hw.launch.py` |
| 正方形航点 | `ros2 launch bringup_square_mission square_mission_hw.launch.py` |
| EGO 避障 | `ros2 launch bringup_ego ego_avoidance_hw.launch.py` |
| EGO 多航点 | `ros2 launch bringup_ego_multi_mission ego_multi_mission_hw.launch.py` |
| EGO + MPC | `ros2 launch bringup_ego_mpc ego_mpc_hw.launch.py` |
| FAST-LIO 定点悬停 | `ros2 launch fastlio_bringup fastlio_hover.launch.py` |

先阅读对应包的 README 和 launch 参数。PX4 输出默认关闭或需要手动解锁；每次只运行一个 Offboard 控制器。实飞前检查定位有效性、控制话题发布者数量、外参、飞行区域和遥控器接管方式。`px4_msgs` 分支须与 PX4 固件版本匹配。

## 上游鸣谢

仓库集成或改编了以下开源项目；具体许可和使用条件以各项目随附的许可证为准。未列为上游的任务逻辑由本仓库维护。

| 项目 | 用途 | 来源 |
| --- | --- | --- |
| Livox ROS Driver 2 | MID-360 驱动 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) |
| Point-LIO | LiDAR-惯性定位 | [HY-LiYihan/Point-LIO](https://github.com/HY-LiYihan/Point-LIO)，原始项目 [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) |
| FAST-LIO | LiDAR-惯性定位 | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) |
| `px4_msgs` | PX4 uORB ROS 2 消息 | [PX4/px4_msgs](https://github.com/PX4/px4_msgs) |
| EGO Planner Swarm | 局部轨迹规划 | [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) |

上游源码保留其许可证和原始 README；本仓库文档遵循 GitHub README 的项目入口、构建、运行和致谢结构，参见 [GitHub README 指南](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-readmes)。
