#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构造整套点到点飞行集成的 ROS 2 启动描述。

    调用：由 `ros2 launch action action.launch.py` 自动调用。函数声明启动参数，
    默认依次启动 Livox MID360s 驱动、FAST-LIO、SUPER 的 `fsm_node` 和本包控制器。
    可用 `start_livox_driver`、`start_fastlio` 关闭相应进程。
    """
    action_share = FindPackageShare("action")
    livox_share = FindPackageShare("livox_ros_driver2")
    fast_lio_share = FindPackageShare("fast_lio")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox_driver = LaunchConfiguration("start_livox_driver")
    livox_driver_launch_file = LaunchConfiguration("livox_driver_launch_file")
    start_fastlio = LaunchConfiguration("start_fastlio")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    super_config = LaunchConfiguration("super_config")
    controller_config = LaunchConfiguration("controller_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    )
    declare_start_livox_driver = DeclareLaunchArgument(
        "start_livox_driver",
        default_value="true",
        description="Start the Livox driver (MID360 by default)",
    )
    declare_livox_driver_launch_file = DeclareLaunchArgument(
        "livox_driver_launch_file",
        default_value="msg_MID360s_launch.py",
        description="Launch file under livox_ros_driver2/launch_ROS2",
    )
    declare_start_fastlio = DeclareLaunchArgument(
        "start_fastlio",
        default_value="true",
        description="Start fast_lio mapping.launch.py in this launch",
    )
    declare_fastlio_config_file = DeclareLaunchArgument(
        "fastlio_config_file",
        default_value="mid360s.yaml",
        description="FAST-LIO sensor configuration file",
    )
    declare_super_config = DeclareLaunchArgument(
        "super_config",
        default_value=PathJoinSubstitution(
            [action_share, "config", "super_real.yaml"]
        ),
    )
    declare_controller_config = DeclareLaunchArgument(
        "controller_config",
        default_value=PathJoinSubstitution(
            [action_share, "config", "action_controller.yaml"]
        ),
    )

    super_node = Node(
        package="super_planner",
        executable="fsm_node",
        name="fsm_node",
        output="screen",
        parameters=[
            {"config_path": super_config, "use_sim_time": use_sim_time}
        ],
    )
    controller_node = Node(
        package="action",
        executable="action_controller.py",
        name="fastlio_super_mavros_controller",
        output="screen",
        parameters=[controller_config, {"use_sim_time": use_sim_time}],
    )
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, "launch", "mapping.launch.py"])
        ),
        launch_arguments={
            "config_file": fastlio_config_file,
            "rviz": "false",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(start_fastlio),
    )
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [livox_share, "launch_ROS2", livox_driver_launch_file]
            )
        ),
        condition=IfCondition(start_livox_driver),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_start_livox_driver,
            declare_livox_driver_launch_file,
            declare_start_fastlio,
            declare_fastlio_config_file,
            declare_super_config,
            declare_controller_config,
            livox_driver_launch,
            fastlio_launch,
            super_node,
            controller_node,
        ]
    )
