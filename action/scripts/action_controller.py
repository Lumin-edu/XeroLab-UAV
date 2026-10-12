#!/usr/bin/env python3

import math
import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from mars_quadrotor_msgs.msg import PolynomialTrajectory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from rigid_transform import normalize_quaternion
from polynomial_trajectory import (
    HEART_BEAT,
    POSITION_TRAJ,
    YAW_COMMAND,
    YAW_TRAJ,
    sample_trajectory,
    trajectory_velocity_bound,
    validate_trajectory,
)
from vision_pose_adapter import odometry_to_vision_pose


VISION_POSE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)


class FastlioSuperMavrosController(Node):
    """连接 FAST-LIO、SUPER 和 MAVROS 的 ROS 2 节点。

    FAST-LIO 的 `Odometry` 经外参补偿后发布为 MAVROS 外部视觉位姿；SUPER
    的完整七阶多项式由本节点按 ROS 时钟采样，再将位置、速度、加速度、yaw 和
    yaw rate 发布为 MAVROS raw local setpoint。目标回调会清空旧轨迹，定时器
    负责预流、超时保持、解锁和 OFFBOARD 请求。
    """

    def __init__(self) -> None:
        """声明参数、建立发布/订阅及服务客户端，并启动控制定时器。
        关键参数单位为秒或 Hz；`base_link_to_base_*` 是机体坐标外参，
        `planner_to_mavros_offset_xyz` 是规划世界原点到 MAVROS 本地原点的米制偏移。
        初始化失败（例如四元数或向量长度非法）会抛出 `ValueError`，节点不会启动。
        """
        super().__init__("fastlio_super_mavros_controller")

        # FAST-LIO 输出的机体位姿；坐标数值必须与 SUPER 的规划世界坐标一致，单位米。
        self.fastlio_odom_topic = str(
            self.declare_parameter("fastlio_odom_topic", "/Odometry").value
        )
        # 送入 MAVROS 外部视觉融合的 PoseStamped 话题；只承载归零后的位姿，不是飞行指令。
        self.vision_pose_topic = str(
            self.declare_parameter(
                "vision_pose_topic", "/mavros/vision_pose/pose"
            ).value
        )
        # SUPER 发布完整 PolynomialTrajectory 的话题；包含分段七阶位置/yaw 多项式和心跳。
        self.planning_trajectory_topic = str(
            self.declare_parameter(
                "planning_trajectory_topic", "/planning_cmd/poly_traj"
            ).value
        )
        # 原样转发经过校验的 SUPER 轨迹，供诊断或其他节点观察，不参与飞控控制闭环。
        self.trajectory_passthrough_topic = str(
            self.declare_parameter(
                "trajectory_passthrough_topic",
                "/action/super_polynomial_trajectory",
            ).value
        )
        # 用户提交任务目标的入口；收到后先缓存，待 OFFBOARD 已建立再发送给 SUPER。
        self.goal_topic = str(
            self.declare_parameter("goal_topic", "/action/goal").value
        )
        # SUPER 的目标输入；PoseStamped 的 xyz 按数值解释为共享 ENU 世界坐标。
        self.planner_goal_topic = str(
            self.declare_parameter("planner_goal_topic", "/goal_pose").value
        )
        # MAVROS 状态反馈，决定是否允许解锁、切换 OFFBOARD 和发送任务目标。
        self.state_topic = str(
            self.declare_parameter("state_topic", "/mavros/state").value
        )
        # MAVROS 本地里程计；用于安全保持位置、判断反馈新鲜度和启动 setpoint 预流。
        self.local_odom_topic = str(
            self.declare_parameter(
                "local_odom_topic", "/mavros/local_position/odom"
            ).value
        )
        # 发给 MAVROS/PX4 的原始本地目标，PositionTarget 同时携带 P/V/A、yaw 和 yaw_rate。
        self.setpoint_topic = str(
            self.declare_parameter(
                "setpoint_topic", "/mavros/setpoint_raw/local"
            ).value
        )
        self.arming_service = str(
            self.declare_parameter("arming_service", "/mavros/cmd/arming").value
        )
        self.mode_service = str(
            self.declare_parameter("mode_service", "/mavros/set_mode").value
        )
        self.output_frame_id = str(
            self.declare_parameter("output_frame_id", "odom").value
        )
        self.control_frame_id = str(
            self.declare_parameter("control_frame_id", "map").value
        )

        # 控制定时器频率；最小 20 Hz，默认 100 Hz，保证 OFFBOARD 连续收到目标。
        self.publish_rate_hz = max(
            20.0, float(self.declare_parameter("publish_rate_hz", 100.0).value)
        )
        # 是否把 SUPER 计算的加速度写入 PositionTarget；关闭时通过 type_mask 忽略 A 字段。
        self.acceleration_feedforward = bool(
            self.declare_parameter("acceleration_feedforward", True).value
        )
        # 对整条 SUPER 多项式做时间缩放，使发布的速度向量模长不超过此值。
        self.max_velocity_mps = float(
            self.declare_parameter("max_velocity_mps", 1.0).value
        )
        if not math.isfinite(self.max_velocity_mps) or self.max_velocity_mps <= 0.0:
            raise ValueError("max_velocity_mps must be finite and greater than zero")
        # 进入 OFFBOARD 前持续发送有效 setpoint 的时间，单位秒；用于满足飞控预流要求。
        self.prestream_sec = max(
            1.0, float(self.declare_parameter("prestream_sec", 2.0).value)
        )
        # SUPER 心跳允许的最大间隔，单位秒；超时后停止使用旧轨迹并保持当前位置。
        self.command_timeout_sec = max(
            0.1, float(self.declare_parameter("command_timeout_sec", 0.5).value)
        )
        # MAVROS 本地位姿反馈的最大允许间隔，单位秒；过期时所有速度/加速度前馈清零。
        self.local_odom_timeout_sec = max(
            0.1,
            float(self.declare_parameter("local_odom_timeout_sec", 1.0).value),
        )
        # 是否由本节点自动解锁；默认 false，避免仅凭软件输入就改变飞控安全状态。
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)
        # 是否在已解锁且条件满足时自动请求 OFFBOARD 模式。
        self.auto_offboard = bool(
            self.declare_parameter("auto_offboard", True).value
        )
        # 解锁和模式服务重复请求的最小间隔，单位秒；防止服务未响应时高频重入。
        self.request_interval_sec = max(
            0.5,
            float(self.declare_parameter("request_interval_sec", 1.0).value),
        )

        # base_link 到 base 的平移外参，单位米；平移向量在 base_link 局部坐标中表达。
        translation = self.declare_parameter(
            "base_link_to_base_translation", [0.0, 0.0, 0.0]
        ).value
        # base_link 到 base 的旋转外参，使用 xyzw 顺序；归一化失败会阻止节点启动。
        rotation = self.declare_parameter(
            "base_link_to_base_rotation_xyzw", [0.0, 0.0, 0.0, 1.0]
        ).value
        if len(translation) != 3:
            raise ValueError("base_link_to_base_translation must contain 3 values")
        self.base_link_to_base_translation = tuple(
            float(value) for value in translation
        )
        self.base_link_to_base_rotation = normalize_quaternion(rotation)

        # SUPER 世界原点到 MAVROS 本地原点的固定平移偏移，单位米；通常应为实测标定值。
        offset = self.declare_parameter(
            "planner_to_mavros_offset_xyz", [0.0, 0.0, 0.0]
        ).value
        if len(offset) != 3:
            raise ValueError("planner_to_mavros_offset_xyz must contain 3 values")
        self.planner_to_mavros_offset = tuple(float(value) for value in offset)

        # 最近一次 MAVROS 状态；初始值未连接、未解锁、非 OFFBOARD。
        self.state = State()
        # SUPER 使用原始 FAST-LIO 世界坐标；这里不能复用带杆臂补偿的视觉基准，
        # 因为外参平移非零时，两者代表的物理原点并不相同。
        # 首个原始 FAST-LIO 世界位置；SUPER 轨迹减去它后才成为 MAVROS 本地目标。
        self.planner_reference: tuple[float, float, float] | None = None
        # 首个完成外参补偿的视觉位置；视觉位姿发布时减去它以建立局部零点。
        self.fastlio_reference: tuple[float, float, float] | None = None
        self.latest_local_pose: tuple[
            tuple[float, float, float], tuple[float, float, float, float]
        ] | None = None
        self.local_odom_received_at = 0.0
        # 最近一条完整 SUPER 多项式；心跳不含系数，因此不能覆盖这个缓存。
        self.latest_trajectory: PolynomialTrajectory | None = None
        # 当前整条轨迹的时间缩放比例；同一常数同时应用到位置采样、速度、加速度和 yaw rate。
        self.trajectory_time_scale = 1.0
        # 最近一次 SUPER HEART_BEAT 的单调时钟时间，用于判断轨迹是否仍在更新。
        self.last_heartbeat_rx = 0.0
        self.last_command_log = 0.0
        self.mission_requested = False
        # 等待飞控进入 OFFBOARD 后再发送的目标，避免轨迹绝对时间在预流阶段提前消耗。
        self.pending_goal: PoseStamped | None = None
        self.goal_forwarded = False
        self.goal_forwarded_at = 0.0
        self.prestream_started: float | None = None
        self.last_request_time = 0.0
        self.arm_request_pending = False
        self.mode_request_pending = False
        # 轨迹/反馈异常后的一次性安全锁存；置位后只发静态保持目标，直到收到新轨迹。
        self.stale_command_latched = False
        self.hold_position: tuple[float, float, float] | None = None
        self.hold_yaw: float | None = None
        self.last_vision_publish = 0.0

        self.vision_pose_pub = self.create_publisher(
            PoseStamped, self.vision_pose_topic, VISION_POSE_QOS
        )
        self.setpoint_pub = self.create_publisher(
            PositionTarget, self.setpoint_topic, 20
        )
        self.trajectory_passthrough_pub = self.create_publisher(
            PolynomialTrajectory,
            self.trajectory_passthrough_topic,
            10,
        )
        self.planner_goal_pub = self.create_publisher(
            PoseStamped,
            self.planner_goal_topic,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.fastlio_odom_topic,
            self.fastlio_odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.local_odom_topic,
            self.local_odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            10,
        )
        self.create_subscription(
            PolynomialTrajectory,
            self.planning_trajectory_topic,
            self.planning_trajectory_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            qos_profile_sensor_data,
        )
        self.arm_client = self.create_client(CommandBool, self.arming_service)
        self.mode_client = self.create_client(SetMode, self.mode_service)
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.timer_callback
        )

        self.get_logger().info(
            "FAST-LIO -> MAVROS vision pose and SUPER -> MAVROS trajectory bridge: "
            f"odom={self.fastlio_odom_topic}, trajectory={self.planning_trajectory_topic}, "
            f"goal={self.goal_topic}->{self.planner_goal_topic}, "
            f"setpoint={self.setpoint_topic}, rate={self.publish_rate_hz:.1f} Hz, "
            f"acceleration_feedforward={self.acceleration_feedforward}"
        )

    @staticmethod
    def finite_vector(values: tuple[float, ...]) -> bool:
        """判断坐标、速度、加速度或角度元组是否全部为有限实数。"""
        return all(math.isfinite(value) for value in values)

    def fastlio_odom_callback(self, msg: Odometry) -> None:
        """处理 FAST-LIO 里程计并限频发布 MAVROS 外部视觉位姿。

        输入位置和姿态来自 `/Odometry`；先拒绝非有限值，再通过
        `odometry_to_vision_pose` 应用 base_link 到 base 的刚体外参。首次有效帧
        分别记录 SUPER 原始坐标基准和视觉位姿基准，后者用于平移到 MAVROS 本地原点。
        姿态无效或发布频率未到时丢弃当前帧，不影响后续回调。
        """
        raw_position = msg.pose.pose.position
        raw_values = (float(raw_position.x), float(raw_position.y), float(raw_position.z))
        if not self.finite_vector(raw_values):
            self.get_logger().warning("Ignoring FAST-LIO odometry with non-finite position")
            return
        try:
            output = odometry_to_vision_pose(
                msg,
                base_link_to_base_translation=self.base_link_to_base_translation,
                base_link_to_base_rotation_xyzw=self.base_link_to_base_rotation,
                output_frame_id=self.output_frame_id,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid FAST-LIO pose: {exc}")
            return

        if self.planner_reference is None:
            self.planner_reference = raw_values
            self.get_logger().info(
                "Captured raw FAST-LIO world origin for SUPER-to-MAVROS mapping"
            )

        p = output.pose.position
        if self.fastlio_reference is None:
            self.fastlio_reference = (p.x, p.y, p.z)
            self.get_logger().info(
                "Captured FAST-LIO world origin for the MAVROS external-pose bridge"
            )
        now = time.monotonic()
        if now - self.last_vision_publish < 1.0 / self.publish_rate_hz:
            return
        self.publish_vision_pose(output)
        self.last_vision_publish = now

    def publish_vision_pose(self, source_pose: PoseStamped) -> None:
        """把带外参的世界位姿减去首帧基准后发布给 MAVROS。

        输出话题是 `vision_pose_topic`，消息类型为 `PoseStamped`；只平移位置，
        保留输入时间戳、frame 和姿态。没有首帧基准时不发布。
        """
        reference = self.fastlio_reference
        if reference is None:
            return
        msg = PoseStamped()
        msg.header = source_pose.header
        msg.pose = source_pose.pose
        msg.pose.position.x -= reference[0]
        msg.pose.position.y -= reference[1]
        msg.pose.position.z -= reference[2]
        self.vision_pose_pub.publish(msg)

    def local_odom_callback(self, msg: Odometry) -> None:
        """缓存 MAVROS 本地位姿并刷新 setpoint 预流状态。

        输入来自 `local_odom_topic`；位置单位为米，四元数按 xyzw 归一化。无效
        数值被忽略。非 OFFBOARD 状态下，当前位置和 yaw 作为安全保持点；连接已建立
        后首次收到有效位姿会开始 `prestream_sec` 秒的 setpoint 预流。
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        position = (float(p.x), float(p.y), float(p.z))
        orientation = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not self.finite_vector(position + orientation):
            return
        try:
            orientation = normalize_quaternion(orientation)
        except ValueError:
            return
        self.latest_local_pose = (position, orientation)
        self.local_odom_received_at = time.monotonic()
        yaw = self.quaternion_to_yaw(orientation)
        if self.state.mode != "OFFBOARD" or self.hold_position is None:
            self.hold_position = position
            self.hold_yaw = yaw
        if self.prestream_started is None and self.state.connected:
            self.prestream_started = time.monotonic()
            self.get_logger().info("MAVROS local pose ready; starting raw setpoint pre-stream")

    def state_callback(self, msg: State) -> None:
        """记录 MAVROS 连接、解锁和模式状态；掉线时重新要求预流。"""
        self.state = msg
        if not msg.connected:
            # MAVROS 断链后必须重新完成一段连续 setpoint 预流，不能沿用断链前计时。
            self.prestream_started = None

    def planning_trajectory_callback(self, msg: PolynomialTrajectory) -> None:
        """接收并校验 SUPER 轨迹，同时记录同一话题上的心跳。

        `POSITION_TRAJ` 消息包含 x/y/z 的分段七阶系数、每段持续时间和起始时间；
        `HEART_BEAT` 只表示规划器仍存活，通常不携带新的系数，因此心跳只能刷新
        `last_heartbeat_rx`，不能替换 `latest_trajectory`。轨迹时间戳早于当前目标
        转发时间的消息会丢弃，避免上一任务的规划结果污染新任务。校验通过后复制缓存，
        并通过诊断话题转发一份，不改变 SUPER 原消息。
        """
        flags = int(msg.type)
        # 使用单调时钟判断“多久没收到心跳”，避免系统时间回拨造成误判。
        if flags & HEART_BEAT:
            self.last_heartbeat_rx = time.monotonic()

        # 没有位置轨迹标志的消息可能只是心跳或其他状态，不能作为可采样轨迹。
        if not flags & POSITION_TRAJ:
            return
        if not self.goal_forwarded:
            return
        trajectory_stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if trajectory_stamp + 1e-6 < self.goal_forwarded_at:
            return
        try:
            validate_trajectory(msg)
            velocity_bound = trajectory_velocity_bound(msg)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid SUPER polynomial trajectory: {exc}")
            return

        # 统一放慢时间轴以限制整条多项式的速度模长。Bernstein 凸包上界覆盖每段
        # 的连续时间区间，而不是只检查某几个离散采样点。
        self.trajectory_time_scale = min(
            1.0,
            self.max_velocity_mps * (1.0 - 1e-9) / velocity_bound
            if velocity_bound > 0.0
            else 1.0,
        )

        # 深拷贝避免 ROS 消息对象被后续回调或中间层复用时改变正在执行的轨迹。
        self.latest_trajectory = deepcopy(msg)
        self.trajectory_passthrough_pub.publish(msg)
        self.stale_command_latched = False
        self.get_logger().info(
            "Cached SUPER seventh-order trajectory: "
            f"pieces={int(msg.piece_num_pos)}, duration={sum(msg.time_pos):.2f}s, "
            f"speed_bound={velocity_bound:.2f}m/s, time_scale={self.trajectory_time_scale:.3f}, "
            f"yaw={'polynomial' if flags & YAW_TRAJ else ('command' if flags & YAW_COMMAND else 'hold')}"
        )

    def goal_callback(self, msg: PoseStamped) -> None:
        """验证并缓存用户目标，等待 OFFBOARD 启动后再发送给 SUPER。

        输入为 `/action/goal` 的世界坐标 xyz 和 yaw 四元数；SUPER 按数值使用坐标，
        因而输出 frame 固定标注为 `world`。目标四元数为零时回退到 yaw=0，并清除旧
        轨迹，避免规划新轨迹期间沿用上一任务的 setpoint。先等飞控进入
        OFFBOARD 再发布目标，避免 SUPER 的绝对轨迹时钟在预流期间提前运行。
        """
        p = msg.pose.position
        q = msg.pose.orientation
        position = (float(p.x), float(p.y), float(p.z))
        orientation = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not self.finite_vector(position):
            self.get_logger().warning("Ignoring goal with non-finite coordinates")
            return
        try:
            orientation = normalize_quaternion(orientation)
        except ValueError:
            orientation = (0.0, 0.0, 0.0, 1.0)
            self.get_logger().warning(
                "Goal quaternion is empty; using yaw 0.0 rad. Supply a quaternion to choose yaw."
            )

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        # SUPER 不读取 frame_id，只直接使用 xyz 数值；这里明确标注坐标契约并保留三轴。
        goal.header.frame_id = "world"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = position
        (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        ) = orientation
        # 新目标到来时清除上一任务的多项式和心跳；规划器重新计算期间只能保持当前位置，
        # 不能继续执行旧轨迹。目标先放入 pending_goal，等 OFFBOARD 确认后再发送，
        # 这样 SUPER 的绝对起始时间不会在人工解锁或预流期间提前消耗。
        self.latest_trajectory = None
        self.trajectory_time_scale = 1.0
        self.last_heartbeat_rx = 0.0
        self.stale_command_latched = False
        self.pending_goal = goal
        self.goal_forwarded = False
        self.goal_forwarded_at = 0.0
        if self.latest_local_pose is not None:
            self.hold_position = self.latest_local_pose[0]
            self.hold_yaw = self.quaternion_to_yaw(self.latest_local_pose[1])
        self.mission_requested = True
        self.get_logger().info(
            f"Queued world-frame goal ({position[0]:.2f}, {position[1]:.2f}, "
            f"{position[2]:.2f}); it will be forwarded after OFFBOARD is active"
        )

    @staticmethod
    def quaternion_to_yaw(q: tuple[float, float, float, float]) -> float:
        """从 xyzw 四元数提取绕 Z 轴的 yaw，返回弧度值。"""
        x, y, z, w = normalize_quaternion(q)
        return math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        )

    def command_is_fresh(self, now: float) -> bool:
        """判断是否同时拥有轨迹缓存和未超时的 SUPER 心跳。"""
        return (
            self.latest_trajectory is not None
            and self.last_heartbeat_rx > 0.0
            and now - self.last_heartbeat_rx <= self.command_timeout_sec
        )

    def local_pose_is_fresh(self, now: float) -> bool:
        """判断 MAVROS 本地位姿反馈是否在允许的秒级超时窗口内。"""
        return (
            self.local_odom_received_at > 0.0
            and now - self.local_odom_received_at <= self.local_odom_timeout_sec
        )

    def world_to_mavros_local(
        self, world_position: tuple[float, float, float]
    ) -> tuple[float, float, float] | None:
        """把 SUPER ENU 世界位置换算为 MAVROS 本地目标位置。

        位置采用 `world - planner_reference + planner_to_mavros_offset`；这里仅对
        位置做原点平移，速度和加速度仍沿同一组已对齐的 ENU 轴表达，不应减原点或
        应用平移。尚未收到 FAST-LIO 首帧时返回 None，调用者会继续发布安全保持点。
        """
        if self.planner_reference is None:
            return None
        return tuple(
            world_position[i]
            - self.planner_reference[i]
            + self.planner_to_mavros_offset[i]
            for i in range(3)
        )

    def publish_setpoint(self, now: float) -> None:
        """把 SUPER 当前时刻的 PVA 和 yaw 前馈发布为 MAVROS raw target。

        完整分段多项式使用 ROS 时间采样；原点平移只作用于位置，速度和加速度仍沿
        已对齐的 ENU 数值轴表达。安全保持分支始终将速度、加速度和 yaw_rate 置零，
        防止旧轨迹的运动前馈继续影响飞控。
        """
        if self.latest_local_pose is None:
            return

        local_position, local_orientation = self.latest_local_pose
        local_pose_fresh = self.local_pose_is_fresh(now)
        position = self.hold_position or local_position
        yaw = self.hold_yaw
        if yaw is None:
            yaw = self.quaternion_to_yaw(local_orientation)

        fresh = self.command_is_fresh(now)
        # position_target 为 None 表示本周期没有可用的新轨迹采样，稍后生成零 PVA
        # 的静态保持目标；这样旧轨迹的速度/加速度不会泄漏到安全分支。
        position_target = None
        if not local_pose_fresh:
            # 保持最近安全位置，不能继续发布已过期轨迹的速度/加速度前馈。
            self.hold_position = local_position
            self.hold_yaw = self.quaternion_to_yaw(local_orientation)
            position = self.hold_position
            yaw = self.hold_yaw
            self.stale_command_latched = True
        if (
            local_pose_fresh
            and self.state.armed
            and self.state.mode == "OFFBOARD"
            and self.mission_requested
            and self.goal_forwarded
            and fresh
            and not self.stale_command_latched
            and self.latest_trajectory is not None
        ):
            try:
                # 使用 ROS clock（而不是 monotonic）匹配 SUPER 消息中的 start_wt；
                # sample_trajectory 会把绝对时间定位到对应 piece 并求出三阶导数以内的值。
                sample_time = self.get_clock().now().nanoseconds * 1e-9
                sample = sample_trajectory(
                    self.latest_trajectory,
                    sample_time,
                    self.hold_yaw if self.hold_yaw is not None else yaw,
                    self.trajectory_time_scale,
                )
            except (TypeError, ValueError, RuntimeError) as exc:
                self.get_logger().error(f"SUPER polynomial sampling failed; holding: {exc}")
                self.stale_command_latched = True
            else:
                # 只有位置需要减去规划原点；速度、加速度是向量，不受平移影响。
                requested = self.world_to_mavros_local(sample.position)
                if requested is not None:
                    position_target = self.make_setpoint(
                        requested,
                        sample.velocity,
                        sample.acceleration,
                        sample.yaw,
                        sample.yaw_rate,
                    )
                    self.hold_position = requested
                    self.hold_yaw = sample.yaw
        elif (
            self.state.armed
            and self.state.mode == "OFFBOARD"
            and self.mission_requested
            and self.latest_trajectory is not None
            and not fresh
        ):
            # SUPER 心跳停止：冻结当前测量位置，并把 P/V/A 前馈全部清零，防止继续执行旧轨迹。
            self.hold_position = local_position
            self.hold_yaw = self.quaternion_to_yaw(local_orientation)
            position = self.hold_position
            yaw = self.hold_yaw
            if not self.stale_command_latched:
                self.get_logger().error(
                    "SUPER polynomial heartbeat timed out; holding current MAVROS local position"
                )
            self.stale_command_latched = True

        if position_target is None:
            position_target = self.make_setpoint(
                position,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                yaw,
                0.0,
            )
        self.setpoint_pub.publish(position_target)

    def make_setpoint(
        self,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        acceleration: tuple[float, float, float],
        yaw: float,
        yaw_rate: float,
    ) -> PositionTarget:
        """把 SUPER 的采样结果编码为 MAVROS `PositionTarget`。

        `position/velocity/acceleration` 的单位分别是米、米每秒、米每二次方秒；
        `yaw/yaw_rate` 的单位分别是弧度、弧度每秒。字段按 MAVROS 的 LOCAL_NED
        消息接口填写；坐标轴转换由 MAVROS/PX4 链路负责，本节点不手动交换 ENU/NED。
        安全保持调用此函数时会传入零速度、零加速度和零 yaw_rate。
        """
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.control_frame_id
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # 0 表示默认使用位置、速度、加速度、yaw、yaw_rate 全部字段。
        target.type_mask = 0
        if not self.acceleration_feedforward:
            target.type_mask |= (
                PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
            )
        # 位置已完成世界原点平移；P/V/A 均来自同一时刻的多项式采样。
        target.position.x, target.position.y, target.position.z = position
        target.velocity.x, target.velocity.y, target.velocity.z = velocity
        (
            target.acceleration_or_force.x,
            target.acceleration_or_force.y,
            target.acceleration_or_force.z,
        ) = acceleration
        target.yaw = float(yaw)
        target.yaw_rate = float(yaw_rate)
        return target

    def request_arm(self) -> None:
        """异步请求 MAVROS 解锁；服务未就绪或已有请求时跳过。"""
        if self.arm_request_pending or not self.arm_client.service_is_ready():
            return
        request = CommandBool.Request()
        request.value = True
        self.arm_request_pending = True
        future = self.arm_client.call_async(request)
        future.add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future) -> None:
        """处理解锁服务结果并记录传输或拒绝错误。"""
        self.arm_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # ROS 服务 future 可能报告通信层传输错误。
            self.get_logger().warning(f"MAVROS arming request failed: {exc}")
            return
        if not response.success:
            self.get_logger().warning(
                f"MAVROS rejected arming request (result={response.result})"
            )

    def request_offboard(self) -> None:
        """异步请求 MAVROS 切换 `OFFBOARD` 模式。"""
        if self.mode_request_pending or not self.mode_client.service_is_ready():
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "OFFBOARD"
        self.mode_request_pending = True
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.mode_result_callback)

    def mode_result_callback(self, future) -> None:
        """处理 OFFBOARD 服务结果并记录传输或拒绝错误。"""
        self.mode_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # ROS 服务 future 可能报告通信层传输错误。
            self.get_logger().warning(f"MAVROS OFFBOARD request failed: {exc}")
            return
        if not response.mode_sent:
            self.get_logger().warning("MAVROS rejected OFFBOARD mode request")

    def timer_callback(self) -> None:
        """以 `publish_rate_hz` 周期运行控制状态机。

        先发布连续 raw setpoint，再检查连接、预流时长、待执行目标和本地位姿；
        按配置请求解锁和 OFFBOARD。确认 armed+OFFBOARD 后才把缓存目标发给 SUPER。
        """
        # monotonic 只用于超时和重试间隔；轨迹多项式采样则使用 ROS clock 时间戳。
        now = time.monotonic()
        self.publish_setpoint(now)

        if now - self.last_command_log >= 1.0:
            self.last_command_log = now
            if self.latest_trajectory is None:
                self.get_logger().info("Waiting for a SUPER polynomial trajectory")
            elif not self.command_is_fresh(now):
                self.get_logger().warning("Waiting for a fresh SUPER polynomial heartbeat")

        # 只有所有前置条件成立，才允许自动解锁、切换模式和发送缓存目标。
        if (
            self.prestream_started is None
            or now - self.prestream_started < self.prestream_sec
            or not self.state.connected
            or not self.mission_requested
            or not self.local_pose_is_fresh(now)
            or self.planner_reference is None
        ):
            return

        if self.auto_arm and not self.state.armed:
            if now - self.last_request_time >= self.request_interval_sec:
                self.request_arm()
                self.last_request_time = now
            return

        # 目标必须在 OFFBOARD 已确认后发送，防止 SUPER 的 start_wt 在飞控准备期间流逝。
        if (
            self.auto_offboard
            and self.state.armed
            and self.state.mode != "OFFBOARD"
            and now - self.last_request_time >= self.request_interval_sec
        ):
            self.request_offboard()
            self.last_request_time = now

        if (
            self.state.armed
            and self.state.mode == "OFFBOARD"
            and self.pending_goal is not None
            and not self.goal_forwarded
        ):
            self.planner_goal_pub.publish(self.pending_goal)
            self.goal_forwarded = True
            self.goal_forwarded_at = self.get_clock().now().nanoseconds * 1e-9
            goal_position = self.pending_goal.pose.position
            self.get_logger().info(
                "MAVROS is armed in OFFBOARD; forwarded queued goal to SUPER at "
                f"({goal_position.x:.2f}, {goal_position.y:.2f}, {goal_position.z:.2f})"
            )


def main(args=None) -> None:
    """初始化 ROS、运行控制节点，并在退出时释放节点和 ROS 上下文。"""
    rclpy.init(args=args)
    node = FastlioSuperMavrosController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
