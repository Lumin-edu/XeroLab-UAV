#!/usr/bin/env python3

"""SUPER 分段七阶轨迹的校验、时间定位和采样工具。

SUPER 的系数按 ``t^7, ..., t, 1`` 降幂排列，并把多个 piece 首尾拼接后
展平到消息数组中。本模块把它恢复成可查询的分段多项式，输出某一绝对时间点
上的位置、速度、加速度、yaw 和 yaw_rate；它不发布 ROS 消息，也不负责坐标轴转换。
"""

from dataclasses import dataclass
import math
from typing import Sequence


# PolynomialTrajectory.type 的位标志；一个消息可以同时携带位置轨迹和心跳。
HEART_BEAT = 1 << 0
POSITION_TRAJ = 1 << 1
YAW_TRAJ = 1 << 2
YAW_COMMAND = 1 << 3


@dataclass(frozen=True)
class TrajectorySample:
    """控制器单次采样所需的 PVA（位置/速度/加速度）和偏航量。"""
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]
    yaw: float
    yaw_rate: float


@dataclass(frozen=True)
class _PiecewisePolynomial:
    """内部表示：绝对起始时刻、每段持续时间和每段降幂系数。"""
    start_time: float
    durations: tuple[float, ...]
    coefficients: tuple[tuple[float, ...], ...]

    @property
    def total_duration(self) -> float:
        return sum(self.durations)


def _finite_values(values: Sequence[float], name: str) -> tuple[float, ...]:
    """将 ROS 数组转成浮点元组，并拒绝空数组、NaN 和无穷大。"""
    result = tuple(float(value) for value in values)
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def _build_piecewise(
    start_time: float,
    durations: Sequence[float],
    flattened_coefficients: Sequence[float],
    piece_count: int,
    order: int,
    name: str,
) -> _PiecewisePolynomial:
    """校验一条轴的分段多项式并按 piece 切分展平系数。

    `piece_count * (order + 1)` 必须等于系数总数；每段时长必须为正，
    `start_time` 使用 ROS 时钟的绝对秒数。任何结构错误都会抛出 ValueError，
    由控制器捕获后切换到安全保持。
    """
    if piece_count < 1:
        raise ValueError(f"{name} must contain at least one polynomial piece")
    if order != 7:
        raise ValueError(f"{name} must be seventh order, got order {order}")
    if len(durations) != piece_count:
        raise ValueError(f"{name} duration count does not match piece count")
    coefficient_count = piece_count * (order + 1)
    if len(flattened_coefficients) != coefficient_count:
        raise ValueError(f"{name} coefficient count does not match piece count")

    finite_durations = _finite_values(durations, f"{name} durations")
    if any(duration <= 0.0 for duration in finite_durations):
        raise ValueError(f"{name} piece durations must be positive")
    finite_coefficients = _finite_values(flattened_coefficients, f"{name} coefficients")
    start_time = float(start_time)
    if not math.isfinite(start_time):
        raise ValueError(f"{name} start time must be finite")

    width = order + 1
    pieces = tuple(
        finite_coefficients[index * width : (index + 1) * width]
        for index in range(piece_count)
    )
    return _PiecewisePolynomial(start_time, finite_durations, pieces)


def validate_trajectory(msg) -> None:
    """确认消息包含完整的 SUPER 位置路径和可选 yaw 路径。

    位置必须包含 x/y/z 三轴且阶数固定为 7；若设置 `YAW_TRAJ`，yaw 也必须是
    合法分段多项式；若设置 `YAW_COMMAND`，则检查常值 yaw/yaw_rate。该函数只
    验证，不修改消息。
    """
    flags = int(msg.type)
    if not flags & POSITION_TRAJ:
        raise ValueError("message does not contain a position trajectory")

    piece_count = int(msg.piece_num_pos)
    _build_piecewise(
        msg.start_wt_pos,
        msg.time_pos,
        msg.coef_pos_x,
        piece_count,
        int(msg.order_pos),
        "x position trajectory",
    )
    expected = piece_count * (int(msg.order_pos) + 1)
    for coefficients, axis in (
        (msg.coef_pos_y, "y position trajectory"),
        (msg.coef_pos_z, "z position trajectory"),
    ):
        if len(coefficients) != expected:
            raise ValueError(f"{axis} coefficient count does not match piece count")
        _finite_values(coefficients, f"{axis} coefficients")

    if flags & YAW_TRAJ:
        _build_piecewise(
            msg.start_wt_yaw,
            msg.time_yaw,
            msg.coef_yaw,
            int(msg.piece_num_yaw),
            int(msg.order_yaw),
            "yaw trajectory",
        )
    elif flags & YAW_COMMAND:
        if not math.isfinite(float(msg.yaw)) or not math.isfinite(float(msg.yaw_rate)):
            raise ValueError("yaw command must be finite")


def _bernstein_component_velocity_bound(
    descending_coefficients: Sequence[float], duration: float
) -> float:
    """计算一个七阶多项式分段在整段内的保守速度分量上界。

    先把关于局部时间 ``t`` 的一阶导数改写成 ``s=t/duration`` 上的六阶
    幂基多项式，再转换为 Bernstein 基。Bernstein 基函数在 [0, 1] 上非负且
    和为 1，因此多项式的值位于 Bernstein 系数的凸包内；最大系数绝对值
    可作为该轴速度的上界。
    """
    order = len(descending_coefficients) - 1
    if order < 1:
        return 0.0
    # 输入按降幂排列；换成 a[k] = t**k 的升幂系数。
    ascending = tuple(float(value) for value in reversed(descending_coefficients))
    degree = order - 1
    power_coefficients = tuple(
        (power + 1) * ascending[power + 1] * duration**power
        for power in range(order)
    )
    bernstein_coefficients = tuple(
        sum(
            power_coefficients[power]
            * math.comb(index, power)
            / math.comb(degree, power)
            for power in range(index + 1)
        )
        for index in range(order)
    )
    return max(abs(value) for value in bernstein_coefficients)


def trajectory_velocity_bound(msg) -> float:
    """返回完整位置多项式的保守速度模长上界（m/s）。

    每个分段分别计算 x/y/z 速度分量上界，再组合为欧氏模长上界并取所有
    分段中的最大值。结果用于对整条轨迹做统一时间缩放，避免只在离散采样点
    限速而漏掉采样点之间的速度峰值。
    """
    validate_trajectory(msg)
    piece_count = int(msg.piece_num_pos)
    order = int(msg.order_pos)
    width = order + 1
    axes = (msg.coef_pos_x, msg.coef_pos_y, msg.coef_pos_z)
    max_bound = 0.0
    for piece_index, duration_value in enumerate(msg.time_pos):
        duration = float(duration_value)
        component_bounds = []
        for coefficients in axes:
            start = piece_index * width
            piece_coefficients = coefficients[start : start + width]
            component_bounds.append(
                _bernstein_component_velocity_bound(piece_coefficients, duration)
            )
        piece_bound = math.sqrt(sum(value * value for value in component_bounds))
        if not math.isfinite(piece_bound):
            raise ValueError("trajectory velocity bound is not finite")
        max_bound = max(max_bound, piece_bound)
    return max_bound


def _evaluate_descending(coefficients: Sequence[float], t: float, derivative: int) -> float:
    """用 Horner 法计算降幂多项式及其指定阶导数。"""
    values = tuple(float(value) for value in coefficients)
    degree = len(values) - 1
    for _ in range(derivative):
        values = tuple(
            values[index] * (degree - index)
            for index in range(len(values) - 1)
        )
        degree -= 1
        if not values:
            return 0.0

    result = values[0]
    for coefficient in values[1:]:
        result = result * t + coefficient
    return result


def _piece_at_time(
    trajectory: _PiecewisePolynomial, absolute_time: float
) -> tuple[int, float]:
    """把绝对时间夹在轨迹有效区间内，并返回 piece 编号和局部时间。

    轨迹开始前使用第一段的 t=0，结束后使用最后一段末端；因此控制器在
    规划器尚未更新但心跳仍存在时不会访问越界系数。
    """
    elapsed = min(
        max(float(absolute_time) - trajectory.start_time, 0.0),
        trajectory.total_duration,
    )
    for index, duration in enumerate(trajectory.durations):
        if elapsed <= duration or index == len(trajectory.durations) - 1:
            return index, min(elapsed, duration)
        elapsed -= duration
    raise RuntimeError("unreachable piece lookup")


def _sample_axis(
    trajectory: _PiecewisePolynomial,
    axis_coefficients: Sequence[float],
    absolute_time: float,
    derivative: int,
) -> float:
    """从一条轴的展平系数中取出对应 piece，并计算位置/速度/加速度。"""
    index, local_time = _piece_at_time(trajectory, absolute_time)
    width = len(trajectory.coefficients[index])
    coefficients = axis_coefficients[index * width : (index + 1) * width]
    return _evaluate_descending(coefficients, local_time, derivative)


def sample_trajectory(
    msg,
    absolute_time: float,
    fallback_yaw: float,
    time_scale: float = 1.0,
) -> TrajectorySample:
    """在 ROS 绝对时间采样 SUPER 完整轨迹。

    ``time_scale`` 把多项式的相对时间按固定比例放慢；位置按缩放时间采样，
    速度、加速度和 yaw_rate 分别按比例的一次方、二次方和一次方缩放。
    返回值沿共享 ENU 数值轴表达；控制器只对位置做世界原点平移。yaw 的优先级
    是：分段 yaw 多项式 > 常值 yaw command > 调用者提供的保持 yaw。采样结果
    含 NaN/Inf 或输入结构非法时抛出异常，由上层进入保持逻辑。
    """
    validate_trajectory(msg)
    absolute_time = float(absolute_time)
    time_scale = float(time_scale)
    if not math.isfinite(absolute_time):
        raise ValueError("sample time must be finite")
    if not math.isfinite(time_scale) or time_scale <= 0.0 or time_scale > 1.0:
        raise ValueError("trajectory time scale must be finite and in (0, 1]")

    count = int(msg.piece_num_pos)
    position_path = _build_piecewise(
        msg.start_wt_pos,
        msg.time_pos,
        msg.coef_pos_x,
        count,
        int(msg.order_pos),
        "x position trajectory",
    )
    axes = (msg.coef_pos_x, msg.coef_pos_y, msg.coef_pos_z)
    scaled_position_time = position_path.start_time + (
        absolute_time - position_path.start_time
    ) * time_scale
    derivatives = tuple(
        tuple(
            _sample_axis(position_path, axis, scaled_position_time, derivative)
            for axis in axes
        )
        for derivative in range(3)
    )
    # Chain rule for t_scaled = t0 + time_scale * (t - t0).
    derivatives = (
        derivatives[0],
        tuple(value * time_scale for value in derivatives[1]),
        tuple(value * time_scale * time_scale for value in derivatives[2]),
    )

    flags = int(msg.type)
    if flags & YAW_TRAJ:
        yaw_path = _build_piecewise(
            msg.start_wt_yaw,
            msg.time_yaw,
            msg.coef_yaw,
            int(msg.piece_num_yaw),
            int(msg.order_yaw),
            "yaw trajectory",
        )
        scaled_yaw_time = yaw_path.start_time + (
            absolute_time - yaw_path.start_time
        ) * time_scale
        yaw_piece, yaw_local_time = _piece_at_time(yaw_path, scaled_yaw_time)
        yaw_coefficients = yaw_path.coefficients[yaw_piece]
        yaw = _evaluate_descending(yaw_coefficients, yaw_local_time, 0)
        yaw_rate = (
            _evaluate_descending(yaw_coefficients, yaw_local_time, 1) * time_scale
        )
    elif flags & YAW_COMMAND:
        yaw = float(msg.yaw)
        yaw_rate = float(msg.yaw_rate) * time_scale
    else:
        yaw = float(fallback_yaw)
        yaw_rate = 0.0

    values = (*derivatives[0], *derivatives[1], *derivatives[2], yaw, yaw_rate)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("sampled trajectory contains non-finite values")
    return TrajectorySample(
        position=derivatives[0],
        velocity=derivatives[1],
        acceleration=derivatives[2],
        yaw=yaw,
        yaw_rate=yaw_rate,
    )
