"""末端任务轨迹、阻尼最小二乘逆运动学与关节参考轨迹生成。"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

from .config import JOINT_NAMES, SIM
from .model import ModelIds, reset_home, site_jacobian, site_pose


@dataclass(frozen=True)
class ReferenceTrajectory:
    """控制周期采样的机械臂参考轨迹。

    Attributes:
        time: 时间戳，单位为 s，形状为 ``(n,)``。
        q: 参考关节角，单位为 rad，形状为 ``(n, 7)``。
        qd: 参考关节角速度，单位为 rad/s，形状为 ``(n, 7)``。
        qdd: 参考关节角加速度，单位为 rad/s²，形状为 ``(n, 7)``。
        ee_position: 参考末端位置，单位为 m，形状为 ``(n, 3)``。
        ee_rotation: 参考末端旋转矩阵序列，形状为 ``(n, 3, 3)``。
    """

    time: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray
    ee_position: np.ndarray
    ee_rotation: np.ndarray

    def sample(
        self, index: int
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """返回指定控制周期的关节参考量和末端目标位姿。

        Args:
            index: 控制周期在轨迹数组中的下标。

        Returns:
            依次返回参考关节角、角速度、角加速度、末端位置和旋转矩阵。
        """
        return (
            self.q[index],
            self.qd[index],
            self.qdd[index],
            self.ee_position[index],
            self.ee_rotation[index],
        )


def _quintic(u: float) -> float:
    """计算端点速度和加速度均为零的五次时间缩放函数。

    Args:
        u: 归一化时间；函数内部将其限制到 ``[0, 1]``。

    Returns:
        五次多项式时间缩放值。
    """
    u = float(np.clip(u, 0.0, 1.0))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def task_position(t: float, home_position: np.ndarray) -> np.ndarray:
    """计算给定时刻的末端位置目标。

    轨迹依次包含五次多项式过渡、一个完整竖直圆周、二次过渡、
    x--y 平面 8 字轨迹和终点保持。

    Args:
        t: 仿真时刻，单位为 s。
        home_position: 末端初始位置，单位为 m，形状为 ``(3,)``。

    Returns:
        世界坐标系中的末端目标位置，单位为 m，形状为 ``(3,)``。
    """
    radius = SIM.circle_radius
    center = home_position + np.array([0.0, 0.0, -0.06])
    start = center + np.array([0.0, radius, 0.0])
    figure8_center = center
    if t < SIM.transition_end:
        return home_position + _quintic(t / SIM.transition_end) * (start - home_position)
    if t <= SIM.circle_end:
        theta = 2.0 * np.pi * SIM.circle_frequency * (t - SIM.transition_end)
        return center + np.array([0.0, radius * np.cos(theta), radius * np.sin(theta)])
    if t < SIM.figure8_start:
        u = (t - SIM.circle_end) / (SIM.figure8_start - SIM.circle_end)
        return start + _quintic(u) * (figure8_center - start)
    if t <= SIM.figure8_end:
        phase = 2.0 * np.pi * SIM.figure8_frequency * (t - SIM.figure8_start)
        return figure8_center + np.array(
            [
                SIM.figure8_radius_x * np.sin(phase),
                SIM.figure8_radius_y * np.sin(2.0 * phase),
                0.0,
            ]
        )
    return figure8_center.copy()


def task_rotation(t: float, home_rotation: np.ndarray) -> np.ndarray:
    """计算给定时刻的末端姿态目标。

    圆周段和 8 字段分别在工具坐标系中叠加平滑的滚转、俯仰
    和偏航变化。各角度在每段起终点的值和一阶导数均为零。

    Args:
        t: 仿真时刻，单位为 s。
        home_rotation: 末端初始旋转矩阵，形状为 ``(3, 3)``。

    Returns:
        世界坐标系中的末端目标旋转矩阵，形状为 ``(3, 3)``。
    """
    if SIM.transition_end <= t <= SIM.circle_end:
        theta = 2.0 * np.pi * SIM.circle_frequency * (t - SIM.transition_end)
        # 最大滚转 12°、俯仰 6°。
        roll = np.deg2rad(6.0) * (1.0 - np.cos(theta))
        pitch = np.deg2rad(6.0) * np.sin(theta) ** 2
        relative = Rotation.from_euler("xy", [roll, pitch]).as_matrix()
        return home_rotation @ relative
    if SIM.figure8_start <= t <= SIM.figure8_end:
        phase = 2.0 * np.pi * SIM.figure8_frequency * (t - SIM.figure8_start)
        # 8 字段最大偏航 16°、滚转 5°，便于观察平面轨迹中的姿态变化。
        roll = np.deg2rad(5.0) * np.sin(phase) ** 2
        yaw = np.deg2rad(8.0) * (1.0 - np.cos(phase))
        relative = Rotation.from_euler("xz", [roll, yaw]).as_matrix()
        return home_rotation @ relative
    return home_rotation.copy()


def _orientation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """用旋转向量表示从当前姿态到目标姿态的误差。"""
    return Rotation.from_matrix(target @ current.T).as_rotvec()


def _solve_pose_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    ids: ModelIds,
    seed: np.ndarray,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
) -> np.ndarray:
    """使用带零空间回中项的阻尼最小二乘法求解七轴逆运动学。

    Args:
        model: MuJoCo 模型。
        data: 逆运动学迭代使用的 MuJoCo 状态。
        ids: FR3 模型元素索引。
        seed: 本次迭代的关节角初值，单位为 rad，形状为 ``(7,)``。
        target_position: 末端目标位置，单位为 m，形状为 ``(3,)``。
        target_rotation: 末端目标旋转矩阵，形状为 ``(3, 3)``。

    Returns:
        满足机械限位约束的关节角近似解，单位为 rad，形状为 ``(7,)``。
    """
    q = seed.copy()
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
    )
    # 在机械限位内侧保留 0.025 rad 裕量，降低插值后触碰限位的风险。
    lower = model.jnt_range[joint_ids, 0] + 0.025
    upper = model.jnt_range[joint_ids, 1] - 0.025
    # 姿态误差权重较低，优先保证圆轨迹的位置精度。
    weights = np.diag([1.0, 1.0, 1.0, 0.35, 0.35, 0.35])
    damping = 2.5e-3

    for _ in range(45):
        data.qpos[ids.joint_qpos] = q
        data.qvel[ids.joint_dof] = 0.0
        mujoco.mj_forward(model, data)
        position, rotation = site_pose(data, ids.site)
        error = np.r_[target_position - position, _orientation_error(target_rotation, rotation)]
        if np.linalg.norm(error[:3]) < 2e-5 and np.linalg.norm(error[3:]) < 2e-4:
            break
        jac = weights @ site_jacobian(model, data, ids.site, ids.joint_dof)
        weighted_error = weights @ error
        # 阻尼伪逆在接近奇异位形时限制关节增量。
        pinv = jac.T @ np.linalg.inv(jac @ jac.T + damping**2 * np.eye(6))
        nullspace = np.eye(7) - pinv @ jac
        # 主任务跟踪末端位姿，零空间任务将冗余关节缓慢拉回舒适初始姿态。
        dq = pinv @ weighted_error + nullspace @ (0.025 * (SIM.home_q - q))
        max_step = np.max(np.abs(dq))
        # 限制单次迭代最大关节变化，避免线性化误差导致数值跳变。
        if max_step > 0.08:
            dq *= 0.08 / max_step
        q = np.clip(q + dq, lower, upper)
    return q


def build_reference(
    model: mujoco.MjModel, ids: ModelIds, *, coarse_dt: float = 0.01
) -> ReferenceTrajectory:
    """生成控制器使用的连续关节参考轨迹。

    先在较粗时间网格上逐点求解逆运动学，再用三次样条生成控制周期
    下的关节角、角速度和角加速度。

    Args:
        model: MuJoCo 模型。
        ids: FR3 模型元素索引。
        coarse_dt: 逆运动学粗采样周期，单位为 s。

    Returns:
        以 ``SIM.control_dt`` 为采样周期的完整参考轨迹。
    """
    data = mujoco.MjData(model)
    reset_home(model, data, ids)
    home_position, home_rotation = site_pose(data, ids.site)
    coarse_time = np.arange(0.0, SIM.duration + coarse_dt / 2, coarse_dt)
    q_coarse = np.empty((coarse_time.size, 7))
    task_coarse = np.empty((coarse_time.size, 3))
    rotation_coarse = np.empty((coarse_time.size, 3, 3))
    q = SIM.home_q.copy()
    for k, t in enumerate(coarse_time):
        desired = task_position(float(t), home_position)
        desired_rotation = task_rotation(float(t), home_rotation)
        q = _solve_pose_ik(model, data, ids, q, desired, desired_rotation)
        q_coarse[k] = q
        task_coarse[k] = desired
        rotation_coarse[k] = desired_rotation

    time = np.arange(0.0, SIM.duration + SIM.control_dt / 2, SIM.control_dt)
    spline = CubicSpline(coarse_time, q_coarse, axis=0, bc_type="clamped")
    q_ref = spline(time)
    qd_ref = spline(time, 1)
    qdd_ref = spline(time, 2)
    ee_ref = np.column_stack(
        [np.interp(time, coarse_time, task_coarse[:, axis]) for axis in range(3)]
    )
    # 在 SO(3) 上用旋转向量插值，避免直接插值矩阵破坏正交性。
    relative_rotation = Rotation.from_matrix(rotation_coarse @ home_rotation.T)
    rotation_vectors = relative_rotation.as_rotvec()
    interpolated_vectors = np.column_stack(
        [np.interp(time, coarse_time, rotation_vectors[:, axis]) for axis in range(3)]
    )
    ee_rotation = Rotation.from_rotvec(interpolated_vectors).as_matrix() @ home_rotation
    return ReferenceTrajectory(
        time=time,
        q=q_ref,
        qd=qd_ref,
        qdd=qdd_ref,
        ee_position=ee_ref,
        ee_rotation=ee_rotation,
    )
