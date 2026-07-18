"""关节空间偏置补偿 PD 与计算力矩控制器。"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .config import DOCKING, SIM
from .model import ModelIds, mass_matrix, site_jacobian, site_pose


@dataclass(frozen=True)
class ControllerGains:
    """两类关节控制器的增益配置。

    Attributes:
        natural_frequency: 计算力矩闭环的目标自然频率，单位为 rad/s。
        damping_ratio: 计算力矩闭环的目标阻尼比，无量纲。
        pd_stiffness: PD 控制器的关节刚度，单位为 N·m/rad，形状为 ``(7,)``。
        pd_damping: PD 控制器的关节阻尼，单位为 N·m·s/rad，形状为 ``(7,)``。
    """

    natural_frequency: float = 18.0
    damping_ratio: float = 1.0
    pd_stiffness: np.ndarray | None = None
    pd_damping: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.pd_stiffness is None:
            object.__setattr__(
                self,
                "pd_stiffness",
                np.array([160, 180, 140, 110, 70, 45, 30], dtype=float),
            )
        if self.pd_damping is None:
            object.__setattr__(
                self,
                "pd_damping",
                np.array([25, 28, 22, 18, 12, 8, 6], dtype=float),
            )


GAINS = ControllerGains()


def computed_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    ids: ModelIds,
    q_ref: np.ndarray,
    qd_ref: np.ndarray,
    qdd_ref: np.ndarray,
) -> np.ndarray:
    """根据完整惯性矩阵和偏置力计算关节转矩。

    Args:
        model: MuJoCo 模型。
        data: 当前 MuJoCo 动力学状态。
        ids: FR3 关节和执行器在模型中的索引。
        q_ref: 参考关节角，单位为 rad，形状为 ``(7,)``。
        qd_ref: 参考关节角速度，单位为 rad/s，形状为 ``(7,)``。
        qdd_ref: 参考关节角加速度，单位为 rad/s²，形状为 ``(7,)``。

    Returns:
        期望关节转矩，单位为 N·m，形状为 ``(7,)``。
    """
    q = data.qpos[ids.joint_qpos]
    qd = data.qvel[ids.joint_dof]
    kp = GAINS.natural_frequency**2
    kd = 2.0 * GAINS.damping_ratio * GAINS.natural_frequency
    desired_acceleration = qdd_ref + kd * (qd_ref - qd) + kp * (q_ref - q)
    matrix = mass_matrix(model, data, ids.joint_dof)
    # qfrc_bias 对应动力学方程中的科氏力、离心力和重力等偏置项 h(q, q_dot)。
    bias = data.qfrc_bias[ids.joint_dof].copy()
    # 计算力矩律：tau = M(q) * qdd_cmd + h(q, q_dot)。
    return matrix @ desired_acceleration + bias


def bias_compensated_pd(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    ids: ModelIds,
    q_ref: np.ndarray,
    qd_ref: np.ndarray,
    qdd_ref: np.ndarray,
) -> np.ndarray:
    """计算带惯性前馈和 MuJoCo 偏置力补偿的关节空间 PD 转矩。

    控制律为 :math:`tau = M(q) ddot{q}_{ref} + K_p e + K_d dot{e} + h(q, dot{q})`，
    其中惯性前馈项 :math:`M(q) ddot{q}_{ref}` 使控制器能够提前输出
    维持参考加速度所需的力矩，无需等待位置或速度误差积累。

    Args:
        model: MuJoCo 模型，用于计算关节空间惯性矩阵。
        data: 当前 MuJoCo 动力学状态。
        ids: FR3 关节和执行器在模型中的索引。
        q_ref: 参考关节角，单位为 rad，形状为 ``(7,)``。
        qd_ref: 参考关节角速度，单位为 rad/s，形状为 ``(7,)``。
        qdd_ref: 参考关节角加速度，单位为 rad/s²，形状为 ``(7,)``。

    Returns:
        期望关节转矩，单位为 N·m，形状为 ``(7,)``。
    """
    q = data.qpos[ids.joint_qpos]
    qd = data.qvel[ids.joint_dof]
    matrix = mass_matrix(model, data, ids.joint_dof)
    return (
        matrix @ qdd_ref
        + GAINS.pd_stiffness * (q_ref - q)
        + GAINS.pd_damping * (qd_ref - qd)
        + data.qfrc_bias[ids.joint_dof]
    )


CONTROLLERS = {"ctc": computed_torque, "pd": bias_compensated_pd}


def task_space_impedance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    ids: ModelIds,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    target_twist: np.ndarray,
    target_acceleration: np.ndarray,
    external_wrench: np.ndarray,
) -> np.ndarray:
    """计算带外力反馈与零空间姿态保持的六维操作空间阻抗转矩。

    阻抗关系为 ``M_d (xdd-xdd_d)+D_d (xd-xd_d)+K_d (x-x_d)=F_ext``。
    阻尼矩阵不采用手工整定，而是根据操作空间惯性矩阵 ``Λ`` 和参考刚度
    ``K_r`` 在线计算临界阻尼：

        ``D_r = 2 √(diag(Λ) ⊙ K_r)``

    该公式为 Ren & Shan (2026) Eq. (29) 在刚度矩阵取对角形式时的退化，
    保证了各自由度的闭环响应均处于临界阻尼状态，无需逐任务调参。
    外力为世界坐标系下的工具接触力/力矩。
    """
    position, rotation = site_pose(data, ids.site)
    jacobian = site_jacobian(model, data, ids.site, ids.joint_dof)
    qd = data.qvel[ids.joint_dof]
    current_twist = jacobian @ qd
    pose_error = np.r_[
        target_position - position,
        Rotation.from_matrix(target_rotation @ rotation.T).as_rotvec(),
    ]
    velocity_error = target_twist - current_twist
    matrix = mass_matrix(model, data, ids.joint_dof)
    inverse_mass_jacobian_t = np.linalg.solve(matrix, jacobian.T)
    operational_inverse = jacobian @ inverse_mass_jacobian_t
    operational_inertia = np.linalg.inv(
        operational_inverse + DOCKING.operational_damping**2 * np.eye(6)
    )
    # 临界阻尼 D_r = 2 √(diag(Λ) ⊙ K_r)，各 DOF 自适应实际惯性。
    lambda_diag = np.diag(operational_inertia)
    damping = 2.0 * np.sqrt(lambda_diag * DOCKING.stiffness)
    commanded_acceleration = (
        target_acceleration
        + (external_wrench + DOCKING.stiffness * pose_error + damping * velocity_error)
        / DOCKING.virtual_mass
    )
    task_wrench = operational_inertia @ commanded_acceleration
    dynamically_consistent_inverse = inverse_mass_jacobian_t @ operational_inertia
    nullspace = np.eye(7) - dynamically_consistent_inverse @ jacobian
    q = data.qpos[ids.joint_qpos]
    posture_torque = DOCKING.nullspace_stiffness * (SIM.home_q - q) - DOCKING.nullspace_damping * qd
    return jacobian.T @ task_wrench + data.qfrc_bias[ids.joint_dof] + nullspace.T @ posture_torque
