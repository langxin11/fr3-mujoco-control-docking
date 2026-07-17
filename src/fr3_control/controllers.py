"""关节空间偏置补偿 PD 与计算力矩控制器。"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .model import ModelIds, mass_matrix


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
    _qdd_ref: np.ndarray,
) -> np.ndarray:
    """计算带 MuJoCo 偏置力补偿的关节空间 PD 转矩。

    Args:
        model: MuJoCo 模型；接口保持统一，本控制器不直接使用该参数。
        data: 当前 MuJoCo 动力学状态。
        ids: FR3 关节和执行器在模型中的索引。
        q_ref: 参考关节角，单位为 rad，形状为 ``(7,)``。
        qd_ref: 参考关节角速度，单位为 rad/s，形状为 ``(7,)``。
        _qdd_ref: 参考关节角加速度；PD 控制器不使用该参数。

    Returns:
        期望关节转矩，单位为 N·m，形状为 ``(7,)``。
    """
    del model
    q = data.qpos[ids.joint_qpos]
    qd = data.qvel[ids.joint_dof]
    return (
        GAINS.pd_stiffness * (q_ref - q)
        + GAINS.pd_damping * (qd_ref - qd)
        + data.qfrc_bias[ids.joint_dof]
    )


CONTROLLERS = {"ctc": computed_torque, "pd": bias_compensated_pd}
