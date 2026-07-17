"""PMSM、磁场定向电流环和减速器的离散执行器模型。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import PMSM


@dataclass
class PMSMDriveBank:
    """七轴 PMSM 磁场定向驱动器的动态状态。

    Attributes:
        current: 七台电机的 q 轴转矩电流，单位为 A，形状为 ``(7,)``。
        integral_error: q 轴电流 PI 控制器的积分误差，单位为 A·s，形状为 ``(7,)``。
    """

    current: np.ndarray
    integral_error: np.ndarray

    @classmethod
    def create(cls) -> PMSMDriveBank:
        """创建电流和积分状态均为零的电机组。

        Returns:
            初始化后的七轴 PMSM 驱动器。
        """
        return cls(current=np.zeros(7), integral_error=np.zeros(7))

    @property
    def kp_current(self) -> float:
        """返回按目标带宽整定的电流环比例增益。"""
        return PMSM.inductance * PMSM.current_bandwidth

    @property
    def ki_current(self) -> float:
        """返回按目标带宽整定的电流环积分增益。"""
        return PMSM.resistance * PMSM.current_bandwidth

    def step(
        self, desired_torque: np.ndarray, joint_velocity: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """推进一次 q 轴电流动态并计算关节输出转矩。

        Args:
            desired_torque: 期望关节转矩，单位为 N·m，形状为 ``(7,)``。
            joint_velocity: 关节角速度，单位为 rad/s，形状为 ``(7,)``。
            dt: 本次离散积分的时间步长，单位为 s。

        Returns:
            依次返回实际关节转矩、端电压、电流参考值和饱和标志；前三项
            形状均为 ``(7,)``，单位依次为 N·m、V 和 A，饱和标志为布尔数组。
        """
        # 在 i_d=0 的表贴式 PMSM 中，i_q 直接产生电磁转矩。
        torque_per_amp = PMSM.efficiency * PMSM.gear_ratio * PMSM.torque_constant
        current_reference = np.clip(
            desired_torque / torque_per_amp, -PMSM.current_limit, PMSM.current_limit
        )
        error = current_reference - self.current
        feedforward = (
            PMSM.back_emf_constant * PMSM.gear_ratio * joint_velocity
            + PMSM.resistance * current_reference
        )
        unsaturated = feedforward + self.kp_current * error + self.ki_current * self.integral_error
        voltage = np.clip(unsaturated, -PMSM.voltage_limit, PMSM.voltage_limit)
        # 反算增益 0.2 为经验调参值；除以比例增益后，电压饱和误差被折算为电流误差。
        self.integral_error += dt * (
            error + 0.2 * (voltage - unsaturated) / max(self.kp_current, 1e-9)
        )
        # 限制积分状态，避免执行器长期饱和时出现积分累积。
        self.integral_error = np.clip(self.integral_error, -0.1, 0.1)
        # 理想 FOC 下的 q 轴等效方程：L_q di_q/dt = u_q - R_s i_q - K_e N q_dot。
        derivative = (
            voltage
            - PMSM.resistance * self.current
            - PMSM.back_emf_constant * PMSM.gear_ratio * joint_velocity
        ) / PMSM.inductance
        self.current += dt * derivative
        self.current = np.clip(self.current, -PMSM.current_limit, PMSM.current_limit)
        joint_torque = np.clip(
            torque_per_amp * self.current, -PMSM.joint_torque_limit, PMSM.joint_torque_limit
        )
        saturated = (np.abs(voltage) >= PMSM.voltage_limit - 1e-9) | (
            np.abs(self.current) >= PMSM.current_limit - 1e-9
        )
        return joint_torque, voltage, current_reference, saturated
