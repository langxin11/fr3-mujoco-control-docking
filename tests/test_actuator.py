"""PMSM 参数、约束和转矩映射的单元测试。"""

import numpy as np

from fr3_control.actuator import PMSMDriveBank
from fr3_control.config import PMSM


def test_motor_parameter_units() -> None:
    """验证数据手册参数完成了正确的 SI 单位换算。"""
    assert np.isclose(PMSM.rotor_inertia, 1.7e-5)
    assert np.isclose(PMSM.back_emf_constant, PMSM.torque_constant, rtol=0.01)
    assert PMSM.pole_pairs == 8


def test_torque_mapping_and_limits() -> None:
    """验证电流、电压限幅及减速器输出转矩映射。"""
    motor = PMSMDriveBank.create()
    torque = np.full(7, 1e4)
    for _ in range(500):
        applied, voltage, current_ref, _ = motor.step(torque, np.zeros(7), 5e-5)
    assert np.all(np.abs(voltage) <= PMSM.voltage_limit)
    assert np.all(np.abs(current_ref) <= PMSM.current_limit)
    expected = PMSM.efficiency * PMSM.gear_ratio * PMSM.torque_constant * motor.current
    expected = np.clip(expected, -PMSM.joint_torque_limit, PMSM.joint_torque_limit)
    assert np.allclose(applied, expected)


def test_back_emf_requires_voltage_at_speed() -> None:
    """验证电机旋转时控制器会补偿反电动势。"""
    motor = PMSMDriveBank.create()
    _, voltage, _, _ = motor.step(np.zeros(7), np.ones(7), 5e-5)
    assert np.all(voltage > 0.0)
