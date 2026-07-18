"""FR3 柔顺对接场景、参考和闭环行为测试。"""

import mujoco
import numpy as np

from fr3_control.config import PMSM
from fr3_control.controllers import task_space_impedance
from fr3_control.model import load_docking_model, reset_docking_home
from fr3_control.simulation import simulate_docking
from fr3_control.trajectory import build_docking_reference


def test_docking_model_reference_and_impedance_torque() -> None:
    """验证对接模型、插入参考和初始阻抗转矩均可用。"""
    model, ids = load_docking_model()
    data = mujoco.MjData(model)
    position, rotation, _, _ = reset_docking_home(model, data, ids)
    reference = build_docking_reference(model, ids, coarse_dt=0.02)
    assert model.nsensor == 2
    assert model.geom_type[ids.tool_contact_geom] == mujoco.mjtGeom.mjGEOM_SDF
    assert model.geom_type[ids.socket_contact_geom] == mujoco.mjtGeom.mjGEOM_SDF
    assert np.all(np.isfinite(reference.q))
    assert np.max(np.abs(np.diff(reference.q, axis=0))) < 0.01
    torque = task_space_impedance(
        model,
        data,
        ids,
        position,
        rotation,
        np.zeros(6),
        np.zeros(6),
        np.zeros(6),
    )
    assert np.all(np.isfinite(torque))
    data.qpos[ids.joint_qpos] = reference.q[-1]
    mujoco.mj_forward(model, data)
    assert data.ncon > 0


def test_impedance_docking_holds_contact_within_limits() -> None:
    """验证阻抗控制完成接触保持且执行器不超过电压和力边界。"""
    log, metrics = simulate_docking("impedance", save=False)
    assert bool(metrics["docking_completed"])
    assert metrics["hold_contact_percent"] >= 90.0
    assert metrics["peak_contact_force_n"] < 10.0
    assert np.all(np.isfinite(log["q"]))
    assert np.max(np.abs(log["voltage"])) <= PMSM.voltage_limit + 1e-8
