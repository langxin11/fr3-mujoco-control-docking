"""MuJoCo 模型、惯性矩阵与参考轨迹的集成测试。"""

import mujoco
import numpy as np

from fr3_control.config import PMSM, SIM
from fr3_control.model import load_model, mass_matrix, reset_home, site_pose
from fr3_control.trajectory import build_reference


def test_model_and_mass_matrix() -> None:
    """验证模型可加载且七轴惯性矩阵对称正定。"""
    model, ids = load_model()
    data = mujoco.MjData(model)
    reset_home(model, data, ids)
    matrix = mass_matrix(model, data, ids.joint_dof)
    assert matrix.shape == (7, 7)
    assert np.allclose(matrix, matrix.T, atol=1e-10)
    assert np.all(np.linalg.eigvalsh(matrix) > 0)
    assert np.allclose(model.actuator_ctrlrange[ids.actuators, 1], PMSM.joint_torque_limit)
    assert np.allclose(model.dof_armature[ids.joint_dof], PMSM.rotor_inertia * PMSM.gear_ratio**2)


def test_reference_is_continuous_and_tracks_task() -> None:
    """验证关节参考连续且正运动学末端误差满足阈值。"""
    model, ids = load_model()
    reference = build_reference(model, ids, coarse_dt=0.02)
    assert reference.time[0] == 0.0
    assert np.isclose(reference.time[-1], SIM.duration)
    assert np.max(np.abs(np.diff(reference.q, axis=0))) < 0.05
    data = mujoco.MjData(model)
    worst = 0.0
    for index in np.linspace(0, reference.time.size - 1, 25, dtype=int):
        reset_home(model, data, ids)
        data.qpos[ids.joint_qpos] = reference.q[index]
        mujoco.mj_forward(model, data)
        actual, _ = site_pose(data, ids.site)
        worst = max(worst, np.linalg.norm(actual - reference.ee_position[index]))
    assert worst < 0.003
