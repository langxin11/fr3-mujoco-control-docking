"""闭环仿真、实验指标计算与结果持久化。"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .actuator import PMSMDriveBank
from .config import DOCKING, RESULTS_DIR, SIM
from .controllers import CONTROLLERS, task_space_impedance
from .model import (
    DockingModelIds,
    load_docking_model,
    load_model,
    reset_docking_home,
    reset_home,
    site_pose,
)
from .trajectory import ReferenceTrajectory, _quintic, build_docking_reference, build_reference


def _metrics(log: dict[str, np.ndarray], scenario: str) -> dict[str, float | str]:
    """从单次实验日志计算报告使用的量化指标。

    Args:
        log: ``simulate`` 生成的时序数据。
        scenario: 实验场景名称，取 ``nominal`` 或 ``disturbance``。

    Returns:
        包含跟踪误差、执行器峰值、饱和率、能量和恢复时间的指标字典。
    """
    ee_norm = np.linalg.norm(log["ee_error"], axis=1)
    orientation_error = log["ee_orientation_error"]
    joint_norm = np.linalg.norm(log["q_ref"] - log["q"], axis=1)
    metrics: dict[str, float | str] = {
        "controller": str(log["controller"]),
        "scenario": scenario,
        "ee_rmse_mm": float(1e3 * np.sqrt(np.mean(ee_norm**2))),
        "ee_max_mm": float(1e3 * np.max(ee_norm)),
        "ee_orientation_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(orientation_error**2)))),
        "ee_orientation_max_deg": float(np.rad2deg(np.max(orientation_error))),
        "joint_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(joint_norm**2 / 7.0)))),
        "peak_current_a": float(np.max(np.abs(log["current"]))),
        "peak_voltage_v": float(np.max(np.abs(log["voltage"]))),
        "saturation_percent": float(100.0 * np.mean(log["saturated"])),
        # 对各电机绝对电功率积分，避免再生功率与耗电功率相互抵消。
        "electrical_energy_j": float(
            np.sum(np.abs(log["current"] * log["voltage"])) * SIM.control_dt
        ),
    }
    # 仅在各任务的稳态跟踪区间统计，避免两段过渡轨迹混入比较。
    segments = {
        "circle": (SIM.transition_end, SIM.circle_end),
        "figure8": (SIM.figure8_start, SIM.figure8_end),
    }
    for name, (start, end) in segments.items():
        mask = (log["time"] >= start) & (log["time"] <= end)
        segment_position = ee_norm[mask]
        segment_orientation = orientation_error[mask]
        metrics[f"{name}_ee_rmse_mm"] = float(1e3 * np.sqrt(np.mean(segment_position**2)))
        metrics[f"{name}_ee_max_mm"] = float(1e3 * np.max(segment_position))
        metrics[f"{name}_orientation_rmse_deg"] = float(
            np.rad2deg(np.sqrt(np.mean(segment_orientation**2)))
        )
        metrics[f"{name}_orientation_max_deg"] = float(np.rad2deg(np.max(segment_orientation)))
    if scenario == "disturbance":
        after = np.flatnonzero(log["time"] >= SIM.disturbance_end)
        # 恢复时间定义为撤力后末端误差首次回到 10 mm 以内所需时间。
        recovered = after[ee_norm[after] < 0.01]
        metrics["recovery_s"] = (
            float(log["time"][recovered[0]] - SIM.disturbance_end) if recovered.size else -1.0
        )
    else:
        metrics["recovery_s"] = 0.0
    return metrics


def _save_run(log: dict[str, np.ndarray], metrics: dict[str, float | str], output: Path) -> None:
    """以 NPZ、JSON 和 CSV 三种格式保存一次实验结果。

    Args:
        log: ``simulate`` 生成的完整时序数据。
        metrics: ``_metrics`` 生成的实验指标。
        output: 不带扩展名的输出路径。
    """
    output.parent.mkdir(parents=True, exist_ok=True)
    arrays = {key: value for key, value in log.items() if isinstance(value, np.ndarray)}
    np.savez_compressed(output.with_suffix(".npz"), **arrays)
    output.with_suffix(".json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    columns = [
        "time",
        *[f"q_ref_{i + 1}" for i in range(7)],
        *[f"q_{i + 1}" for i in range(7)],
        "ee_ref_x",
        "ee_ref_y",
        "ee_ref_z",
        "ee_x",
        "ee_y",
        "ee_z",
        "ee_error_norm",
        "ee_orientation_error_deg",
    ]
    with output.with_suffix(".csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        for index, time in enumerate(log["time"]):
            writer.writerow(
                [
                    time,
                    *log["q_ref"][index],
                    *log["q"][index],
                    *log["ee_ref"][index],
                    *log["ee"][index],
                    np.linalg.norm(log["ee_error"][index]),
                    np.rad2deg(log["ee_orientation_error"][index]),
                ]
            )


def _docking_contact_wrench(
    model: mujoco.MjModel, data: mujoco.MjData, ids: DockingModelIds
) -> tuple[np.ndarray, int]:
    """汇总作用在移动对接件上的世界坐标系接触力/力矩。"""
    wrench = np.zeros(6)
    count = 0
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        if {contact.geom1, contact.geom2} != {ids.tool_contact_geom, ids.socket_contact_geom}:
            continue
        local_wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, contact_index, local_wrench)
        frame = contact.frame.reshape(3, 3)
        # ``mj_contactForce`` 的正号对应 geom1 受到的约束反作用；对接对中
        # tool 为 geom1 时，转换为“环境作用在工具上”的外力需取相反号。
        sign = -1.0 if contact.geom1 == ids.tool_contact_geom else 1.0
        wrench[:3] += sign * frame.T @ local_wrench[:3]
        wrench[3:] += sign * frame.T @ local_wrench[3:]
        count += 1
    return wrench, count


def _docking_metrics(log: dict[str, np.ndarray]) -> dict[str, float | str]:
    """计算柔顺对接的接触、插入和执行器量化指标。"""
    position_error = np.linalg.norm(log["ee_error"], axis=1)
    contact_force = np.linalg.norm(log["contact_wrench"][:, :3], axis=1)
    contact_torque = np.linalg.norm(log["contact_wrench"][:, 3:], axis=1)
    contact_indices = np.flatnonzero(log["contact_count"] > 0)
    hold_mask = log["time"] >= DOCKING.insertion_end
    hold_contact_percent = 100.0 * np.mean(log["contact_count"][hold_mask] > 0)
    return {
        "controller": str(log["controller"]),
        "scenario": "docking",
        "ee_rmse_mm": float(1e3 * np.sqrt(np.mean(position_error**2))),
        "ee_final_mm": float(1e3 * position_error[-1]),
        "ee_orientation_final_deg": float(np.rad2deg(log["ee_orientation_error"][-1])),
        "peak_contact_force_n": float(np.max(contact_force)),
        "steady_contact_force_n": float(np.mean(contact_force[hold_mask])),
        "final_contact_force_n": float(contact_force[-1]),
        "peak_contact_torque_nm": float(np.max(contact_torque)),
        "contact_start_s": float(log["time"][contact_indices[0]]) if contact_indices.size else -1.0,
        "contact_duration_s": float(SIM.control_dt * contact_indices.size),
        "hold_contact_percent": float(hold_contact_percent),
        "docking_completed": bool(contact_indices.size and hold_contact_percent >= 90.0),
        "peak_current_a": float(np.max(np.abs(log["current"]))),
        "peak_voltage_v": float(np.max(np.abs(log["voltage"]))),
        "saturation_percent": float(100.0 * np.mean(log["saturated"])),
        "electrical_energy_j": float(
            np.sum(np.abs(log["current"] * log["voltage"])) * SIM.control_dt
        ),
    }


def simulate(
    controller_name: str,
    scenario: str,
    *,
    reference: ReferenceTrajectory | None = None,
    save: bool = True,
) -> tuple[dict[str, np.ndarray], dict[str, float | str]]:
    """运行一个控制器与实验场景组合的闭环仿真。

    Args:
        controller_name: 控制器名称，取 ``pd`` 或 ``ctc``。
        scenario: 场景名称，取 ``nominal`` 或 ``disturbance``。
        reference: 可复用的参考轨迹；为 ``None`` 时现场生成。
        save: 是否将日志和指标写入 ``results`` 目录。

    Returns:
        时序日志和由该日志计算得到的指标字典。

    Raises:
        ValueError: 控制器名称或场景名称不受支持。
    """
    if controller_name not in CONTROLLERS:
        raise ValueError(f"unknown controller: {controller_name}")
    if scenario not in {"nominal", "disturbance"}:
        raise ValueError(f"unknown scenario: {scenario}")

    model, ids = load_model()
    data = mujoco.MjData(model)
    reset_home(model, data, ids)
    if reference is None:
        reference = build_reference(model, ids)
    controller = CONTROLLERS[controller_name]
    motor = PMSMDriveBank.create()
    count = reference.time.size
    log = {
        "time": reference.time.copy(),
        "q_ref": reference.q.copy(),
        "qd_ref": reference.qd.copy(),
        "qdd_ref": reference.qdd.copy(),
        "ee_ref": reference.ee_position.copy(),
        "ee_rot_ref": reference.ee_rotation.copy(),
        "q": np.zeros((count, 7)),
        "qd": np.zeros((count, 7)),
        "ee": np.zeros((count, 3)),
        "ee_rotation": np.zeros((count, 3, 3)),
        "ee_error": np.zeros((count, 3)),
        "ee_orientation_error": np.zeros(count),
        "desired_torque": np.zeros((count, 7)),
        "applied_torque": np.zeros((count, 7)),
        "current": np.zeros((count, 7)),
        "current_ref": np.zeros((count, 7)),
        "voltage": np.zeros((count, 7)),
        "saturated": np.zeros((count, 7), dtype=bool),
        "external_force": np.zeros((count, 3)),
        "controller": controller_name,
    }
    # 外层控制器按 1 ms 更新，电机模型和 MuJoCo 按 50 μs 子步积分。
    control_stride = round(SIM.control_dt / SIM.physics_dt)
    total_steps = round(SIM.duration / SIM.physics_dt)
    desired_torque = np.zeros(7)
    voltage = np.zeros(7)
    current_ref = np.zeros(7)
    saturated = np.zeros(7, dtype=bool)
    applied_torque = np.zeros(7)

    for physics_step in range(total_steps + 1):
        control_index = physics_step // control_stride
        is_control_tick = physics_step % control_stride == 0
        t = physics_step * SIM.physics_dt
        if is_control_tick:
            q_ref, qd_ref, qdd_ref, target, target_rotation = reference.sample(control_index)
            desired_torque = controller(model, data, ids, q_ref, qd_ref, qdd_ref)
            data.mocap_pos[ids.target_mocap] = target
            target_quaternion = Rotation.from_matrix(target_rotation).as_quat()
            data.mocap_quat[ids.target_mocap] = target_quaternion[[3, 0, 1, 2]]
            position, rotation = site_pose(data, ids.site)
            # 在本控制周期积分前记录状态；执行器量对应上一物理子步的输出。
            log["q"][control_index] = data.qpos[ids.joint_qpos]
            log["qd"][control_index] = data.qvel[ids.joint_dof]
            log["ee"][control_index] = position
            log["ee_rotation"][control_index] = rotation
            log["ee_error"][control_index] = target - position
            rotation_error = target_rotation @ rotation.T
            log["ee_orientation_error"][control_index] = np.arccos(
                np.clip((np.trace(rotation_error) - 1.0) / 2.0, -1.0, 1.0)
            )
            log["desired_torque"][control_index] = desired_torque
            log["applied_torque"][control_index] = applied_torque
            log["current"][control_index] = motor.current
            log["current_ref"][control_index] = current_ref
            log["voltage"][control_index] = voltage
            log["saturated"][control_index] = saturated
            if scenario == "disturbance" and SIM.disturbance_start <= t <= SIM.disturbance_end:
                log["external_force"][control_index, 2] = -SIM.disturbance_force
        if physics_step == total_steps:
            break

        data.xfrc_applied[:] = 0.0
        if scenario == "disturbance" and SIM.disturbance_start <= t <= SIM.disturbance_end:
            # 在 hand 刚体质心处施加世界坐标系负 z 方向恒力。
            data.xfrc_applied[ids.hand_body, 2] = -SIM.disturbance_force
        applied_torque, voltage, current_ref, saturated = motor.step(
            desired_torque, data.qvel[ids.joint_dof].copy(), SIM.physics_dt
        )
        data.ctrl[ids.actuators] = applied_torque
        mujoco.mj_step(model, data)

    metrics = _metrics(log, scenario)
    if save:
        _save_run(log, metrics, RESULTS_DIR / f"{controller_name}_{scenario}")
    return log, metrics


def run_all_experiments() -> list[dict[str, float | str]]:
    """运行两种控制器在两种场景下的四组实验。

    Returns:
        按 PD、计算力矩控制器以及标称、扰动场景排列的四组指标。
    """
    model, ids = load_model()
    reference = build_reference(model, ids)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        RESULTS_DIR / "reference.npz",
        time=reference.time,
        q=reference.q,
        qd=reference.qd,
        qdd=reference.qdd,
        ee_position=reference.ee_position,
        ee_rotation=reference.ee_rotation,
        ee_twist=reference.ee_twist,
        ee_acceleration=reference.ee_acceleration,
    )
    summaries = []
    for controller in ("pd", "ctc"):
        for scenario in ("nominal", "disturbance"):
            _, metrics = simulate(controller, scenario, reference=reference, save=True)
            summaries.append(metrics)
    (RESULTS_DIR / "summary.json").write_text(
        json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summaries


def simulate_docking(
    controller_name: str,
    *,
    reference: ReferenceTrajectory | None = None,
    save: bool = True,
) -> tuple[dict[str, np.ndarray], dict[str, float | str]]:
    """运行 FR3 柔顺对接；支持刚性 CTC 与任务空间阻抗控制。"""
    if controller_name not in {"ctc", "impedance"}:
        raise ValueError(f"unknown docking controller: {controller_name}")
    model, ids = load_docking_model()
    data = mujoco.MjData(model)
    reset_docking_home(model, data, ids)
    if reference is None:
        reference = build_docking_reference(model, ids)
    motor = PMSMDriveBank.create()
    count = reference.time.size
    log = {
        "time": reference.time.copy(),
        "q_ref": reference.q.copy(),
        "qd_ref": reference.qd.copy(),
        "qdd_ref": reference.qdd.copy(),
        "ee_ref": reference.ee_position.copy(),
        "ee_rot_ref": reference.ee_rotation.copy(),
        "q": np.zeros((count, 7)),
        "qd": np.zeros((count, 7)),
        "ee": np.zeros((count, 3)),
        "ee_rotation": np.zeros((count, 3, 3)),
        "ee_error": np.zeros((count, 3)),
        "ee_orientation_error": np.zeros(count),
        "desired_torque": np.zeros((count, 7)),
        "applied_torque": np.zeros((count, 7)),
        "current": np.zeros((count, 7)),
        "current_ref": np.zeros((count, 7)),
        "voltage": np.zeros((count, 7)),
        "saturated": np.zeros((count, 7), dtype=bool),
        "contact_wrench": np.zeros((count, 6)),
        "contact_count": np.zeros(count, dtype=int),
        "controller": controller_name,
    }
    physics_dt = model.opt.timestep
    control_stride = round(SIM.control_dt / physics_dt)
    total_steps = round(DOCKING.duration / physics_dt)
    desired_torque = np.zeros(7)
    voltage = np.zeros(7)
    current_ref = np.zeros(7)
    saturated = np.zeros(7, dtype=bool)
    applied_torque = np.zeros(7)
    contact_latched = False
    contact_latch_time = 0.0
    contact_target: np.ndarray | None = None
    contact_target_rotation: np.ndarray | None = None

    for physics_step in range(total_steps + 1):
        control_index = physics_step // control_stride
        if physics_step % control_stride == 0:
            q_ref, qd_ref, qdd_ref, target, target_rotation = reference.sample(control_index)
            contact_wrench, contact_count = _docking_contact_wrench(model, data, ids)
            time_value = reference.time[control_index]
            if contact_count > 0 and not contact_latched:
                contact_latched = True
                contact_latch_time = time_value
                contact_target, contact_target_rotation = site_pose(data, ids.site)
            if controller_name == "impedance" and contact_target is not None:
                # 接触后保持首次接触的位姿，由力反馈提供小压紧力，避免继续跟踪预插入轨迹。
                target = contact_target
                target_rotation = contact_target_rotation
            if controller_name == "ctc":
                desired_torque = CONTROLLERS["ctc"](model, data, ids, q_ref, qd_ref, qdd_ref)
            else:
                desired_contact_wrench = np.zeros(6)
                if contact_latched:
                    # 环境接触力的目标方向与插入方向相反；未达到该力时会继续轻推。
                    force_scale = _quintic(
                        (time_value - contact_latch_time) / DOCKING.contact_force_ramp_s
                    )
                    desired_contact_wrench[:3] = -DOCKING.contact_hold_force * target_rotation[:, 2]
                    desired_contact_wrench[:3] *= force_scale
                desired_torque = task_space_impedance(
                    model,
                    data,
                    ids,
                    target,
                    target_rotation,
                    reference.ee_twist[control_index],
                    reference.ee_acceleration[control_index],
                    contact_wrench - desired_contact_wrench,
                )
            data.mocap_pos[ids.target_mocap] = target
            target_quaternion = Rotation.from_matrix(target_rotation).as_quat()
            data.mocap_quat[ids.target_mocap] = target_quaternion[[3, 0, 1, 2]]
            position, rotation = site_pose(data, ids.site)
            log["q"][control_index] = data.qpos[ids.joint_qpos]
            log["qd"][control_index] = data.qvel[ids.joint_dof]
            log["ee"][control_index] = position
            log["ee_rotation"][control_index] = rotation
            log["ee_error"][control_index] = target - position
            rotation_error = target_rotation @ rotation.T
            log["ee_orientation_error"][control_index] = np.arccos(
                np.clip((np.trace(rotation_error) - 1.0) / 2.0, -1.0, 1.0)
            )
            log["desired_torque"][control_index] = desired_torque
            log["applied_torque"][control_index] = applied_torque
            log["current"][control_index] = motor.current
            log["current_ref"][control_index] = current_ref
            log["voltage"][control_index] = voltage
            log["saturated"][control_index] = saturated
            log["contact_wrench"][control_index] = contact_wrench
            log["contact_count"][control_index] = contact_count
        if physics_step == total_steps:
            break
        applied_torque, voltage, current_ref, saturated = motor.step(
            desired_torque, data.qvel[ids.joint_dof].copy(), physics_dt
        )
        data.ctrl[ids.actuators] = applied_torque
        mujoco.mj_step(model, data)

    metrics = _docking_metrics(log)
    if save:
        _save_run(log, metrics, RESULTS_DIR / f"docking_{controller_name}")
    return log, metrics


def run_docking_experiments() -> list[dict[str, float | str]]:
    """运行刚性 CTC 与阻抗控制的两组 FR3 对接实验。"""
    model, ids = load_docking_model()
    reference = build_docking_reference(model, ids)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        RESULTS_DIR / "docking_reference.npz",
        time=reference.time,
        q=reference.q,
        qd=reference.qd,
        qdd=reference.qdd,
        ee_position=reference.ee_position,
        ee_rotation=reference.ee_rotation,
        ee_twist=reference.ee_twist,
        ee_acceleration=reference.ee_acceleration,
    )
    summaries = [
        simulate_docking(controller, reference=reference, save=True)[1]
        for controller in ("ctc", "impedance")
    ]
    (RESULTS_DIR / "docking_summary.json").write_text(
        json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summaries
