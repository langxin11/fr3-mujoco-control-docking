"""闭环仿真、实验指标计算与结果持久化。"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from .actuator import PMSMDriveBank
from .config import RESULTS_DIR, SIM
from .controllers import CONTROLLERS
from .model import load_model, reset_home, site_pose
from .trajectory import ReferenceTrajectory, build_reference


def _metrics(log: dict[str, np.ndarray], scenario: str) -> dict[str, float | str]:
    """从单次实验日志计算报告使用的量化指标。

    Args:
        log: ``simulate`` 生成的时序数据。
        scenario: 实验场景名称，取 ``nominal`` 或 ``disturbance``。

    Returns:
        包含跟踪误差、执行器峰值、饱和率、能量和恢复时间的指标字典。
    """
    ee_norm = np.linalg.norm(log["ee_error"], axis=1)
    joint_norm = np.linalg.norm(log["q_ref"] - log["q"], axis=1)
    metrics: dict[str, float | str] = {
        "controller": str(log["controller"]),
        "scenario": scenario,
        "ee_rmse_mm": float(1e3 * np.sqrt(np.mean(ee_norm**2))),
        "ee_max_mm": float(1e3 * np.max(ee_norm)),
        "joint_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(joint_norm**2 / 7.0)))),
        "peak_current_a": float(np.max(np.abs(log["current"]))),
        "peak_voltage_v": float(np.max(np.abs(log["voltage"]))),
        "saturation_percent": float(100.0 * np.mean(log["saturated"])),
        # 对各电机绝对电功率积分，避免再生功率与耗电功率相互抵消。
        "electrical_energy_j": float(
            np.sum(np.abs(log["current"] * log["voltage"])) * SIM.control_dt
        ),
    }
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
                ]
            )


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
        "q": np.zeros((count, 7)),
        "qd": np.zeros((count, 7)),
        "ee": np.zeros((count, 3)),
        "ee_error": np.zeros((count, 3)),
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
            q_ref, qd_ref, qdd_ref, target = reference.sample(control_index)
            desired_torque = controller(model, data, ids, q_ref, qd_ref, qdd_ref)
            data.mocap_pos[ids.target_mocap] = target
            position, _ = site_pose(data, ids.site)
            # 在本控制周期积分前记录状态；执行器量对应上一物理子步的输出。
            log["q"][control_index] = data.qpos[ids.joint_qpos]
            log["qd"][control_index] = data.qvel[ids.joint_dof]
            log["ee"][control_index] = position
            log["ee_error"][control_index] = target - position
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
