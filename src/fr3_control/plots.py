"""读取实验结果并生成课程报告使用的静态图表。"""

from __future__ import annotations

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401  # 导入时向 Matplotlib 注册 SciencePlots 样式。
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.axes3d import Axes3D

from .config import DOCKING, FIGURES_DIR, RESULTS_DIR, SIM


def _load(name: str) -> dict[str, np.ndarray]:
    """读取指定实验的 NPZ 日志。

    Args:
        name: 不含扩展名的实验名称。

    Returns:
        从字段名映射到数组副本的字典。
    """
    with np.load(RESULTS_DIR / f"{name}.npz") as archive:
        return {key: archive[key] for key in archive.files}


def _style() -> None:
    """设置适合中英文科学图表的 SciencePlots 全局样式。"""
    # no-latex 保留 SciencePlots 排版，同时避免生成图片依赖本机 LaTeX 环境。
    plt.style.use(["science", "no-latex", "cjk-sc-font"])
    plt.rcParams.update(
        {
            "mathtext.fontset": "stix",
            "axes.unicode_minus": False,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "figure.dpi": 140,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
        }
    )


def _draw_orientation_frames(
    axis: Axes3D,
    positions: np.ndarray,
    rotations: np.ndarray,
    *,
    count: int = 8,
    length: float = 0.022,
) -> None:
    """在参考轨迹上稀疏绘制末端三维姿态标架。

    Args:
        axis: 用于绘制轨迹的 Matplotlib 三维坐标轴。
        positions: 参考末端位置序列，形状为 ``(n, 3)``。
        rotations: 参考末端旋转矩阵序列，形状为 ``(n, 3, 3)``。
        count: 每段轨迹上姿态标架的数量。
        length: 每根坐标轴箭头的长度，单位为 m。
    """
    # 轨迹起终点重合，因此不重复绘制最后一个姿态标架。
    indices = np.unique(np.linspace(0, positions.shape[0] - 2, count, dtype=int))
    colors = ("#d62728", "#2ca02c", "#1f77b4")
    for index in indices:
        origin = positions[index]
        rotation = rotations[index]
        axis.scatter(*origin, color="#4a4a4a", s=5, depthshade=False)
        for coordinate, color in enumerate(colors):
            direction = length * rotation[:, coordinate]
            axis.quiver(
                *origin,
                *direction,
                color=color,
                linewidth=0.9,
                arrow_length_ratio=0.3,
            )


def generate_plots() -> list[str]:
    """生成轨迹、误差、关节响应和指标对比图。

    Returns:
        所有已生成图片的路径字符串。
    """
    _style()
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    pd_nom = _load("pd_nominal")
    ctc_nom = _load("ctc_nominal")
    pd_dist = _load("pd_disturbance")
    ctc_dist = _load("ctc_disturbance")
    outputs: list[str] = []

    segments = (
        ("圆轨迹", SIM.transition_end, SIM.circle_end, 20, -35),
        ("$x$–$y$ 平面 8 字轨迹", SIM.figure8_start, SIM.figure8_end, 32, -55),
    )
    fig = plt.figure(figsize=(10.2, 4.8))
    for plot_index, (title, start, end, elevation, azimuth) in enumerate(segments, start=1):
        axis = fig.add_subplot(1, 2, plot_index, projection="3d")
        mask = (ctc_nom["time"] >= start) & (ctc_nom["time"] <= end)
        reference = ctc_nom["ee_ref"][mask]
        # 三维坐标面只保留少量主网格，避免网格遮挡轨迹。
        axis.minorticks_off()
        for coordinate_axis in (axis.xaxis, axis.yaxis, axis.zaxis):
            coordinate_axis.set_major_locator(MaxNLocator(nbins=4))
        center = 0.5 * (reference.min(axis=0) + reference.max(axis=0))
        half_span = 0.56 * np.max(np.ptp(reference, axis=0))
        axis.set(
            xlim=(center[0] - half_span, center[0] + half_span),
            ylim=(center[1] - half_span, center[1] + half_span),
            zlim=(center[2] - half_span, center[2] + half_span),
        )
        axis.set_box_aspect((1, 1, 1))
        axis.view_init(elev=elevation, azim=azimuth)
        axis.plot(*reference.T, "k--", lw=2, label="参考")
        axis.plot(*pd_nom["ee"][mask].T, lw=1.3, label="PD")
        axis.plot(*ctc_nom["ee"][mask].T, lw=1.3, label="计算力矩")
        _draw_orientation_frames(axis, reference, ctc_nom["ee_rot_ref"][mask])
        axis.set(xlabel="$x$ / m", ylabel="$y$ / m", zlabel="$z$ / m", title=title)
        axis.legend(fontsize=8)
    path = FIGURES_DIR / "trajectory_3d.png"
    fig.savefig(path)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(2, 2, figsize=(7.4, 6.3), sharex="row", layout="constrained")
    for row, (title, start, end, _, _) in enumerate(segments):
        for data, label in ((pd_nom, "PD"), (ctc_nom, "计算力矩")):
            mask = (data["time"] >= start) & (data["time"] <= end)
            local_time = data["time"][mask] - start
            position_error = 1e3 * np.linalg.norm(data["ee_error"][mask], axis=1)
            orientation_error = np.rad2deg(data["ee_orientation_error"][mask])
            axes[row, 0].plot(local_time, position_error, label=label)
            axes[row, 1].plot(local_time, orientation_error, label=label)
        axes[row, 0].set(ylabel="位置误差 / mm", title=f"{title}：位置")
        axes[row, 1].set(ylabel="姿态误差 / °", title=f"{title}：姿态")
        axes[row, 0].legend()
        axes[row, 1].legend()
    axes[1, 0].set(xlabel="段内时间 / s")
    axes[1, 1].set(xlabel="段内时间 / s")
    path = FIGURES_DIR / "segment_tracking_error.png"
    fig.savefig(path)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.7), sharex=True)
    for data, label in ((pd_nom, "PD"), (ctc_nom, "计算力矩")):
        error = 1e3 * np.linalg.norm(data["ee_error"], axis=1)
        axes[0].plot(data["time"], error, label=label)
    for data, label in ((pd_dist, "PD+外力"), (ctc_dist, "计算力矩+外力")):
        error = 1e3 * np.linalg.norm(data["ee_error"], axis=1)
        axes[1].plot(data["time"], error, label=label)
    axes[0].set(ylabel="误差 / mm", title="标称场景末端误差")
    axes[1].set(xlabel="时间 / s", ylabel="误差 / mm", title="15 N 外力抗扰场景")
    axes[1].axvspan(SIM.disturbance_start, SIM.disturbance_end, alpha=0.15, color="red")
    for axis in axes:
        axis.legend()
    path = FIGURES_DIR / "tracking_error.png"
    fig.savefig(path)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.7), sharex=True)
    for joint in range(7):
        axes[0].plot(
            ctc_nom["time"],
            np.rad2deg(ctc_nom["q"][:, joint]),
            lw=0.9,
            label=f"q{joint + 1}",
        )
        axes[1].plot(ctc_nom["time"], ctc_nom["current"][:, joint], lw=0.9)
    axes[0].set(ylabel="关节角 / °", title="计算力矩控制关节响应")
    axes[1].set(xlabel="时间 / s", ylabel="q 轴电流 / A", title="PMSM 转矩电流")
    axes[0].legend(ncol=4, fontsize=8)
    path = FIGURES_DIR / "joint_current.png"
    fig.savefig(path)
    plt.close(fig)
    outputs.append(str(path))

    summaries = json.loads((RESULTS_DIR / "summary.json").read_text(encoding="utf-8"))
    labels = [f"{item['controller'].upper()}\n{item['scenario']}" for item in summaries]
    values = [item["ee_rmse_mm"] for item in summaries]
    fig, axis = plt.subplots(figsize=(6.4, 3.8))
    bars = axis.bar(labels, values, color=["#4c78a8", "#72b7b2", "#f58518", "#e45756"])
    axis.bar_label(bars, fmt="%.2f")
    axis.set(ylabel="末端 RMSE / mm", title="控制器量化对比")
    path = FIGURES_DIR / "metrics_comparison.png"
    fig.savefig(path)
    plt.close(fig)
    outputs.append(str(path))
    return outputs


def generate_docking_plots() -> list[str]:
    """生成柔顺对接的接触力、位姿偏差和量化对比图。"""
    _style()
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    ctc = _load("docking_ctc")
    impedance = _load("docking_impedance")
    outputs: list[str] = []

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.8), sharex=True)
    for data, label in ((ctc, "关节空间逆动力学跟踪"), (impedance, "笛卡尔空间阻抗控制")):
        contact_force = np.linalg.norm(data["contact_wrench"][:, :3], axis=1)
        position_error = 1e3 * np.linalg.norm(data["ee_error"], axis=1)
        axes[0].plot(data["time"], contact_force, label=label)
        axes[1].plot(data["time"], position_error, label=label)
    axes[0].axhline(DOCKING.contact_hold_force, color="#4a4a4a", ls="--", lw=1, label="阻抗目标力")
    axes[0].set(ylabel="接触力 / N", title="对接过程接触力")
    axes[1].set(xlabel="时间 / s", ylabel="末端偏差 / mm", title="对接位姿偏差")
    for axis in axes:
        axis.axvspan(DOCKING.insertion_end, DOCKING.duration, color="#72b7b2", alpha=0.12)
        axis.legend()
    path = FIGURES_DIR / "docking_contact_response.png"
    fig.savefig(path)
    plt.close(fig)
    outputs.append(str(path))

    summaries = json.loads((RESULTS_DIR / "docking_summary.json").read_text(encoding="utf-8"))
    labels = ["关节空间\n逆动力学跟踪", "笛卡尔空间\n阻抗控制"]
    peak = [item["peak_contact_force_n"] for item in summaries]
    steady = [item["steady_contact_force_n"] for item in summaries]
    final_error = [item["ee_final_mm"] for item in summaries]
    x = np.arange(len(labels))
    width = 0.36
    fig, axes = plt.subplots(1, 2, figsize=(7.3, 3.5), layout="constrained")
    peak_bars = axes[0].bar(x - width / 2, peak, width, label="峰值")
    steady_bars = axes[0].bar(x + width / 2, steady, width, label="保持段均值")
    axes[0].bar_label(peak_bars, fmt="%.2f", fontsize=8)
    axes[0].bar_label(steady_bars, fmt="%.2f", fontsize=8)
    axes[0].set(xticks=x, xticklabels=labels, ylabel="接触力 / N", title="接触力对比")
    axes[0].legend(fontsize=8)
    error_bars = axes[1].bar(labels, final_error, color=["#4c78a8", "#72b7b2"])
    axes[1].bar_label(error_bars, fmt="%.2f", fontsize=8)
    axes[1].set(ylabel="最终位置偏差 / mm", title="柔顺偏差")
    path = FIGURES_DIR / "docking_metrics.png"
    fig.savefig(path)
    plt.close(fig)
    outputs.append(str(path))

    # --- 6D 期望 vs 实际轨迹 ---
    _draw_docking_6d_trajectory(ctc, impedance)
    outputs.append(str(FIGURES_DIR / "docking_6d_trajectory.png"))
    # --- 6D 分轴偏差曲线 ---
    _draw_docking_6d_deviation(ctc, impedance)
    outputs.append(str(FIGURES_DIR / "docking_6d_deviation.png"))

    return outputs


def _draw_docking_6d_trajectory(
    ctc: dict[str, np.ndarray],
    impedance: dict[str, np.ndarray],
) -> None:
    """绘制柔顺对接 6-DOF 期望与实际末端轨迹对比。

    三行两列：位置 (x, y, z) + RPY 姿态角 (roll, pitch, yaw)，
    每张子图同时展示关节空间逆动力学与笛卡尔空间阻抗控制的实际轨迹及共同参考轨迹。
    """
    from scipy.spatial.transform import Rotation as R  # noqa: N817

    ref_rpy = R.from_matrix(ctc["ee_rot_ref"]).as_euler("xyz")
    ctc_rpy = R.from_matrix(ctc["ee_rotation"]).as_euler("xyz")
    impedance_rpy = R.from_matrix(impedance["ee_rotation"]).as_euler("xyz")

    pos_labels = ("$x$", "$y$", "$z$")
    rpy_labels = ("roll", "pitch", "yaw")

    fig, axes = plt.subplots(3, 2, figsize=(7.8, 7.5), sharex=True, layout="constrained")
    for idx, (data, label, color) in enumerate((
        (ctc, "关节空间逆动力学实际", "#e45756"),
        (impedance, "笛卡尔空间阻抗实际", "#4c78a8"),
    )):
        for row in range(3):
            ax = axes[row, 0]
            ax.plot(
                data["time"], 1e3 * data["ee_ref"][:, row], "k--", lw=1.2, label="参考" if idx == 0 and row == 0 else ""
            )
            ax.plot(data["time"], 1e3 * data["ee"][:, row], lw=1.0, color=color, label=label if row == 0 else "")
            ax.set_ylabel(f"{pos_labels[row]} / mm")
            axes[2, 0].set_xlabel("时间 / s")
        axes[0, 0].set_title("末端位置")
        axes[0, 0].legend(fontsize=7)

    for controller_data, rpy_data in (
        ((ctc, "关节空间逆动力学实际", "#e45756"), ctc_rpy),
        ((impedance, "笛卡尔空间阻抗实际", "#4c78a8"), impedance_rpy),
    ):
        data, label, color = controller_data
        for row in range(3):
            ax = axes[row, 1]
            ax.plot(
                data["time"],
                np.rad2deg(ref_rpy[:, row]),
                "k--",
                lw=1.2,
                label="参考" if label == "关节空间逆动力学实际" and row == 0 else "",
            )
            ax.plot(
                data["time"],
                np.rad2deg(rpy_data[:, row]),
                lw=1.0,
                color=color,
                label=label if row == 0 else "",
            )
            ax.set_ylabel(f"{rpy_labels[row]} / °")
            axes[2, 1].set_xlabel("时间 / s")
        axes[0, 1].set_title("末端姿态 (RPY)")
        axes[0, 1].legend(fontsize=7)

    axes[2, 0].set_xlabel("时间 / s")
    axes[2, 1].set_xlabel("时间 / s")
    # 标注插入与保持段
    for col in range(2):
        for row in range(3):
            axes[row, col].axvspan(DOCKING.insertion_end, DOCKING.duration, color="#72b7b2", alpha=0.10)
    fig.savefig(FIGURES_DIR / "docking_6d_trajectory.png")
    plt.close(fig)


def _draw_docking_6d_deviation(
    ctc: dict[str, np.ndarray],
    impedance: dict[str, np.ndarray],
) -> None:
    """绘制柔顺对接 6-DOF 分轴偏差曲线。

    三行两列：位置偏差 (Δx, Δy, Δz) + RPY 姿态角偏差 (Δroll, Δpitch, Δyaw)，
    每张子图对比刚性 CTC 与阻抗控制的逐轴跟踪误差。
    """
    from scipy.spatial.transform import Rotation as R  # noqa: N817

    # 位置误差直接从日志获取（参考 − 实际，单位 m）。
    # 姿态误差：参考 RPY − 实际 RPY，回绕到 [−π, π]。
    ref_rpy = R.from_matrix(ctc["ee_rot_ref"]).as_euler("xyz")
    ctc_rpy = R.from_matrix(ctc["ee_rotation"]).as_euler("xyz")
    impedance_rpy = R.from_matrix(impedance["ee_rotation"]).as_euler("xyz")
    ctc_rpy_error = np.rad2deg(
        (ref_rpy - ctc_rpy + np.pi) % (2 * np.pi) - np.pi
    )
    impedance_rpy_error = np.rad2deg(
        (ref_rpy - impedance_rpy + np.pi) % (2 * np.pi) - np.pi
    )

    pos_labels = ("$\\Delta x$", "$\\Delta y$", "$\\Delta z$")
    rpy_labels = ("$\\Delta$ roll", "$\\Delta$ pitch", "$\\Delta$ yaw")

    fig, axes = plt.subplots(3, 2, figsize=(7.8, 7.5), sharex=True, layout="constrained")
    for row in range(3):
        axes[row, 0].plot(
            ctc["time"], 1e3 * ctc["ee_error"][:, row], lw=1.0, color="#e45756", label="CTC" if row == 0 else ""
        )
        axes[row, 0].plot(
            impedance["time"],
            1e3 * impedance["ee_error"][:, row],
            lw=1.0,
            color="#4c78a8",
            label="阻抗" if row == 0 else "",
        )
        axes[row, 0].set_ylabel(f"{pos_labels[row]} / mm")
        axes[row, 0].axvspan(DOCKING.insertion_end, DOCKING.duration, color="#72b7b2", alpha=0.10)

        axes[row, 1].plot(
            ctc["time"], ctc_rpy_error[:, row], lw=1.0, color="#e45756", label="CTC" if row == 0 else ""
        )
        axes[row, 1].plot(
            impedance["time"],
            impedance_rpy_error[:, row],
            lw=1.0,
            color="#4c78a8",
            label="阻抗" if row == 0 else "",
        )
        axes[row, 1].set_ylabel(f"{rpy_labels[row]} / °")
        axes[row, 1].axvspan(DOCKING.insertion_end, DOCKING.duration, color="#72b7b2", alpha=0.10)

    axes[0, 0].set_title("位置偏差")
    axes[0, 1].set_title("姿态偏差 (RPY)")
    axes[0, 0].legend(fontsize=7)
    axes[0, 1].legend(fontsize=7)
    axes[2, 0].set_xlabel("时间 / s")
    axes[2, 1].set_xlabel("时间 / s")
    fig.savefig(FIGURES_DIR / "docking_6d_deviation.png")
    plt.close(fig)
