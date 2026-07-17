"""读取实验结果并生成课程报告使用的静态图表。"""

from __future__ import annotations

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401  # 导入时向 Matplotlib 注册 SciencePlots 样式。
from matplotlib.ticker import MaxNLocator

from .config import FIGURES_DIR, RESULTS_DIR, SIM


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

    fig = plt.figure(figsize=(6.8, 5.2))
    axis = fig.add_subplot(111, projection="3d")
    # 三维坐标面只保留少量主网格，避免 SciencePlots 次网格遮挡轨迹。
    axis.minorticks_off()
    for coordinate_axis in (axis.xaxis, axis.yaxis, axis.zaxis):
        coordinate_axis.set_major_locator(MaxNLocator(nbins=5))
    # 三轴使用相同物理尺度，避免将毫米级 x 误差视觉放大到圆轨迹直径大小。
    reference_min = ctc_nom["ee_ref"].min(axis=0)
    reference_max = ctc_nom["ee_ref"].max(axis=0)
    center = 0.5 * (reference_min + reference_max)
    half_span = 0.55 * np.max(reference_max - reference_min)
    axis.set(
        xlim=(center[0] - half_span, center[0] + half_span),
        ylim=(center[1] - half_span, center[1] + half_span),
        zlim=(center[2] - half_span, center[2] + half_span),
    )
    axis.set_box_aspect((1, 1, 1))
    axis.plot(*ctc_nom["ee_ref"].T, "k--", lw=2, label="期望轨迹")
    axis.plot(*pd_nom["ee"].T, lw=1.3, label="PD")
    axis.plot(*ctc_nom["ee"].T, lw=1.3, label="计算力矩")
    axis.set(xlabel="x / m", ylabel="y / m", zlabel="z / m", title="末端空间轨迹")
    axis.legend()
    path = FIGURES_DIR / "trajectory_3d.png"
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
