"""将仿真日志渲染为带实时误差面板的演示视频。"""

from __future__ import annotations

import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .config import RESULTS_DIR, VIDEO_DIR
from .model import load_model, reset_home


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    """按可用顺序加载支持中英文显示的字体。

    Args:
        size: 字号，单位为像素。

    Returns:
        可供 Pillow 绘制文字使用的字体对象。
    """
    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for candidate in candidates:
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size=size)
    return ImageFont.load_default()


def _side_panel(
    time: np.ndarray,
    error_mm: np.ndarray,
    index: int,
    label: str,
    height: int = 540,
    width: int = 240,
) -> Image.Image:
    """绘制单帧右侧的实验状态与误差曲线面板。

    Args:
        time: 实验时间戳，单位为 s。
        error_mm: 各时间戳对应的末端位置误差，单位为 mm。
        index: 当前视频帧对应的日志下标。
        label: 当前实验名称。
        height: 面板高度，单位为像素。
        width: 面板宽度，单位为像素。

    Returns:
        RGB 格式的 Pillow 图像。
    """
    panel = Image.new("RGB", (width, height), "#f5f7fa")
    draw = ImageDraw.Draw(panel)
    draw.text((16, 18), "Franka FR3 · 7-DOF", fill="#172b4d", font=_font(20))
    draw.text((16, 50), label, fill="#344563", font=_font(14))
    draw.text((16, 85), f"time: {time[index]:5.2f} s", fill="#172b4d", font=_font(18))
    draw.text((16, 115), f"EE error: {error_mm[index]:6.2f} mm", fill="#172b4d", font=_font(16))
    x0, y0, x1, y1 = 18, 180, width - 16, 430
    draw.rectangle((x0, y0, x1, y1), outline="#97a0af", width=1)
    maximum = max(10.0, float(np.percentile(error_mm, 99.5)))
    draw.text((x0, y0 - 26), "Tracking error", fill="#344563", font=_font(15))
    if index > 1:
        xs = x0 + (time[: index + 1] / time[-1]) * (x1 - x0)
        ys = y1 - np.clip(error_mm[: index + 1] / maximum, 0, 1) * (y1 - y0)
        points = list(zip(xs.astype(int), ys.astype(int), strict=True))
        draw.line(points, fill="#e45756", width=3)
    draw.text((x0, y1 + 10), "green: desired position", fill="#1f7a3b", font=_font(13))
    draw.text((x0, y1 + 34), "red: end-effector", fill="#bf2600", font=_font(13))
    return panel


def _title_frame() -> Image.Image:
    """绘制完整演示视频的片头画面。"""
    frame = Image.new("RGB", (960, 540), "#f5f7fa")
    draw = ImageDraw.Draw(frame)
    draw.text((480, 145), "7-DOF Franka Research 3", anchor="mm", fill="#172b4d", font=_font(38))
    draw.text(
        (480, 205),
        "Inverse-dynamics trajectory tracking",
        anchor="mm",
        fill="#344563",
        font=_font(27),
    )
    draw.text(
        (480, 295),
        "PD vs. computed-torque control",
        anchor="mm",
        fill="#0052cc",
        font=_font(24),
    )
    draw.text(
        (480, 340),
        "Nominal circle tracking and 15 N disturbance tests",
        anchor="mm",
        fill="#344563",
        font=_font(18),
    )
    return frame


def _summary_frame() -> Image.Image:
    """根据四组实验指标绘制视频汇总画面。"""
    summaries = json.loads((RESULTS_DIR / "summary.json").read_text(encoding="utf-8"))
    nominal = {item["controller"]: item for item in summaries if item["scenario"] == "nominal"}
    reduction = (
        100.0
        * (nominal["pd"]["ee_rmse_mm"] - nominal["ctc"]["ee_rmse_mm"])
        / nominal["pd"]["ee_rmse_mm"]
    )
    frame = Image.new("RGB", (960, 540), "#f5f7fa")
    draw = ImageDraw.Draw(frame)
    draw.text((480, 60), "Experiment summary", anchor="mm", fill="#172b4d", font=_font(34))
    headers = ("Controller", "Scenario", "EE RMSE", "Max error", "Recovery")
    xs = (80, 250, 440, 620, 810)
    for x, header in zip(xs, headers, strict=True):
        draw.text((x, 130), header, anchor="mm", fill="#344563", font=_font(18))
    for row, item in enumerate(summaries):
        y = 195 + 70 * row
        values = (
            str(item["controller"]).upper(),
            str(item["scenario"]),
            f"{item['ee_rmse_mm']:.2f} mm",
            f"{item['ee_max_mm']:.2f} mm",
            "--" if item["scenario"] == "nominal" else f"{item['recovery_s']:.3f} s",
        )
        color = "#0052cc" if item["controller"] == "ctc" else "#172b4d"
        for x, value in zip(xs, values, strict=True):
            draw.text((x, y), value, anchor="mm", fill=color, font=_font(17))
    draw.text(
        (480, 495),
        f"Computed-torque control reduces nominal RMSE by {reduction:.1f}%.",
        anchor="mm",
        fill="#1f7a3b",
        font=_font(20),
    )
    return frame


def render_video(run_name: str = "all", output: Path | None = None, fps: int = 24) -> Path:
    """将一个或全部实验日志渲染为 MP4 视频。

    Args:
        run_name: 实验名称；传入 ``all`` 时依次渲染四组实验并添加片头片尾。
        output: 输出视频路径；为 ``None`` 时使用项目默认路径。
        fps: 输出视频帧率。

    Returns:
        已生成视频的路径。
    """
    output = output or VIDEO_DIR / "fr3_control_demo.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)
    model, ids = load_model()
    data = mujoco.MjData(model)
    reset_home(model, data, ids)
    renderer = mujoco.Renderer(model, height=540, width=720)
    writer = imageio.get_writer(
        output,
        fps=fps,
        codec="libx264",
        bitrate="900k",
        quality=None,
        ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
        macro_block_size=None,
    )
    try:
        if run_name == "all":
            title = np.asarray(_title_frame())
            for _ in range(5 * fps):
                writer.append_data(title)
            runs = ("pd_nominal", "ctc_nominal", "pd_disturbance", "ctc_disturbance")
        else:
            runs = (run_name,)
        for selected in runs:
            with np.load(RESULTS_DIR / f"{selected}.npz") as archive:
                log = {key: archive[key] for key in archive.files}
            frame_times = np.arange(0.0, log["time"][-1], 1.0 / fps)
            frame_indices = np.searchsorted(log["time"], frame_times).clip(0, log["time"].size - 1)
            error_mm = 1e3 * np.linalg.norm(log["ee_error"], axis=1)
            label = selected.replace("_", " · ").upper()
            for index in frame_indices:
                data.qpos[ids.joint_qpos] = log["q"][index]
                data.qvel[ids.joint_dof] = log["qd"][index]
                data.mocap_pos[ids.target_mocap] = log["ee_ref"][index]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera="report")
                robot = Image.fromarray(renderer.render())
                panel = _side_panel(log["time"], error_mm, int(index), label)
                frame = Image.new("RGB", (960, 540))
                frame.paste(robot, (0, 0))
                frame.paste(panel, (720, 0))
                writer.append_data(np.asarray(frame))
        if run_name == "all":
            summary = np.asarray(_summary_frame())
            for _ in range(8 * fps):
                writer.append_data(summary)
    finally:
        writer.close()
        renderer.close()
    return output
