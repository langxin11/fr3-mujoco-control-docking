"""项目命令行入口及子命令定义。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .plots import generate_docking_plots, generate_plots
from .simulation import run_all_experiments, run_docking_experiments, simulate, simulate_docking
from .video import render_docking_interface_preview, render_docking_video, render_video


def _parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。

    Returns:
        配置完成的顶层参数解析器。
    """
    parser = argparse.ArgumentParser(description="FR3 inverse-dynamics control project")
    commands = parser.add_subparsers(dest="command", required=True)
    single = commands.add_parser("simulate", help="run one controller/scenario pair")
    single.add_argument("--controller", choices=("pd", "ctc"), required=True)
    single.add_argument("--scenario", choices=("nominal", "disturbance"), required=True)
    docking = commands.add_parser("simulate-docking", help="run one compliant docking controller")
    docking.add_argument("--controller", choices=("ctc", "impedance"), required=True)
    commands.add_parser("run-experiments", help="run all four experiments")
    commands.add_parser(
        "run-docking-experiments", help="run rigid and compliant docking experiments"
    )
    commands.add_parser("plot-results", help="generate report figures")
    commands.add_parser("plot-docking-results", help="generate compliant docking figures")
    video = commands.add_parser("render-video", help="render the MP4 demonstration")
    video.add_argument("--run", default="all")
    video.add_argument("--output", type=Path)
    docking_video = commands.add_parser("render-docking-video", help="render compliant docking MP4")
    docking_video.add_argument("--output", type=Path)
    interface_preview = commands.add_parser(
        "render-docking-interface-preview",
        help="render the arm-free docking-interface alignment preview",
    )
    interface_preview.add_argument("--output", type=Path)
    return parser


def main() -> None:
    """解析命令行参数并分派到对应的实验、绘图或视频流程。"""
    args = _parser().parse_args()
    if args.command == "simulate":
        _, metrics = simulate(args.controller, args.scenario)
        print(json.dumps(metrics, ensure_ascii=False, indent=2))
    elif args.command == "simulate-docking":
        _, metrics = simulate_docking(args.controller)
        print(json.dumps(metrics, ensure_ascii=False, indent=2))
    elif args.command == "run-experiments":
        print(json.dumps(run_all_experiments(), ensure_ascii=False, indent=2))
    elif args.command == "run-docking-experiments":
        print(json.dumps(run_docking_experiments(), ensure_ascii=False, indent=2))
    elif args.command == "plot-results":
        print("\n".join(generate_plots()))
    elif args.command == "plot-docking-results":
        print("\n".join(generate_docking_plots()))
    elif args.command == "render-video":
        print(render_video(args.run, args.output))
    elif args.command == "render-docking-video":
        print(render_docking_video(args.output))
    elif args.command == "render-docking-interface-preview":
        print(render_docking_interface_preview(args.output))


if __name__ == "__main__":
    main()
