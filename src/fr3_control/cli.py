"""项目命令行入口及子命令定义。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .plots import generate_plots
from .simulation import run_all_experiments, simulate
from .video import render_video


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
    commands.add_parser("run-experiments", help="run all four experiments")
    commands.add_parser("plot-results", help="generate report figures")
    video = commands.add_parser("render-video", help="render the MP4 demonstration")
    video.add_argument("--run", default="all")
    video.add_argument("--output", type=Path)
    return parser


def main() -> None:
    """解析命令行参数并分派到对应的实验、绘图或视频流程。"""
    args = _parser().parse_args()
    if args.command == "simulate":
        _, metrics = simulate(args.controller, args.scenario)
        print(json.dumps(metrics, ensure_ascii=False, indent=2))
    elif args.command == "run-experiments":
        print(json.dumps(run_all_experiments(), ensure_ascii=False, indent=2))
    elif args.command == "plot-results":
        print("\n".join(generate_plots()))
    elif args.command == "render-video":
        print(render_video(args.run, args.output))


if __name__ == "__main__":
    main()
