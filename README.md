# 7 自由度 Franka Research 3 机械臂逆动力学控制

本项目是“机器人系统设计与应用”高阶期末作业。它使用 MuJoCo Menagerie 的 Franka
Research 3 v2 模型，在 PMSM 磁场定向电流环与减速器约束下比较重力/偏置补偿 PD 和
计算力矩控制，任务为末端三维圆轨迹跟踪，并包含 15 N 外力抗扰实验。

## 快速开始

```bash
uv sync --all-groups
uv run fr3-control run-experiments
uv run fr3-control plot-results
MUJOCO_GL=egl uv run fr3-control render-video
uv run pytest
```

单独运行实验：

```bash
uv run fr3-control simulate --controller ctc --scenario disturbance
```

## 输出

- `results/`：四组实验的 NPZ、CSV 和 JSON 指标。
- `figures/`：报告使用的结果图。
- `video/fr3_control_demo.mp4`：演示视频。
- `report/main.typ` 与 `report/期末作业报告.pdf`：报告源文件和 PDF。

报告中的姓名和学号已经填写；余老师知识点仍使用显式占位符，提交前必须替换。

## 模型与许可

`assets/franka_fr3_v2` 来自 Google DeepMind 的
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)，其原始许可证保留在
模型目录中。模型的位置执行器被替换为受 FR3 官方关节转矩限制约束的力矩执行器。

## 执行器模型边界

FR3 厂商资料未公开各关节电机的相电阻、电感、磁链和真实减速比，因此本项目不声称复现
FR3 内部驱动器。电气部分采用理想 FOC、`id=0` 的表贴式 PMSM q 轴等效模型，参数取自
公开的 48 V Maxon EC-i 52 无刷电机；减速比为明确标注的工程假设，关节转矩上限则采用
FR3 官方公开值。该处理用于研究执行器电流动态和限幅对运动控制的影响。
