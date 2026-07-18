# 7 自由度 Franka Research 3 机械臂逆动力学控制

本项目是“机器人系统设计与应用”高阶期末作业。它使用 MuJoCo Menagerie 的 Franka
Research 3 v2 模型，在 PMSM 磁场定向电流环与减速器约束下比较重力/偏置补偿 PD 和
计算力矩控制，任务为末端竖直圆与 x--y 平面 8 字组合位姿轨迹跟踪，并包含 15 N 外力抗扰实验。
此外，项目提供独立的 FR3 柔顺对接场景，对比刚性 CTC 与操作空间阻抗控制在接触峰值、保持力和柔顺位姿偏差上的权衡。

## 快速开始

```bash
uv sync --all-groups
uv run fr3-control run-experiments
uv run fr3-control plot-results
MUJOCO_GL=egl uv run fr3-control render-video
uv run fr3-control run-docking-experiments
uv run fr3-control plot-docking-results
MUJOCO_GL=egl uv run fr3-control render-docking-video
uv run pytest
```

单独运行实验：

```bash
uv run fr3-control simulate --controller ctc --scenario disturbance
uv run fr3-control simulate-docking --controller impedance
```

## 输出

- `results/`：四组实验的 NPZ、CSV 和 JSON 指标。
- `figures/`：报告使用的结果图。
- `video/fr3_control_demo.mp4`：演示视频。
- `video/fr3_compliant_docking.mp4`：刚性 CTC 与阻抗控制的柔顺对接视频。
- `report/main.typ` 与 `report/期末作业报告.pdf`：报告源文件和 PDF。

报告中的姓名和学号已经填写；余老师知识点仍使用显式占位符，提交前必须替换。

## 模型与许可

`assets/franka_fr3_v2` 来自 Google DeepMind 的
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)，其原始许可证保留在
模型目录中。模型的位置执行器被替换为受 FR3 官方关节转矩限制约束的力矩执行器。

柔顺对接使用 `assets/franka_fr3_v2/scene_docking.xml`，其中的对接件 STL 已获作者授权，来源、
用途与接触建模边界记录在 `THIRD_PARTY_NOTICES.md`。仿真直接以该 STL 网格生成 MuJoCo 原生
SDF 接触几何，因此可保留其非凸对接轮廓；该实验验证的是刚体接触下的柔顺接近与保持，不代表完整机械锁止结构的有限元或公差级复现。

## 执行器模型边界

FR3 厂商资料未公开各关节电机的相电阻、电感、磁链和真实减速比，因此本项目不声称复现
FR3 内部驱动器。电气部分采用理想 FOC、`id=0` 的表贴式 PMSM q 轴等效模型，参数取自
公开的 48 V Maxon EC-i 52 无刷电机；减速比为明确标注的工程假设，关节转矩上限则采用
FR3 官方公开值。该处理用于研究执行器电流动态和限幅对运动控制的影响。
