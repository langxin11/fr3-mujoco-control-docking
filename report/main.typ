#set document(title: "基于 MuJoCo 的 7 自由度 Franka Research 3 机械臂冗余运动规划与逆动力学轨迹跟踪控制", author: "肖大亮")
#set page(paper: "a4", margin: (left: 2.6cm, right: 2.4cm, top: 2.5cm, bottom: 2.5cm), numbering: "1")
#set text(font: ("Times New Roman", "FZShuSong-Z01", "Noto Serif CJK SC"), size: 12pt, lang: "zh", region: "CN")
#set par(
  justify: true,
  leading: 0.75em,
  first-line-indent: (amount: 2em, all: true),
)
#set heading(numbering: "1.1")
#show heading.where(level: 1): it => {
  pagebreak(weak: true)
  set text(size: 16pt, weight: "bold")
  block(above: 1.0em, below: 0.8em)[#it]
}
#show heading.where(level: 2): set text(size: 14pt, weight: "bold")
#show figure.caption: set text(size: 10.5pt)
#show table.cell: set text(size: 10.5pt)
#set math.equation(numbering: "(1)")

#let metrics = json("../results/summary.json")
#let pd-nom = metrics.at(0)
#let pd-dis = metrics.at(1)
#let ctc-nom = metrics.at(2)
#let ctc-dis = metrics.at(3)
#let f2(x) = str(calc.round(x, digits: 2))
#let reduction(before, after) = f2(100 * (before - after) / before)

#align(center + horizon)[
  #text(size: 20pt, weight: "bold")[2026 年机器人系统设计与应用]
  #v(1.2cm)
  #text(size: 24pt, weight: "bold")[期末结课作业报告]
  #v(2.2cm)
  #text(size: 20pt, weight: "bold")[基于 MuJoCo 的 7 自由度 Franka Research 3 机械臂]
  #v(0.35cm)
  #text(size: 20pt, weight: "bold")[冗余运动规划与逆动力学轨迹跟踪控制]
  #v(2.5cm)
  #table(
    columns: (4cm, 7cm),
    stroke: none,
    inset: 8pt,
    align: (right, left),
    [作业类型：], [高阶作业],
    [姓名：], [肖大亮],
    [学号：], [3220250097],
    [完成日期：], [2026 年 7 月],
  )
]

#pagebreak()
#outline(title: [目录], depth: 2)

= 作业类型与涉及知识点

研究对象为七自由度 Franka Research 3（FR3）串联机械臂，系统集成了多刚体动力学、永磁同步电机（PMSM）磁场定向控制（FOC）等效模型和减速器。运动目标为末端在三维空间竖直平面内跟踪半径为 0.08 m、频率 0.2 Hz 的圆轨迹。控制算法采用逆动力学中的计算力矩控制（Computed Torque Control，CTC），并以偏置力补偿 PD 作为基线。为模拟工程实际，轨迹规划处理了七自由度冗余性，仿真中引入电压与电流限幅、FR3 关节转矩限制、减速器效率、转子惯量和 15 N 末端恒定外力等非理想因素。

#figure(
  table(
    columns: (1.2cm, 3cm, 10.2cm),
    stroke: 0.5pt,
    inset: 7pt,
    align: (center, center, left),
    table.header([序号], [课程内容], [本项目中的对应知识点]),
    [1], [陈老师], [轨迹规划、正逆运动学、多刚体动力学、逆动力学控制、跟踪误差分析],
    [2], [李老师], [由直流电机基本电磁关系推广至 PMSM 的 q 轴等效模型、反电动势、电机惯量、减速器、电流环和多速率离散控制],
    [3], [余老师], [[待补充第 12--16 周知识点及其与本项目的对应关系]],
  ),
  caption: [三位老师知识点与项目内容的对应关系],
)

项目使用 Google DeepMind 维护的 MuJoCo Menagerie `franka_fr3_v2` 模型 @menagerie2022。该模型由 Franka 公开的 FR3 URDF 生成，保留连杆惯性、几何和关节约束；FR3 厂商资料给出的额定负载为 3 kg、最大臂展为 855 mm，并在七轴配置连杆侧力矩传感器 @franka2022fr3。MuJoCo 作为统一正向动力学仿真器，其核心方法见 @todorov2012mujoco。所有控制、绘图和视频程序均使用 Python 编写，没有使用 Simulink。

== 论文概述

本报告建立了 FR3 七自由度刚体、PMSM/FOC 工程等效驱动器和减速器的统一闭环模型，并严格区分了厂商公开的技术参数与工程假设。通过阻尼伪逆和零空间优化生成连续的冗余关节轨迹，在相同执行器约束下定量比较 PD 与计算力矩控制，并使用 15 N 末端外力检验了抗扰性能。全部实验代码提供统一命令行接口，可一条命令复现四组实验、图表和视频。

= 研究目标与总体方案

== 研究目标

研究目标是使 FR3 末端在保持姿态不变的同时跟踪半径为 0.08 m、频率为 0.2 Hz 的空间圆。仿真总时长为 8 s：前 2 s 使用五次多项式从初始位姿平滑运动至圆轨迹起点；2--7 s 完成一周圆轨迹；7--8 s 保持终点。标称实验用于评价基本精度，抗扰实验在 4.5--5.5 s 对机械臂末端施加竖直向下 15 N 外力。

量化指标包括末端位置均方根误差（RMSE）、最大末端误差、关节角 RMSE、峰值电流、峰值电压、执行器饱和比例、电能指标和撤力后恢复时间。预设成功条件为计算力矩控制标称末端 RMSE 小于 5 mm、相对 PD 至少降低 30%，并在撤力后 0.5 s 内恢复至 10 mm 误差范围。

== 系统结构

#figure(
  block(fill: luma(96%), stroke: 0.6pt, inset: 10pt, radius: 3pt)[
    #align(center)[
      #grid(columns: (1.8fr, 0.25fr, 1.8fr, 0.25fr, 1.8fr), gutter: 5pt,
        block(fill: rgb("dbeafe"), inset: 8pt)[期望末端圆轨迹\冗余逆运动学],
        align(horizon)[→],
        block(fill: rgb("dcfce7"), inset: 8pt)[PD / 计算力矩\期望关节力矩],
        align(horizon)[→],
        block(fill: rgb("ffedd5"), inset: 8pt)[FOC 电流 PI、PMSM\减速器与限幅],
      )
      #v(0.3cm)
      #grid(columns: (2fr, 0.25fr, 2fr), gutter: 5pt,
        block(fill: rgb("f3e8ff"), inset: 8pt)[MuJoCo 七自由度多刚体动力学\外力扰动],
        align(horizon)[→],
        block(fill: rgb("fce7f3"), inset: 8pt)[关节与末端状态反馈\误差和能量统计],
      )
    ]
  ],
  caption: [机械臂运动控制系统总体结构],
)

控制周期采用多速率结构：MuJoCo 和 FOC q 轴电流环以 0.05 ms 更新，外层运动控制器以 1 ms 更新。外层控制器给出期望关节力矩，力矩通过减速器参数换算为期望 q 轴电流，电流 PI 计算 q 轴电压；PMSM 实际转矩电流产生的电磁转矩经减速器作用于机械臂。

= 运动学与轨迹规划

== 正运动学与雅可比矩阵

设七个关节角构成向量 $q in RR^7$，末端位姿写作齐次变换 $T_0^e(q)$。正运动学由各关节刚体变换依次相乘得到。末端线速度和角速度由几何雅可比矩阵给出：

$ mat(v_e, omega_e) = J(q) dot(q), quad J(q) in RR^(6 times 7). $

由于任务空间为六维而机械臂有七个关节，系统存在一个冗余自由度。冗余使末端任务存在无穷多组局部关节速度解，可利用零空间改善关节姿态并远离限位。

== 阻尼伪逆与零空间运动

本项目采用阻尼最小二乘伪逆

$ J^"+" = J^T (J J^T + lambda^2 I)^(-1) $

并用下式求取每次逆运动学迭代的关节增量：

$ Delta q = J^"+" e + (I-J^"+"J) k_n (q_h-q). $

其中 $e$ 同时包含末端位置误差和姿态旋转向量误差，$q_h$ 为舒适初始姿态，$k_n=0.025$ 为零空间回中系数，阻尼系数 $lambda=2.5 times 10^(-3)$。每步关节增量限制为 0.08 rad，并在机械限位内保留 0.025 rad 裕量。首先以 10 ms 间隔求解离散逆运动学，再用三次样条插值得到 1 ms 控制周期下连续的 $q_d$、$dot(q)_d$ 和 $dot.double(q)_d$。

== 轨迹连续性

过渡阶段使用五次时间缩放

$ s(u)=10u^3-15u^4+6u^5, quad u in [0,1], $

保证过渡两端的速度和加速度为零。圆轨迹位于末端初始位置附近的 $y$--$z$ 平面，固定 $x$ 坐标和末端姿态。离线逆解之后抽取 25 个时刻重新计算正运动学，最大位置重构误差小于 3 mm；同时检查相邻关节参考角变化，避免由逆解分支切换造成跳变。

#figure(
  image("../figures/trajectory_3d.png", width: 82%),
  caption: [期望圆轨迹及两种控制器的实际末端轨迹],
)

= 多刚体、电机与减速器建模

== 七自由度机械臂动力学

机械臂关节空间动力学写为 @featherstone2008：

$ M(q) dot.double(q) + C(q,dot(q))dot(q) + g(q) = tau + J_v(q)^T F_"ext". $

其中 $M(q)$ 为对称正定质量矩阵，$C(q,dot(q))dot(q)$ 为科氏力与离心力，$g(q)$ 为重力，$tau$ 为执行器关节力矩，$F_"ext"$ 为末端外力。仿真中由 MuJoCo 的组合刚体算法展开质量矩阵，并从偏置力向量取得科氏、离心和重力项。测试程序逐次验证质量矩阵的对称性和正定性。

FR3 v2 模型保留了各连杆质量、质心和完整惯性张量。关节碰撞、几何网格和机械限位来自 Menagerie；原始位置执行器被替换为力矩执行器，使控制输入不再是关节位置指令，而是 PMSM 电流经减速器产生的关节力矩。关节力矩绝对值限制采用 FR3 FCI 公开值：前四轴为 87 N·m，后三轴为 12 N·m @frankafci2026。

== PMSM 与 FOC 工程等效模型

现代机器人关节通常采用无刷伺服驱动。仿真采用表贴式 PMSM 在理想转子位置检测、理想 FOC 且 $i_d=0$ 条件下的 q 轴模型描述转矩通道：

$ L_q dot(i_q) + R_s i_q + K_e omega_m = u_q, quad tau_m=K_t i_q. $

FR3 厂商资料没有公开各关节电机的相电阻、电感、磁链和真实减速比，因此报告中不将任何电气参数表述为 FR3 官方内部参数。为建立可复现实验，选用公开的 48 V Maxon EC-i 52、180 W 无刷电机作为代表性参数源 @maxon2025eci52：$R_s=0.284 Omega$，$L_q=0.45 "mH"$，$K_t=0.0919 N dot m/A$，$K_e=(1/104) V/"rpm" approx 0.0918 V dot s/"rad"$，极对数为 7，转子惯量 $J_m=1.70 times 10^(-5) "kg" dot m^2$。该模型用于考察电流动态、反电动势和电压限幅对控制的影响，而非复刻 FR3 私有驱动器。

== 减速器与电流控制

由于 FR3 真实减速比同样未公开，七个关节的工程等效减速比分别设为 $[100,100,100,100,50,50,50]$，效率设为 $eta=0.9$，并通过前述 FR3 官方关节转矩上限截断最终输出。电机与关节侧变量满足

$ omega_m=N dot(q), quad tau=eta N K_t i, quad J_"ref"=J_m N^2. $

q 轴电流 PI 的带宽取 $omega_c=2000 "rad"/s$，根据一阶电流模型设 $K_"pi"=L_q omega_c$、$K_"ii"=R_s omega_c$，并加入反电动势和电阻压降前馈。48 V 等效电压饱和时使用反算抗积分饱和；峰值电流由关节转矩限制反算得到，确保最终关节力矩不超过 FR3 公开边界。

#figure(
  image("../figures/joint_current.png", width: 92%),
  caption: [计算力矩控制下的七关节响应与 PMSM q 轴电流],
)

= 控制器设计与参数选择

== 偏置力补偿 PD

基线控制器采用

$ tau_"PD"=K_p(q_d-q)+K_d(dot(q)_d-dot(q))+h(q,dot(q)), $

其中 $h=C dot(q)+g$ 为当前状态的动力学偏置项。使用偏置补偿可以避免比较结果仅由静态重力误差决定，使对比更集中于惯性耦合补偿和动态跟踪性能。七个关节的刚度参数为 $[160,180,140,110,70,45,30]$，阻尼参数为 $[25,28,22,18,12,8,6]$。

== 计算力矩控制

计算力矩控制利用完整模型进行非线性反馈线性化 @siciliano2009：

$ tau_"CTC"=M(q)[dot.double(q)_d+K_d(dot(q)_d-dot(q))+K_p(q_d-q)]+h(q,dot(q)). $

当模型准确且执行器不饱和时，闭环误差满足

$ dot.double(e)+K_d dot(e)+K_p e=0. $

按二阶标准形式设置 $K_p=omega_n^2 I$、$K_d=2 zeta omega_n I$。在 $omega_n in {12,15,18,22,28}$ 的确定性网格中，综合末端 RMSE、最大误差和饱和率，最终选择 $omega_n=18 "rad"/s$、$zeta=1$。更高带宽虽然可以继续降低理想跟踪误差，但会增加峰值电流和模型不确定性敏感度。

两种控制器使用完全相同的参考轨迹、初始状态、物理步长、电机、减速器和电压/电流限制。控制器输出期望关节力矩，实际力矩必须经过电流动态后作用于 MuJoCo 模型——该约束避免了”理想力矩源”假设掩盖执行器带宽的实际情况。

= 仿真结果与分析

== 标称轨迹跟踪

#figure(
  image("../figures/tracking_error.png", width: 94%),
  caption: [标称与 15 N 外力场景下的末端位置误差],
)

标称场景中，PD 的末端 RMSE 为 #f2(pd-nom.at("ee_rmse_mm")) mm，最大误差为 #f2(pd-nom.at("ee_max_mm")) mm；计算力矩控制的末端 RMSE 为 #f2(ctc-nom.at("ee_rmse_mm")) mm，最大误差为 #f2(ctc-nom.at("ee_max_mm")) mm。计算力矩控制将 RMSE 降低约 #reduction(pd-nom.at("ee_rmse_mm"), ctc-nom.at("ee_rmse_mm"))%，并将最大误差降低约 #reduction(pd-nom.at("ee_max_mm"), ctc-nom.at("ee_max_mm"))%，满足小于 5 mm 且相对 PD 降低 30% 的预设目标。

PD 的关节 RMSE 略小，并不与末端结论矛盾：七自由度系统中，不同关节误差组合通过雅可比映射后对末端位置影响不同；计算力矩控制更准确地补偿了多轴惯性耦合，使任务空间轨迹更接近期望圆。计算力矩控制峰值电流达到 #f2(ctc-nom.at("peak_current_a")) A，但饱和点仅占 #f2(ctc-nom.at("saturation_percent"))%，主要发生在参考加速度变化较快的短暂时刻。

== 外力抗扰性能

15 N 竖直向下外力作用期间，PD 的末端 RMSE 增至 #f2(pd-dis.at("ee_rmse_mm")) mm，最大误差为 #f2(pd-dis.at("ee_max_mm")) mm；计算力矩控制的 RMSE 为 #f2(ctc-dis.at("ee_rmse_mm")) mm，最大误差为 #f2(ctc-dis.at("ee_max_mm")) mm。相对 PD，计算力矩控制将抗扰场景 RMSE 降低约 #reduction(pd-dis.at("ee_rmse_mm"), ctc-dis.at("ee_rmse_mm"))%。

撤去外力后，PD 恢复到 10 mm 误差以内需要 #f2(pd-dis.at("recovery_s")) s，计算力矩控制仅需 #f2(ctc-dis.at("recovery_s")) s。后者的误差反馈位于模型补偿后的等效线性系统内，闭环带宽更为一致。计算力矩控制本身并非鲁棒控制，若质量、摩擦或减速器效率存在显著建模误差，其优势可能减小；本实验验证的是反馈抗扰能力而非参数自适应能力。

#figure(
  image("../figures/metrics_comparison.png", width: 76%),
  caption: [四组实验末端 RMSE 量化比较],
)

== 指标汇总

#figure(
  table(
    columns: (2.1cm, 2.4cm, 2.3cm, 2.3cm, 2.2cm, 2.2cm),
    stroke: 0.5pt,
    inset: 5pt,
    align: center,
    table.header([控制器], [场景], [RMSE/mm], [最大误差/mm], [峰值电流/A], [恢复时间/s]),
    [PD], [标称], [#f2(pd-nom.at("ee_rmse_mm"))], [#f2(pd-nom.at("ee_max_mm"))], [#f2(pd-nom.at("peak_current_a"))], [--],
    [计算力矩], [标称], [#f2(ctc-nom.at("ee_rmse_mm"))], [#f2(ctc-nom.at("ee_max_mm"))], [#f2(ctc-nom.at("peak_current_a"))], [--],
    [PD], [15 N 外力], [#f2(pd-dis.at("ee_rmse_mm"))], [#f2(pd-dis.at("ee_max_mm"))], [#f2(pd-dis.at("peak_current_a"))], [#f2(pd-dis.at("recovery_s"))],
    [计算力矩], [15 N 外力], [#f2(ctc-dis.at("ee_rmse_mm"))], [#f2(ctc-dis.at("ee_max_mm"))], [#f2(ctc-dis.at("peak_current_a"))], [#f2(ctc-dis.at("recovery_s"))],
  ),
  caption: [控制器实验指标汇总],
)

= 结论与展望

本报告完成了一个七自由度 FR3 机械臂的高阶运动控制仿真。系统并非理想关节力矩源，而是包含了 PMSM q 轴电感、电阻、反电动势、FOC 电流 PI、电压/电流限制、转子惯量、减速器效率和 FR3 官方关节转矩限制的完整执行器链。通过冗余逆运动学生成连续参考轨迹，在相同条件下比较偏置力补偿 PD 与计算力矩控制。报告严格区分了 FR3 官方整机限制、Menagerie 刚体数据和代表性 PMSM 工程假设，避免将通用电机参数误标为 FR3 内部参数。

实验表明，计算力矩控制在标称圆轨迹下取得 #f2(ctc-nom.at("ee_rmse_mm")) mm 末端 RMSE，比 PD 降低约 #reduction(pd-nom.at("ee_rmse_mm"), ctc-nom.at("ee_rmse_mm"))%；在 15 N 外力下取得 #f2(ctc-dis.at("ee_rmse_mm")) mm RMSE，比 PD 降低约 #reduction(pd-dis.at("ee_rmse_mm"), ctc-dis.at("ee_rmse_mm"))%，撤力后恢复时间约为 #f2(ctc-dis.at("recovery_s")) s。结果说明，对于多轴耦合显著的冗余机械臂，利用质量矩阵和动力学偏置项进行补偿能够显著改善任务空间跟踪与抗扰恢复。

后续工作可沿以下方向推进：引入连杆质量和负载的不确定性，考察鲁棒或自适应计算力矩控制的效果；在零空间中纳入可操作度最大化与碰撞约束；将离线逆运动学替换为在线优化控制，在统一框架中同时处理轨迹生成、关节限位、电流约束和障碍物规避。

= 附录：复现说明

项目使用 Python 3.12 和 `uv` 管理依赖，主要命令如下：

- `uv run fr3-control run-experiments`：生成四组实验的 NPZ、CSV 和 JSON 数据；
- `uv run fr3-control plot-results`：生成报告插图；
- `MUJOCO_GL=egl uv run fr3-control render-video`：渲染演示视频（需要 EGL 支持）；
- `uv run pytest`：验证电机单位换算、限幅、质量矩阵、轨迹连续性和完整闭环仿真。

所有实验使用固定参数和确定性初始状态。报告中的数值直接从 `results/summary.json` 读取，重新运行实验并编译 Typst 后可自动更新。

#bibliography("references.bib", title: [参考文献], style: "ieee")
