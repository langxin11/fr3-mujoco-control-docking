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
#let docking-metrics = json("../results/docking_summary.json")
#let docking-ctc = docking-metrics.at(0)
#let docking-impedance = docking-metrics.at(1)
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

= 引言

研究对象为七自由度 Franka Research 3（FR3）串联机械臂，运动目标由半径 0.10 m、频率 0.2 Hz 的 $y$--$z$ 竖直圆和 $x$--$y$ 平面 8 字轨迹组成，并同步完成平滑的滚转、俯仰与偏航姿态变化。控制算法采用逆动力学中的计算力矩控制（Computed Torque Control, CTC），并以带惯性前馈的偏置力补偿 PD 作为基线，在相同执行器约束下进行定量比较。

为使仿真更贴近工程实际，系统并非假设理想关节力矩源，而是建立了包含多刚体动力学、永磁同步电机（PMSM）、磁场定向控制（FOC）等效模型和减速器的完整执行器链。轨迹规划处理了七自由度冗余性，仿真中引入了电压与电流限幅、FR3 关节转矩限制、减速器效率、转子惯量和 15 N 末端恒定外力等非理想因素。

项目使用 Google DeepMind 维护的 MuJoCo Menagerie `franka_fr3_v2` 模型 @menagerie2022。该模型由 Franka 公开的 FR3 URDF 生成，保留了连杆惯性、几何与关节约束；FR3 厂商资料给出的额定负载为 3 kg、最大臂展为 855 mm，七轴均配置连杆侧力矩传感器 @franka2024fr3。MuJoCo 作为统一正向动力学仿真器的核心方法见 @todorov2012mujoco。所有控制、绘图和视频程序均使用 Python 编写。

本报告的主要工作包括：建立了 FR3 七自由度刚体、PMSM/FOC 工程等效驱动器和减速器的统一闭环模型，并严格区分了厂商公开的技术参数与工程假设；通过阻尼伪逆和零空间优化生成了连续的冗余关节轨迹；在相同执行器约束下定量比较了 PD 与计算力矩控制的跟踪精度与抗扰性能；进一步引入操作空间阻抗控制和受授权对接件外观，构造刚性 CTC 与柔顺接触的对接比较实验。全部实验代码提供统一命令行接口，可一条命令复现实验、图表和视频。

= 系统总体方案

本章给出控制回路的整体结构，后续各章沿信号正向传播路径依次展开：运动学规划（第 3 章）生成期望关节轨迹，控制器（第 5 章）根据跟踪误差计算期望关节力矩，执行器链将力矩指令转化为实际关节力矩驱动被控对象（第 4 章），最终由实验（第 6 章）验证整体性能。

== 系统架构

#figure(
  block(fill: luma(96%), stroke: 0.6pt, inset: 10pt, radius: 3pt)[
    #align(center)[
      #grid(columns: (1.8fr, 0.25fr, 1.8fr, 0.25fr, 1.8fr), gutter: 5pt,
        block(fill: rgb("dbeafe"), inset: 8pt)[期望末端组合位姿轨迹\冗余逆运动学],
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

控制系统按照"期望轨迹 → 控制器 → 执行器 → 被控对象 → 状态反馈"的闭环回路组织。蓝色模块对应第 3 章运动学规划，绿色模块对应第 5 章控制器设计，橙/紫色模块共同对应第 4 章被控对象建模。

== 多速率控制时序

控制周期采用多速率结构：MuJoCo 物理仿真和 FOC q 轴电流环以 0.05 ms 更新，外层运动控制器以 1 ms 更新。外层控制器给出期望关节力矩，该力矩通过减速器参数换算为期望 q 轴电流；电流 PI 调节器计算 q 轴电压，PMSM 实际转矩电流产生的电磁转矩经减速器作用于机械臂。

= 运动学与轨迹规划

被控对象在关节空间运动，而控制目标定义在末端任务空间，因此需要运动学模型建立两者的映射关系。本章从正运动学出发，建立任务空间与关节空间的微分映射（雅可比矩阵），进而处理七自由度冗余性以生成连续的关节空间参考轨迹。

== 正运动学与雅可比矩阵

设七个关节角构成向量 $q in RR^7$，末端位姿写作齐次变换 $T_0^e(q)$。正运动学由各关节刚体变换依次相乘得到。末端线速度和角速度由几何雅可比矩阵给出：

$ mat(v_e, omega_e) = J(q) dot(q), quad J(q) in RR^(6 times 7). $

由于任务空间为六维而机械臂有七个关节，系统存在一个冗余自由度：末端运动仅约束雅可比映射的六维行空间，其七维零空间中任意关节速度分量不改变末端状态。这一特性为改善关节姿态、远离机械限位提供了自由度，但也意味着需在无穷多组局部关节速度解中选择合适的一组。

== 阻尼伪逆与零空间运动

采用阻尼最小二乘伪逆来构造逆运动学解：

$ J^"+" = J^T (J J^T + lambda^2 I)^(-1) $

每次逆运动学迭代的关节增量由两项叠加：主项将末端误差映射至关节空间，零空间项利用冗余自由度将关节向舒适姿态回中：

$ Delta q = J^"+" e + (I-J^"+"J) k_n (q_h-q). $

其中 $e$ 同时包含末端位置误差和姿态旋转向量误差，$q_h$ 为舒适初始姿态，$k_n=0.025$ 为零空间回中系数，阻尼系数 $lambda=2.5 times 10^(-3)$。每步关节增量限制为 0.08 rad，并在机械限位内保留 0.025 rad 裕量。

== 轨迹生成与连续性

第一段任务是位于末端初始位置附近、半径为 0.10 m 的 $y$--$z$ 竖直圆，$x$ 坐标保持不变；第二段是 $x$--$y$ 平面 8 字轨迹，x 和 y 向半宽分别为 0.10 m 和 0.07 m。圆周段叠加最大 12° 滚转和 6° 俯仰，8 字段叠加最大 16° 偏航和 5° 滚转；姿态角在各段起终点的值与一阶导数均为零。仿真总时长为 14 s：0--1.5 s 过渡至圆起点，1.5--6.5 s 完成圆轨迹，6.5--8 s 过渡至 8 字中心，8--13 s 完成 8 字轨迹，13--14 s 保持终点。

过渡阶段使用五次时间缩放函数

$ s(u)=10u^3-15u^4+6u^5, quad u in [0,1], $

保证过渡两端的速度和加速度均为零，避免启停冲击。离线逆运动学以 10 ms 间隔求解离散关节参考点，再用三次样条插值得到 1 ms 控制周期下连续的 $q_d$、$dot(q)_d$ 和 $dot.double(q)_d$。离线逆解后抽取 25 个时刻重新计算正运动学，最大位置重构误差小于 3 mm、最大姿态重构误差小于 0.5°；同时检查相邻关节参考角变化，确认未出现逆解分支切换引起的跳变。

= 被控对象建模

上一章解决了"期望关节轨迹从哪来"的问题。然而，从期望关节力矩到实际关节运动之间存在一系列物理环节：电机将电压转化为电流与电磁转矩，减速器将高速低扭矩转换为低速高扭矩，最终驱动多刚体机械臂。本章沿信号正向传播路径，从内向外依次建立各环节的数学模型。

== 七自由度机械臂动力学

机械臂关节空间动力学描述关节力矩与关节运动之间的关系 @featherstone2008：

$ M(q) dot.double(q) + C(q,dot(q))dot(q) + g(q) = tau + J_v(q)^T F_"ext". $

其中 $M(q)$ 为对称正定质量矩阵，$C(q,dot(q))dot(q)$ 为科氏力与离心力项，$g(q)$ 为重力项，$tau$ 为执行器输出的关节力矩，$F_"ext"$ 为末端外力。仿真中由 MuJoCo 的组合刚体算法展开质量矩阵，并从偏置力向量取得科氏、离心和重力项，测试程序逐次验证了质量矩阵的对称性和正定性。

FR3 v2 模型保留了各连杆质量、质心和完整惯性张量。关节碰撞、几何网格和机械限位来自 Menagerie；原始位置执行器被替换为力矩执行器，使控制输入不再是关节位置指令，而是 PMSM 电流经减速器产生的关节力矩。关节力矩绝对值限制采用 FR3 FCI 公开值：前四轴为 87 N·m，后三轴为 12 N·m @frankafci。

== PMSM 与 FOC 工程等效模型

上述动力学模型中的 $tau$ 由电机经减速器产生。现代机器人关节普遍采用无刷伺服驱动，仿真中采用表贴式 PMSM 在理想转子位置检测、理想 FOC 且 $i_d=0$ 条件下的 q 轴模型描述转矩通道：

$ L_q dot(i_q) + R_s i_q + K_e omega_m = u_q, quad tau_m=K_t i_q. $

FR3 厂商资料未公开各关节电机的相电阻、电感、磁链和真实减速比，因此报告中不将任何电气参数表述为 FR3 官方内部参数。为建立可复现实验，选用公开的 48 V Maxon EC-i 52、180 W 无刷电机作为代表性参数源 @maxoneci52：$R_s=0.284 Omega$，$L_q=0.45 "mH"$，$K_t=0.0919 N dot m/A$，$K_e=(1/104) V/"rpm" approx 0.0918 V dot s/"rad"$，极对数为 8，转子惯量 $J_m=1.70 times 10^(-5) "kg" dot m^2$。该模型用于考察电流动态、反电动势和电压限幅对控制的影响，而非复刻 FR3 私有驱动器。

== 减速器与电流控制

电机产生的电磁转矩经减速器放大后作用于关节。由于 FR3 真实减速比同样未公开，七个关节的工程等效减速比分别设为 $[100,100,100,100,50,50,50]$，效率设为 $eta=0.9$，并通过前述 FR3 官方关节转矩上限截断最终输出。电机与关节侧变量满足

$ omega_m=N dot(q), quad tau=eta N K_t i, quad J_"ref"=J_m N^2. $

至此，从电压到关节运动的完整物理因果链已建立：$u_q$ 经 $R_s$、$L_q$ 决定 $i_q$，$i_q$ 经 $K_t$ 决定 $tau_m$，$tau_m$ 经 $N$ 和 $eta$ 决定 $tau$，$tau$ 经机械臂动力学决定 $q$。在此过程中，反电动势 $K_e omega_m$ 形成与转速相关的负反馈，电压饱和和电流限制则引入非线性。

q 轴电流 PI 的带宽取 $omega_c=2000 "rad"/s$，根据一阶电流模型设 $K_"pi"=L_q omega_c$、$K_"ii"=R_s omega_c$，并加入反电动势和电阻压降前馈。48 V 等效电压饱和时使用反算抗积分饱和；峰值电流由关节转矩限制反算得到，确保最终关节力矩不超过 FR3 公开边界。

= 控制器设计

前两章分别建立了运动学映射和被控对象模型，本章在此基础上设计控制器：给定期望关节轨迹 $q_d$ 和实际关节状态 $q$、$dot(q)$，计算期望关节力矩 $tau_d$，使跟踪误差收敛。

== 带惯性前馈的偏置力补偿 PD

基线控制器在关节空间 PD 反馈的基础上，引入惯性前馈项 $M(q)dot.double(q)_d$，构成前馈-反馈复合结构：

$ tau_"PD"=M(q)dot.double(q)_d+K_p(q_d-q)+K_d(dot(q)_d-dot(q))+h(q,dot(q)), $

其中 $h=C dot(q)+g$ 为当前状态的科氏力与重力偏置项。惯性前馈项 $M(q)dot.double(q)_d$ 使控制器能够提前输出维持参考加速度所需的力矩，无需等待位置或速度误差积累；$K_p$ 和 $K_d$ 则构成对角反馈增益，用于抑制剩余的跟踪误差和外力扰动。与计算力矩控制相比，该结构保留了前馈补偿中的惯性耦合信息，但反馈部分不使用质量矩阵进行关节解耦，两者之间的剩余差距可归因于反馈通道的惯性解耦和高增益。

引入偏置补偿的目的是使对比集中于惯性耦合补偿和动态跟踪性能，而非静态重力误差——若基线不含偏置项，PD 的跟踪误差将主要由重力项决定，无法公允地比较两种控制策略在多轴耦合场景下的差异。

七个关节的刚度参数为 $[160,180,140,110,70,45,30]$，阻尼参数为 $[25,28,22,18,12,8,6]$。这些参数通过试凑选取，权衡了响应速度与饱和率。

== 计算力矩控制

计算力矩控制（Computed Torque Control, CTC）在控制理论中更通用的名称是反馈线性化控制（Feedback Linearization Control）或逆动力学控制（Inverse Dynamics Control），CTC 是它在机器人关节空间的具体实现 @siciliano2009。三者层次关系为：反馈线性化是最上位的非线性控制框架，逆动力学控制强调用动力学模型抵消被控对象的非线性项，CTC 则特指将逆动力学计算得到的力矩直接作为控制指令。

CTC 的控制律为

$ tau_"CTC"=M(q)[dot.double(q)_d+K_d(dot(q)_d-dot(q))+K_p(q_d-q)]+h(q,dot(q)). $

其核心思想是先用 $M(q)$ 和 $h(q,dot(q))$ 抵消被控对象的非线性动态，使等效闭环系统简化为线性二阶误差动力学。当模型准确且执行器不饱和时，将控制律代入式 (5) 的机械臂动力学方程 $M(q)dot.double(q)+h(q,dot(q))=tau$ 可得

$ M(q)dot.double(q)+h=M(q)[dot.double(q)_d+K_d dot(e)+K_p e]+h, $
$ M(q)dot.double(q)=M(q)[dot.double(q)_d+K_d dot(e)+K_p e], $
$ dot.double(q)=dot.double(q)_d+K_d dot(e)+K_p e quad (M(q) "恒可逆"), $
$ dot.double(e)+K_d dot(e)+K_p e=0. $

原本非线性、强耦合的七自由度机械臂动力学，在 CTC 作用下被"线性化"为一个解耦的二阶线性误差系统——这正是反馈线性化名称的由来。

按二阶标准形式设置 $K_p=omega_n^2 I$、$K_d=2 zeta omega_n I$。在 $omega_n in {12,15,18,22,28}$ 的确定性网格中，综合末端 RMSE、最大误差和饱和率，最终选择 $omega_n=18 "rad"/s$、$zeta=1$。更高带宽可继续降低理想跟踪误差，但会推高峰值电流并增大模型不确定性敏感度。

== 操作空间阻抗控制

=== 刚性控制的接触缺陷

首先分析计算力矩控制（§5.2）在接触场景下的根本局限。将 CTC 控制律 $tau_"CTC"=M(q)dot.double(q)_c+h(q,dot(q))$ 代入机械臂动力学方程，并假设模型补偿准确 $(tilde(h)=h)$，闭环关节动力学为：

$ dot.double(q)=dot.double(q)_c+M(q)^(-1)J(q)^T F_"ext". $

左乘 $J M^(-1)$ 并利用运动学关系，映射到任务空间加速度：

$ dot.double(x)=dot.double(x)_d+Lambda^(-1)F_"ext", quad Lambda = (J M^(-1)J^T)^(-1). $

该式表明 CTC 的闭环对外力呈现纯惯性响应 $Lambda^(-1)F_"ext"$，缺乏 $D_d dot(e)$ 和 $K_d e$ 项来调节接触力。在精密对接等接触密集任务中，这种无阻尼特性将导致接触力过大甚至碰撞不稳定 @ren2026unified。因此，刚性跟踪控制器不能满足对接任务对柔顺交互的根本需求。

=== 阻抗控制律与临界阻尼设计

柔顺对接采用六维操作空间阻抗关系 @hogan1985impedance：

$ Lambda (dot.double(x)-dot.double(x)_d)+D_r (dot(x)-dot(x)_d)+K_r (x-x_d)=F_"ext"-F_d, $

其中 $x$ 同时包含末端位置和旋转向量，$F_"ext"$ 为 MuJoCo 接触对计算得到、转换到世界坐标系的工具受力/力矩，$F_d$ 为接触锁定后沿插入反方向的 7 N 虚拟保持力。与 §5.1 中人为指定对角阻尼不同（$[50,50,50,10,10,10]$），此处阻尼矩阵 $D_r$ 按 Ren & Shan (2026) @ren2026unified 的临界阻尼设计在线计算：

$ D_(r,i) = 2 sqrt(Lambda_(i i) dot K_(r,i)), $

该式源自 $D_r=sqrt(Lambda)sqrt(K_r)+sqrt(K_r)sqrt(Lambda)$ 在 $K_r$ 取对角形式时的退化，保证各自由度闭环响应均处于临界阻尼，免除了逐任务调参的需要。

与轨迹跟踪实验中各向同性的 PD 增益不同，对接任务的刚度采用#strong[各向异性设计]：在世界坐标系中，横向（$x$、$y$）参考刚度设为 200 N/m、最低 80 N/m，以维持工具与母端轴线的高精度对中；轴向（$z$，即插入方向）参考刚度保持 100 N/m、最低 30 N/m，允许足够的轴向柔顺以吸收接触冲击。转动三轴刚度保持 25 N·m/rad、最低 8 N·m/rad。该设计遵循"横向对中优先、轴向吸震优先"的对接原则——1 N 的横向接触力在最低横向刚度下仅产生约 12.5 mm 的偏移，而同样的力在轴向最低刚度下允许约 33 mm 的柔顺位移。

首次接触后，位置参考冻结在接触位姿，控制器仅以力反馈继续小幅压紧，避免原预插入轨迹对非凸网格造成额外冲击。控制器利用实时质量矩阵和雅可比构造操作空间惯性 $Lambda=(J M^(-1)J^T+lambda_o^2 I)^(-1)$，再以 $tau=J^T Lambda a_"cmd"+h+N^T tau_0$ 输出关节转矩；$tau_0$ 为回中零空间阻尼项。阻尼由上述 $D_r$ 公式在线确定。

== 比较框架

两种控制器使用完全相同的参考轨迹、初始状态、物理步长、电机参数、减速器参数和电压/电流限制。控制器输出期望关节力矩，实际力矩必须经过电流动态和减速器后作用于 MuJoCo 模型——该约束避免了"理想力矩源"假设掩盖执行器带宽的实际情况。以下实验将在这套统一框架下评估两种控制器的性能差异。

= 仿真实验与结果分析

== 实验设置

实验包含两种场景。标称场景运行 14 s，用于评价竖直圆、平面 8 字及同步姿态变化的基本跟踪精度。抗扰场景在圆周段的 4.5--5.5 s 对末端施加竖直向下的 15 N 恒定外力，其余条件与标称场景一致。

量化评价指标包括：（1）末端位置均方根误差（RMSE）与最大误差；（2）末端姿态 RMSE 与最大误差；（3）关节角 RMSE；（4）峰值电流与峰值电压；（5）执行器饱和时间比例；（6）撤力后恢复到 10 mm 误差以内所需时间。预设成功条件为：计算力矩控制在标称场景下的末端位置 RMSE 小于 5 mm，且在撤力后 0.5 s 内恢复至 10 mm 误差范围；同时通过惯性前馈 PD 与 CTC 的对照，分离前馈补偿与反馈解耦各自对跟踪精度的贡献。

== 标称轨迹跟踪

#figure(
  image("../figures/trajectory_3d.png", width: 100%),
  caption: [标称场景下圆轨迹与 8 字轨迹的分开对比（RGB 箭头表示参考末端 $x$--$y$--$z$ 姿态标架）],
)

图中左侧仅保留 1.5--6.5 s 的 $y$--$z$ 圆轨迹，右侧仅保留 8--13 s 的 $x$--$y$ 平面 8 字轨迹，不包含启动和段间过渡。分开绘制避免了两个平面轨迹在同一三维坐标系中互相遮挡。每条参考轨迹上等时间间隔绘制 8 个末端姿态标架，红、绿、蓝箭头依次表示工具坐标系的 $x$、$y$、$z$ 轴，用于直观展示轨迹运动过程中的姿态变化。

#figure(
  image("../figures/segment_tracking_error.png", width: 96%),
  caption: [圆轨迹与 8 字轨迹的位置、姿态误差分段对比],
)

#figure(
  table(
    columns: (2.2cm, 2.2cm, 2.6cm, 2.6cm, 2.8cm),
    stroke: 0.5pt,
    inset: 5pt,
    align: center,
    table.header([轨迹段], [控制器], [位置 RMSE/mm], [位置最大误差/mm], [姿态 RMSE/°]),
    [圆轨迹], [PD], [#f2(pd-nom.at("circle_ee_rmse_mm"))], [#f2(pd-nom.at("circle_ee_max_mm"))], [#f2(pd-nom.at("circle_orientation_rmse_deg"))],
    [圆轨迹], [计算力矩], [#f2(ctc-nom.at("circle_ee_rmse_mm"))], [#f2(ctc-nom.at("circle_ee_max_mm"))], [#f2(ctc-nom.at("circle_orientation_rmse_deg"))],
    [8 字轨迹], [PD], [#f2(pd-nom.at("figure8_ee_rmse_mm"))], [#f2(pd-nom.at("figure8_ee_max_mm"))], [#f2(pd-nom.at("figure8_orientation_rmse_deg"))],
    [8 字轨迹], [计算力矩], [#f2(ctc-nom.at("figure8_ee_rmse_mm"))], [#f2(ctc-nom.at("figure8_ee_max_mm"))], [#f2(ctc-nom.at("figure8_orientation_rmse_deg"))],
  ),
  caption: [标称场景分轨迹段量化指标],
)

在圆轨迹段，PD 和计算力矩控制的位置 RMSE 分别为 #f2(pd-nom.at("circle_ee_rmse_mm")) mm 和 #f2(ctc-nom.at("circle_ee_rmse_mm")) mm，计算力矩控制降低约 #reduction(pd-nom.at("circle_ee_rmse_mm"), ctc-nom.at("circle_ee_rmse_mm"))%。在 8 字轨迹段，两者的位置 RMSE 分别为 #f2(pd-nom.at("figure8_ee_rmse_mm")) mm 和 #f2(ctc-nom.at("figure8_ee_rmse_mm")) mm，降低约 #reduction(pd-nom.at("figure8_ee_rmse_mm"), ctc-nom.at("figure8_ee_rmse_mm"))%。8 字段包含更频繁的曲率变化，因而两种控制器的位置 RMSE 均略高于各自的圆轨迹结果。姿态指标则呈现不同结果：PD 在圆和 8 字段的姿态 RMSE 分别为 #f2(pd-nom.at("circle_orientation_rmse_deg"))° 和 #f2(pd-nom.at("figure8_orientation_rmse_deg"))°，均明显小于计算力矩控制的 #f2(ctc-nom.at("circle_orientation_rmse_deg"))° 和 #f2(ctc-nom.at("figure8_orientation_rmse_deg"))°。PD 对角增益对腕部关节的较低刚度使姿态跟踪更平滑，而计算力矩控制中高统一的 $K_p=324$ 在驱动较小惯量的腕部关节时引入了轻微超调。

标称场景中，PD 的末端 RMSE 为 #f2(pd-nom.at("ee_rmse_mm")) mm，最大误差为 #f2(pd-nom.at("ee_max_mm")) mm；计算力矩控制的末端 RMSE 为 #f2(ctc-nom.at("ee_rmse_mm")) mm，最大误差为 #f2(ctc-nom.at("ee_max_mm")) mm。计算力矩控制将 RMSE 降低约 #reduction(pd-nom.at("ee_rmse_mm"), ctc-nom.at("ee_rmse_mm"))%，最大误差降低约 #reduction(pd-nom.at("ee_max_mm"), ctc-nom.at("ee_max_mm"))%，两者均满足末端 RMSE 小于 5 mm 的预设目标。PD 在引入惯性前馈后已将跟踪精度提升至与 CTC 接近的水平，剩余差距主要来自反馈通道的对角增益结构和较低的等效刚度——CTC 通过 $M(q)$ 将高增益误差项映射为关节力矩，实现了更完整的惯性解耦。

PD 的关节 RMSE 为 #f2(pd-nom.at("joint_rmse_deg"))°，小于计算力矩控制的 #f2(ctc-nom.at("joint_rmse_deg"))°，这并不与末端精度结论矛盾：七自由度系统中，不同关节误差组合经雅可比映射后对末端位姿的贡献不同。PD 通过直接约束单关节偏差获得了较低的关节 RMSE，但反馈通道缺少惯性解耦，少量末端误差仍来自关节间动力学耦合；计算力矩控制以略高的关节偏差换取更协调的多轴运动。两种控制器的峰值电流均为 #f2(ctc-nom.at("peak_current_a")) A，饱和时间占比小于 0.01%，对完整实验影响可忽略。

#figure(
  image("../figures/joint_current.png", width: 92%),
  caption: [标称场景下计算力矩控制的关节响应与 PMSM q 轴电流],
)

== 外力抗扰性能

#figure(
  image("../figures/tracking_error.png", width: 94%),
  caption: [标称与 15 N 外力场景下的末端位置误差],
)

15 N 竖直向下外力作用期间，PD 的末端 RMSE 增至 #f2(pd-dis.at("ee_rmse_mm")) mm，最大误差为 #f2(pd-dis.at("ee_max_mm")) mm；计算力矩控制的 RMSE 为 #f2(ctc-dis.at("ee_rmse_mm")) mm，最大误差为 #f2(ctc-dis.at("ee_max_mm")) mm。相对 PD，计算力矩控制将抗扰场景 RMSE 降低约 #reduction(pd-dis.at("ee_rmse_mm"), ctc-dis.at("ee_rmse_mm"))%。

撤去外力后，PD 恢复到 10 mm 误差以内需要 #f2(pd-dis.at("recovery_s")) s，计算力矩控制仅需 #f2(ctc-dis.at("recovery_s")) s。惯性前馈项 $M(q)dot.double(q)_d$ 主要用于补偿参考加速度，对突发外力扰动无直接抑制作用；抗扰性能几乎完全依赖反馈增益的刚度水平。计算力矩控制等效的 $K_p=324$ 远高于 PD 的对角刚度（尤其是腕部关节，差距达 3--10 倍），其误差反馈位于模型补偿后的等效线性系统内、闭环带宽更为一致，因此抗扰和恢复显著更快。这一对比清晰地分离了前馈和反馈的不同角色：前馈决定跟踪精度，反馈增益决定抗扰能力。计算力矩控制不是严格意义上的鲁棒控制——若质量、摩擦或减速器效率存在显著建模误差，其优势可能减小；本实验验证的是在准确模型条件下的反馈抗扰能力。

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

标称场景下计算力矩控制的末端 RMSE 为 #f2(ctc-nom.at("ee_rmse_mm")) mm，带惯性前馈的 PD 为 #f2(pd-nom.at("ee_rmse_mm")) mm，两者均满足小于 5 mm 的预设指标；抗扰场景下 CTC 为 #f2(ctc-dis.at("ee_rmse_mm")) mm，撤力后 #f2(ctc-dis.at("recovery_s")) s 内恢复至 10 mm 以内。两种控制器的峰值电流均为 #f2(ctc-nom.at("peak_current_a")) A，瞬时饱和占比小于 0.01%，因此标称跟踪性能的剩余差异来自反馈通道的增益结构，而非执行器容量限制。抗扰性能的巨大差异则揭示了前馈与反馈的本质分工：$M(q)dot.double(q)_d$ 消除了跟踪滞后，但只有高反馈增益才能有效抑制突发外力。

= 柔顺对接扩展实验

== 场景与评价方法

在不改变上述组合轨迹实验的前提下，另建独立的 `scene_docking.xml`。末端和固定端均使用经原作者授权的对接件网格 @langxin2026docking；MuJoCo 以该 STL 直接生成原生 SDF 碰撞几何，从而保留非凸键槽轮廓及多点接触。该刚体接触模型用于评估柔顺接近、法向接触和保持力，不宣称复刻完整锁止机构的材料变形、公差和有限元细节。

对接总时长为 11 s：0--1 s 保持初始位姿，1--5 s 沿工具坐标系 z 轴靠近，5--7 s 从 55 mm 端面间距受控插入至 45 mm，7--11 s 保持。首次 SDF 接触出现后，位置参考锁定在该接触位姿，7 N 虚拟保持力在 0.4 s 内按五次时间缩放升起。阻抗控制器采用 §5.3 所述的 $Lambda$ 基临界阻尼和 sigmoid 自适应刚度（$k_alpha=0.5$，$K_min$ 平动 30 N/m、转动 8 N·m/rad），在自由空间保持高跟踪刚度、接触后自动柔化以降低冲击力。

参照 Ren & Shan (2026) @ren2026unified 的三层评价体系，指标按 mission 优先级组织：

#figure(
  table(
    columns: (1.5cm, 5.0cm, 6.0cm),
    stroke: 0.5pt,
    inset: 5pt,
    align: (center, left, left),
    table.header([层级], [指标], [判据]),
    [物理交互安全], [峰值轴向力 $|F_z|_"max"$、稳态轴向力 $|F_z|_"fin"$、稳态接触力均值], [越低越安全；保持段接触 >90% 视为完成],
    [内部系统安全], [峰值电流、峰值电压、饱和率], [不触发硬件限幅为通过],
    [任务跟踪性能], [最终位置/姿态偏差], [允许适度偏移以换取柔顺交互],
  ),
  caption: [柔顺对接三层评价指标],
)

评价侧重点与 §6.1 不同：对接不以毫米级跟踪精度为目标——允许的柔顺偏差正是吸收接触冲击、保护工件的手段。

== 接触结果与讨论

#figure(
  image("../figures/docking_contact_response.png", width: 92%),
  caption: [刚性 CTC 与自适应阻抗控制的接触力和末端偏差；浅绿色区域为保持段],
)

#figure(
  image("../figures/docking_metrics.png", width: 92%),
  caption: [柔顺对接的接触力与最终位置偏差量化对比],
)

#figure(
  table(
    columns: (1.4cm, 1.7cm, 1.7cm, 1.7cm, 1.7cm, 1.7cm, 1.7cm),
    stroke: 0.5pt,
    inset: 4pt,
    align: center,
    table.header([控制器], [接触/s], [峰值力/N], [保持力/N], [横向/mm], [轴向/mm], [姿态/°]),
    [刚性 CTC], [#f2(docking-ctc.at("contact_start_s"))], [#f2(docking-ctc.at("peak_contact_force_n"))], [#f2(docking-ctc.at("steady_contact_force_n"))], [#f2(docking-ctc.at("lateral_error_final_mm"))], [#f2(docking-ctc.at("axial_error_final_mm"))], [#f2(docking-ctc.at("alignment_angle_deg"))],
    [阻抗控制], [#f2(docking-impedance.at("contact_start_s"))], [#f2(docking-impedance.at("peak_contact_force_n"))], [#f2(docking-impedance.at("steady_contact_force_n"))], [#f2(docking-impedance.at("lateral_error_final_mm"))], [#f2(docking-impedance.at("axial_error_final_mm"))], [#f2(docking-impedance.at("alignment_angle_deg"))],
  ),
  caption: [柔顺对接量化指标（横向 = 垂直于插入轴的偏差，轴向 = 沿插入方向）],
)

#figure(
  table(
    columns: (1.6cm, 2.2cm, 2.4cm, 2.6cm, 2.4cm),
    stroke: 0.5pt,
    inset: 5pt,
    align: center,
    table.header([控制器], [插入深度/mm], [保持接触/%], [保持段接触点数], [峰值电流/A]),
    [刚性 CTC], [#f2(docking-ctc.at("insertion_depth_mm"))], [#f2(docking-ctc.at("hold_contact_percent"))], [#f2(docking-ctc.at("avg_contact_points_hold"))], [#f2(docking-ctc.at("peak_current_a"))],
    [阻抗控制], [#f2(docking-impedance.at("insertion_depth_mm"))], [#f2(docking-impedance.at("hold_contact_percent"))], [#f2(docking-impedance.at("avg_contact_points_hold"))], [#f2(docking-impedance.at("peak_current_a"))],
  ),
  caption: [柔顺对接补充指标],
)

两种控制器均通过配置的完成判据。刚性 CTC 的接触始于 #f2(docking-ctc.at("contact_start_s")) s，峰值接触力为 #f2(docking-ctc.at("peak_contact_force_n")) N，保持段均值为 #f2(docking-ctc.at("steady_contact_force_n")) N——其闭环仅提供 $Lambda^(-1)F_"ext"$ 的纯惯性响应，接触力主要由非凸网格几何和物理参数决定，横向偏差 #f2(docking-ctc.at("lateral_error_final_mm")) mm 表明刚性推入时工具也出现了明显侧移。

阻抗控制在 #f2(docking-impedance.at("contact_start_s")) s 首次接触，得益于#strong[各向异性刚度]（横向 200→80 N/m 对中、轴向 100→30 N/m 吸震），横向偏差仅 #f2(docking-impedance.at("lateral_error_final_mm")) mm——比旧版等向刚度方案（11.1 mm）降低约 54%，也比刚性 CTC（#f2(docking-ctc.at("lateral_error_final_mm")) mm）更小。峰值和保持段接触力分别为 #f2(docking-impedance.at("peak_contact_force_n")) N 与 #f2(docking-impedance.at("steady_contact_force_n")) N，相对 CTC 分别降低约 #reduction(docking-ctc.at("peak_contact_force_n"), docking-impedance.at("peak_contact_force_n"))% 和 #reduction(docking-ctc.at("steady_contact_force_n"), docking-impedance.at("steady_contact_force_n"))%。轴向偏差 #f2(docking-impedance.at("axial_error_final_mm")) mm 对应 #f2(docking-impedance.at("insertion_depth_mm")) mm 的插入深度——工具在自适应柔化后受控滑入母端凹槽，远超 CTC 的 #f2(docking-ctc.at("insertion_depth_mm")) mm。保持段平均接触点数为 #f2(docking-impedance.at("avg_contact_points_hold"))，低于 CTC 的 #f2(docking-ctc.at("avg_contact_points_hold"))，说明柔顺接触的力分布更集中在少量接触点——非凸 SDF 网格的多点接触特性需要更高的接触刚度才能充分激发。

三层指标中，阻抗控制在"物理交互安全"层以极低的峰值力和保持力全面占优，且在"任务跟踪性能"层以更小的横向偏差体现了各向异性对中的有效性；CTC 靠刚性贴合取得更深插入和更多接触点，但代价是 3--5 倍的接触力。两者在"内部系统安全"层均无饱和。该对比表明：#strong[对接质量不能仅以总位置偏差排序]——横向偏差衡量对中精度，插入深度衡量配合程度，接触力衡量安全性，三者构成不可互相替代的多目标评价空间。

= 结论与展望

本报告完成了一个七自由度 FR3 机械臂的高阶运动控制仿真。系统沿"期望轨迹 → 控制器 → 执行器链 → 多刚体动力学 → 状态反馈"的闭环信号路径逐层建模，包含 PMSM q 轴电感、电阻、反电动势、FOC 电流 PI、电压/电流限制、转子惯量、减速器效率和 FR3 官方关节转矩限制，并非简化为理想关节力矩源。通过冗余逆运动学生成连续参考轨迹，在相同执行器约束下比较了带惯性前馈的偏置力补偿 PD 与计算力矩控制，并严格区分了 FR3 官方整机限制、Menagerie 刚体数据和代表性 PMSM 工程假设。

实验表明，在标称组合位姿轨迹下，带惯性前馈的 PD 取得 #f2(pd-nom.at("ee_rmse_mm")) mm 末端位置 RMSE，计算力矩控制取得 #f2(ctc-nom.at("ee_rmse_mm")) mm，仅进一步降低约 #reduction(pd-nom.at("ee_rmse_mm"), ctc-nom.at("ee_rmse_mm"))%。两者在跟踪精度上已十分接近，剩余差距来自 CTC 通过 $M(q)$ 实现的反馈通道惯性解耦和更高的等效刚度。抗扰场景则呈现截然不同的结果：在 15 N 外力下 CTC 取得 #f2(ctc-dis.at("ee_rmse_mm")) mm RMSE，比 PD 降低约 #reduction(pd-dis.at("ee_rmse_mm"), ctc-dis.at("ee_rmse_mm"))%，撤力后恢复时间约 #f2(ctc-dis.at("recovery_s")) s。这一对比清晰地分离了前馈与反馈在运动控制中的不同角色：$M(q)dot.double(q)_d$ 前馈项几乎完全消除了动态跟踪滞后，使 PD 从原来 3.62 mm 的 RMSE 大幅降至接近 CTC 的水平；但外力扰动抑制几乎完全依赖反馈增益——CTC 等效的 $K_p=324$ 远高于 PD 的对角刚度，因而抗扰和恢复性能显著更优。

独立柔顺对接实验进一步显示，操作空间阻抗控制以可接受的末端让位换取更低的峰值和保持接触力；这为接触敏感任务提供了与刚性轨迹跟踪不同的控制取舍。

后续工作可沿以下方向推进：引入连杆质量和负载的不确定性，考察鲁棒或自适应计算力矩控制的效果；在零空间中纳入可操作度最大化与碰撞约束；将离线逆运动学替换为在线优化控制，在统一框架中同时处理轨迹生成、关节限位、电流约束和障碍物规避。

= 附录 A：复现说明

项目使用 Python 3.12 和 `uv` 管理依赖，主要命令如下：

- `uv run fr3-control run-experiments`：生成四组实验的 NPZ、CSV 和 JSON 数据；
- `uv run fr3-control plot-results`：生成报告插图；
- `MUJOCO_GL=egl uv run fr3-control render-video`：渲染演示视频（需要 EGL 支持）；
- `uv run fr3-control run-docking-experiments`：运行刚性 CTC 与阻抗控制的柔顺对接实验；
- `uv run fr3-control plot-docking-results`：生成对接接触力与偏差图；
- `MUJOCO_GL=egl uv run fr3-control render-docking-video`：渲染柔顺对接对比视频；
- `uv run pytest`：验证电机单位换算、限幅、质量矩阵、轨迹连续性和完整闭环仿真。

所有实验使用固定参数和确定性初始状态。报告中的数值直接从 `results/summary.json` 读取，重新运行实验并编译 Typst 后可自动更新。

= 附录 B：课程知识点对照

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

#{
  set text(lang: "en", region: "US")
  bibliography("references.bib", title: [参考文献], style: "ieee")
}
