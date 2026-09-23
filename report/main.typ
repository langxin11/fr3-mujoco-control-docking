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
#import "@preview/mitex:0.2.7": mitex

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

研究对象为七自由度 Franka Research 3（FR3）机械臂。末端依次跟踪半径 0.10 m、频率 0.2 Hz 的 $y$--$z$ 圆和 $x$--$y$ 平面 8 字轨迹，并同步改变姿态。控制器采用计算力矩控制（Computed Torque Control, CTC），以带惯性前馈的偏置力补偿 PD 为基线，在标称及 15 N 外力场景下比较跟踪性能。

仿真基于 MuJoCo Menagerie `franka_fr3_v2` @menagerie2022 和 MuJoCo 多刚体动力学 @todorov2012mujoco，保留连杆惯性、几何、关节限位及 FR3 公开的关节力矩边界 @franka2024fr3 @frankafci。系统进一步包含 PMSM/FOC 等效模型、减速器、电流与电压限幅，不采用理想力矩源。主要工作为：冗余逆运动学轨迹生成；机械臂—电机—减速器统一建模；偏置补偿 PD 与 CTC 对比；关节空间逆动力学与笛卡尔阻抗控制的柔顺对接实验。

= 系统建模与轨迹规划

== 总体架构与多速率时序

#figure(
  image("../figures/system_architecture.svg", width: 100%),
  caption: [机械臂运动控制系统总体结构],
)

闭环信号链为 $(q_d,dot(q)_d,dot.double(q)_d) -> tau_d -> tau -> (q,dot(q))$。外层运动控制周期为 1 ms；MuJoCo 物理步长与 FOC q 轴电流环周期均为 0.05 ms。

== 冗余运动学与轨迹生成

设七个关节角构成向量 $q in RR^7$，末端位姿写作齐次变换 $T_0^e(q)$。正运动学由各关节刚体变换依次相乘得到。末端线速度和角速度由几何雅可比矩阵给出：

$ mat(v_e, omega_e) = J(q) dot(q), quad J(q) in RR^(6 times 7). $

其中 $v_e$、$omega_e$ 分别为末端线速度和角速度，$J(q)$ 将关节速度映射为六维末端速度。由于机械臂有 7 个关节而任务空间只有 6 维，系统保留一个可用于姿态优化的冗余自由度。

采用阻尼伪逆与零空间回中项处理 $6 times 7$ 雅可比矩阵：

$ J^"+"=J^T(JJ^T+lambda^2 I)^(-1), quad
  Delta q=J^"+"e+(I-J^"+"J)k_n(q_h-q). $

$J^+e$ 将末端位姿误差转换为关节修正量；投影矩阵 $I-J^+J$ 将回中运动限制在雅可比零空间内，因而不改变末端任务。阻尼 $lambda$ 用于限制奇异位形附近伪逆解的放大。

参数为 $lambda=2.5 times 10^(-3)$、$k_n=0.025$，单步增量限制为 0.08 rad，关节限位裕量为 0.025 rad。任务时序为“过渡—圆—过渡—8 字—保持”，对应区间 $[0,1.5]$、$[1.5,6.5]$、$[6.5,8]$、$[8,13]$、$[13,14]$ s。过渡函数

$ s(u)=10u^3-15u^4+6u^5, quad u in [0,1] $

保证端点速度、加速度为零。逆运动学以 10 ms 离线求解，再用三次样条生成 1 ms 间隔的 $(q_d,dot(q)_d,dot.double(q)_d)$；正运动学复核的位置和姿态误差分别小于 3 mm 和 0.5°。

== 机械臂—执行器统一模型

机械臂关节空间动力学描述关节力矩与关节运动之间的关系 @featherstone2008：

$ M(q) dot.double(q) + C(q,dot(q))dot(q) + g(q) = tau + J_v(q)^T F_"ext". $

$M(q)$ 表示关节空间惯性，$C(q,dot(q))dot(q)$ 为科氏/离心力，$g(q)$ 为重力，$J_v^T F_"ext"$ 将末端外力映射为关节力矩。$M$ 对称正定，保证给定合力矩时关节加速度唯一。

MuJoCo 计算 $M$ 及偏置力 $h=C dot(q)+g$；测试验证 $M$ 的对称正定性。力矩限幅为

$ |tau|_"max"=[87,87,87,87,12,12,12] "N·m". $

在理想 FOC 且 $i_d=0$ 条件下，PMSM、电机侧速度和减速器输出满足

$ L_q dot(i_q)+R_s i_q+K_e omega_m=u_q, quad
  tau_m=K_t i_q, quad
  omega_m=N dot(q), quad
  tau=eta N K_t i_q. $

第一式表示 q 轴电压依次用于建立电流、克服电阻压降和反电动势；$i_q$ 经转矩常数 $K_t$ 产生电机力矩。减速比 $N$ 放大输出力矩，效率 $eta$ 描述传动损耗，由此形成 $u_q -> i_q -> tau_m -> tau$ 的执行器链。

电气参数取自 48 V Maxon EC-i 52 代表性电机 @maxoneci52：

#mitex(`
  \left\{
  \begin{aligned}
    R_s &= 0.284\,\Omega,\quad
    L_q = 0.45\,\mathrm{mH},\quad
    K_t = 0.0919\,\mathrm{N}\cdot\mathrm{m/A},\\
    K_e &= 0.0918\,\mathrm{V}\cdot\mathrm{s/rad},\quad
    J_m = 1.70\times10^{-5}\,\mathrm{kg}\cdot\mathrm{m^2}.
  \end{aligned}
  \right.
`)

工程等效减速比 $N=[100,100,100,100,50,50,50]$，效率 $eta=0.9$。FR3 未公开上述内部电气参数与真实减速比，故其仅用于可复现的执行器动态研究。电流 PI 取

$ omega_c=2000 "rad/s", quad K_"pi"=L_q omega_c, quad K_"ii"=R_s omega_c, $

并加入反电动势前馈、48 V 电压限幅和反算抗积分饱和。

= 控制器设计

== 重力/偏置补偿 PD 与惯性前馈

令 $e=q_d-q$、$dot(e)=dot(q)_d-dot(q)$。重力补偿、偏置补偿及本项目基线控制律依次为

$ tau_"PD+g"=K_p(q_d-q)+K_d(dot(q)_d-dot(q))+g(q). $

$ tau_"PD+h"=K_p(q_d-q)+K_d(dot(q)_d-dot(q))+h(q,dot(q)). $

$ tau_"base"=M(q)dot.double(q)_d+K_p e+K_d dot(e)+h(q,dot(q)). $

$tau_"PD+g"$ 只消除准确模型下的静态重力误差，$tau_"PD+h"$ 进一步补偿速度相关的科氏/离心项；实验基线再用 $M dot.double(q)_d$ 提前提供参考加速度所需力矩。未知外力仍需由 $K_p e+K_d dot(e)$ 抵消，因此反馈增益直接影响抗扰性能。增益为

$ K_p="diag"(160,180,140,110,70,45,30), quad
  K_d="diag"(25,28,22,18,12,8,6). $

== 计算力矩控制

计算力矩控制是机器人关节空间的逆动力学/反馈线性化控制 @siciliano2009：

$ tau_"CTC"=M(q)[dot.double(q)_d+K_d dot(e)+K_p e]+h(q,dot(q)). $

方括号内为由参考加速度和误差反馈组成的虚拟关节加速度；$M(q)$ 将其转换为关节力矩，$h$ 抵消名义非线性偏置项。

模型准确、无外力且执行器不饱和，则

$ M dot.double(q)+h=tau_"CTC" quad => quad
  dot.double(e)+K_d dot(e)+K_p e=0. $

该式表明原非线性耦合系统被化为由 $K_p$、$K_d$ 决定的二阶误差系统；其解耦结论依赖动力学模型准确且力矩未饱和。

取 $K_p=omega_n^2 I$、$K_d=2zeta omega_n I$；由 $omega_n in {12,15,18,22,28}$ 网格搜索确定 $omega_n=18 "rad/s"$、$zeta=1$，即 $K_p=324 I$、$K_d=36 I$。

#pagebreak(weak: true)
== 笛卡尔空间阻抗控制

CTC 在接触下的任务空间响应近似为

$ dot.double(x)=dot.double(x)_c+Lambda^(-1)F_"ext", quad
  Lambda=(J M^(-1)J^T)^(-1), $

$Lambda$ 是机械臂在末端六维空间中呈现的等效惯性。CTC 对接触力只产生 $Lambda^(-1)F_"ext"$ 的惯性响应，缺少可独立调节的末端刚度和阻尼，因而难以限制接触冲击。柔顺对接采用六维阻抗关系 @hogan1985impedance：

$ M_d(dot.double(x)-dot.double(x)_d)+D_r(dot(x)-dot(x)_d)
  +K_r(x-x_d)=F_"ext"-F_d. $

$M_d$、$D_r$、$K_r$ 分别规定期望末端惯性、阻尼和刚度，$F_d$ 为目标保持力。该关系允许末端在外力下产生受控位移，以牺牲少量跟踪精度换取柔顺接触。

操作空间惯性、接触力软化因子、有效刚度和临界阻尼为 @ren2026unified

$ Lambda=(J M^(-1)J^T+lambda_o^2 I)^(-1), quad
  alpha=2 sigma(k_alpha norm(F_"ext"))-1, $

$ K_r="clip"((1-alpha)K_"ref",K_"min",K_"ref"), quad
  D_(r,i)=2sqrt(Lambda_(i i)K_(r,i)). $

$ K_"ref"="diag"(800,800,400,80,80,80), quad
  K_"min"="diag"(80,80,40,10,10,10). $

$alpha$ 随接触力幅值增大，使 $K_r$ 从 $K_"ref"$ 降至 $K_"min"$；$D_(r,i)$ 按当前等效惯性和刚度取临界阻尼，抑制接触振荡。横向刚度高于插入方向刚度，以同时维持轴线对中并吸收轴向冲击。

$ M_d="diag"(10,10,10,1,1,1), quad k_alpha=0.5. $

#mitex(`
  \begin{aligned}
    P_N &= I-M^{-1}J^\top\Lambda J,\\
    \tau &= J^\top\Lambda a_{\mathrm{cmd}}+h+P_N^\top\tau_0.
  \end{aligned}
`)

其中 $J^T Lambda a_"cmd"$ 产生任务空间控制力矩，$P_N$ 为动态一致零空间投影矩阵，$P_N^T tau_0$ 在不破坏末端任务的零空间内保持关节姿态。

= 仿真实验与结果分析

== 实验设置

标称场景运行 14 s；抗扰场景在 $t in [4.5,5.5]$ s 施加沿重力方向的 15 N 末端外力。评价量为位置/姿态 RMSE 与最大误差、关节 RMSE、峰值电流/电压、饱和率及撤力后恢复至 10 mm 的时间。成功条件为标称位置 RMSE $<5$ mm、撤力后 0.5 s 内恢复。

== 轨迹跟踪与抗扰结果

#figure(
  image("../figures/trajectory_3d.png", width: 92%),
  caption: [圆轨迹与 8 字轨迹跟踪；RGB 箭头表示参考末端姿态],
)

#figure(
  image("../figures/segment_tracking_error.png", width: 90%),
  caption: [圆轨迹与 8 字轨迹的位置、姿态误差分段对比],
)

圆轨迹位置 RMSE 由 PD 的 #f2(pd-nom.at("circle_ee_rmse_mm")) mm 降至 CTC 的 #f2(ctc-nom.at("circle_ee_rmse_mm")) mm；8 字轨迹由 #f2(pd-nom.at("figure8_ee_rmse_mm")) mm 降至 #f2(ctc-nom.at("figure8_ee_rmse_mm")) mm。CTC 的统一高带宽提高位置精度，但腕部轻微超调使其姿态 RMSE 高于 PD。

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

#figure(
  image("../figures/joint_current.png", width: 88%),
  caption: [标称场景下计算力矩控制的关节响应与 PMSM q 轴电流],
)

#figure(
  grid(
    columns: (1.32fr, 1fr),
    gutter: 8pt,
    image("../figures/tracking_error.png", width: 100%),
    image("../figures/metrics_comparison.png", width: 100%),
  ),
  caption: [标称与 15 N 外力场景的末端误差及 RMSE 汇总],
)

15 N 外力下，CTC 将 RMSE 从 #f2(pd-dis.at("ee_rmse_mm")) mm 降至 #f2(ctc-dis.at("ee_rmse_mm")) mm，并在撤力后 #f2(ctc-dis.at("recovery_s")) s 恢复至 10 mm 内。两种控制器峰值电流相同且饱和率小于 0.01%，故差异来自 CTC 的反馈惯性解耦与更高等效刚度，而非执行器容量。该实验评价固定外力下的反馈抗扰能力，不构成参数不确定性意义下的鲁棒性证明。

= 柔顺对接扩展实验

== 场景与评价方法

独立场景 `scene_docking.xml` 使用经授权的非凸对接件网格 @langxin2026docking，并以 MuJoCo SDF 保留键槽轮廓和多点接触。公、母端安装偏航均为 40°，母端再绕插入轴错开 45°。11 s 任务依次为保持（0--1 s）、接近（1--5 s）、插入（5--7 s）和接触保持（7--11 s）；参考端面间距由 55 mm 减至 45 mm。首次接触后冻结位姿参考，并在 1.5 s 内升起 5 N 保持力。

评价向量为

$ cal(M)={F_"peak",F_"hold",d_"ins",e_"lat",e_R,p_"contact",I_"peak"}, $

分别表示峰值/保持接触力、插入深度、横向/姿态偏差、保持接触率和峰值电流；$p_"contact">90%$ 判定任务完成。对接允许有限位姿让步，以接触安全和插入完成度为主要目标。

== 接触结果与讨论

#figure(
  image("../figures/docking_contact_response.png", width: 88%),
  caption: [逆动力学跟踪与笛卡尔阻抗控制的接触响应],
)

#figure(
  grid(
    columns: (0.72fr, 1.28fr),
    gutter: 10pt,
    image("../figures/docking_interface_alignment.png", width: 100%),
    image("../figures/docking_metrics.png", width: 100%),
  ),
  caption: [对接接口相对位姿与接触性能量化比较],
)

#figure(
  table(
    columns: (2.2cm, 2.1cm, 1.6cm, 1.6cm, 1.6cm, 1.8cm, 1.7cm),
    stroke: 0.5pt,
    inset: 4pt,
    align: center,
    table.header([控制器], [峰值/保持力 N], [插入/mm], [横向/mm], [姿态/°], [保持接触/%], [峰值电流/A]),
    [逆动力学], [#f2(docking-ctc.at("peak_contact_force_n")) / #f2(docking-ctc.at("steady_contact_force_n"))], [#f2(docking-ctc.at("insertion_depth_mm"))], [#f2(docking-ctc.at("lateral_error_final_mm"))], [#f2(docking-ctc.at("alignment_angle_deg"))], [#f2(docking-ctc.at("hold_contact_percent"))], [#f2(docking-ctc.at("peak_current_a"))],
    [笛卡尔阻抗], [#f2(docking-impedance.at("peak_contact_force_n")) / #f2(docking-impedance.at("steady_contact_force_n"))], [#f2(docking-impedance.at("insertion_depth_mm"))], [#f2(docking-impedance.at("lateral_error_final_mm"))], [#f2(docking-impedance.at("alignment_angle_deg"))], [#f2(docking-impedance.at("hold_contact_percent"))], [#f2(docking-impedance.at("peak_current_a"))],
  ),
  caption: [柔顺对接量化指标],
)

两种控制器均满足完成判据。阻抗控制将峰值和保持力分别降低 #reduction(docking-ctc.at("peak_contact_force_n"), docking-impedance.at("peak_contact_force_n"))% 和 #reduction(docking-ctc.at("steady_contact_force_n"), docking-impedance.at("steady_contact_force_n"))%，插入深度由 #f2(docking-ctc.at("insertion_depth_mm")) mm 增至 #f2(docking-impedance.at("insertion_depth_mm")) mm；代价是横向偏差增至 #f2(docking-impedance.at("lateral_error_final_mm")) mm。两者均未触发执行器饱和，说明阻抗控制以有限让位换取了更低接触力和更深插入。

= 结论与展望

本文建立了 FR3 多刚体—PMSM/FOC—减速器闭环模型，并完成冗余轨迹规划、偏置补偿 PD 与 CTC 对比。标称位置 RMSE 分别为 #f2(pd-nom.at("ee_rmse_mm")) mm 和 #f2(ctc-nom.at("ee_rmse_mm")) mm；15 N 外力下 CTC 为 #f2(ctc-dis.at("ee_rmse_mm")) mm，较 PD 降低 #reduction(pd-dis.at("ee_rmse_mm"), ctc-dis.at("ee_rmse_mm"))%，撤力恢复时间为 #f2(ctc-dis.at("recovery_s")) s。结果表明惯性前馈决定标称跟踪滞后，质量矩阵反馈解耦及增益决定外力抑制能力。

柔顺对接中，笛卡尔阻抗控制以有限横向让步换取更低接触力和更深插入。后续可在相同不确定性集合下研究负载自适应或鲁棒补偿，并将碰撞约束纳入在线冗余优化。

= 附录

== 复现说明

项目使用 Python 3.12 与 `uv`；主要命令为：

- `uv run fr3-control run-experiments` / `plot-results`：轨迹实验与绘图；
- `uv run fr3-control run-docking-experiments` / `plot-docking-results`：对接实验与绘图；
- `MUJOCO_GL=egl uv run fr3-control render-video` / `render-docking-video`：视频渲染；
- `uv run pytest`：模型、轨迹、执行器和闭环验证。

实验采用确定性初始状态；报告数值由 `results/summary.json` 与 `results/docking_summary.json` 自动读取。

== 课程知识点对照

#figure(
  table(
    columns: (1.2cm, 3cm, 10.2cm),
    stroke: 0.5pt,
    inset: 5pt,
    align: (center, center, left),
    table.header([序号], [课程内容], [本项目中的对应知识点]),
    [1], [陈老师], [轨迹规划、正逆运动学、多刚体动力学、逆动力学控制、跟踪误差分析],
    [2], [李老师], [由直流电机基本电磁关系推广至 PMSM 的 q 轴等效模型、反电动势、电机惯量、减速器、电流环和多速率离散控制],
    [3], [余老师], [重力补偿 PD、偏置力补偿、逆动力学/计算力矩控制及反馈线性化；本项目通过带惯性前馈的偏置补偿 PD 与计算力矩控制的对比，分析模型补偿、反馈解耦和外力抗扰性能。],
  ),
  caption: [三位老师知识点与项目内容的对应关系],
)

#{
  set text(lang: "en", region: "US")
  bibliography("references.bib", title: [参考文献], style: "ieee")
}
