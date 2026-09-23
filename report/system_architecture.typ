#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge

#set page(width: 1600pt, height: 590pt, margin: 0pt)
#set text(
  font: "Noto Sans CJK SC",
  size: 16pt,
  fill: rgb("394b5d"),
)

#let ink = rgb("26384a")
#let muted = rgb("52687d")
#let disturbance = rgb("b34c4c")

#let architecture-node(accent, title, line1, line2) = block(
  width: 250pt,
  height: 160pt,
  fill: white,
  stroke: (paint: ink, thickness: 1.2pt),
  radius: 3pt,
  inset: 0pt,
)[
  #block(width: 100%, height: 7pt, fill: accent)[]
  #v(16pt)

  #align(center)[
    #text(
      size: 25pt,
      weight: "bold",
      fill: rgb("182635"),
    )[#title]

    #v(10pt)

    #text(size: 18pt)[#line1]
    #linebreak()
    #text(size: 18pt)[#line2]
  ]
]

#block(
  width: 1545pt,
  height: 560pt,
  fill: rgb("fbfcfd"),
  stroke: (paint: rgb("9aabb9"), thickness: 0.8pt),
  radius: 5pt,
  inset: 26pt,
)[
  #text(
    size: 18pt,
    weight: "bold",
    fill: muted,
  )[闭环控制信号链]

  #v(7pt)

  #line(
    length: 100%,
    stroke: (
      paint: rgb("c4d0da"),
      thickness: 0.7pt,
    ),
  )

  #v(28pt)

  #align(center)[
    #diagram(
      spacing: 43pt,
      edge-stroke: (
        paint: ink,
        thickness: 1.2pt,
      ),
      edge-corner-radius: 6pt,
      mark-scale: 75%,

      // 主节点
      node(
        (0, 0),
        architecture-node(
          rgb("3d6f9e"),
          [轨迹与逆运动学],
          [末端组合位姿轨迹],
          [七自由度冗余求解],
        ),
        name: <trajectory>,
        stroke: none,
        inset: 0pt,
      ),

      node(
        (1, 0),
        architecture-node(
          rgb("3f8b6a"),
          [运动控制器],
          [关节空间逆动力学],
          [笛卡尔空间阻抗],
        ),
        name: <controller>,
        stroke: none,
        inset: 0pt,
      ),

      node(
        (2, 0),
        architecture-node(
          rgb("b87832"),
          [驱动与传动链],
          [FOC 电流 PI · PMSM],
          [减速器、电压/电流限幅],
        ),
        name: <actuator>,
        stroke: none,
        inset: 0pt,
      ),

      node(
        (3, 0),
        architecture-node(
          rgb("705a9c"),
          [被控对象],
          [MuJoCo FR3 七自由度],
          [多刚体动力学与接触],
        ),
        name: <plant>,
        stroke: none,
        inset: 0pt,
      ),

      node(
        (4, 0),
        architecture-node(
          rgb("63798b"),
          [状态反馈与评价],
          [关节/末端状态估计],
          [误差、能量、接触力],
        ),
        name: <feedback>,
        stroke: none,
        inset: 0pt,
      ),

      // 前向控制通道
      edge(
        <trajectory.east>,
        <controller.west>,
        $q^d, dot(q)^d, dot.double(q)^d$,
        "-|>",
        label-side: center,
      ),

      edge(
        <controller.east>,
        <actuator.west>,
        $tau^d$,
        "-|>",
        label-side: center,
      ),

      edge(
        <actuator.east>,
        <plant.west>,
        $tau$,
        "-|>",
        label-side: center,
      ),

      edge(
        <plant.east>,
        <feedback.west>,
        $z$,
        "-|>",
        label-side: center,
      ),

      // 状态反馈路径：
      // 从最右侧节点底部下降，向左折返，再进入控制器
      edge(
        <feedback.south>,
        "d,lll,u",
        "-|>",
        [
          $q, dot(q), x, dot(x)$；
          接触力与跟踪误差
        ],
        stroke: (
          paint: muted,
          thickness: 1.1pt,
          dash: "dashed",
        ),
        label-side: center,
      ),

      // 外部扰动
      edge(
        (3, -1),
        <plant.north>,
        $F_"ext"$,
        "-|>",
        stroke: (
          paint: disturbance,
          thickness: 1.2pt,
        ),
        label-side: left,
      ),
    )
  ]

  #v(20pt)

  #text(size: 16pt, fill: muted)[
    实线：前向控制通道　　虚线：测量反馈通道　　
    #text(fill: disturbance)[红色：外部扰动输入]
  ]
]