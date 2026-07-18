// 机械臂控制系统架构图的可复现 Typst 源。
// 编译：typst compile --root . report/system_architecture.typ figures/system_architecture.svg

#set page(width: 1600pt, height: 590pt, margin: 0pt)
#set text(font: "Noto Sans CJK SC", size: 16pt, fill: rgb("394b5d"))

#let ink = rgb("26384a")
#let muted = rgb("52687d")

#let node(accent, title, line1, line2) = block(
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
    #text(size: 25pt, weight: "bold", fill: rgb("182635"))[#title]
    #v(10pt)
    #text(size: 18pt)[#line1]
    #linebreak()
    #text(size: 18pt)[#line2]
  ]
]

#let arrow(signal) = align(center)[
  #text(size: 16pt, fill: muted)[#signal]
  #v(2pt)
  #text(size: 32pt, fill: ink)[→]
]

#block(
  width: 1545pt,
  height: 560pt,
  fill: rgb("fbfcfd"),
  stroke: (paint: rgb("9aabb9"), thickness: 0.8pt),
  radius: 5pt,
  inset: 26pt,
)[
  #text(size: 18pt, weight: "bold", fill: muted)[闭环控制信号链]
  #v(7pt)
  #line(length: 100%, stroke: (paint: rgb("c4d0da"), thickness: 0.7pt))
  #v(42pt)
  #align(center)[
    #grid(
      columns: (250pt, 43pt, 250pt, 43pt, 250pt, 43pt, 250pt, 43pt, 250pt),
      column-gutter: 5pt,
      align: center + top,
      node(rgb("3d6f9e"), [轨迹与逆运动学], [末端组合位姿轨迹], [七自由度冗余求解]),
      arrow([$q^d, dot(q)^d, dot.double(q)^d$]),
      node(rgb("3f8b6a"), [运动控制器], [关节空间逆动力学], [笛卡尔空间阻抗]),
      arrow([$tau^d$]),
      node(rgb("b87832"), [驱动与传动链], [FOC 电流 PI · PMSM], [减速器、电压/电流限幅]),
      arrow([$tau$]),
      node(rgb("705a9c"), [被控对象], [MuJoCo FR3 七自由度], [多刚体动力学与接触]),
      arrow([$z$]),
      node(rgb("63798b"), [状态反馈与评价], [关节/末端状态估计], [误差、能量、接触力]),
    )
  ]
  #v(20pt)
  #align(right)[#text(size: 18pt, fill: rgb("b34c4c"))[↑ $F_"ext"$]]
  #v(27pt)
  #align(center)[
    #text(size: 19pt, fill: muted)[⟵ ┄ ┄ ┄　$ q, dot(q), x, dot(x)$；接触力与跟踪误差　┄ ┄ ┄]
  ]
  #v(26pt)
  #align(left)[
    #text(size: 16pt, fill: muted)[实线：前向控制通道　　虚线：测量反馈通道　　红色：外部扰动输入]
  ]
]
