# 第三方资产说明

## 对接件网格

`assets/docking/dock_1_17_new.STL` 经作者（用户师兄）授权，复用于本课程项目的 FR3 柔顺对接仿真。
原始实现参考：<https://github.com/langxin11/compliant_docking_simulation>，分支
`fix/simulation-stability-improvements`。本仓库仅将该网格用于对接件的可视化与 MuJoCo
SDF 可视化建模。为获得稳定、可重复的法向接触力，物理接触使用了与对接端面尺寸
匹配的 MuJoCo 圆柱代理；控制器、FR3 场景、实验与报告内容在本项目中独立实现。
