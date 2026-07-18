# 第三方资产说明

## 对接件网格

`assets/docking/dock_1_17_new.STL` 经作者（用户师兄）授权，复用于本课程项目的 FR3 柔顺对接仿真。
原始实现参考：<https://github.com/langxin11/compliant_docking_simulation>，分支
`fix/simulation-stability-improvements`。本仓库将该网格用于对接件的可视化与 MuJoCo 原生
SDF 碰撞建模。物理接触直接由该 STL 网格生成的 SDF 几何计算，以保留其非凸对接轮廓；
控制器、FR3 场景、实验与报告内容在本项目中独立实现。
