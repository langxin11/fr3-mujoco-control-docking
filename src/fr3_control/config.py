"""项目路径、电机参数与仿真参数的集中配置。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = ROOT / "assets" / "franka_fr3_v2" / "scene.xml"
DOCKING_MODEL_PATH = ROOT / "assets" / "franka_fr3_v2" / "scene_docking.xml"
RESULTS_DIR = ROOT / "results"
FIGURES_DIR = ROOT / "figures"
VIDEO_DIR = ROOT / "video"


@dataclass(frozen=True)
class PMSMConfig:
    """表贴式永磁同步电机、逆变器及减速器的工程等效参数。

    电气参数取自公开的 48 V Maxon EC-i 52 无刷电机，而不是 FR3
    未公开的内部驱动参数。模型假设理想转子位置检测、理想磁场定向控制
    且 ``i_d = 0``，只显式积分转矩电流 ``i_q``。

    Attributes:
        resistance: q 轴等效端电阻，单位为 Ω。
        inductance: q 轴等效端电感，单位为 H。
        torque_constant: 电机转矩常数，单位为 N·m/A。
        back_emf_constant: 反电动势常数，单位为 V·s/rad。
        rotor_inertia: 单台电机的转子惯量，单位为 kg·m²。
        pole_pairs: 电机极对数。
        gear_ratio: 七个关节的工程假设减速比，形状为 ``(7,)``。
        efficiency: 减速器传动效率，无量纲。
        voltage_limit: 逆变器 q 轴等效电压绝对值上限，单位为 V。
        nominal_current: 参考电机连续电流，单位为 A。
        joint_torque_limit: FR3 官方关节转矩绝对值上限，单位为 N·m。
        current_bandwidth: 电流环设计带宽，单位为 rad/s。
    """

    resistance: float = 0.284
    inductance: float = 0.45e-3
    torque_constant: float = 0.0919
    # 将参考电机的转速常数 104 rpm/V 换算为 SI 制反电动势常数。
    back_emf_constant: float = (1.0 / 104.0) / (2.0 * np.pi / 60.0)
    rotor_inertia: float = 170e-7
    pole_pairs: int = 8
    gear_ratio: np.ndarray = field(
        default_factory=lambda: np.array([100, 100, 100, 100, 50, 50, 50], dtype=float)
    )
    efficiency: float = 0.9
    voltage_limit: float = 48.0
    nominal_current: float = 4.59
    joint_torque_limit: np.ndarray = field(
        default_factory=lambda: np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)
    )
    current_bandwidth: float = 2000.0

    @property
    def current_limit(self) -> np.ndarray:
        """返回与 FR3 关节峰值转矩限制一致的 q 轴峰值电流。"""
        return self.joint_torque_limit / (self.efficiency * self.gear_ratio * self.torque_constant)


@dataclass(frozen=True)
class SimulationConfig:
    """轨迹和仿真实验配置。

    Attributes:
        physics_dt: MuJoCo 物理积分步长，单位为 s。
        control_dt: 外层控制器采样周期，单位为 s。
        duration: 单次实验总时长，单位为 s。
        transition_end: 起点过渡段结束时刻，单位为 s。
        circle_end: 圆周跟踪段结束时刻，单位为 s。
        circle_radius: 末端圆轨迹半径，单位为 m。
        circle_frequency: 末端圆轨迹频率，单位为 Hz。
        figure8_start: 平面 8 字轨迹开始时刻，单位为 s。
        figure8_end: 平面 8 字轨迹结束时刻，单位为 s。
        figure8_radius_x: 8 字轨迹的 x 向半宽，单位为 m。
        figure8_radius_y: 8 字轨迹的 y 向半宽，单位为 m。
        figure8_frequency: 8 字轨迹的基波频率，单位为 Hz。
        disturbance_start: 外力扰动开始时刻，单位为 s。
        disturbance_end: 外力扰动结束时刻，单位为 s。
        disturbance_force: 末端沿负 z 轴的扰动力大小，单位为 N。
        home_q: 七个机械臂关节的初始角度，单位为 rad，形状为 ``(7,)``。
    """

    physics_dt: float = 5e-5
    control_dt: float = 1e-3
    duration: float = 14.0
    transition_end: float = 1.5
    circle_end: float = 6.5
    circle_radius: float = 0.10
    circle_frequency: float = 0.2
    figure8_start: float = 8.0
    figure8_end: float = 13.0
    figure8_radius_x: float = 0.10
    figure8_radius_y: float = 0.07
    figure8_frequency: float = 0.2
    disturbance_start: float = 4.5
    disturbance_end: float = 5.5
    disturbance_force: float = 15.0
    home_q: np.ndarray = field(
        default_factory=lambda: np.array([0.0, -0.45, 0.0, -2.15, 0.0, 1.75, 0.78], dtype=float)
    )


@dataclass(frozen=True)
class DockingConfig:
    """FR3 柔顺对接实验的场景、轨迹和阻抗参数。

    母端在每次复位时按 ``start_distance`` 沿工具局部 z 轴自动放置，
    由此保证同一套参数可随 FR3 初始位姿一起变化。参考轨迹先保持，
    再靠近至接触距离，随后以小过盈继续插入并保持。
    """

    # SDF 接触场景以 5 kHz 物理积分（每个 1 kHz 控制周期 5 个子步）平衡接触搜索成本和稳定性。
    physics_dt: float = 2e-4
    duration: float = 11.0
    settle_end: float = 1.0
    approach_end: float = 5.0
    insertion_end: float = 7.0
    start_distance: float = 0.160
    # 原生 SDF 网格在工具标记点相距约 66 mm 时开始接触；参考保持小过盈并由力反馈限压。
    contact_distance: float = 0.055
    insertion_distance: float = 0.045
    contact_hold_force: float = 7.0
    contact_force_ramp_s: float = 0.4
    virtual_mass: np.ndarray = field(
        default_factory=lambda: np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0], dtype=float)
    )
    stiffness: np.ndarray = field(
        default_factory=lambda: np.array([100.0, 100.0, 100.0, 25.0, 25.0, 25.0], dtype=float)
    )
    damping: np.ndarray = field(
        default_factory=lambda: np.array([50.0, 50.0, 50.0, 10.0, 10.0, 10.0], dtype=float)
    )
    nullspace_stiffness: float = 4.0
    nullspace_damping: float = 2.5
    operational_damping: float = 2e-4


PMSM = PMSMConfig()
SIM = SimulationConfig()
DOCKING = DockingConfig()
JOINT_NAMES = tuple(f"fr3v2_joint{i}" for i in range(1, 8))
ACTUATOR_NAMES = tuple(f"actuator{i}" for i in range(1, 8))
