"""MuJoCo FR3 v2 模型的加载、复位和常用动力学运算。"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .config import (
    ACTUATOR_NAMES,
    DOCKING,
    DOCKING_INTERFACE_AXIAL_OFFSET,
    DOCKING_INTERFACE_YAW,
    DOCKING_MODEL_PATH,
    JOINT_NAMES,
    MODEL_PATH,
    SIM,
)


@dataclass(frozen=True)
class ModelIds:
    """仿真所需模型元素的索引集合。

    Attributes:
        joint_qpos: 七个机械臂关节在 ``qpos`` 中的地址，形状为 ``(7,)``。
        joint_dof: 七个机械臂关节在 ``qvel`` 中的自由度地址，形状为 ``(7,)``。
        actuators: 七个机械臂转矩执行器的 ID，形状为 ``(7,)``。
        site: 末端执行器标记点 ``ee_site`` 的 ID。
        hand_body: 末端扰动力所施加刚体 ``fr3v2_link7`` 的 ID。
        target_mocap: 期望位置可视化刚体对应的 mocap ID。
    """

    joint_qpos: np.ndarray
    joint_dof: np.ndarray
    actuators: np.ndarray
    site: int
    hand_body: int
    target_mocap: int


@dataclass(frozen=True)
class DockingModelIds(ModelIds):
    """柔顺对接场景中额外元素的索引集合。"""

    socket_mocap: int
    socket_site: int
    force_sensor: int
    torque_sensor: int
    tool_contact_geom: int
    socket_contact_geom: int


def load_model() -> tuple[mujoco.MjModel, ModelIds]:
    """加载 FR3 v2 场景并解析仿真所需的命名元素。

    Returns:
        MuJoCo 模型以及该模型对应的元素索引集合。

    Raises:
        RuntimeError: 模型缺少任一必需的关节、执行器、刚体或标记点。
    """
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.opt.timestep = SIM.physics_dt
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
    )
    actuator_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ACTUATOR_NAMES]
    )
    target_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    ids = ModelIds(
        joint_qpos=model.jnt_qposadr[joint_ids].copy(),
        joint_dof=model.jnt_dofadr[joint_ids].copy(),
        actuators=actuator_ids,
        site=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site"),
        hand_body=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fr3v2_link7"),
        target_mocap=int(model.body_mocapid[target_body]),
    )
    if min(*ids.joint_qpos, *ids.actuators, ids.site, ids.hand_body, ids.target_mocap) < 0:
        raise RuntimeError("FR3 v2 model is missing one or more required named elements")
    return model, ids


def load_docking_model() -> tuple[mujoco.MjModel, DockingModelIds]:
    """加载包含 FR3、对接件、接触对和力/力矩传感器的独立场景。"""
    model = mujoco.MjModel.from_xml_path(str(DOCKING_MODEL_PATH))
    model.opt.timestep = DOCKING.physics_dt
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
    )
    actuator_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ACTUATOR_NAMES]
    )
    reference_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "docking_reference")
    socket_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "docking_socket")
    ids = DockingModelIds(
        joint_qpos=model.jnt_qposadr[joint_ids].copy(),
        joint_dof=model.jnt_dofadr[joint_ids].copy(),
        actuators=actuator_ids,
        site=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "docking_ee_site"),
        hand_body=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "docking_tool"),
        target_mocap=int(model.body_mocapid[reference_body]),
        socket_mocap=int(model.body_mocapid[socket_body]),
        socket_site=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "docking_socket_site"),
        force_sensor=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "docking_force_sensor"),
        torque_sensor=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "docking_torque_sensor"),
        tool_contact_geom=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "docking_tool_sdf"),
        socket_contact_geom=mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "docking_socket_sdf"
        ),
    )
    required = (
        *ids.joint_qpos,
        *ids.actuators,
        ids.site,
        ids.hand_body,
        ids.target_mocap,
        ids.socket_mocap,
        ids.socket_site,
        ids.force_sensor,
        ids.torque_sensor,
        ids.tool_contact_geom,
        ids.socket_contact_geom,
    )
    if min(required) < 0:
        raise RuntimeError("docking model is missing one or more required named elements")
    return model, ids


def reset_home(model: mujoco.MjModel, data: mujoco.MjData, ids: ModelIds) -> None:
    """将仿真状态复位到项目规定的初始姿态。

    Args:
        model: MuJoCo 模型。
        data: 待复位的 MuJoCo 动力学状态。
        ids: 与 ``model`` 对应的元素索引集合。
    """
    mujoco.mj_resetData(model, data)
    data.qpos[ids.joint_qpos] = SIM.home_q
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def reset_docking_home(
    model: mujoco.MjModel, data: mujoco.MjData, ids: DockingModelIds
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """复位 FR3，并由工具初始位姿设置固定母端的位姿。

    Returns:
        依次返回工具初始位置、初始旋转矩阵、母端位置和母端旋转矩阵。
    """
    mujoco.mj_resetData(model, data)
    data.qpos[ids.joint_qpos] = SIM.home_q
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    tool_position, tool_rotation = site_pose(data, ids.site)
    approach_axis = tool_rotation[:, 2]
    socket_position = tool_position + DOCKING.start_distance * approach_axis
    # 母端先继承工具端的安装偏航，再绕插入轴错开 45° 并翻转端面；这样可视与
    # SDF 碰撞网格使用同一公母接口坐标系，凸缘和键槽以互补姿态严格相对。
    mesh_yaw = Rotation.from_euler("z", DOCKING_INTERFACE_YAW).as_matrix()
    axial_offset = Rotation.from_euler("z", DOCKING_INTERFACE_AXIAL_OFFSET).as_matrix()
    socket_rotation = tool_rotation @ mesh_yaw @ axial_offset @ np.diag([1.0, -1.0, -1.0])
    data.mocap_pos[ids.socket_mocap] = socket_position
    socket_quaternion = Rotation.from_matrix(socket_rotation).as_quat()
    data.mocap_quat[ids.socket_mocap] = socket_quaternion[[3, 0, 1, 2]]
    data.mocap_pos[ids.target_mocap] = tool_position
    tool_quaternion = Rotation.from_matrix(tool_rotation).as_quat()
    data.mocap_quat[ids.target_mocap] = tool_quaternion[[3, 0, 1, 2]]
    mujoco.mj_forward(model, data)
    return tool_position, tool_rotation, socket_position, socket_rotation


def site_pose(data: mujoco.MjData, site_id: int) -> tuple[np.ndarray, np.ndarray]:
    """读取标记点在世界坐标系中的位置和旋转矩阵。

    Args:
        data: 已完成正向运动学计算的 MuJoCo 状态。
        site_id: 待读取标记点的 ID。

    Returns:
        位置向量和旋转矩阵的副本，形状分别为 ``(3,)`` 和 ``(3, 3)``。
    """
    return data.site_xpos[site_id].copy(), data.site_xmat[site_id].reshape(3, 3).copy()


def site_jacobian(
    model: mujoco.MjModel, data: mujoco.MjData, site_id: int, dof_ids: np.ndarray
) -> np.ndarray:
    """计算标记点关于指定自由度的六维几何雅可比矩阵。

    Args:
        model: MuJoCo 模型。
        data: 当前 MuJoCo 动力学状态。
        site_id: 目标标记点 ID。
        dof_ids: 需要保留的自由度地址。

    Returns:
        平移雅可比与旋转雅可比按行拼接的矩阵，形状为 ``(6, len(dof_ids))``。
    """
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    return np.vstack((jacp[:, dof_ids], jacr[:, dof_ids]))


def mass_matrix(model: mujoco.MjModel, data: mujoco.MjData, dof_ids: np.ndarray) -> np.ndarray:
    """提取指定自由度对应的关节空间惯性矩阵。

    Args:
        model: MuJoCo 模型。
        data: 当前 MuJoCo 动力学状态。
        dof_ids: 需要保留的自由度地址。

    Returns:
        指定自由度的对称惯性子矩阵。
    """
    full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, full)
    return full[np.ix_(dof_ids, dof_ids)]
