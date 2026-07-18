"""验证无机械臂接口检查场景和完整对接场景使用同一公母坐标约定。"""

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from fr3_control.config import DOCKING_INTERFACE_AXIAL_OFFSET, DOCKING_INTERFACE_MODEL_PATH
from fr3_control.model import load_docking_model, reset_docking_home


def test_arm_free_interface_scene_has_opposed_and_aligned_mating_frames() -> None:
    """公母网格共享安装偏航，且端面法向相对、接口基座沿 z 轴分离 55 mm。"""
    model = mujoco.MjModel.from_xml_path(str(DOCKING_INTERFACE_MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.njnt == 0

    plug_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "interface_plug_visual")
    socket_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "interface_socket_visual")
    plug_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "interface_plug")
    socket_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "interface_socket")
    plug_rotation = data.geom_xmat[plug_geom].reshape(3, 3)
    socket_rotation = data.geom_xmat[socket_geom].reshape(3, 3)
    mesh_quaternion = model.mesh_quat[model.geom_dataid[plug_geom]]
    mesh_rotation = Rotation.from_quat(mesh_quaternion[[1, 2, 3, 0]]).as_matrix()
    mating_rotation = Rotation.from_euler("z", DOCKING_INTERFACE_AXIAL_OFFSET).as_matrix() @ np.diag(
        [1.0, -1.0, -1.0]
    )
    opposing_faces_in_mesh_frame = mesh_rotation.T @ mating_rotation @ mesh_rotation

    assert np.allclose(socket_rotation, plug_rotation @ opposing_faces_in_mesh_frame, atol=1e-6)
    assert np.allclose(data.xpos[socket_body] - data.xpos[plug_body], [0.0, 0.0, 0.055])


def test_full_docking_scene_uses_the_same_visual_and_collision_alignment() -> None:
    """完整场景中，工具 SDF、工具可视网格和母端 SDF 坐标系一致。"""
    model, ids = load_docking_model()
    data = mujoco.MjData(model)
    reset_docking_home(model, data, ids)
    tool_visual = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "docking_tool_visual")
    tool_rotation = data.geom_xmat[ids.tool_contact_geom].reshape(3, 3)
    visual_rotation = data.geom_xmat[tool_visual].reshape(3, 3)
    socket_rotation = data.geom_xmat[ids.socket_contact_geom].reshape(3, 3)
    mesh_quaternion = model.mesh_quat[model.geom_dataid[ids.tool_contact_geom]]
    mesh_rotation = Rotation.from_quat(mesh_quaternion[[1, 2, 3, 0]]).as_matrix()
    mating_rotation = Rotation.from_euler("z", DOCKING_INTERFACE_AXIAL_OFFSET).as_matrix() @ np.diag(
        [1.0, -1.0, -1.0]
    )
    opposing_faces_in_mesh_frame = mesh_rotation.T @ mating_rotation @ mesh_rotation

    assert np.allclose(visual_rotation, tool_rotation, atol=1e-6)
    assert np.allclose(socket_rotation, tool_rotation @ opposing_faces_in_mesh_frame, atol=1e-6)
