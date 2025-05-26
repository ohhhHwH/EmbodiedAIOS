import gymnasium as gym
import numpy as np
import mujoco
import os


class CtrlByMujoco:
    ee_site_name = "ee_site"

    def __init__(
        self,
        joint_num,
        joint_lower_limits,
        joint_upper_limits,
        sim_steps=10,
        render_mode=None,
        model_path=os.path.abspath(
            os.path.join(
                "./src/piper_description", "mujoco_model", "piper_description.xml"
            )
        ),
    ):
        self.joint_num = joint_num
        self.sim_steps = sim_steps

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.actuator_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"joint{i+1}")
            for i in range(self.joint_num)
        ]

        self.render_mode = render_mode
        if self.render_mode:
            self.renderer = mujoco.Renderer(self.model)
            self.cam = mujoco.MjvCamera()
            # 视距，拉远看整个机械臂
            self.cam.distance = 2.0

        # 两个夹爪的 ID
        self.joint7_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "joint7"
        )
        self.joint8_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "joint8"
        )

        self.joint_lower_limits = joint_lower_limits
        self.joint_upper_limits = joint_upper_limits

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)

    def send_a_step(self):
        for _ in range(self.sim_steps):
            mujoco.mj_forward(self.model, self.data)
            mujoco.mj_step(self.model, self.data)

    def set_joint(self, joint_id2positions: dict):
        """
        返回超出关节限制的量
        """
        excess = {}
        for idx, (joint_id, position) in enumerate(joint_id2positions.items()):
            if joint_id in self.actuator_ids:
                # note: 直接修改qpos但不修改ctrl，相当于让关节瞬移到目标位置
                # 但仿真器又仿真ctrl它回到原点，所以每步step都几乎没动
                # self.data.qpos[:JOINT_NUM] = qpos
                self.data.ctrl[joint_id] = np.clip(
                    position,
                    self.joint_lower_limits[idx],
                    self.joint_upper_limits[idx],
                )
                excess[joint_id] = self.data.ctrl[joint_id] - position
            else:
                raise ValueError(f"Invalid joint ID: {joint_id}")
        return excess

    def add_joint(self, joint_id2action: dict):
        """
        返回超出关节限制的量
        """
        excess = {}
        for idx, (joint_id, action) in enumerate(joint_id2action.items()):
            if joint_id in self.actuator_ids:
                new_qpos = self.data.qpos[joint_id] + action
                self.data.ctrl[joint_id] = np.clip(
                    new_qpos,
                    self.joint_lower_limits[idx],
                    self.joint_upper_limits[idx],
                )
                excess[joint_id] = self.data.ctrl[joint_id] - new_qpos
            else:
                raise ValueError(f"Invalid joint ID: {joint_id}")
        return excess

    def get_joint(self):
        """
        不包括夹爪关节
        """
        return np.array([self.data.qpos[joint_id] for joint_id in self.actuator_ids])

    def get_ee_pos(self):
        return np.array(self.data.site(self.ee_site_name).xpos.copy())

    def get_ee_rotmat(self):
        return self.data.site(self.ee_site_name).xmat.reshape(3, 3).copy()

    def render(self):
        if self.render_mode is None:
            return
        if self.render_mode == "rgb_array":
            self.renderer.update_scene(self.data, camera=self.cam)
            return self.renderer.render()
        elif self.render_mode == "human":
            if not hasattr(self, "viewer"):
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.sync()
        else:
            raise ValueError("Invalid render mode. Use 'rgb_array' or 'human'.")
