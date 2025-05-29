import numpy as np
import time
from piper_sdk import C_PiperInterface_V2
from ctrl_base import CtrlBase


class CtrlByPiperSDK(CtrlBase):
    def __init__(
        self,
        joint_num: int,
        joint_lower_limits: list[float],
        joint_upper_limits: list[float],
    ):
        super().__init__(joint_num, joint_lower_limits, joint_upper_limits)
        self.actuator_ids = [i for i in range(self.joint_num)]
        # 统一是弧度
        self.joints_state_ctrl = np.zeros(self.joint_num, dtype=np.float32)

        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        if not self._enable_fun():
            raise RuntimeError("Failed to enable Piper arm.")
        self.reset()
        # print(self.piper.GetArmGripperMsgs())
        # print(self.piper.GetAllMotorAngleLimitMaxSpd())

    def __del__(self):
        """
        确保在对象销毁时禁用机械臂
        """
        self.reset()
        time.sleep(2)
        self.disable()

    def reset(self):
        self.joints_state_ctrl = np.zeros(self.joint_num, dtype=np.float32)
        # self.piper.MotionCtrl_1(emergency_stop=0x02, track_ctrl=0, grag_teach_ctrl=0)
        # TODO: 看看mit模式是什么，据说响应速度更快
        self.piper.MotionCtrl_2(
            ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=50, is_mit_mode=0x00
        )
        # self.piper.EndPoseCtrl(X=0, Y=0, Z=0, RX=0, RY=0, RZ=0)
        self.set_joint({id: self.joints_state_ctrl[id] for id in self.actuator_ids})
        self.send_a_step()
        self.set_gripper(close=False)

    def send_a_step(self):
        ctrl = np.degrees(self.joints_state_ctrl) * 1000.0
        self.piper.JointCtrl(
            joint_1=int(ctrl[0]),
            joint_2=int(ctrl[1]),
            joint_3=int(ctrl[2]),
            joint_4=int(ctrl[3]),
            joint_5=int(ctrl[4]),
            joint_6=int(ctrl[5]),
        )

    def set_joint(self, joint_id2positions: dict[str, float]) -> dict[str, float]:
        excess = {}
        for idx, (joint_id, position) in enumerate(joint_id2positions.items()):
            if joint_id in self.actuator_ids:
                self.joints_state_ctrl[joint_id] = np.clip(
                    position,
                    self.joint_lower_limits[idx],
                    self.joint_upper_limits[idx],
                )
                excess[joint_id] = self.joints_state_ctrl[joint_id] - position
            else:
                raise ValueError(f"Invalid joint ID: {joint_id}")
        return excess

    def add_joint(self, joint_id2action: dict[str, float]) -> dict[str, float]:
        excess = {}
        self.joints_state_ctrl = self.get_joint()
        for idx, (joint_id, action) in enumerate(joint_id2action.items()):
            if joint_id in self.actuator_ids:
                new_qpos = self.joints_state_ctrl[joint_id] + action
                self.joints_state_ctrl[joint_id] = np.clip(
                    new_qpos,
                    self.joint_lower_limits[idx],
                    self.joint_upper_limits[idx],
                )
                excess[joint_id] = self.joints_state_ctrl[joint_id] - new_qpos
            else:
                raise ValueError(f"Invalid joint ID: {joint_id}")
        return excess

    def get_joint(self) -> np.ndarray:
        joints = self.piper.GetArmJointMsgs()
        return np.radians(
            np.array(
                [
                    joints.joint_state.joint_1,
                    joints.joint_state.joint_2,
                    joints.joint_state.joint_3,
                    joints.joint_state.joint_4,
                    joints.joint_state.joint_5,
                    joints.joint_state.joint_6,
                ],
                dtype=np.float32,
            )
            / 1000.0
        )

    def get_gripper(self) -> float:
        gripper_msgs = self.piper.GetArmGripperMsgs()
        return gripper_msgs.gripper_state.grippers_angle / 1000.0 / 1000.0

    def get_ee_pos(self) -> np.ndarray:
        end_pose = self.piper.GetArmEndPoseMsgs().end_pose
        return (
            np.array(
                [end_pose.X_axis, end_pose.Y_axis, end_pose.Z_axis],
                dtype=np.float32,
            )
            / 1000.0
            / 1000.0
        )

    def get_ee_quat(self) -> np.ndarray:
        end_pose = self.piper.GetArmEndPoseMsgs().end_pose
        return (
            self._euler2quat([end_pose.RX_axis, end_pose.RY_axis, end_pose.RZ_axis])
            / 1000.0
        )

    def set_gripper(self, close: bool = True):
        # 50 mm
        self.piper.GripperCtrl(
            0 if close else 100 * 1000,
            gripper_effort=1000,
            gripper_code=0x01,
            set_zero=0,
        )

    def render(self):
        print("Rendering is not supported in Piper SDK control mode.")

    def _enable_fun(self) -> bool:
        """
        使能机械臂并检测使能状态,尝试5s,如果使能超时则返回False
        """
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not (enable_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            self.piper.EnableArm(7)
            msgs = self.piper.GetArmLowSpdInfoMsgs()
            enable_flag = (
                msgs.motor_1.foc_status.driver_enable_status
                and msgs.motor_2.foc_status.driver_enable_status
                and msgs.motor_3.foc_status.driver_enable_status
                and msgs.motor_4.foc_status.driver_enable_status
                and msgs.motor_5.foc_status.driver_enable_status
                and msgs.motor_6.foc_status.driver_enable_status
            )
            print("使能状态:", enable_flag)
            print("--------------------")
            # 检查是否超过超时时间
            if elapsed_time > timeout:
                print("超时....")
                elapsed_time_flag = True
                # enable_flag = True
                break
            time.sleep(1)
        if elapsed_time_flag:
            print("程序自动使能超时")
        return enable_flag

    def disable(self):
        """
        禁用机械臂
        """
        self.piper.DisableArm()
        self.piper.GripperCtrl(0, 1000, 0x02, 0)
        self.piper.DisconnectPort()
        print("Piper arm disabled.")

    @staticmethod
    def _euler2quat(euler_angles: list[float]) -> np.ndarray:
        """
        将欧拉角转换为四元数
        :param euler_angles: 欧拉角 [roll, pitch, yaw], 单位为角度
        :return: 四元数 [qx, qy, qz, qw]
        """
        roll, pitch, yaw = np.radians(euler_angles)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return np.array([qx, qy, qz, qw], dtype=np.float32)


if __name__ == "__main__":
    piper_ctrl = CtrlByPiperSDK(
        6,
        joint_lower_limits=[-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944],
        joint_upper_limits=[2.618, 3.14, 0.0, 1.745, 1.22, 2.0944],
    )
    time.sleep(1)
    print("Initial joint positions:", piper_ctrl.get_joint())
    print("Initial gripper position:", piper_ctrl.get_gripper())
    print("Initial end-effector position:", piper_ctrl.get_ee_pos())
    print("Initial end-effector orientation (quaternion):", piper_ctrl.get_ee_quat())
    piper_ctrl.set_joint(
        {i: [0.5, 0.5, -0.7, 0.3, -0.2, 0.5, 0.08][i] for i in range(6)}
    )
    piper_ctrl.send_a_step()
    print("Updated joint positions:", piper_ctrl.get_joint())
    print("Updated gripper position:", piper_ctrl.get_gripper())
    print("Updated end-effector position:", piper_ctrl.get_ee_pos())
    print("Updated end-effector orientation (quaternion):", piper_ctrl.get_ee_quat())
    piper_ctrl.set_gripper(close=True)
    print("Gripper closed.")
    time.sleep(1)
    piper_ctrl.set_gripper(close=False)
    print("Gripper opened.")
    piper_ctrl.set_joint({i: [0.0, 0.0, -0.0, 0.0, 0.2, 0.0, 0.0][i] for i in range(6)})
    piper_ctrl.send_a_step()
    time.sleep(1)
