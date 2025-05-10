# 文件：piper_rl_node.py 中添加此类

import gym
import gym.spaces
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as RosTime

# maximize z 模型的JOINTLOWERLIMIT[2]是用的-2*np.pi
JOINTLOWERLIMIT = [-np.pi, 0.0, -2 * np.pi, -np.pi, -np.pi, -np.pi]
JOINTUPPERLIMIT = [np.pi, np.pi, 0.0, np.pi, np.pi, np.pi]


class MyRobotEnv(gym.Env):
    def __init__(self):
        super(MyRobotEnv, self).__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node("my_robot_rl_env")

        # 初始状态（6个关节 + 目标点）
        # self.joint_state = [0.0] * 6
        self.current_joint_states = [0.0] * 7  # 包括夹爪joint7
        self.target_point = [0.5, 0.0, 0.5]

        # 订阅 joint_states 与 目标点
        self.node.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, 10
        )
        self.node.create_subscription(
            PointStamped, "/base_target_point", self.target_cb, 10
        )
        self.arm_pub = self.node.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.node.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )

        # 动作空间：每个关节的角度范围 + 夹爪动作（0/1)
        # self.action_space = gym.spaces.Box(
        #     low=np.array(
        #         JOINTLOWERLIMIT, dtype=np.float32
        #     ),
        #     high=np.array(JOINTUPPERLIMIT, dtype=np.float32),
        #     dtype=np.float32,
        # )

        # 动作空间：每个关节的角度增量范围（-0.1 ~ 0.1 rad）+ 夹爪动作（0/1)
        self.action_space = gym.spaces.Box(
            low=np.array([-0.1] * 6 + [0.0], dtype=np.float32),
            high=np.array([0.1] * 6 + [1.0], dtype=np.float32),
            dtype=np.float32,
        )
        # maximize z
        # self.action_space = gym.spaces.Box(
        #     low=np.array([-0.1] * 4, dtype=np.float32),
        #     high=np.array([0.1] * 4, dtype=np.float32),
        #     dtype=np.float32,
        # )

        # 动作空间 Δx, Δy, Δz (单位米)，夹爪动作(0/1)
        # self.action_space = gym.spaces.Box(
        #     low=np.array([-0.02, -0.02, -0.02, 0]),
        #     high=np.array([0.02, 0.02, 0.02, 1]),
        #     dtype=np.float32,
        # )

        # 观测空间：6个关节的角度
        # maximize z
        # self.observation_space = gym.spaces.Box(
        #     low=np.array(JOINTLOWERLIMIT, dtype=np.float32),
        #     high=np.array(JOINTUPPERLIMIT, dtype=np.float32),
        #     dtype=np.float32,
        # )

        # 观测空间：6个关节角度 + 目标点 xyz
        self.observation_space = gym.spaces.Box(
            low=np.array(JOINTLOWERLIMIT + [-np.inf] * 3),
            high=np.array(JOINTUPPERLIMIT + [np.inf] * 3),
            dtype=np.float32,
        )
        # 观测空间：当前点 xyz + 目标点 xyz
        # self.observation_space = gym.spaces.Box(
        #     low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32
        # )
        # self.observation_space = gym.spaces.Box(
        #     low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32
        # )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def send_arm_goal(self, delta_xyz):
        # 末端当前位置
        # ee_pos = self.get_ee_pose()
        # target_pos = [ee_pos[0] + delta_xyz[0], ee_pos[1] + delta_xyz[1], ee_pos[2] + delta_xyz[2]]

        # 简化IK：只控制joint2, joint3, joint5（你可以加更复杂的IK）
        # 这里直接简单推 joint2，joint3来模拟z方向伸缩
        new_joint = self.current_joint_states[:6].copy()
        # Example IK: adjust joint2 for z, joint1 for x,y（伪IK，真实需要更完善）
        new_joint[1] -= delta_xyz[2] * 5
        new_joint[0] += delta_xyz[1] * 5
        new_joint[2] += delta_xyz[0] * 5
        for i in range(3):
            new_joint[i] = min(new_joint[i], JOINTUPPERLIMIT[i])
            new_joint[i] = max(new_joint[i], JOINTLOWERLIMIT[i])

        self.send_arm_joint_goal(new_joint)

    def send_arm_joint(self, delta_joint):
        new_joint = self.current_joint_states[:6].copy()
        for i in range(6):
            new_joint[i] += delta_joint[i]
            new_joint[i] = min(new_joint[i], JOINTUPPERLIMIT[i])
            new_joint[i] = max(new_joint[i], JOINTLOWERLIMIT[i])
        # new_joint[5] = np.pi
        self.send_arm_joint_goal(new_joint)

    def send_arm_joint_goal(self, target_joint):
        traj = JointTrajectory()
        traj.joint_names = [f"joint{i+1}" for i in range(6)]
        point = JointTrajectoryPoint()
        point.positions = target_joint
        point.time_from_start.sec = 1
        traj.points.append(point)
        # self.current_joint_states[:6] = target_joint

        self.arm_pub.publish(traj)

    def joint_state_cb(self, msg):
        name2index = {name: i for i, name in enumerate(msg.name)}
        self.current_joint_states = [
            msg.position[name2index[f"joint{i+1}"]] for i in range(6)
        ]

    def target_cb(self, msg):
        self.target_point = [msg.point.x, msg.point.y, msg.point.z]

    # 获取当前末端执行器位置
    def get_ee_pose(self):
        try:
            latest_time = self.node.get_clock().now().to_msg()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                "gripper_base",  # source frame
                rclpy.time.Time(),  # ← 获取最近的可用变换
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            pos = trans.transform.translation
            return [pos.x, pos.y, pos.z]
        except TransformException as e:
            self.node.get_logger().warn(f"TF transform failed: {e}")
            return [0.0, 0.0, 0.0]

    # def _get_obs(self):
    #     return np.array(self.joint_state + self.target_point, dtype=np.float32)
    #

    def _get_obs(self):
        return np.array(
            self.current_joint_states[:6]
            + [
                self.target_point[0],
                self.target_point[1],
                self.target_point[2],
            ],
            dtype=np.float32,
        )
        # maximize z
        # return np.array(
        #     self.current_joint_states[:6],
        #     dtype=np.float32,
        # )

    def control_gripper(self, close=True):

        traj = JointTrajectory()
        traj.joint_names = ["joint7"]  # 夹爪

        point = JointTrajectoryPoint()
        point.positions = [0.0 if close else 0.02]  # 0.3 为闭合程度，按需调整
        point.time_from_start.sec = 1
        traj.points.append(point)

        self.gripper_pub.publish(traj)

        # 等待夹爪运动生效
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.25)

    def reset(self):
        self.control_gripper(close=True)
        # 后续可加入机械臂重置逻辑
        # 等待机械臂运动生效
        rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._get_obs()

    def step(self, action):
        # delta_xyz = action[:3].tolist()
        # gripper_action = action[3]
        # self.send_arm_goal(delta_xyz)

        delta_joint = action[:6].tolist()
        gripper_action = action[6]
        self.send_arm_joint(delta_joint)
        # maximize z
        # delta_joint = [0] + action.tolist() + [0]
        # self.send_arm_joint(delta_joint)

        if gripper_action > 0.5:
            self.control_gripper(close=True)

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=1.0)

        obs = self._get_obs()
        ee_pos = self.get_ee_pose()
        goal_pos = self.target_point

        dist = np.linalg.norm(np.array(ee_pos) - np.array(goal_pos))
        manhattan_dist = np.abs(np.array(ee_pos) - np.array(goal_pos)).sum()

        # joint = self.current_joint_states
        # maximize z
        # reward = (
        #     ee_pos[2] * 10
        #     - abs(joint[1] - np.pi / 2)
        #     - abs(joint[2] - (-np.pi))
        #     - abs(joint[4])
        # )
        # if ee_pos[2] < 0.2:
        #     reward -= 10.0
        # if ee_pos[2] > 0.6:
        #     reward **= 2
        # if ee_pos[2] > 0.7:
        #     done = True
        # else:
        #     done = False
        reward = -5.0 * dist
        if dist < 0.3:
            reward += 3.0
        if dist < 0.2:
            reward += 4.0
        if dist < 0.1:
            reward += 5.0
        if dist < 0.05:
            reward += 20.0
        if dist < 0.02:
            reward += 50.0
            if gripper_action > 0.5:
                reward += 100.0
                done = True
            else:
                done = False
        else:
            done = False
        # === 控制台输出：可视化当前状态 ===
        self.node.get_logger().info(
            f"\n"
            + f"📍 末端位置: x={ee_pos[0]:.3f}, y={ee_pos[1]:.3f}, z={ee_pos[2]:.3f}\n"
            + f"🎯 目标点:  x={goal_pos[0]:.3f}, y={goal_pos[1]:.3f}, z={goal_pos[2]:.3f}\n"
            + f"📏 当前距离: {dist:.4f} m, 曼哈顿距离: {manhattan_dist:.4f}m\n"
            + f'🤖 当前关节: {[f"{i:.2f}" for i in self.current_joint_states]}\n'
            + f"💰 当前奖励: {reward:.4f}\n"
            + f"{'✅ 成功抓取!' if done else ''}"
        )

        return obs, reward, done, {}

    # def forward_kinematics_estimate(self, joints):
    #     # 简化计算，用 joint1/joint2 的角度估个方向向量模拟末端位置
    #     x = 0.4 + 0.2 * np.cos(joints[0])
    #     y = 0.0 + 0.2 * np.sin(joints[0])
    #     z = 0.4 + 0.1 * np.sin(joints[1])
    #     return [x, y, z]


if __name__ == "__main__":
    env = MyRobotEnv()
    obs = env.reset()
    print("Start Obs:", obs)
    action = np.zeros(6)
    action[1] = 0.1  # 向下移动关节2试试
    obs, reward, done, _ = env.step(action)
    print("New Obs:", obs)
    print("Reward:", reward, "Done:", done)
    print("张开夹爪")
    env.control_gripper(close=False)
    time.sleep(1)
    print("闭合夹爪")
    env.control_gripper(close=True)
    pose = env.get_ee_pose()
    print("末端位姿:", pose)
