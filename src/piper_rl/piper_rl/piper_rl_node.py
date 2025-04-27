# 文件：piper_rl_node.py 中添加此类

import gym
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


class MyRobotEnv(gym.Env):
    def __init__(self):
        super(MyRobotEnv, self).__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node('my_robot_rl_env')

        # 初始状态（6个关节 + 目标点）
        # self.joint_state = [0.0] * 6
        self.current_joint_states = [0.0] * 7  # 包括夹爪joint7
        self.target_point = [0.5, 0.0, 0.5]

        # 订阅 joint_states 与 目标点
        self.node.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.node.create_subscription(PointStamped, '/base_target_point', self.target_cb, 10)
        self.arm_pub = self.node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.node.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # 动作空间：每个关节的角度增量范围（-0.1 ~ 0.1 rad）
        # self.action_space = gym.spaces.Box(low=-0.1, high=0.1, shape=(6,), dtype=np.float32)

        # 动作空间 Δx, Δy, Δz (单位米)，夹爪动作(0/1)
        self.action_space = gym.spaces.Box(low=np.array([-0.02, -0.02, -0.02, 0]),
                                           high=np.array([0.02, 0.02, 0.02, 1]),
                                           dtype=np.float32)


        # 观测空间：6个关节角度 + 目标点 xyz
        # self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(15,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        # self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def send_arm_goal(self, delta_xyz):
        # 末端当前位置
        ee_pos = self.get_ee_pose()
        target_pos = [ee_pos[0] + delta_xyz[0], ee_pos[1] + delta_xyz[1], ee_pos[2] + delta_xyz[2]]

        # 简化IK：只控制joint2, joint3, joint5（你可以加更复杂的IK）
        # 这里直接简单推 joint2，joint3来模拟z方向伸缩
        new_joint = self.current_joint_states[:6].copy()
        # Example IK: adjust joint2 for z, joint1 for x,y（伪IK，真实需要更完善）
        new_joint[1] -= delta_xyz[2] * 5
        new_joint[0] += delta_xyz[1] * 5
        new_joint[2] += delta_xyz[0] * 5

        traj = JointTrajectory()
        traj.joint_names = [f'joint{i+1}' for i in range(6)]
        point = JointTrajectoryPoint()
        point.positions = new_joint
        point.time_from_start.sec = 1
        traj.points.append(point)

        self.arm_pub.publish(traj)



    def joint_state_cb(self, msg):
        name2index = {name: i for i, name in enumerate(msg.name)}
        try:
            self.joint_state = [msg.position[name2index[f'joint{i+1}']] for i in range(6)]
        except:
            pass  # 防止 joint 数量不足导致异常

    def target_cb(self, msg):
        self.target_point = [msg.point.x, msg.point.y, msg.point.z]

    def get_ee_pose(self):
        try:
            latest_time = self.node.get_clock().now().to_msg()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'gripper_base',    # source frame
                rclpy.time.Time(),  # ← 获取最近的可用变换
                timeout=rclpy.duration.Duration(seconds=1.0)
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
        ee_pos = self.get_ee_pose()
        dx = self.target_point[0] - ee_pos[0]
        dy = self.target_point[1] - ee_pos[1]
        dz = self.target_point[2] - ee_pos[2]
        return np.array(self.current_joint_states[:6] + [ee_pos[0], ee_pos[1], ee_pos[2],
                                                          self.target_point[0], self.target_point[1], self.target_point[2]], dtype=np.float32)


    # def _get_obs(self):
    #     ee_pos = self.get_ee_pose()
    #     dx = self.target_point[0] - ee_pos[0]
    #     dy = self.target_point[1] - ee_pos[1]
    #     dz = self.target_point[2] - ee_pos[2]
    #
    #     return np.array(
    #         self.current_joint_states[:6] +
    #         [ee_pos[0], ee_pos[1], ee_pos[2]] +
    #         [self.target_point[0], self.target_point[1], self.target_point[2]] +
    #         [dx, dy, dz],
    #         dtype=np.float32
    #     )

    def control_gripper(self, close=True):

        traj = JointTrajectory()
        traj.joint_names = ['joint7']  # 夹爪

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
        rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._get_obs()


    # def step(self, action):
    #     # 当前状态加上动作增量
    #     target_joint = [self.joint_state[i] + float(action[i]) for i in range(6)]
    #
    #     # 构造轨迹消息
    #     traj = JointTrajectory()
    #     traj.joint_names = [f'joint{i+1}' for i in range(6)]
    #
    #     point = JointTrajectoryPoint()
    #     point.positions = target_joint
    #     point.time_from_start.sec = 1
    #     traj.points.append(point)
    #
    #     # 发布到 arm_controller
    #     pub = self.node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    #     pub.publish(traj)
    #
    #     # 等待机械臂运动生效
    #     for _ in range(5):
    #         rclpy.spin_once(self.node, timeout_sec=0.1)
    #
    #     # 计算末端与目标距离（简化为关节状态 + 目标点）
    #     obs = self._get_obs()
    #     ee_pos = self.get_ee_pose()
    #     # ee_hint = self.forward_kinematics_estimate(obs[:6])  # 简化
    #     goal = obs[6:9]
    #     dist = np.linalg.norm(np.array(ee_pos) - np.array(goal))
    #
    #     # 奖励函数
    #     reward = -dist
    #     # 奖励 shaping
    #     if dist < 0.1:
    #         reward += 0.5
    #     if dist < 0.05:
    #         reward += 1.0
    #     if dist < 0.03:
    #         reward += 2.0
    #     if dist < 0.02:
    #         reward += 5.0
    #         self.control_gripper(close=True)
    #         done = True
    #     else:
    #         done = False
    #
    #
    #     # === 控制台输出：可视化当前状态 ===
    #     self.node.get_logger().info(
    #         f"\n" +
    #         f"📍 末端位置: x={ee_pos[0]:.3f}, y={ee_pos[1]:.3f}, z={ee_pos[2]:.3f}\n" +
    #         f"🎯 目标点:  x={goal[0]:.3f}, y={goal[1]:.3f}, z={goal[2]:.3f}\n" +
    #         f"📏 当前距离: {dist:.4f} m\n" +
    #         f"💰 当前奖励: {reward:.4f}\n" +
    #         f"{'✅ 成功抓取!' if done else ''}"
    #     )
    #
    #     return obs, reward, done, {}


    def step(self, action):
        delta_xyz = action[:3]
        gripper_action = action[3]

        self.send_arm_goal(delta_xyz)

        if gripper_action > 0.5:
            self.control_gripper(close=True)

        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.3)

        obs = self._get_obs()
        ee_pos = self.get_ee_pose()
        goal_pos = self.target_point

        dist = np.linalg.norm(np.array(ee_pos) - np.array(goal_pos))

        reward = -5.0 * dist
        if dist < 0.1:
            reward += 0.5
        if dist < 0.05:
            reward += 2.0
        if dist < 0.02:
            reward += 5.0
            if gripper_action > 0.5:
                reward += 10.0
                done = True
            else:
                done = False
        else:
            done = False
        # === 控制台输出：可视化当前状态 ===
        self.node.get_logger().info(
            f"\n" +
            f"📍 末端位置: x={ee_pos[0]:.3f}, y={ee_pos[1]:.3f}, z={ee_pos[2]:.3f}\n" +
            f"🎯 目标点:  x={goal_pos[0]:.3f}, y={goal_pos[1]:.3f}, z={goal_pos[2]:.3f}\n" +
            f"📏 当前距离: {dist:.4f} m\n" +
            f"💰 当前奖励: {reward:.4f}\n" +
            f"{'✅ 成功抓取!' if done else ''}"
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