# plan_fixer/plan_fixer/goal_pose_refresher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPoseRefresher(Node):
    def __init__(self):
        super().__init__('goal_pose_refresher')

        # 订阅原始goal pose
        self.sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        # 发布更新后的goal pose
        self.pub = self.create_publisher(PoseStamped, '/goal_pose_fixed', 10)

        # 缓存最新收到的goal
        self.latest_goal = None
        self.last_goal_time = self.get_clock().now()

        # 每秒检查一次是否需要重新发布
        self.timer = self.create_timer(1.0, self.timer_callback)

    def goal_callback(self, msg: PoseStamped):
        # 收到新的goal
        self.latest_goal = msg
        self.last_goal_time = self.get_clock().now()
        self.get_logger().info('Received new goal_pose.')

    def timer_callback(self):
        now = self.get_clock().now()

        # 如果有记录的goal
        if self.latest_goal is not None:
            elapsed_time = (now.nanoseconds - self.last_goal_time.nanoseconds) * 1e-9  # 转成秒
            if elapsed_time > 5.0:
                # 超过5秒没有新的goal，重新发一次
                new_goal = PoseStamped()
                new_goal.header.frame_id = self.latest_goal.header.frame_id
                new_goal.header.stamp = now.to_msg()
                new_goal.pose = self.latest_goal.pose

                self.pub.publish(new_goal)
                self.last_goal_time = now  # 更新last_goal_time
                self.get_logger().info('Republished old goal_pose with updated timestamp.')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseRefresher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
