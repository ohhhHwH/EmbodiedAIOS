import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPoseRefresher(Node):
    def __init__(self):
        super().__init__('goal_pose_refresher')
        self.sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose_del', 10)

        self.latest_goal = None
        self.last_goal_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def goal_callback(self, msg: PoseStamped):
        self.latest_goal = msg
        self.last_goal_time = self.get_clock().now()
        self.get_logger().info('Received new goal_pose.')

    def timer_callback(self):
        now = self.get_clock().now()
        if self.latest_goal is not None:
            elapsed_time = (now.nanoseconds - self.last_goal_time.nanoseconds) * 1e-9
            if elapsed_time > 5.0:
                new_goal = PoseStamped()
                new_goal.header.frame_id = self.latest_goal.header.frame_id
                new_goal.header.stamp = now.to_msg()
                new_goal.pose = self.latest_goal.pose
                self.pub.publish(new_goal)
                self.last_goal_time = now
                self.get_logger().info('Republished old goal_pose with updated timestamp.')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseRefresher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
