import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathHeaderFixer(Node):
    def __init__(self):
        super().__init__('path_header_fixer')
        self.sub = self.create_subscription(Path, '/plan', self.callback, 10)
        self.pub = self.create_publisher(Path, '/plan_fixed', 10)

    def callback(self, msg: Path):
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        for pose in msg.poses:
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = now
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathHeaderFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
