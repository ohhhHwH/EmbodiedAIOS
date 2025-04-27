# plan_fixer/plan_fixer/fix_path_header.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathHeaderFixer(Node):
    def __init__(self):
        super().__init__('path_header_fixer')
        self.sub = self.create_subscription(Path, '/plan', self.callback, 10)
        self.pub = self.create_publisher(Path, '/plan_fixed', 10)

    def callback(self, msg: Path):
        for pose in msg.poses:
            pose.header.frame_id = msg.header.frame_id
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathHeaderFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
