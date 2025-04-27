import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import subprocess
import json
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime

class VLMMapperNode(Node):
    def __init__(self):
        super().__init__('vlm_mapper')
        self.bridge = CvBridge()
        self.image_received = False
        self.target_point = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.target_sub = self.create_subscription(PointStamped, '/camera_target_point', self.target_callback, 10)

        self.timer = self.create_timer(5.0, self.timer_callback)  # 每5秒触发一次检测

        self.get_logger().info("📸 VLM 图像识别与坐标记录节点启动")

    def image_callback(self, msg):
        if not self.image_received:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite('test.jpg', cv_image)
            self.image_received = True
            self.get_logger().info("✅ 图像保存为 test.jpg")

    def target_callback(self, msg):
        self.target_point = msg

    def timer_callback(self):
        if not self.image_received or self.target_point is None:
            self.get_logger().warn("⚠️ 尚未准备好图像或目标点")
            return

        # 调用大模型处理图像
        result = self.call_doubao()
        if result is None:
            self.get_logger().error("❌ doubao.py 执行失败")
            return

        # 坐标转换
        try:
            target_in_base = self.tf_buffer.transform(self.target_point, 'base_link', timeout=rclpy.duration.Duration(seconds=1.0))
            x = target_in_base.point.x
            y = target_in_base.point.y
            z = target_in_base.point.z
            self.get_logger().info(f"📍 目标在 base_link 坐标系下的位置: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        except Exception as e:
            self.get_logger().error(f"❌ TF坐标转换失败: {str(e)}")
            return

        # 存储结果
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        with open("vlm_log.txt", "a") as f:
            f.write(f"[{timestamp}] 📷 位置: ({x:.2f}, {y:.2f}, {z:.2f})\n")
            for item in result:
                f.write(f"  - {item}\n")
            f.write("\n")
        self.get_logger().info("📝 已记录图像识别结果与坐标信息")

        # 重置标志
        self.image_received = False
        self.target_point = None

    def call_doubao(self):
        try:
            output = subprocess.check_output(["python3", "doubao.py"])
            return json.loads(output)
        except Exception as e:
            self.get_logger().error(f"调用 doubao.py 失败: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = VLMMapperNode()
    rclpy.spin(node)
    rclpy.shutdown()
