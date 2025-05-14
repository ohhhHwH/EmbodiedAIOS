import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Empty          # 触发器消息类型，可替换为你需要的类型

import tf2_ros
import tf2_geometry_msgs
import subprocess
import json
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
import s3img
# 通过 pip install volcengine-python-sdk[ark] 安装方舟SDK
from volcenginesdkarkruntime import Ark
from typing import Dict, List
from piper_msgs.msg import AllObjectPos


# 替换 <Model> 为模型的Model ID
vlmmodel="doubao-1.5-vision-pro-32k-250115"


#  @TODO 设计一个服务，如果收到vlm识别的请求，就读取摄像头数据，和当前的摄像头给的目标点的坐标，然后返回当前的一些内容，并且将他们的坐标锚定到目标点坐标附近（or直接给目标点坐标

class VLMMapperNode(Node):
    def __init__(self):
        super().__init__('vlm_mapper')
        self.bridge = CvBridge()
        self.image_received = False
        self.target_point = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)

        # 初始化Ark客户端，从环境变量中读取您的API Key
        self.vlmclient = Ark(api_key=os.getenv('ARK_API_KEY'), )

        # ① 订阅目标检测结果
        # 存放最近一次接收到的解析结果
        self.latest_data: Dict[str, Any] | None = None

        # 创建订阅；callback 指向类方法 self.parse_object_points
        self.subscription = self.create_subscription(
            AllObjectPos,
            "/piper_vision/all_object_points",
            self.parse_object_points,  # <—— callback
            qos_profile=10,
        )

        # ② 订阅“触发器”话题 —— 收到就调用 self.on_trigger
        self.trigger_sub = self.create_subscription(
            Empty,                        # 也可以用 Bool、String 等
            "/detection_trigger",         # 发布方决定的触发话题名
            self.on_trigger,              # 收到触发消息就执行检测逻辑
            qos_profile=10,
        )


        self.get_logger().info("📸 VLM 图像识别与坐标记录节点启动")


    # ---------------------- 回调 ----------------------
    def parse_object_points(self, msg: AllObjectPos) -> None:
        """把消息解析为 dict 并写入 self.latest_data"""

        # 时间戳转为可读格式
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        time_str = datetime.fromtimestamp(t).isoformat(timespec="milliseconds")

        objs: List[Dict[str, Any]] = []
        for i, name in enumerate(msg.names):
            p: Point = msg.points[i]
            w = msg.widths[i] if i < len(msg.widths) else None
            h = msg.heights[i] if i < len(msg.heights) else None
            objs.append(
                {
                    "name": name,
                    "position": {"x": p.x, "y": p.y, "z": p.z},
                    "size": {"width": w, "height": h},
                }
            )

        # 缓存
        self.latest_data = {
            "time": time_str,
            "frame": msg.header.frame_id,
            "objects": objs,
        }

        # 可选：打印或做其他处理
        self.get_logger().debug(f"最新数据已更新: {self.latest_data}")



    # ---------- 响应拍照上传给vlm来进行检测 ----------
    def on_trigger(self, _msg):
        """
        当收到触发器话题时执行。这里复用原先的 timer_callback 逻辑。
        """
        if not self.image_received or self.latest_data is None:
            self.get_logger().warn("还没收到任何目标数据，忽略本次触发")
            return

        # 调用大模型处理图像
        result = self.call_doubao()
        if result is None:
            self.get_logger().error("❌ doubao vlm 执行失败")
            return


        # 坐标转换
        #  @TODO 需要转换为世界坐标，而不是base_link
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
        self.get_logger().info(f"🎯 触发检测，当前缓存数据:\n{self.latest_data}")



    def image_callback(self, msg):
        if not self.image_received:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            now_time = datetime.now()
            cv2.imwrite(f'{now_time}-pic.jpg', cv_image)
            self.image_received = True
            self.get_logger().info(f"✅ 图像保存为 {now_time}-pic.jpg")


    def call_doubao(self):
        try:
            # 创建一个对话请求
            response = self.vlmclient.chat.completions.create(
                # 指定您部署了视觉理解大模型的推理接入点ID
                model=vlmmodel,
                messages=[
                    {
                        # 指定消息的角色为用户
                        "role": "user",
                        "content": [
                            # 文本消息，希望模型根据图片信息回答的问题
                            {"type": "text", "text": "你是一个智能楼宇测绘员，请提炼出这张照片中的物品、门牌号、以及所有有价值的信息。以list的形式返回给我"},
                            # 图片信息，希望模型理解的图片
                            {"type": "image_url", "image_url": {"url": f"{s3img.upload_file()}"}
                             },
                        ],
                    }
                ],
            )

            print(response.choices[0].message.content)
            output = response.choices[0].message.content
            return json.loads(output)
        except Exception as e:
            self.get_logger().error(f"调用 doubao.py 失败: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = VLMMapperNode()
    rclpy.spin(node)
    rclpy.shutdown()
