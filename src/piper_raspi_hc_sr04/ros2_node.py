#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 wheatfox
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from driver import MultiHCSR04Driver
import yaml
import os


class HCSR04Node(Node):
    def __init__(self):
        super().__init__("hc_sr04_node")

        # Load sensor configurations from YAML file in the same directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "sensors.yaml")

        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
                self.sensor_configs = config["sensors"]  # Keep as list
                self.get_logger().info(
                    f"successfully loaded sensor configurations from {config_path}"
                )
        except Exception as e:
            self.get_logger().error(f"failed to load sensor configurations: {str(e)}")
            raise

        self.sensor_publishers = {}
        for sensor in self.sensor_configs:
            sensor_name = sensor["name"]
            topic_name = f"ultrasonic/{sensor_name}"
            self.sensor_publishers[sensor_name] = self.create_publisher(Range, topic_name, 10)
            self.get_logger().info(
                f"created publisher for {sensor_name} on topic {topic_name}"
            )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.driver = MultiHCSR04Driver(self.sensor_configs)
        self.get_logger().info(
            f"HC-SR04 node started with {len(self.sensor_configs)} sensors"
        )

    def timer_callback(self):
        distances = self.driver.get_all_distances()

        for sensor_name, distance in distances.items():
            if distance is not None:
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = sensor_name
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.1
                msg.min_range = 0.02
                msg.max_range = 4.0
                msg.range = float(distance)

                self.sensor_publishers[sensor_name].publish(msg)
                self.get_logger().debug(
                    f"published distance for {sensor_name}: {distance} cm"
                )
            else:
                self.get_logger().warn(
                    f"failed to get distance measurement for {sensor_name}"
                )

    def cleanup(self):
        self.driver.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = HCSR04Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
