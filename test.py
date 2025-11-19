#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import math
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32 as RosInt32
from sensor_msgs.msg import NavSatFix, NavSatStatus


class UiTestPublisher(Node):
    def __init__(self):
        super().__init__("ui_test_publisher")

        # Publishers
        self.pub_asr = self.create_publisher(RosString, "/user_speech", 10)
        self.pub_mode = self.create_publisher(RosInt32, "/behavior_mode", 10)
        self.pub_gps = self.create_publisher(NavSatFix, "/gps_raw", 10)

        # Timer: publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("UI Test Publisher running...")

    def timer_callback(self):
        # ========== (1) ASR Message ==========
        asr_samples = [
            "Hello robot!",
            "Take me to the lake.",
            "What's the weather today?",
            "你好 小白机器人",
            "",
            "I'm hungry",
        ]
        asr_msg = RosString()
        asr_msg.data = random.choice(asr_samples)
        self.pub_asr.publish(asr_msg)
        self.get_logger().info(f"[TEST] Published ASR: {asr_msg.data}")

        # ========== (2) Behavior Mode ==========
        mode = random.choice([0, 1, 2, 3, 4])
        mode_msg = RosInt32()
        mode_msg.data = mode
        self.pub_mode.publish(mode_msg)
        self.get_logger().info(f"[TEST] behavior_mode = {mode}")

        # ========== (3) GPS Raw Fix ==========
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps"

        gps_state = random.choice(["OK", "WEAK", "LOST", "SEARCH"])

        if gps_state == "LOST":
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
            gps_msg.latitude = 0.0
            gps_msg.longitude = 0.0

        elif gps_state == "SEARCH":
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.latitude = 0.0
            gps_msg.longitude = 0.0

        elif gps_state == "WEAK":
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.latitude = 39.123 + random.random() / 100
            gps_msg.longitude = 116.392 + random.random() / 100
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        elif gps_state == "OK":
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.latitude = 39.123 + random.random() / 100
            gps_msg.longitude = 116.392 + random.random() / 100
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
            gps_msg.position_covariance = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]

        self.pub_gps.publish(gps_msg)
        self.get_logger().info(
            f"[TEST] gps = {gps_state}, lat={gps_msg.latitude:.6f}, lon={gps_msg.longitude:.6f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = UiTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

