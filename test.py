#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32 as RosInt32
from std_msgs.msg import Float32 as RosFloat32
from sensor_msgs.msg import NavSatFix, NavSatStatus

from pino_msgs.msg import AudioMSG


class UiTestPublisher(Node):
    def __init__(self):
        super().__init__("ui_test_publisher")

        # Publishers
        self.pub_asr     = self.create_publisher(RosString, "/user_speech", 10)
        self.pub_llm     = self.create_publisher(AudioMSG,  "/audio_cmd", 10)
        self.pub_mode    = self.create_publisher(RosInt32,  "/behavior_mode", 10)
        self.pub_gps     = self.create_publisher(NavSatFix, "/gps_raw", 10)
        self.pub_batt    = self.create_publisher(RosFloat32, "/battery", 10)
        self.pub_target  = self.create_publisher(RosString, "/nav_target", 10)

        # Timer: publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("UI Test Publisher running...")

    def timer_callback(self):

        # ========== (1) ASR Message ==========
        asr_samples = [
            "Hello robot! 🤖",
            "带我去芙蓉湖 🌊",
            "Where is 大雁塔? 🏯",
            "你好 小白机器人 🐶",
            "I'm hungry 🍔",
            "",
        ]
        asr_msg = RosString()
        asr_msg.data = random.choice(asr_samples)
        self.pub_asr.publish(asr_msg)
        self.get_logger().info(f"[TEST] ASR: {asr_msg.data}")

        # ========== (2) LLM Response (AudioMSG) ==========
        llm_samples = [
            "好的，现在带你去芙蓉湖 🌊",
            "正在为你规划路线，请稍候 🗺️",
            "我在这里！需要帮忙吗？ 😊",
            "正在进入自主导航模式 🧭",
            "请跟我来，我带你去看大雁塔！ 🏯",
        ]
        llm_msg = AudioMSG()
        llm_msg.text = random.choice(llm_samples)
        self.pub_llm.publish(llm_msg)
        self.get_logger().info(f"[TEST] LLM: {llm_msg.text}")

        # ========== (3) Behavior Mode ==========
        mode = random.choice([0, 1, 2, 3, 4])
        mode_msg = RosInt32()
        mode_msg.data = mode
        self.pub_mode.publish(mode_msg)
        self.get_logger().info(f"[TEST] Mode: {mode}")

        # ========== (4) GPS Raw Fix ==========
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
            f"[TEST] GPS: {gps_state}, lat={gps_msg.latitude:.6f}, lon={gps_msg.longitude:.6f}"
        )

        # ========== (5) Battery ==========
        batt = RosFloat32()
        batt.data = random.uniform(3.0, 100.0)  # percent
        self.pub_batt.publish(batt)
        self.get_logger().info(f"[TEST] Battery: {batt.data:.1f}%")

        # ========== (6) Navigation Target ==========
        target_samples = [
            "大雁塔",
            "芙蓉湖",
            "南门入口",
            "游客中心",
            "随机巡逻",
            "等待指令",
        ]
        target_msg = RosString()
        target_msg.data = random.choice(target_samples)
        self.pub_target.publish(target_msg)
        self.get_logger().info(f"[TEST] Target: {target_msg.data}")


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

