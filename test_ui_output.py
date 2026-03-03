#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script to publish messages to robot_ui topics.
Run this alongside robot_ui.py to test the UI display.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32, Bool
from sensor_msgs.msg import NavSatFix
from pino_msgs.msg import AudioMSG
import time


class UiTestPublisher(Node):
    def __init__(self):
        super().__init__('ui_test_publisher')

        # Publishers matching robot_ui subscriptions
        self.asr_pub = self.create_publisher(String, '/user_speech', 10)
        self.llm_pub = self.create_publisher(AudioMSG, '/audio_cmd', 10)
        self.mode_pub = self.create_publisher(Int32, '/behavior_mode', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps_raw', 10)
        self.target_pub = self.create_publisher(String, '/nav_target', 10)
        self.language_pub = self.create_publisher(Bool, '/language', 10)

        # Subscribe to button press events (to see UI output)
        self.create_subscription(String, '/ui_button_press', self.on_button, 10)
        self.create_subscription(Int32, '/motion_cmd', self.on_motion_cmd, 10)
        self.create_subscription(Bool, '/language_cmd', self.on_language_cmd, 10)

        self.current_language = False  # False=Chinese, True=English

        self.get_logger().info('UI Test Publisher ready!')
        self.get_logger().info('Commands:')
        self.get_logger().info('  1 - Send ASR message')
        self.get_logger().info('  2 - Send LLM message')
        self.get_logger().info('  3 - Send battery update')
        self.get_logger().info('  4 - Send GPS update')
        self.get_logger().info('  5 - Send mode update')
        self.get_logger().info('  6 - Send nav target')
        self.get_logger().info('  7 - Toggle language (CN/EN)')
        self.get_logger().info('  a - Send all test messages')
        self.get_logger().info('  q - Quit')

    def on_button(self, msg: String):
        self.get_logger().info(f'[UI BUTTON] {msg.data}')

    def on_motion_cmd(self, msg: Int32):
        cmd_names = {
            0: 'STOP', 1: 'STAND', 2: 'SIT',
            3: 'FORWARD', 4: 'BACKWARD', 5: 'TURN_LEFT', 6: 'TURN_RIGHT',
            14: 'DANCE', 16: 'SHAKE_HAND'
        }
        name = cmd_names.get(msg.data, f'UNKNOWN({msg.data})')
        self.get_logger().info(f'[MOTION CMD] {msg.data} ({name})')

    def on_language_cmd(self, msg: Bool):
        lang = "English" if msg.data else "Chinese"
        self.get_logger().info(f'[LANGUAGE CMD] {lang}')
        # Echo back as language confirmation
        self.current_language = msg.data
        self.send_language(msg.data)

    def send_asr(self, text: str = "你好，我想去芙蓉园"):
        msg = String()
        msg.data = text
        self.asr_pub.publish(msg)
        self.get_logger().info(f'Sent ASR: {text}')

    def send_llm(self, text: str = "好的，我来为您介绍大唐芙蓉园。"):
        msg = AudioMSG()
        msg.cmd = 'speak'
        msg.text = text
        msg.voice = 'zf_xiaoyi'
        msg.volume = 1.0
        msg.speed = 1.0
        self.llm_pub.publish(msg)
        self.get_logger().info(f'Sent LLM: {text}')

    def send_battery(self, level: float = 85.0):
        msg = Float32()
        msg.data = level
        self.battery_pub.publish(msg)
        self.get_logger().info(f'Sent Battery: {level}')

    def send_gps(self, status: int = 0, lat: float = 34.2234, lon: float = 108.9456):
        msg = NavSatFix()
        msg.status.status = status
        msg.latitude = lat
        msg.longitude = lon
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        self.gps_pub.publish(msg)
        self.get_logger().info(f'Sent GPS: status={status}, lat={lat}, lon={lon}')

    def send_mode(self, code: int = 2):
        msg = Int32()
        msg.data = code
        self.mode_pub.publish(msg)
        modes = {0: '空闲', 1: '移动中', 2: '交互中', 3: '错误', 4: '待机'}
        self.get_logger().info(f'Sent Mode: {code} ({modes.get(code, "未知")})')

    def send_target(self, target: str = "紫云楼"):
        msg = String()
        msg.data = target
        self.target_pub.publish(msg)
        self.get_logger().info(f'Sent Target: {target}')

    def send_language(self, is_english: bool):
        msg = Bool()
        msg.data = is_english
        self.language_pub.publish(msg)
        lang = "English" if is_english else "Chinese"
        self.get_logger().info(f'Sent Language: {lang}')

    def send_all(self):
        """Send a sequence of test messages"""
        self.get_logger().info('--- Sending all test messages ---')

        self.send_battery(75.0)
        time.sleep(0.2)

        self.send_gps(0, 34.2234, 108.9456)
        time.sleep(0.2)

        self.send_mode(2)
        time.sleep(0.2)

        self.send_asr("请问芙蓉园有什么好玩的？")
        time.sleep(0.5)

        self.send_llm("大唐芙蓉园是西安著名的皇家园林景区。")
        time.sleep(0.5)

        self.send_llm("这里有紫云楼、仕女馆、御苑门等著名景点。")
        time.sleep(0.5)

        self.send_target("紫云楼")

        self.get_logger().info('--- All test messages sent ---')


def main():
    rclpy.init()
    node = UiTestPublisher()

    # Test messages in Chinese
    asr_messages = [
        "你好",
        "请问芙蓉园有什么表演？",
        "我想听一首诗",
        "给我讲个笑话",
        "带我去紫云楼",
    ]

    llm_messages = [
        "您好！我是导游机器人小白，很高兴为您服务。",
        "大唐芙蓉园今天有水幕电影表演，晚上8点开始。",
        "让我为您朗诵一首李白的《静夜思》。",
        "好的，我给您讲个笑话：为什么程序员总是分不清万圣节和圣诞节？因为Oct 31 = Dec 25！",
        "好的，紫云楼在园区东北方向，请跟我来。",
    ]

    asr_idx = 0
    llm_idx = 0

    try:
        while rclpy.ok():
            cmd = input("\nEnter command (1-7, a, q): ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == '1':
                node.send_asr(asr_messages[asr_idx % len(asr_messages)])
                asr_idx += 1
            elif cmd == '2':
                node.send_llm(llm_messages[llm_idx % len(llm_messages)])
                llm_idx += 1
            elif cmd == '3':
                import random
                node.send_battery(random.uniform(20.0, 100.0))
            elif cmd == '4':
                node.send_gps(0, 34.2234, 108.9456)
            elif cmd == '5':
                import random
                node.send_mode(random.randint(0, 4))
            elif cmd == '6':
                targets = ["紫云楼", "仕女馆", "御苑门", "芳林苑"]
                import random
                node.send_target(random.choice(targets))
            elif cmd == '7':
                node.current_language = not node.current_language
                node.send_language(node.current_language)
            elif cmd == 'a':
                node.send_all()
            else:
                print("Unknown command. Use 1-7, a, or q.")

            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
