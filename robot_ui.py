#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, threading, signal
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame, QGridLayout, QTextEdit, QSizePolicy, QScrollArea
)
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32 as RosInt32
from std_msgs.msg import Float32 as RosFloat32
from std_msgs.msg import Bool as RosBool
from sensor_msgs.msg import NavSatFix, Joy

from pino_msgs.msg import AudioMSG   # contains .text


# ============================================================
<<<<<<< HEAD
# Motion Command Mapping (matches motion_server.py)
# ============================================================
MOTION_COMMANDS = {
    "STOP": 0,
    "STAND": 1,
    "SIT": 2,
    # In this UI, FORWARD button enters wandering/navigation prepare mode.
    "FORWARD": 30,
    "HAND_HEART": 11,  # use shake_body instead of hand_heart
    "SPACE_STEP": 12,
    "DANCE1": 14,
    "SHAKE_HAND": 16,  # wave_hand in motion_server.py
    "DANCE2": 19,
    "NAVIGATE_PREPARE": 20,
    "WANDER": 30,
}


# ============================================================
=======
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7
# ✅ FONT SELECTOR (Chinese + Emoji safe)
# ============================================================
def pick_chinese_font():
    """
    Auto select best Chinese-capable font on this system.
    """
    db = QFontDatabase()
    families = db.families()

    PRIORITY = [
        "Noto Sans CJK SC",     # Ubuntu / Jetson
<<<<<<< HEAD
        "Noto Sans CJK TC",
        "WenQuanYi Zen Hei",    # Common Linux
        "WenQuanYi Micro Hei",
        "Droid Sans Fallback",  # Android/Linux fallback
        "AR PL UMing CN",       # Arphic fonts
        "AR PL UKai CN",
=======
        "WenQuanYi Zen Hei",    # Common Linux
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7
        "PingFang SC",         # macOS
        "Hiragino Sans GB",
        "Microsoft YaHei",     # Windows
        "SimHei",
<<<<<<< HEAD
        "SimSun",
        "Source Han Sans SC",
        "Sans"                 # Generic fallback
    ]

    print(f"[FONT] Available font families: {len(families)}")

    # Check for fonts with "Chinese", "CJK", or "Han" in the name
    chinese_fonts = [f for f in families if any(keyword in f for keyword in ["Chinese", "CJK", "Han", "Hei", "Song", "Ming", "Kai"])]
    if chinese_fonts:
        print(f"[FONT] Found potential Chinese fonts: {chinese_fonts[:5]}")

    for f in PRIORITY:
        if f in families:
            print(f"[FONT] ✓ Using Chinese font: {f}")
            return f

    # Last resort: try to find any font with CJK support
    for f in chinese_fonts:
        print(f"[FONT] ✓ Using detected Chinese font: {f}")
        return f

    print("[FONT] ⚠ No Chinese font found! Using 'Sans' fallback.")
    print("[FONT] Install Chinese fonts with: sudo apt-get install fonts-noto-cjk")
    return "Sans"


def pick_emoji_font():
    """
    Find emoji font on the system.
    """
    db = QFontDatabase()
    families = db.families()

    EMOJI_FONTS = [
        "Noto Color Emoji",
        "Apple Color Emoji",
        "Segoe UI Emoji",
        "Noto Emoji",
        "EmojiOne Color",
        "Symbola"
    ]

    for f in EMOJI_FONTS:
        if f in families:
            print(f"[FONT] ✓ Using emoji font: {f}")
            return f

    print("[FONT] ⚠ No emoji font found")
    return None


def create_font_with_emoji(base_font_name: str, size: int, weight: int = QFont.Normal):
    """
    Create a font with emoji fallback support.
    """
    font = QFont(base_font_name, size, weight)

    # Try to add emoji font as fallback
    emoji_font = pick_emoji_font()
    if emoji_font:
        # Set font families list to include emoji fallback
        try:
            font.setFamilies([base_font_name, emoji_font, "Sans"])
        except AttributeError:
            # Older Qt versions don't have setFamilies
            font.setFamily(base_font_name)

    font.setStyleHint(QFont.SansSerif)
    font.setStyleStrategy(QFont.PreferAntialias)
    return font


FONT_CN = None   # resolved in main()
FONT_EMOJI = None  # resolved in main()
=======
        "Source Han Sans SC"
    ]

    for f in PRIORITY:
        if f in families:
            print(f"[FONT] Using Chinese font: {f}")
            return f

    print("[FONT] ⚠ No Chinese font found! Falling back to default system font.")
    return QApplication.font().family()


FONT_CN = None   # resolved in main()
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7


# ============================================================
# Simple Color Panel Widget
# ============================================================
class ColorPanel(QFrame):
    def __init__(self, title_text: str, body_font_size=26, gradient="blue"):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)

        self.title = QLabel(title_text)
<<<<<<< HEAD
        self.title.setFont(create_font_with_emoji(FONT_CN, body_font_size, QFont.Bold))

        self.value = QLabel("—")
        self.value.setFont(create_font_with_emoji(FONT_CN, body_font_size))
=======
        self.title.setFont(QFont(FONT_CN, 18, QFont.Bold))

        self.value = QLabel("—")
        self.value.setFont(QFont(FONT_CN, body_font_size))
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7
        self.value.setWordWrap(True)

        h = QHBoxLayout()
        h.setContentsMargins(12, 8, 12, 8)
        h.setSpacing(6)
        h.addWidget(self.title, stretch=0)
        h.addWidget(self.value, stretch=1)
        self.setLayout(h)

        # Pastel themes
        self.normal_css = {
            "blue": """
                QFrame { border-radius:12px;
                         background:#ffffff; color:#000000;
                         border:1px solid #d6e6ff; }
                QLabel { color:#000000; }
            """,
            "green": """
                QFrame { border-radius:12px;
                         background:#ffffff; color:#000000;
                         border:1px solid #d8f0dd; }
                QLabel { color:#000000; }
            """,
            "yellow": """
                QFrame { border-radius:12px;
                         background:#ffffff; color:#000000;
                         border:1px solid #f2e4bf; }
                QLabel { color:#000000; }
            """,
            "purple": """
                QFrame { border-radius:12px;
                         background:#ffffff; color:#000000;
                         border:1px solid #e6def7; }
                QLabel { color:#000000; }
            """
        }.get(gradient, "blue")

        self.alert_css = """
            QFrame { border-radius:12px; background:#ff4c4c; color:white; }
            QLabel { color:white; }
        """

        self.setStyleSheet(self.normal_css)

    def set_text(self, text: str):
        self.value.setText(text)

    def set_alert(self, on: bool):
        self.setStyleSheet(self.alert_css if on else self.normal_css)


# ============================================================
# Signal Bus (Thread-Safe for Qt)
# ============================================================
class UiBus(QObject):
    sig_asr     = pyqtSignal(str)
    sig_llm     = pyqtSignal(str)
    sig_mode    = pyqtSignal(int)
    sig_gps     = pyqtSignal(object)
    sig_battery = pyqtSignal(float)
    sig_target  = pyqtSignal(str)
    sig_alert   = pyqtSignal(bool)
    sig_language = pyqtSignal(bool)  # False=Chinese, True=English


# ============================================================
# ROS2 Node
# ============================================================
class RobotUiNode(Node):
    def __init__(self, bus: UiBus):
        super().__init__("robot_dashboard_ui_node")
        self.bus = bus

        self.create_subscription(RosString,  "/user_speech", self.on_asr, 10)
        self.create_subscription(AudioMSG,   "/audio_cmd",   self.on_llm, 10)
        self.create_subscription(RosInt32,   "/behavior_mode", self.on_mode, 10)
        self.create_subscription(NavSatFix,  "/gps_raw",     self.on_gps, 10)
        self.create_subscription(RosFloat32, "battery_level", self.on_battery, 10)
        self.create_subscription(RosString,  "/nav_target",  self.on_target, 10)
        self.create_subscription(RosBool,    "/language",    self.on_language, 10)

        # Publisher for button events (debug)
        self.button_pub = self.create_publisher(RosString, "/ui_button_press", 10)

        # Publishers for motion control
        self.motion_cmd_pub = self.create_publisher(RosInt32, "motion_cmd", 10)
        self.local_joystick_pub = self.create_publisher(Joy, "local_joystick", 10)
        self.enable_local_joystick = True
        self._local_joystick_blocked_logged = False

        # Publisher for language command (False=Chinese, True=English)
        self.language_cmd_pub = self.create_publisher(RosBool, "/language_cmd", 10)

        self._behavior_code = 0
        self._gps_state = "UNKNOWN"
        self._any_alert = False

    def publish_button_event(self, button_name: str):
        """Publish button press event to ROS2 topic (debug)"""
        msg = RosString()
        msg.data = f"{button_name} button is pressing"
        self.button_pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def publish_motion_cmd(self, cmd: int):
        """Publish motion command to /motion_cmd topic"""
        msg = RosInt32()
        msg.data = cmd
        self.motion_cmd_pub.publish(msg)
        self.get_logger().info(f"Motion command: {cmd}")

    def publish_local_joystick(self, x: float = 0.0, y: float = 0.0):
        """Publish joystick command to /local_joystick topic"""
        if not self.enable_local_joystick:
            if not self._local_joystick_blocked_logged:
                self.get_logger().info("Local joystick publishing disabled in robot_ui")
                self._local_joystick_blocked_logged = True
            return
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [x, y]
        msg.buttons = []
        self.local_joystick_pub.publish(msg)

    def publish_language_cmd(self, is_english: bool):
        """Publish language command (False=Chinese, True=English)"""
        msg = RosBool()
        msg.data = is_english
        self.language_cmd_pub.publish(msg)
        lang_str = "English" if is_english else "Chinese"
        self.get_logger().info(f"Language command: {lang_str}")

    # ----------------- Callback Handlers -------------------
    def on_asr(self, msg: RosString):
        text = msg.data.strip()
        self.bus.sig_asr.emit(text)

    def on_llm(self, msg: AudioMSG):
        text = msg.text.strip()
        self.bus.sig_llm.emit(text)

    def on_language(self, msg: RosBool):
        """Receive language setting from ASR (False=Chinese, True=English)"""
        self.bus.sig_language.emit(msg.data)

    def on_mode(self, msg: RosInt32):
        code = int(msg.data)
        self._behavior_code = code
        self.bus.sig_mode.emit(code)
        self._update_alerts()

    def on_battery(self, msg: RosFloat32):
        self.bus.sig_battery.emit(float(msg.data))

    def on_target(self, msg: RosString):
        self.bus.sig_target.emit(msg.data.strip())

    def on_gps(self, msg: NavSatFix):
        status_code = int(getattr(msg.status, "status", 0))
        lat = msg.latitude
        lon = msg.longitude

        if status_code < 0:
            state, hint = "LOST", "No fix"
        elif lat == 0.0 and lon == 0.0:
            state, hint = "SEARCH", "Waiting for coordinates"
        else:
            cov = int(getattr(msg, "position_covariance_type", 0))
            if cov in (NavSatFix.COVARIANCE_TYPE_KNOWN,
                       NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN):
                state, hint = "OK", "Fix good"
            else:
                state, hint = "WEAK", "Low confidence"

        self._gps_state = state
        self.bus.sig_gps.emit({
            "state": state,
            "hint": hint,
            "lat": lat,
            "lon": lon
        })
        self._update_alerts()

    def _update_alerts(self):
        any_alert = (self._behavior_code == 3 or self._gps_state == "LOST")
        if any_alert != self._any_alert:
            self._any_alert = any_alert
            self.bus.sig_alert.emit(any_alert)


# ============================================================
# Main Dashboard UI
# ============================================================
class Dashboard(QWidget):
    def __init__(self, node=None, scale=1.0):
        super().__init__()
        self.setWindowTitle("Robot Dashboard")
        self._s = scale  # scale factor relative to 800x480 baseline
        self.node = node
        self.is_chinese = True  # Default to Chinese
        self.is_running = False  # Track start/stop state

<<<<<<< HEAD
        # Button label mappings
        self.button_labels = {
            "HAND_HEART": {"en": "🫨 SHAKE BODY", "cn": "🫨 摇摆"},
            "FORWARD": {"en": "⬆️ FORWARD", "cn": "⬆️ 前进"},
            "SHAKE_HAND": {"en": "🤝 SHAKE HAND", "cn": "🤝 握手"},
            "SIT": {"en": "🧎 SIT", "cn": "🧎 坐下"},
            "STOP": {"en": "🛑 STOP", "cn": "🛑 停止"},
            "STAND": {"en": "🧍 STAND", "cn": "🧍 站立"},
            "SPACE_STEP": {"en": "🚶 SPACE STEP", "cn": "🚶 太空步"},
            "DANCE1": {"en": "💃 DANCE1", "cn": "💃 舞蹈1"},
            "DANCE2": {"en": "🕺 DANCE2", "cn": "🕺 舞蹈2"}
        }

        # Define button types
        self.action_buttons = {"HAND_HEART", "SHAKE_HAND", "SIT", "STOP", "STAND", "SPACE_STEP", "DANCE1", "DANCE2"}
        self.movement_buttons = set()

        # Timer management for buttons
        self.movement_timer = None   # Single 10Hz timer for movement publishing
        self.button_pressed = {}     # Track which buttons are currently pressed

        # Don't set font-family in stylesheet to avoid overriding the Chinese font
=======
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7
        self.setStyleSheet("""
            QWidget { background: #ffffff; color:#000000; }
            QPushButton {
                background:#ffffff;
                color:#000000;
                border:2px solid #b5c8e8;
                border-radius:8px;
                padding:8px;
                text-align:center;
            }
            QPushButton[button_key="STOP"] {
                background:#ff4a4a;
                color:#ffffff;
                font-weight:bold;
                border:2px solid #cc2f2f;
            }
            QPushButton:pressed { background:#eef5ff; }
        """)

<<<<<<< HEAD
        # Header items
        self.label_batt = QLabel("🔋 --%")
        self.label_lang = QPushButton("中 / EN")
        self.label_lang.setStyleSheet("QPushButton { background:transparent; border:none; color:#4CAF50; font-weight:bold; }")
        self.label_lang.clicked.connect(self.toggle_language)
        self.label_mode = QLabel("ROBOT MODE")
        self.label_gps = QLabel("🛰 GPS: UNKNOWN")

        small_font = create_font_with_emoji(FONT_CN, int(12 * self._s))
        self.label_batt.setFont(small_font)
        self.label_lang.setFont(small_font)
        self.label_mode.setFont(small_font)
        self.label_gps.setFont(small_font)

        # FORWARD button in header
        self.buttons = {}
        self.forward_btn = QPushButton(self.button_labels["FORWARD"]["cn"])
        self.forward_btn.setFont(create_font_with_emoji(FONT_CN, int(12 * self._s)))
        self.forward_btn.setProperty("button_key", "FORWARD")
        self.forward_btn.pressed.connect(lambda k="FORWARD": self.on_button_press(k))
        self.forward_btn.released.connect(lambda k="FORWARD": self.on_button_release(k))
        self.buttons["FORWARD"] = self.forward_btn

        header = QHBoxLayout()
        header.setSpacing(1)
        header.addWidget(self.label_batt, alignment=Qt.AlignLeft)
        header.addStretch(1)
        header.addWidget(self.forward_btn, alignment=Qt.AlignCenter)
        header.addStretch(1)
        header.addWidget(self.label_lang, alignment=Qt.AlignCenter)
        header.addStretch(1)
        header.addWidget(self.label_mode, alignment=Qt.AlignCenter)
        header.addStretch(1)
        header.addWidget(self.label_gps, alignment=Qt.AlignRight)

        # ASR panel
        self.panel_asr = ColorPanel("🎤", body_font_size=int(18 * self._s), gradient="blue")
        self.panel_asr.setMinimumHeight(int(66 * self._s))
        self.panel_asr.setMaximumHeight(int(80 * self._s))

        # LLM response (scrollable)
        self.llm_frame = QFrame()
        self.llm_frame.setStyleSheet("QFrame { border-radius:12px; background:#ffffff; border:1px solid #d6e6ff; }")
        self.llm_frame.setMinimumHeight(int(66 * self._s))
        self.llm_frame.setMaximumHeight(int(80 * self._s))
        llm_layout = QHBoxLayout()
        llm_layout.setContentsMargins(12, 8, 12, 8)
        llm_layout.setSpacing(6)
        self.llm_title = QLabel("🤖")
        self.llm_title.setFont(create_font_with_emoji(FONT_CN, int(18 * self._s), QFont.Bold))
        self.llm_text = QTextEdit()
        self.llm_text.setReadOnly(True)
        self.llm_text.setFont(create_font_with_emoji(FONT_CN, int(14 * self._s)))
        self.llm_text.setStyleSheet("QTextEdit { background: transparent; border: none; color: #101010; }")
        self.llm_text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        llm_layout.addWidget(self.llm_title, stretch=0)
        llm_layout.addWidget(self.llm_text, stretch=1)
        self.llm_frame.setLayout(llm_layout)
=======
        self.panel_mode   = ColorPanel("Robot Mode",    gradient="yellow", body_font_size=28)
        self.panel_gps    = ColorPanel("GPS Status",    gradient="green",  body_font_size=28)
        self.panel_batt   = ColorPanel("Battery",       gradient="purple", body_font_size=28)
        self.panel_asr    = ColorPanel("ASR (用户)",    gradient="blue",   body_font_size=24)
        self.panel_llm    = ColorPanel("LLM 回复",      gradient="blue",   body_font_size=24)
        self.panel_target = ColorPanel("目标点",        gradient="yellow", body_font_size=24)

        layout = QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        row1 = QHBoxLayout()
        row1.setSpacing(10)
        row1.addWidget(self.panel_mode, 1)
        row1.addWidget(self.panel_gps, 1)
        row1.addWidget(self.panel_batt, 1)

        row2 = QHBoxLayout()
        row2.setSpacing(10)
        row2.addWidget(self.panel_asr, 1)
        row2.addWidget(self.panel_llm, 1)

        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addWidget(self.panel_target)
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7

        middle_row = QHBoxLayout()
        middle_row.setSpacing(1)
        middle_row.addWidget(self.panel_asr, stretch=1)
        middle_row.addWidget(self.llm_frame, stretch=1)

        middle_row_container = QFrame()
        middle_row_container.setLayout(middle_row)
        middle_row_container.setMinimumHeight(int(66 * self._s))
        middle_row_container.setMaximumHeight(int(80 * self._s))

        # Buttons grid (STOP spans full center column)
        btn_grid = QGridLayout()
        btn_grid.setSpacing(1)
        btn_grid.setColumnStretch(0, 1)
        btn_grid.setColumnStretch(1, 2)
        btn_grid.setColumnStretch(2, 1)
        btn_grid.setRowStretch(0, 1)
        btn_grid.setRowStretch(1, 1)
        btn_grid.setRowStretch(2, 1)

        # Grid layout (STOP spans 2 rows in center):
        # Row 0: HAND_HEART  | STOP (spans 2 rows) | SHAKE_HAND
        # Row 1: SIT         |                      | STAND
        # Row 2: SPACE_STEP  | DANCE1               | DANCE2
        grid_buttons = {
            "HAND_HEART":  (0, 0, 1, 1),
            "STOP":        (0, 1, 2, 1),  # rowSpan=2
            "SHAKE_HAND":  (0, 2, 1, 1),
            "SIT":         (1, 0, 1, 1),
            "STAND":       (1, 2, 1, 1),
            "SPACE_STEP":  (2, 0, 1, 1),
            "DANCE1":      (2, 1, 1, 1),
            "DANCE2":      (2, 2, 1, 1),
        }

        print(f"[DEBUG] Creating buttons with font: {FONT_CN}")
        print(f"[DEBUG] Sample button label: {self.button_labels['SIT']['cn']}")

        for key, (row, col, rowspan, colspan) in grid_buttons.items():
            label_text = self.button_labels[key]["cn"]
            btn = QPushButton(label_text)
            btn.setFont(create_font_with_emoji(FONT_CN, int(18 * self._s)))
            if key == "STOP":
                btn.setMinimumWidth(int(220 * self._s))
                btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setMinimumHeight(int(70 * self._s))
            btn.setProperty("button_key", key)
            btn.pressed.connect(lambda k=key: self.on_button_press(k))
            btn.released.connect(lambda k=key: self.on_button_release(k))
            btn_grid.addWidget(btn, row, col, rowspan, colspan)
            self.buttons[key] = btn

        # Main layout assembly
        main = QVBoxLayout()
        main.setContentsMargins(1, 1, 1, 1)
        main.setSpacing(1)
        main.addLayout(header)
        main.addWidget(middle_row_container, stretch=0)

        btn_container = QFrame()
        btn_container.setLayout(btn_grid)
        main.addWidget(btn_container, stretch=2)

        self.setLayout(main)

    def toggle_language(self):
        """Toggle between Chinese and English and publish command"""
        self.is_chinese = not self.is_chinese
        self._update_ui_language()

        # Publish language command to ASR (False=Chinese, True=English)
        if self.node:
            self.node.publish_button_event("LANGUAGE")
            self.node.publish_language_cmd(not self.is_chinese)
        else:
            print("[WARN] Language toggle pressed but ROS node is not available")

    def _update_ui_language(self):
        """Update UI elements based on current language setting"""
        lang = "cn" if self.is_chinese else "en"

        # Update all button labels
        for key, btn in self.buttons.items():
            btn.setText(self.button_labels[key][lang])

        # Update language indicator
        if self.is_chinese:
            self.label_lang.setText("中文")
        else:
            self.label_lang.setText("EN")

    def on_language(self, is_english: bool):
        """Handle language change from ASR (False=Chinese, True=English)"""
        self.is_chinese = not is_english
        self._update_ui_language()

    def on_button_press(self, button_key):
        """Handle button press - visual feedback and start action/movement"""
        btn = self.buttons[button_key]
        self.button_pressed[button_key] = True

        # Visual feedback - make button darker
        btn.setProperty("pressed_state", "true")
        btn.style().unpolish(btn)
        btn.style().polish(btn)
        if button_key == "STOP":
            btn.setStyleSheet("QPushButton { background:#c0392b; color:#ffffff; border-radius:8px; padding:8px; }")
        else:
            btn.setStyleSheet("QPushButton { background:#808080; color:#101010; border-radius:8px; padding:8px; }")

        # Debug publish
        if self.node:
            self.node.publish_button_event(button_key)

        # Forward button switches robot to wander mode immediately.
        if button_key == "FORWARD" and self.node:
            self.node.publish_motion_cmd(MOTION_COMMANDS["FORWARD"])
            return

        # Handle movement buttons (continuous at 10Hz while pressed)
        if button_key in self.movement_buttons:
            # Send initial movement command immediately
            self._send_movement()

            # Create or restart 10Hz timer for continuous sending
            if self.movement_timer is None:
                self.movement_timer = QTimer()
                self.movement_timer.timeout.connect(self._send_movement)
                self.movement_timer.start(100)  # 10Hz = every 100ms

    def on_button_release(self, button_key):
        """Handle button release - stop movement and restore visual"""
        btn = self.buttons[button_key]
        self.button_pressed[button_key] = False

        # Visual feedback - restore normal color
        btn.setProperty("pressed_state", "false")
        btn.style().unpolish(btn)
        btn.style().polish(btn)
        btn.setStyleSheet("")  # Reset to parent stylesheet

        # Stop movement timer and send neutral joystick when no movement buttons pressed
        if button_key in self.movement_buttons:
            if not self._any_movement_pressed():
                if self.movement_timer:
                    self.movement_timer.stop()
                    self.movement_timer = None
                if self.node:
                    self.node.publish_local_joystick(0.0, 0.0)
            else:
                self._send_movement()

        # Trigger action buttons on release
        if button_key in self.action_buttons:
            self._execute_action(button_key)

    def _execute_action(self, button_key):
        """Execute action command after delay"""
        if not self.node:
            return

        if button_key == "SIT":
            self.node.publish_motion_cmd(MOTION_COMMANDS["SIT"])
        elif button_key == "HAND_HEART":
            self.node.publish_motion_cmd(MOTION_COMMANDS["HAND_HEART"])
        elif button_key == "STAND":
            self.node.publish_motion_cmd(MOTION_COMMANDS["STAND"])
        elif button_key == "SPACE_STEP":
            self.node.publish_motion_cmd(MOTION_COMMANDS["SPACE_STEP"])
        elif button_key == "DANCE1":
            self.node.publish_motion_cmd(MOTION_COMMANDS["DANCE1"])
        elif button_key == "DANCE2":
            self.node.publish_motion_cmd(MOTION_COMMANDS["DANCE2"])
        elif button_key == "SHAKE_HAND":
            self.node.publish_motion_cmd(MOTION_COMMANDS["SHAKE_HAND"])
        elif button_key == "STOP":
            # Toggle start/stop
            self.is_running = not self.is_running
            if self.is_running:
                self.node.publish_motion_cmd(MOTION_COMMANDS["STAND"])
            else:
                self.node.publish_motion_cmd(MOTION_COMMANDS["STOP"])

    def _any_movement_pressed(self):
        return any(self.button_pressed.get(k, False) for k in self.movement_buttons)

    def _send_movement(self):
        """Send movement command (called at 10Hz while any movement button pressed)"""
        if not self.node:
            return

        forward = self.button_pressed.get("FORWARD", False)
        backward = self.button_pressed.get("BACKWARD", False)
        left = self.button_pressed.get("TURN_LEFT", False)
        right = self.button_pressed.get("TURN_RIGHT", False)

        if forward and not backward:
            x = 0.5
        elif backward and not forward:
            x = -0.5
        else:
            x = 0.0

        if left and not right:
            y = -0.5
        elif right and not left:
            y = 0.5
        else:
            y = 0.0
        if( abs(x) + abs(y) < 0.5 ):
            return

        self.node.publish_local_joystick(x, y)

    def on_asr(self, text: str):
        self.panel_asr.set_text(text if text else "(空)")
<<<<<<< HEAD

    def on_llm(self, text: str):
        # append and keep scroll at bottom
        if not text:
            return
        cur = self.llm_text.toPlainText()
        if cur:
            cur += "\n\n"
        cur += text
        self.llm_text.setPlainText(cur)
        self.llm_text.moveCursor(self.llm_text.textCursor().End)
=======
        self.panel_asr.set_alert(text.strip() == "")

    def on_llm(self, text: str):
        self.panel_llm.set_text(text if text else "(空)")
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7

    def on_mode(self, code: int):
        mapping = {
            0: "空闲",
            1: "移动中",
            2: "交互中",
            3: "错误",
            4: "待机"
        }
        txt = mapping.get(code, f"未知({code})")
<<<<<<< HEAD
        self.label_mode.setText(f"ROBOT MODE: {txt}")
=======
        self.panel_mode.set_text(txt)
        self.panel_mode.set_alert(code == 3)
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7

    def on_gps(self, summary: dict):
        s = summary.get("state", "UNKNOWN")
        hint = summary.get("hint", "")
        self.label_gps.setText(f"🛰 GPS: {s} — {hint}")

    def on_battery(self, val: float):
<<<<<<< HEAD
        if val is None:
            self.label_batt.setText("🔋 --")
            return
        self.label_batt.setText(f"🔋 {val:.1f}")

    def on_target(self, target: str):
        # show in ASR/target panel small suffix
        if target:
            existing = self.panel_asr.value.text()
            self.panel_asr.set_text(existing + f"\n目标: {target}")

    def on_any_alert(self, alert: bool):
        # Do not change background on alerts
=======
        self.panel_batt.set_text("100% 🔋")
        self.panel_batt.set_alert(False)

    def on_target(self, target: str):
        self.panel_target.set_text(target if target else "(无目标)")

    def on_any_alert(self, alert: bool):
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7
        pass

    def closeEvent(self, event):
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return super().closeEvent(event)


# ============================================================
# MAIN
# ============================================================
def main():
<<<<<<< HEAD
    global FONT_CN, FONT_EMOJI

    app = QApplication(sys.argv)
    FONT_CN = pick_chinese_font()
    FONT_EMOJI = pick_emoji_font()

    # Set application-wide font to support Chinese characters and emojis
    app_font = create_font_with_emoji(FONT_CN, 12)
    app.setFont(app_font)
    print(f"[FONT] Application font set to: {FONT_CN} with emoji support")

    # Setup signal handlers for Ctrl+C
    def signal_handler(sig, frame):
        print("\n[INFO] Received interrupt signal, shutting down...")
        app.quit()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Compute scale factor relative to 800x480 baseline
    screen = app.primaryScreen().size()
    scale = min(screen.width() / 800.0, screen.height() / 480.0)
    print(f"[UI] Screen: {screen.width()}x{screen.height()}, scale: {scale:.2f}")
=======
    global FONT_CN

    app = QApplication(sys.argv)
    FONT_CN = pick_chinese_font()

    ui = Dashboard()
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7

    rclpy.init(args=None)
    bus = UiBus()
    node = RobotUiNode(bus)

<<<<<<< HEAD
    ui = Dashboard(node=node, scale=scale)

=======
>>>>>>> fc4351a8c49dda2d485b32d4f02bb00ff28c05e7
    bus.sig_asr.connect(ui.on_asr)
    bus.sig_llm.connect(ui.on_llm)
    bus.sig_mode.connect(ui.on_mode)
    bus.sig_gps.connect(ui.on_gps)
    bus.sig_battery.connect(ui.on_battery)
    bus.sig_target.connect(ui.on_target)
    bus.sig_alert.connect(ui.on_any_alert)
    bus.sig_language.connect(ui.on_language)

    def spin():
        rclpy.spin(node)
        node.destroy_node()

    threading.Thread(target=spin, daemon=True).start()

    # Create a timer to allow Python to process signals
    # This is necessary for Ctrl+C to work properly with PyQt5
    timer = QTimer()
    timer.start(500)  # Fire every 500ms
    timer.timeout.connect(lambda: None)  # Do nothing, just let Python process signals

    ui.resize(screen.width(), screen.height())
    ui.showFullScreen()

    code = app.exec_()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(code)


if __name__ == "__main__":
    main()
