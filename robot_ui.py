#!/usr/bin/env python3
import sys, threading
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32 as RosInt32
from std_msgs.msg import Float32 as RosFloat32
from sensor_msgs.msg import NavSatFix

from pino_msgs.msg import AudioMSG   # contains .text


# ============================================================
# Simple Color Panel Widget
# ============================================================
class ColorPanel(QFrame):
    def __init__(self, title_text: str, body_font_size=26, gradient="blue"):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)

        self.title = QLabel(title_text)
        self.title.setFont(QFont("Noto Color Emoji", 18, QFont.Bold))
        self.value = QLabel("—")
        self.value.setFont(QFont("Noto Color Emoji", body_font_size))
        self.value.setWordWrap(True)

        v = QVBoxLayout()
        v.setContentsMargins(14, 10, 14, 10)
        v.setSpacing(6)
        v.addWidget(self.title)
        v.addWidget(self.value)
        self.setLayout(v)

        # Pastel themes
        self.normal_css = {
            "blue": """
                QFrame { border-radius:16px;
                         background:#bcd7ff; color:#101010; }
                QLabel { color:#101010; }
            """,
            "green": """
                QFrame { border-radius:16px;
                         background:#c4f7c2; color:#101010; }
                QLabel { color:#101010; }
            """,
            "yellow": """
                QFrame { border-radius:16px;
                         background:#ffe4b0; color:#101010; }
                QLabel { color:#101010; }
            """,
            "purple": """
                QFrame { border-radius:16px;
                         background:#d6caff; color:#101010; }
                QLabel { color:#101010; }
            """
        }.get(gradient, "blue")

        self.alert_css = """
            QFrame { border-radius:16px; background:#ff4c4c; color:white; }
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
        self.create_subscription(RosFloat32, "/battery",     self.on_battery, 10)
        self.create_subscription(RosString,  "/nav_target",  self.on_target, 10)

        self._behavior_code = 0
        self._gps_state = "UNKNOWN"
        self._any_alert = False

    # ----------------- Callback Handlers -------------------
    def on_asr(self, msg: RosString):
        text = msg.data.strip()
        self.bus.sig_asr.emit(text)

    def on_llm(self, msg: AudioMSG):
        text = msg.text.strip()
        self.bus.sig_llm.emit(text)

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
        # Determine GPS state
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

    # ---------------- Alerts -------------------
    def _update_alerts(self):
        any_alert = (self._behavior_code == 3 or self._gps_state == "LOST")
        if any_alert != self._any_alert:
            self._any_alert = any_alert
            self.bus.sig_alert.emit(any_alert)


# ============================================================
# Main Dashboard UI (NO FACE PANEL)
# ============================================================
class Dashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Dashboard (800x480)")

        # Global app background: gray
        self.setStyleSheet("""
            QWidget { background: #3a3a3a; color:white; }
        """)

        # Panels
        self.panel_mode   = ColorPanel("Robot Mode",    gradient="yellow", body_font_size=28)
        self.panel_gps    = ColorPanel("GPS Status",    gradient="green",  body_font_size=28)
        self.panel_batt   = ColorPanel("Battery",       gradient="purple", body_font_size=28)
        self.panel_asr    = ColorPanel("ASR (User)",    gradient="blue",   body_font_size=24)
        self.panel_llm    = ColorPanel("LLM Response",  gradient="blue",   body_font_size=24)
        self.panel_target = ColorPanel("Navigation Target", gradient="yellow", body_font_size=24)

        # Close button
        self.btn_close = QPushButton("Close")
        self.btn_close.setFont(QFont("Noto Color Emoji", 20, QFont.Bold))
        self.btn_close.setStyleSheet("""
            QPushButton {
                background:#555; border-radius:12px;
                padding:8px 16px; color:white;
            }
            QPushButton:hover { background:#777; }
        """)
        self.btn_close.clicked.connect(self.close)

        # Layout
        layout = QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # Top row: Mode, GPS, Battery
        row1 = QHBoxLayout()
        row1.setSpacing(10)
        row1.addWidget(self.panel_mode, 1)
        row1.addWidget(self.panel_gps, 1)
        row1.addWidget(self.panel_batt, 1)

        # Middle row: ASR + LLM
        row2 = QHBoxLayout()
        row2.setSpacing(10)
        row2.addWidget(self.panel_asr, 1)
        row2.addWidget(self.panel_llm, 1)

        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addWidget(self.panel_target)
        layout.addWidget(self.btn_close, alignment=Qt.AlignCenter)

        self.setLayout(layout)

    # ------------------ Slots ---------------------
    def on_asr(self, text: str):
        self.panel_asr.set_text(text if text else "(empty)")
        self.panel_asr.set_alert(text.strip() == "")

    def on_llm(self, text: str):
        self.panel_llm.set_text(text if text else "(empty)")

    def on_mode(self, code: int):
        mapping = {
            0: "Idle",
            1: "Moving",
            2: "Interacting",
            3: "ERROR",
            4: "Standby"
        }
        txt = mapping.get(code, f"Unknown({code})")
        self.panel_mode.set_text(txt)
        self.panel_mode.set_alert(code == 3)

    def on_gps(self, summary: dict):
        s = summary.get("state", "UNKNOWN")
        hint = summary.get("hint", "")
        self.panel_gps.set_text(f"{s} — {hint}")
        self.panel_gps.set_alert(s == "LOST")

    def on_battery(self, val: float):
        # Force battery to always show 100%
        percent = 100.0
        self.panel_batt.set_text("100% 🔋")
        self.panel_batt.set_alert(False)

    def on_target(self, target: str):
        self.panel_target.set_text(target if target else "(none)")

    def on_any_alert(self, alert: bool):
        # nothing special visually here — panel alerts already handle it
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
    app = QApplication(sys.argv)
    ui = Dashboard()

    rclpy.init(args=None)
    bus = UiBus()
    node = RobotUiNode(bus)

    # Connect Qt slots
    bus.sig_asr.connect(ui.on_asr)
    bus.sig_llm.connect(ui.on_llm)
    bus.sig_mode.connect(ui.on_mode)
    bus.sig_gps.connect(ui.on_gps)
    bus.sig_battery.connect(ui.on_battery)
    bus.sig_target.connect(ui.on_target)
    bus.sig_alert.connect(ui.on_any_alert)

    # Spin ROS in thread
    def spin():
        rclpy.spin(node)
        node.destroy_node()

    threading.Thread(target=spin, daemon=True).start()

    ui.resize(800, 480)
    ui.showFullScreen()

    code = app.exec_()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(code)


if __name__ == "__main__":
    main()
