#!/usr/bin/env python3
import sys, threading
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame
)
from PyQt5.QtGui import QFont, QMovie
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32 as RosInt32
from sensor_msgs.msg import NavSatFix


# =========================
# Files (white-on-black visor GIFs)
# =========================
FACE_GIFS = {
    "IDLE":   "ld_idle.gif",
    "HAPPY":  "ld_happy.gif",
    "LISTEN": "ld_listen.gif",
    "SAD":    "ld_sad.gif",
    "SLEEP":  "ld_sleep.gif",
}

# Behavior mapping: /behavior_mode (Int32)
# 0 idle, 1 moving, 2 interacting/listening, 3 error, 4 sleep
BEHAVIOR_TO_FACE = {
    0: ("IDLE",   "IDLE"),
    1: ("MOVING", "HAPPY"),
    2: ("INTERACT", "LISTEN"),
    3: ("ERROR", "SAD"),
    4: ("STANDBY", "SLEEP"),
}


# =========================
# Simple colored panel widget
# =========================
class ColorPanel(QFrame):
    def __init__(self, title_text: str, body_font_size=26, gradient="blue"):
        super().__init__()
        self.setFrameShape(QFrame.NoFrame)

        self.title = QLabel(title_text)
        self.title.setFont(QFont("Arial", 18, QFont.Bold))
        self.value = QLabel("—")
        self.value.setFont(QFont("Arial", body_font_size))
        self.value.setWordWrap(True)

        v = QVBoxLayout()
        v.setContentsMargins(14, 10, 14, 10)
        v.setSpacing(6)
        v.addWidget(self.title)
        v.addWidget(self.value)
        self.setLayout(v)

        # Pastel gradients (normal) and red (alert)
        self.normal_css = {
            "blue":  """
                QFrame { border-radius:16px;
                         background: qlineargradient(x1:0,y1:0,x2:1,y2:1,
                           stop:0 #8EC5FC, stop:1 #E0C3FC); color: #101010; }
                QLabel { color:#101010; }
            """,
            "green": """
                QFrame { border-radius:16px;
                         background: qlineargradient(x1:0,y1:0,x2:1,y2:1,
                           stop:0 #a1ffce, stop:1 #faffd1); color: #101010; }
                QLabel { color:#101010; }
            """,
            "yellow":"""
                QFrame { border-radius:16px;
                         background: qlineargradient(x1:0,y1:0,x2:1,y2:1,
                           stop:0 #f6d365, stop:1 #fda085); color: #101010; }
                QLabel { color:#101010; }
            """
        }[gradient]

        self.alert_css = """
            QFrame { border-radius:16px; background: #ff4c4c; color: white; }
            QLabel { color: white; }
        """

        self.setStyleSheet(self.normal_css)

    def set_text(self, text: str):
        self.value.setText(text)

    def set_alert(self, on: bool):
        self.setStyleSheet(self.alert_css if on else self.normal_css)


# =========================
# Signals to cross threads
# =========================
class UiBus(QObject):
    sig_asr   = pyqtSignal(str)
    sig_mode  = pyqtSignal(int)
    sig_gps   = pyqtSignal(object)  # dict summary
    sig_alert = pyqtSignal(bool)    # any alert active?


# =========================
# ROS2 node (background)
# =========================
class RobotUiNode(Node):
    def __init__(self, bus: UiBus):
        super().__init__("robot_dashboard_ui_node")
        self.bus = bus

        self.create_subscription(RosString, "/user_speech", self.on_asr, 10)
        self.create_subscription(RosInt32,  "/behavior_mode", self.on_mode, 10)
        self.create_subscription(NavSatFix, "/gps_raw", self.on_gps, 10)

        self._any_alert = False
        self._behavior_code = 0
        self._gps_state = "UNKNOWN"

    def _update_any_alert(self):
        any_alert = (self._behavior_code == 3) or (self._gps_state == "LOST")
        if any_alert != self._any_alert:
            self._any_alert = any_alert
            self.bus.sig_alert.emit(any_alert)

    def on_asr(self, msg: RosString):
        text = msg.data.strip() or "(empty)"
        self.bus.sig_asr.emit(text)

    def on_mode(self, msg: RosInt32):
        self._behavior_code = int(msg.data)
        self.bus.sig_mode.emit(self._behavior_code)
        self._update_any_alert()

    def on_gps(self, msg: NavSatFix):
        # status.status: -1 = NO_FIX, 0 = FIX (impl-specific)
        status_code = int(getattr(msg.status, "status", 0))
        lat, lon = msg.latitude, msg.longitude

        if status_code < 0:
            state, hint = "LOST", "No fix"
        elif (lat == 0.0 and lon == 0.0):
            state, hint = "SEARCH", "Waiting for coords"
        else:
            cov_type = int(getattr(msg, "position_covariance_type", 0))
            if cov_type in (NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
                            NavSatFix.COVARIANCE_TYPE_KNOWN):
                state, hint = "OK", "Fix good"
            else:
                state, hint = "WEAK", "Unknown covariance"

        self._gps_state = state
        summary = {"state": state, "hint": hint, "lat": lat, "lon": lon}
        self.bus.sig_gps.emit(summary)
        self._update_any_alert()


# =========================
# Main Dashboard UI
# =========================
class Dashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Dashboard (800x480)")

        # App background (subtle)
        self.setStyleSheet("""
            QWidget { background: qlineargradient(x1:0,y1:0,x2:1,y2:1,
                         stop:0 #a18cd1, stop:1 #fbc2eb); }
        """)

        # ===== Face area (pure black, centered, white-only animation)
        self.face_frame = QFrame()
        self.face_frame.setStyleSheet("""
            QFrame { background: #000000; border-radius: 16px; }
        """)
        self.face = QLabel()
        self.face.setAlignment(Qt.AlignCenter)
        self.face.setMinimumHeight(230)  # big on 800x480
        self._movie = None
        v_face = QVBoxLayout(self.face_frame)
        v_face.setContentsMargins(10, 10, 10, 10)
        v_face.addWidget(self.face)
        self.set_face(FACE_GIFS["IDLE"])

        # ===== Info panels
        self.panel_status = ColorPanel("Robot Status", gradient="yellow", body_font_size=28)
        self.panel_gps    = ColorPanel("GPS",          gradient="green", body_font_size=28)
        self.panel_asr    = ColorPanel("ASR",          gradient="blue",  body_font_size=24)

        # Close button
        self.btn_close = QPushButton("Close")
        self.btn_close.setFont(QFont("Arial", 20, QFont.Bold))
        self.btn_close.setStyleSheet("""
            QPushButton {
                background: rgba(0,0,0,0.35); color: white;
                border: 2px solid rgba(255,255,255,0.6);
                border-radius: 12px; padding: 8px 18px;
            }
            QPushButton:hover { background: rgba(0,0,0,0.5); }
        """)
        self.btn_close.clicked.connect(self.close)

        # Layout
        top = QVBoxLayout()
        top.setContentsMargins(12, 12, 12, 12)
        top.setSpacing(10)

        top.addWidget(self.face_frame)

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(self.panel_status, 1)
        row.addWidget(self.panel_gps, 1)
        top.addLayout(row)

        top.addWidget(self.panel_asr)
        top.addWidget(self.btn_close, alignment=Qt.AlignCenter)

        self.setLayout(top)

        # State
        self._behavior_code = 0
        self._any_alert = False
        self._gps_state = "UNKNOWN"

        # When ASR text arrives, briefly flash LISTEN face if not in error
        self._listen_timer = QTimer()
        self._listen_timer.setSingleShot(True)
        self._listen_timer.timeout.connect(self._restore_face_from_behavior)

    # ===== Face helpers
    def set_face(self, path: str):
        if not Path(path).exists():
            # Graceful fallback: keep previous face
            return
        if self._movie:
            self._movie.stop()
        self._movie = QMovie(path)
        self.face.setMovie(self._movie)
        self._movie.start()

    def _restore_face_from_behavior(self):
        label, face_key = BEHAVIOR_TO_FACE.get(self._behavior_code, ("UNKNOWN", "IDLE"))
        self.set_face(FACE_GIFS.get(face_key, FACE_GIFS["IDLE"]))

    # ===== Slots (connected to UiBus)
    def on_asr(self, text: str):
        self.panel_asr.set_text(text if len(text) <= 400 else (text[:397] + "..."))
        # Optional alert rule (only if clearly bad): empty ASR turns panel red briefly
        asr_bad = (text.strip() == "")
        self.panel_asr.set_alert(asr_bad)

        # Briefly show LISTEN face if no global alert
        if not self._any_alert and self._behavior_code != 3:
            self.set_face(FACE_GIFS["LISTEN"])
            self._listen_timer.start(1200)  # revert after 1.2s

    def on_mode(self, code: int):
        self._behavior_code = code
        label, face_key = BEHAVIOR_TO_FACE.get(code, ("UNKNOWN", "IDLE"))
        self.panel_status.set_text(label)

        # Status alert if error
        is_error = (code == 3)
        self.panel_status.set_alert(is_error)

        # Only change face if no global alert active
        if not self._any_alert:
            self.set_face(FACE_GIFS.get(face_key, FACE_GIFS["IDLE"]))

    def on_gps(self, summary: dict):
        state = summary.get("state", "UNKNOWN")
        hint  = summary.get("hint", "")
        lat   = summary.get("lat", float("nan"))
        lon   = summary.get("lon", float("nan"))
        self._gps_state = state

        self.panel_gps.set_text(f"{state} — {hint}")

        gps_alert = (state == "LOST")
        self.panel_gps.set_alert(gps_alert)

        # If GPS lost, force SAD face
        if gps_alert:
            self.set_face(FACE_GIFS["SAD"])
        else:
            # If no overall alert, keep behavior face
            if not self._any_alert:
                self._restore_face_from_behavior()

    def on_any_alert(self, any_alert: bool):
        self._any_alert = any_alert
        if any_alert:
            self.set_face(FACE_GIFS["SAD"])
        else:
            self._restore_face_from_behavior()

    # Ensure ROS shuts down on closing
    def closeEvent(self, event):
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return super().closeEvent(event)


# =========================
# Main
# =========================
def main():
    # Qt
    app = QApplication(sys.argv)
    ui = Dashboard()

    # ROS2
    rclpy.init(args=None)
    bus = UiBus()
    node = RobotUiNode(bus)

    # Wire signals
    bus.sig_asr.connect(ui.on_asr)
    bus.sig_mode.connect(ui.on_mode)
    bus.sig_gps.connect(ui.on_gps)
    bus.sig_alert.connect(ui.on_any_alert)

    # Spin ROS in background
    def ros_spin():
        rclpy.spin(node)
        node.destroy_node()
    th = threading.Thread(target=ros_spin, daemon=True)
    th.start()

    # 800x480 full-screen UI
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
