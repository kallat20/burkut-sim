#!/usr/bin/env python3
import sys
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from cv_bridge import CvBridge

# PX4 Mesajları
from px4_msgs.msg import (
    VehicleStatus, VehicleLocalPosition, VehicleAttitude,
    BatteryStatus, VehicleCommand
)
from sensor_msgs.msg import Image

# Proje Mesajları
from burkut_msgs.msg import WaypointArray, Waypoint, ObstacleArray

# ROS2 sistem logları
from rcl_interfaces.msg import Log as RosLog

# MAVLink STATUSTEXT dinleyici (QGC tarzı araç mesajları)
_mavutil = None
_PYMAVLINK_OK = False
try:
    from pymavlink import mavutil as _mavutil  # type: ignore[no-redef]
    _PYMAVLINK_OK = True
except ImportError:
    pass

# PX4 nav_state decode tablosu
NAV_STATE = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
    4: "AUTO_LOITER", 5: "AUTO_RTL", 14: "OFFBOARD",
    17: "TAKEOFF", 18: "LAND",
}

QSS = """
QWidget { background-color: #0B0E14; color: #E0E0E0; font-family: 'Segoe UI', sans-serif; }
QFrame#SidePanel { border: 1px solid #1A1D26; background-color: #11141D; border-radius: 5px; }
QLabel#Header { color: #82AAFF; font-weight: bold; font-size: 10px; }
QLabel#TeleValue { font-family: 'Consolas'; font-size: 13px; color: #FFFFFF; }
QPushButton { background-color: #1F2330; border: 1px solid #333747; padding: 8px; border-radius: 4px; font-weight: bold; }
QPushButton:hover { background-color: #333747; }
QPushButton#Armed { background-color: #1A3A1A; color: #55FF55; border: 1px solid #2A6A2A; }
QPushButton#Emergency { background-color: #451414; color: #FF8888; border: 1px solid #722222; }
QTextEdit#Log { background-color: #05070A; border: none; font-family: 'Consolas'; font-size: 11px; color: #A0A0A0; }
QTableWidget { background-color: #0D1017; gridline-color: #1A1D26; border: none; font-family: 'Consolas'; font-size: 11px; }
QHeaderView::section { background-color: #11141D; color: #82AAFF; padding: 4px; border: none; }
QDoubleSpinBox { background-color: #1F2330; border: 1px solid #333747; padding: 4px; border-radius: 3px; }
QLineEdit { background-color: #0D1017; border: 1px solid #333747; padding: 5px;
            border-radius: 3px; color: #C8F0C8; font-family: 'Consolas'; font-size: 12px; }
QLineEdit:focus { border: 1px solid #82AAFF; }
QTabWidget::pane { border: 1px solid #1A1D26; border-radius: 3px; }
QTabBar::tab { background-color: #11141D; color: #82AAFF; padding: 3px 10px;
               border: 1px solid #1A1D26; font-size: 10px; font-weight: bold; }
QTabBar::tab:selected { background-color: #1F2330; color: #FFFFFF; border-bottom: 2px solid #82AAFF; }
QTabBar::tab:hover { background-color: #1A1D26; }
QComboBox { background-color: #1F2330; border: 1px solid #333747; padding: 4px;
            border-radius: 3px; color: #E0E0E0; font-family: 'Consolas'; font-size: 11px; }
QComboBox::drop-down { border: none; width: 16px; }
"""


class MavlinkListener(QThread):
    """
    Ayrı bir thread'de pymavlink ile MAVLink STATUSTEXT mesajlarını dinler.
    PX4 simülasyonu genelde UDP 14550'e gönderir (QGC varsayılanı).
    Birden fazla alıcıya izin vermek için SO_REUSEPORT desteklenmeli.
    """
    msg_received = pyqtSignal(int, str)   # severity (0-7), text
    status_changed = pyqtSignal(bool, str)  # connected, info string

    # MAVLink STATUSTEXT severity → renk ve etiket
    SEV = {
        0: ("#FF2222", "EMRG"),
        1: ("#FF2222", "ALRT"),
        2: ("#FF5555", "CRIT"),
        3: ("#FF8800", "ERR "),
        4: ("#FFCC44", "WARN"),
        5: ("#E0E0E0", "NOTE"),
        6: ("#A0A0A0", "INFO"),
        7: ("#6A6A8A", "DBG "),
    }
    # MAVLink bağlantısı
    def __init__(self, connection_str: str = "udpin:0.0.0.0:14550"):
        super().__init__()
        self.connection_str = connection_str
        self._running = False

    def run(self):
        if not _PYMAVLINK_OK:
            self.status_changed.emit(False, "pymavlink kurulu değil: pip install pymavlink")
            return
        try:
            mav = _mavutil.mavlink_connection(self.connection_str)  # type: ignore[union-attr]
            self.status_changed.emit(False, f"Heartbeat bekleniyor… ({self.connection_str})")
            mav.wait_heartbeat(timeout=8)  # type: ignore[call-arg]
            self.status_changed.emit(True, f"Bağlandı  sysid={mav.target_system}")  # type: ignore[union-attr]
            self._running = True
            while self._running:
                msg = mav.recv_match(type=["STATUSTEXT"], blocking=True, timeout=1.0)  # type: ignore[call-arg]
                if msg and self._running:
                    text = msg.text.rstrip("\x00").strip()
                    if text:
                        self.msg_received.emit(int(msg.severity), text)
        except Exception as exc:
            self.status_changed.emit(False, f"Hata: {exc}")

    def stop(self):
        self._running = False
        self.quit()
        self.wait(2000)


class HUDWidget(QWidget):
    """Merkezi HUD ve Kamera Ekranı"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image = None
        self.pitch = self.roll = self.yaw = 0.0
        self.alt = self.speed = 0.0
        self.hud_color = QColor(0, 255, 0, 200)

    def update_data(self, roll, pitch, yaw, alt, speed, img=None):
        self.roll, self.pitch, self.yaw = roll, pitch, yaw
        self.alt, self.speed = alt, speed
        self.image = img
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2

        # Kamera arka planı
        if self.image is not None:
            rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            ih, iw, ch = rgb.shape
            qt_img = QImage(rgb.data, iw, ih, ch * iw, QImage.Format.Format_RGB888)
            painter.drawImage(
                self.rect(),
                qt_img.scaled(w, h, Qt.AspectRatioMode.KeepAspectRatioByExpanding)
            )
        else:
            painter.fillRect(self.rect(), QColor(8, 10, 14))
            painter.setPen(QColor(50, 60, 80))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                             "KAMERA YAYINI BEKLENİYOR\n/camera/image_raw")

        painter.setPen(QPen(self.hud_color, 2))

        # Yapay ufuk — roll + pitch
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-math.degrees(self.roll))
        pitch_px = math.degrees(self.pitch) * 8
        for deg in range(-30, 31, 10):
            y = int(-deg * 8 + pitch_px)
            if deg == 0:
                painter.drawLine(-120, y, 120, y)
            else:
                painter.drawLine(-50, y, 50, y)
                painter.drawText(55, y + 5, str(deg))
        painter.restore()

        # Sabit uçak sembolü
        painter.setPen(QPen(self.hud_color, 3))
        painter.drawLine(cx - 40, cy, cx - 12, cy)
        painter.drawLine(cx + 12, cy, cx + 40, cy)
        painter.drawEllipse(cx - 5, cy - 5, 10, 10)

        # Hız şeridi (sol)
        painter.setPen(QPen(self.hud_color, 1))
        painter.drawRect(30, cy - 80, 55, 160)
        painter.drawText(30, cy - 88, "SPD m/s")
        painter.drawText(38, cy + 8, f"{self.speed:.1f}")

        # İrtifa şeridi (sağ)
        painter.drawRect(w - 90, cy - 80, 55, 160)
        painter.drawText(w - 90, cy - 88, "ALT m")
        painter.drawText(w - 82, cy + 8, f"{self.alt:.1f}")

        # Yaw göstergesi (alt orta)
        painter.drawText(cx - 40, h - 15, f"YAW: {math.degrees(self.yaw):.1f}°")


class DroneGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BÜRKÜT YER KONTROL İSTASYONU")
        self.resize(1500, 860)
        self.setStyleSheet(QSS)
        self.cv_bridge = CvBridge()

        # Telemetri durumu
        self.roll = self.pitch = self.yaw = 0.0
        self.alt = self.speed = 0.0
        self.current_frame = None
        self.armed = False
        self._prev_armed = False
        self._prev_nav_state = -1
        self.echo_process = QProcess()     # topic echo subprocess
        self.mav_listener: MavlinkListener | None = None

        self.init_ui()
        self.init_ros()

    # ──────────────────────────────────────────────────────────
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(4)
        root.setContentsMargins(6, 6, 6, 6)

        # ── TOP BAR ──────────────────────────────────────────
        top = QHBoxLayout()
        self.lbl_mode = QLabel("DURUM: BAĞLANTI YOK")
        self.lbl_nav  = QLabel("NAV: ---")
        self.lbl_batt = QLabel("BATT: --V (--%)")
        self.lbl_pos  = QLabel("POS  x=0  y=0  z=0")
        for lbl in [self.lbl_mode, self.lbl_nav, self.lbl_batt, self.lbl_pos]:
            lbl.setObjectName("TeleValue")
            top.addWidget(lbl)
            top.addStretch(1)
        root.addLayout(top)

        # ── ANA GÖVDE ────────────────────────────────────────
        body = QHBoxLayout()

        # ── SOL PANEL ────────────────────────────────────────
        left = QFrame(); left.setObjectName("SidePanel")
        lv = QVBoxLayout(left)
        lv.setSpacing(6)

        lv.addWidget(self._hdr("UÇUŞ KONTROL"))
        self.btn_arm_toggle = QPushButton("ARM")
        self.btn_arm_toggle.setMinimumHeight(40)
        self.btn_arm_toggle.clicked.connect(self.cmd_arm_toggle)
        lv.addWidget(self.btn_arm_toggle)

        lv.addWidget(self._hdr("MODLAR"))
        self.btn_offboard = QPushButton("OFFBOARD")
        self.btn_offboard.clicked.connect(self.cmd_set_offboard)
        lv.addWidget(self.btn_offboard)

        self.btn_rtl = QPushButton("RTL")
        self.btn_rtl.clicked.connect(self.cmd_rtl)
        lv.addWidget(self.btn_rtl)

        self.btn_land = QPushButton("İNİŞ (LAND)")
        self.btn_land.clicked.connect(self.cmd_land)
        lv.addWidget(self.btn_land)

        self.btn_auto = QPushButton("AUTO")
        self.btn_auto.clicked.connect(self.cmd_auto)
        lv.addWidget(self.btn_auto)

        lv.addWidget(self._hdr("OTOPILOT"))
        self.airspd = QDoubleSpinBox()
        self.airspd.setRange(0, 100); self.airspd.setValue(15); self.airspd.setPrefix("AIRSPEED: ")
        lv.addWidget(self.airspd)

        self.alt_sp = QDoubleSpinBox()
        self.alt_sp.setRange(1, 200); self.alt_sp.setValue(20); self.alt_sp.setPrefix("ALT: ")
        lv.addWidget(self.alt_sp)

        # Manuel Waypoint
        lv.addWidget(self._hdr("MANUEL WAYPOINT"))
        self.wp_x = QDoubleSpinBox()
        self.wp_x.setRange(-500, 500); self.wp_x.setPrefix("X: ")
        self.wp_y = QDoubleSpinBox()
        self.wp_y.setRange(-500, 500); self.wp_y.setPrefix("Y: ")
        self.wp_z = QDoubleSpinBox()
        self.wp_z.setRange(1, 200); self.wp_z.setValue(20); self.wp_z.setPrefix("Z: ")
        self.wp_r = QDoubleSpinBox()
        self.wp_r.setRange(0.5, 50); self.wp_r.setValue(2.0); self.wp_r.setPrefix("R_acc: ")
        for sb in [self.wp_x, self.wp_y, self.wp_z, self.wp_r]:
            lv.addWidget(sb)
        self.btn_send_wp = QPushButton("WAYPOINT GÖNDER")
        self.btn_send_wp.clicked.connect(self.cmd_send_waypoint)
        lv.addWidget(self.btn_send_wp)

        lv.addStretch()
        btn_kill = QPushButton("⚠  ACİL DURDURMA")
        btn_kill.setObjectName("Emergency")
        btn_kill.clicked.connect(self.cmd_kill)
        lv.addWidget(btn_kill)
        body.addWidget(left, 2)

        # ── ORTA: HUD ────────────────────────────────────────
        self.hud = HUDWidget()
        body.addWidget(self.hud, 5)

        # ── SAĞ PANEL ────────────────────────────────────────
        right = QFrame(); right.setObjectName("SidePanel")
        rv = QVBoxLayout(right)
        rv.setSpacing(4)

        rv.addWidget(self._hdr("AKTİF WAYPOINTLER  (topic: waypoints)"))
        self.wp_table = QTableWidget(0, 5)
        self.wp_table.setHorizontalHeaderLabels(["X", "Y", "Z", "Yaw", "R_acc"])
        self.wp_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.wp_table.setMaximumHeight(180)
        rv.addWidget(self.wp_table)

        rv.addWidget(self._hdr("ALGILANAN ENGELLER  (topic: pole_test)"))
        self.obs_table = QTableWidget(0, 5)
        self.obs_table.setHorizontalHeaderLabels(["X", "Y", "Z", "Radius", "Conf"])
        self.obs_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.obs_table.setMaximumHeight(180)
        rv.addWidget(self.obs_table)

        rv.addWidget(self._hdr("HIZ VEKTÖRLERİ"))
        self.lbl_vel = QLabel("Vx: 0.00   Vy: 0.00   Vz: 0.00")
        self.lbl_vel.setObjectName("TeleValue")
        rv.addWidget(self.lbl_vel)

        rv.addStretch()
        body.addWidget(right, 2)
        root.addLayout(body)

        # ── ALT BÖLÜM: Sekmeli Mesaj + Konsol ───────────────
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(4)

        # Sol — Sekmeli panel (Topic Echo + PX4 & Görev Mesajları)
        left_msg_frame = QFrame(); left_msg_frame.setObjectName("SidePanel")
        left_msg_frame.setFixedHeight(160)
        lm_vbox = QVBoxLayout(left_msg_frame)
        lm_vbox.setContentsMargins(4, 4, 4, 4)
        lm_vbox.setSpacing(2)

        self.left_tabs = QTabWidget()

        # ── Sekme 1: ROS Topic Echo ───────────────────────
        echo_tab = QWidget()
        et_vbox = QVBoxLayout(echo_tab)
        et_vbox.setContentsMargins(4, 4, 4, 4)
        et_vbox.setSpacing(2)
        self.echo_out = QTextEdit()
        self.echo_out.setObjectName("Log")
        self.echo_out.setReadOnly(True)
        et_vbox.addWidget(self.echo_out)
        self.left_tabs.addTab(echo_tab, "TOPIC ECHO")

        # ── Sekme 2: PX4 & Görev Mesajları ──────────────────
        px4_tab = QWidget()
        pt_vbox = QVBoxLayout(px4_tab)
        pt_vbox.setContentsMargins(4, 4, 4, 4)
        pt_vbox.setSpacing(2)
        self.px4_msg_out = QTextEdit()
        self.px4_msg_out.setObjectName("Log")
        self.px4_msg_out.setReadOnly(True)
        pt_vbox.addWidget(self.px4_msg_out)
        self.left_tabs.addTab(px4_tab, "PX4 & GÖREV")

        lm_vbox.addWidget(self.left_tabs)

        # ── Tab bar sol köşesi — Topic Echo kontrolleri ───────
        echo_corner = QWidget()
        ec_lay = QHBoxLayout(echo_corner)
        ec_lay.setContentsMargins(4, 0, 0, 0)
        ec_lay.setSpacing(3)
        self.echo_topic_combo = QComboBox()
        self.echo_topic_combo.setEditable(True)
        self.echo_topic_combo.setFixedWidth(160)
        self.echo_topic_combo.setFixedHeight(22)
        self.echo_topic_combo.addItems([
            '/fmu/out/vehicle_status',
            '/fmu/out/vehicle_local_position',
            '/fmu/out/vehicle_attitude',
            '/fmu/out/battery_status',
            'waypoints',
            'pole_test',
        ])
        self.btn_echo_toggle = QPushButton("▶")
        self.btn_echo_toggle.setFixedSize(26, 22)
        self.btn_echo_toggle.clicked.connect(self.toggle_topic_echo)
        btn_echo_clear = QPushButton("✕")
        btn_echo_clear.setFixedSize(22, 22)
        btn_echo_clear.clicked.connect(lambda: self.echo_out.clear())
        ec_lay.addWidget(self.echo_topic_combo)
        ec_lay.addWidget(self.btn_echo_toggle)
        ec_lay.addWidget(btn_echo_clear)
        self.left_tabs.setCornerWidget(echo_corner, Qt.Corner.TopLeftCorner)

        # ── Tab bar sağ köşesi — MAVLink bağlantısı ───────────
        mav_corner = QWidget()
        mc_lay = QHBoxLayout(mav_corner)
        mc_lay.setContentsMargins(0, 0, 4, 0)
        mc_lay.setSpacing(3)
        self.mav_conn_input = QLineEdit("udpin:0.0.0.0:14550")
        self.mav_conn_input.setFixedWidth(150)
        self.mav_conn_input.setFixedHeight(22)
        self.btn_mav_toggle = QPushButton("▶ BAĞLAN")
        self.btn_mav_toggle.setFixedSize(70, 22)
        self.btn_mav_toggle.clicked.connect(self.toggle_mavlink)
        self.lbl_mav_status = QLabel("● YOK")
        self.lbl_mav_status.setStyleSheet("color:#FF5555; font-size:10px;")
        btn_mav_clear = QPushButton("✕")
        btn_mav_clear.setFixedSize(22, 22)
        btn_mav_clear.clicked.connect(lambda: self.px4_msg_out.clear())
        mc_lay.addWidget(self.mav_conn_input)
        mc_lay.addWidget(self.btn_mav_toggle)
        mc_lay.addWidget(self.lbl_mav_status)
        mc_lay.addWidget(btn_mav_clear)
        self.left_tabs.setCornerWidget(mav_corner, Qt.Corner.TopRightCorner)

        bottom_layout.addWidget(left_msg_frame, 1)

        # Sağ — PX4 Konsol (komut gönder + /rosout log)
        con_frame = QFrame(); con_frame.setObjectName("SidePanel")
        con_frame.setFixedHeight(160)
        cf_vbox = QVBoxLayout(con_frame)
        cf_vbox.setContentsMargins(6, 4, 6, 4)
        cf_vbox.setSpacing(2)
        cf_vbox.addWidget(self._hdr("PX4 KONSOL  (arm | disarm | rtl | land | offboard | cmd <id>)"))
        self.log_screen = QTextEdit()
        self.log_screen.setObjectName("Log")
        self.log_screen.setReadOnly(True)
        cf_vbox.addWidget(self.log_screen)
        con_input_row = QHBoxLayout()
        self.console_input = QLineEdit()
        self.console_input.setPlaceholderText("Komut girin ve Enter'a basın...")
        self.console_input.returnPressed.connect(self.console_send)
        btn_con_send = QPushButton("GÖNDER")
        btn_con_send.setFixedWidth(80)
        btn_con_send.clicked.connect(self.console_send)
        con_input_row.addWidget(self.console_input)
        con_input_row.addWidget(btn_con_send)
        cf_vbox.addLayout(con_input_row)
        bottom_layout.addWidget(con_frame, 1)

        root.addLayout(bottom_layout)

    def _hdr(self, text):
        lbl = QLabel(text)
        lbl.setObjectName("Header")
        return lbl

    # ──────────────────────────────────────────────────────────
    def init_ros(self):
        rclpy.init()
        self.node = Node('burkut_gcs_node')

        # PX4 uDDS BEST_EFFORT QoS (zorunlu)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── SUBSCRIBERS ──────────────────────────────────────
        # PX4 telemetri
        self.node.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.cb_status, px4_qos)
        self.node.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.cb_pos, px4_qos)
        self.node.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.cb_attitude, px4_qos)
        self.node.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self.cb_battery, px4_qos)

        # Kamera (default QoS - RELIABLE)
        self.node.create_subscription(
            Image, '/camera/image_raw', self.cb_image, 10)

        # Burkut proje topic'leri (default QoS)
        self.node.create_subscription(
            WaypointArray, 'waypoints', self.cb_waypoints, 10)
        self.node.create_subscription(
            ObstacleArray, 'pole_test', self.cb_obstacles, 10)

        # ROS2 sistem logları — tüm node'ların mesajları (/rosout)
        rosout_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        self.node.create_subscription(
            RosLog, '/rosout', self.cb_rosout, rosout_qos)

        # ── PUBLISHERS ───────────────────────────────────────
        # PX4 komut kanalı
        self.cmd_pub = self.node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        # Manuel waypoint override (planning node ile aynı topic)
        self.wp_pub = self.node.create_publisher(
            WaypointArray, 'waypoints', 10)

        # Qt timer → ROS spin (10ms)
        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._ros_spin)
        self.spin_timer.start(10)

    def _ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.hud.update_data(
            self.roll, self.pitch, self.yaw,
            self.alt, self.speed, self.current_frame
        )

    # ── CALLBACKS ────────────────────────────────────────────
    def cb_status(self, msg: VehicleStatus):
        self.armed = (msg.arming_state == 2)
        arm_txt = "ARMED" if self.armed else "DISARMED"
        nav_txt = NAV_STATE.get(msg.nav_state, f"#{msg.nav_state}")
        self.lbl_mode.setText(f"DURUM: {arm_txt}")
        self.lbl_nav.setText(f"NAV: {nav_txt}")

        # Arm/Disarm durum değişikliği → PX4 & Görev sekmesine yaz
        if self.armed != self._prev_armed:
            if self.armed:
                self.px4_msg_out.append(
                    "<span style='color:#FF5555'>[EVENT] Armed by external command</span>")
            else:
                self.px4_msg_out.append(
                    "<span style='color:#55FF55'>[EVENT] Disarmed by external command</span>")
            self._prev_armed = self.armed

        # Nav state değişikliği → PX4 & Görev sekmesine yaz
        if msg.nav_state != self._prev_nav_state:
            self.px4_msg_out.append(
                f"<span style='color:#82AAFF'>[NAV] Mode → {nav_txt}</span>")
            self._prev_nav_state = msg.nav_state

        # Toggle butonunu güncelle
        if self.armed:
            self.btn_arm_toggle.setText("DISARM")
            self.btn_arm_toggle.setObjectName("Armed")
        else:
            self.btn_arm_toggle.setText("ARM")
            self.btn_arm_toggle.setObjectName("")
        self.btn_arm_toggle.style().unpolish(self.btn_arm_toggle)
        self.btn_arm_toggle.style().polish(self.btn_arm_toggle)

    def cb_pos(self, msg: VehicleLocalPosition):
        # PX4 NED → z aşağı pozitif, irtifa için -z
        self.alt   = -msg.z
        self.speed = math.sqrt(msg.vx**2 + msg.vy**2)
        self.lbl_pos.setText(
            f"POS  x={msg.x:.1f}  y={msg.y:.1f}  z={msg.z:.1f}")
        self.lbl_vel.setText(
            f"Vx:{msg.vx:.2f}   Vy:{msg.vy:.2f}   Vz:{msg.vz:.2f}")

    def cb_attitude(self, msg: VehicleAttitude):
        q = msg.q
        # Quaternion → Euler (NED frame)
        self.roll  = math.atan2(
            2.0 * (q[0]*q[1] + q[2]*q[3]),
            1.0 - 2.0 * (q[1]**2 + q[2]**2))
        self.pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (q[0]*q[2] - q[3]*q[1]))))
        self.yaw   = math.atan2(
            2.0 * (q[0]*q[3] + q[1]*q[2]),
            1.0 - 2.0 * (q[2]**2 + q[3]**2))

    def cb_battery(self, msg: BatteryStatus):
        self.lbl_batt.setText(
            f"BATT: {msg.voltage_v:.1f}V ({int(msg.remaining * 100)}%)")

    def cb_image(self, msg: Image):
        try:
            self.current_frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self._log(f"[HATA] Kamera dönüşüm: {e}")

    def cb_waypoints(self, msg: WaypointArray):
        """Planning node'dan gelen aktif waypointleri tabloya yaz"""
        self.wp_table.setRowCount(0)
        for wp in msg.waypoints:
            row = self.wp_table.rowCount()
            self.wp_table.insertRow(row)
            for col, val in enumerate([wp.x, wp.y, wp.z, wp.yaw, wp.acceptance_radius]):
                self.wp_table.setItem(row, col, QTableWidgetItem(f"{val:.2f}"))

    def cb_rosout(self, msg: RosLog):
        """Tüm ROS2 node'larının log mesajlarını sol panele yaz"""
        # Seviye → renk ve etiket
        level_map = {
            10: ("#6A6A8A", "DEBUG"),
            20: ("#A0A0A0", "INFO "),
            30: ("#FFCC44", "WARN "),
            40: ("#FF5555", "ERROR"),
            50: ("#FF2222", "FATAL"),
        }
        color, label = level_map.get(msg.level, ("#A0A0A0", "INFO "))
        # Node adını kısalt (son segment)
        node_name = msg.name.split(".")[-1] if msg.name else "?"
        line = (
            f"<span style='color:{color}'>"
            f"[{label}] [{node_name}] {msg.msg}"
            f"</span>"
        )
        self.log_screen.append(line)
        # Log çok uzarsa eski satırları temizle (max 300 satır)
        doc = self.log_screen.document()
        while doc.blockCount() > 300:
            cursor = self.log_screen.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.select(cursor.SelectionType.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

        # Görev ile ilgili node'ları PX4 & Görev sekmesine de yaz
        _mission_keywords = ('flight_controller', 'planning_test',
                             'pole_ai', 'burkut_gcs')
        if any(k in msg.name for k in _mission_keywords):
            self.px4_msg_out.append(
                f"<span style='color:{color}'>[{label}][{node_name}] {msg.msg}</span>"
            )

    def cb_obstacles(self, msg: ObstacleArray):
        """Perception node'dan gelen engelleri tabloya yaz"""
        self.obs_table.setRowCount(0)
        for obs in msg.obstacles:
            row = self.obs_table.rowCount()
            self.obs_table.insertRow(row)
            for col, val in enumerate([obs.x, obs.y, obs.z, obs.radius, obs.confidence]):
                self.obs_table.setItem(row, col, QTableWidgetItem(f"{val:.2f}"))

    # ── KOMUTLAR ─────────────────────────────────────────────
    def cmd_arm_toggle(self):
        if self.armed:
            self._send_vehicle_cmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=0.0)
            self._log("[KOMUT] DISARM gönderildi.")
        else:
            if QMessageBox.question(
                    self, 'ONAY', "Motorlar ARM edilsin mi?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            ) == QMessageBox.StandardButton.Yes:
                self._send_vehicle_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
                self._log("[KOMUT] ARM gönderildi.")

    def console_send(self):
        """Konsoldan gelen metni parse edip VehicleCommand olarak gönder"""
        raw = self.console_input.text().strip()
        if not raw:
            return
        self.log_screen.append(f"<span style='color:#82AAFF'>▶ {raw}</span>")
        self.console_input.clear()

        cmd_map = {
            "arm":      (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0),
            "disarm":   (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0),
            "rtl":      (VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,  0.0, 0.0),
            "land":     (VehicleCommand.VEHICLE_CMD_DO_LAND_START,          0.0, 0.0),
            "offboard": (VehicleCommand.VEHICLE_CMD_DO_SET_MODE,            1.0, 6.0),
        }

        parts = raw.lower().split()
        key = parts[0]

        if key in cmd_map:
            command, p1, p2 = cmd_map[key]
            for part in parts[1:]:
                if part.startswith("p1="):
                    try: p1 = float(part[3:])
                    except ValueError: pass
                elif part.startswith("p2="):
                    try: p2 = float(part[3:])
                    except ValueError: pass
            self._send_vehicle_cmd(command, p1=p1, p2=p2)
            self.log_screen.append(
                f"  <span style='color:#55FF55'>✓ Gönderildi → cmd={command} p1={p1} p2={p2}</span>")
        elif key == "cmd":
            try:
                cmd_id = int(parts[1])
                p1, p2 = 0.0, 0.0
                for part in parts[2:]:
                    if part.startswith("p1="):
                        p1 = float(part[3:])
                    elif part.startswith("p2="):
                        p2 = float(part[3:])
                self._send_vehicle_cmd(cmd_id, p1=p1, p2=p2)
                self.log_screen.append(
                    f"  <span style='color:#55FF55'>✓ Gönderildi → cmd={cmd_id} p1={p1} p2={p2}</span>")
            except (IndexError, ValueError):
                self.log_screen.append(
                    "  <span style='color:#FF5555'>✗ Sözdizimi: cmd &lt;id&gt; [p1=..] [p2=..]</span>")
        else:
            self.log_screen.append(
                f"  <span style='color:#FF5555'>✗ Bilinmeyen: '{key}'"
                f"  |  Geçerli: arm, disarm, rtl, land, offboard, cmd &lt;id&gt;</span>")

    def cmd_set_offboard(self):
        # DO_SET_MODE: param1=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), param2=6 (PX4 OFFBOARD)
        self._send_vehicle_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
        self._log("[KOMUT] OFFBOARD modu talep edildi.")

    def cmd_rtl(self):
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self._log("[KOMUT] RTL gönderildi.")

    def cmd_land(self):
        # interface.py ile aynı komut
        self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_LAND_START)
        self._log("[KOMUT] LAND komutu gönderildi.")

    def cmd_auto(self):
        # PX4 AUTO.MISSION modu (custom_mode=4)
        self._send_vehicle_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=4.0)
        self._log("[KOMUT] AUTO MISSION modu talep edildi.")

    def cmd_kill(self):
        if QMessageBox.critical(
                self, '⚠ ACİL DURDURMA',
                "MOTORLAR ANINDA DURDURALACAK!\nDevam edilsin mi?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        ) == QMessageBox.StandardButton.Yes:
            # param2=21196 → PX4 force disarm magic number
            self._send_vehicle_cmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                p1=0.0, p2=21196.0)
            self._log("[ACİL] Force DISARM gönderildi!")

    def cmd_send_waypoint(self):
        """GUI'den manuel tek waypoint gönder (planning node'u bypass eder)"""
        msg = WaypointArray()
        wp = Waypoint()
        wp.x = self.wp_x.value()
        wp.y = self.wp_y.value()
        wp.z = self.wp_z.value()
        wp.yaw = 0.0
        wp.acceptance_radius = float(self.wp_r.value())
        wp.fark = 0.0
        msg.waypoints.append(wp)
        self.wp_pub.publish(msg)
        self._log(
            f"[WP] x={wp.x:.1f}  y={wp.y:.1f}  z={wp.z:.1f}  r={wp.acceptance_radius:.1f} gönderildi.")

    def _send_vehicle_cmd(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command    = command
        msg.param1     = float(p1)
        msg.param2     = float(p2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _log(self, text):
        """GUI komut/durum mesajlarını sağ konsola yaz"""
        self.log_screen.append(f"<span style='color:#4499FF'>{text}</span>")

    # ── TOPIC ECHO ───────────────────────────────────────────
    def toggle_topic_echo(self):
        if self.echo_process.state() == QProcess.ProcessState.Running:
            self.echo_process.kill()
            self.echo_process.waitForFinished(500)
            self.btn_echo_toggle.setText("▶")
            self.echo_out.append(
                "<span style='color:#FFCC44'>── Durduruldu ──</span>")
        else:
            topic = self.echo_topic_combo.currentText().strip()
            if not topic:
                return
            self.echo_process.readyReadStandardOutput.connect(self.on_echo_output)
            self.echo_process.readyReadStandardError.connect(self.on_echo_error)
            self.echo_process.start("ros2", ["topic", "echo", topic])
            self.btn_echo_toggle.setText("■")
            self.echo_out.append(
                f"<span style='color:#82AAFF'>── Dinleniyor: {topic} ──</span>")

    def on_echo_output(self):
        data = self.echo_process.readAllStandardOutput().data().decode("utf-8", errors="replace")
        text = data.strip()
        if not text:
            return
        self.echo_out.append(f"<span style='color:#C0C0C0'>{text}</span>")
        # Max 200 satır
        doc = self.echo_out.document()
        while doc.blockCount() > 200:
            cursor = self.echo_out.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.select(cursor.SelectionType.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def on_echo_error(self):
        data = self.echo_process.readAllStandardError().data().decode("utf-8", errors="replace")
        if data.strip():
            self.echo_out.append(
                f"<span style='color:#FF5555'>{data.strip()}</span>")

    # ── MAVLink STATUSTEXT ───────────────────────────────────
    def toggle_mavlink(self):
        if self.mav_listener and self.mav_listener.isRunning():
            self.mav_listener.stop()
            self.mav_listener = None
            self.btn_mav_toggle.setText("▶ BAĞLAN")
            self.lbl_mav_status.setText("● YOK")
            self.lbl_mav_status.setStyleSheet("color:#FF5555; font-size:10px;")
            self.px4_msg_out.append(
                "<span style='color:#FFCC44'>── MAVLink bağlantısı kesildi ──</span>")
        else:
            conn_str = self.mav_conn_input.text().strip() or "udpin:0.0.0.0:14550"
            self.mav_listener = MavlinkListener(conn_str)
            self.mav_listener.msg_received.connect(self.on_mavlink_message)
            self.mav_listener.status_changed.connect(self.on_mavlink_status)
            self.mav_listener.start()
            self.btn_mav_toggle.setText("■ KES")
            self.lbl_mav_status.setText("● ...")
            self.lbl_mav_status.setStyleSheet("color:#FFCC44; font-size:10px;")
            # Sekmeye geç
            self.left_tabs.setCurrentIndex(1)

    def on_mavlink_message(self, severity: int, text: str):
        color, label = MavlinkListener.SEV.get(severity, ("#A0A0A0", "INFO"))
        self.px4_msg_out.append(
            f"<span style='color:{color}'>[{label}] {text}</span>")
        # Çok uzarsa temizle
        doc = self.px4_msg_out.document()
        while doc.blockCount() > 400:
            cursor = self.px4_msg_out.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.select(cursor.SelectionType.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def on_mavlink_status(self, connected: bool, info: str):
        if connected:
            self.lbl_mav_status.setText("● BAĞLI")
            self.lbl_mav_status.setStyleSheet("color:#55FF55; font-size:10px;")
            self.px4_msg_out.append(
                f"<span style='color:#55FF55'>── {info} ──</span>")
        else:
            self.lbl_mav_status.setText("● YOK")
            self.lbl_mav_status.setStyleSheet("color:#FF5555; font-size:10px;")
            self.px4_msg_out.append(
                f"<span style='color:#FF5555'>── {info} ──</span>")

    def closeEvent(self, event):
        self.spin_timer.stop()
        if self.echo_process.state() == QProcess.ProcessState.Running:
            self.echo_process.kill()
            self.echo_process.waitForFinished(500)
        if self.mav_listener and self.mav_listener.isRunning():
            self.mav_listener.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gcs = DroneGCS()
    gcs.show()
    sys.exit(app.exec())