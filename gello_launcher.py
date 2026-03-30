#!/usr/bin/env python3
"""
GELLO xArm7 Launcher
Professional GUI for servo onboarding, calibration, and simulation launch.
Cross-platform: Linux and Windows.
"""

import importlib
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QCursor, QImage, QPainter, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QFileDialog, QFrame, QGraphicsDropShadowEffect, QHBoxLayout,
    QLabel, QMainWindow, QProgressBar, QPushButton, QSizePolicy, QStackedWidget,
    QVBoxLayout, QWidget,
)

# ─────────────────────────────── Constants ────────────────────────────────────

BASE_DIR        = Path(__file__).parent
CONFIG_SIM      = BASE_DIR / "configs" / "xarm_sim_test.yaml"
CONFIG_REAL     = BASE_DIR / "configs" / "xarm_real.yaml"
PROFILE_PATH    = BASE_DIR / "gello_profile.json"
MODEL_PATH      = BASE_DIR / "third_party" / "mujoco_menagerie" / "ufactory_xarm7" / "xarm7.xml"
NUM_JOINTS      = 7
BAUD_RATE       = 57600
CAL_RESIDUAL_WARN = np.pi / 6  # 30° — warn if joint is far from a π/2 multiple

JOINT_INFO = [
    ("Joint 1", "Base",
     "The lowest servo — rotates the entire arm left and right around the vertical axis. "
     "Located at the very bottom of the GELLO arm."),
    ("Joint 2", "Shoulder",
     "The shoulder servo — swings the arm forward and backward. "
     "This is the first joint above the base."),
    ("Joint 3", "Upper Arm",
     "The upper-arm servo — extends or retracts the upper arm segment. "
     "Roughly mid-way up the arm."),
    ("Joint 4", "Elbow",
     "The elbow servo — bends the lower arm section up and down."),
    ("Joint 5", "Forearm Rotation",
     "Rotates the forearm around its own axis. "
     "Usually the second-to-last large joint."),
    ("Joint 6", "Wrist Bend",
     "Tilts the wrist up and down. Penultimate joint."),
    ("Joint 7", "Wrist Rotation",
     "Rotates the wrist. The last joint before the gripper servo."),
    ("Gripper", "End-effector",
     "The gripper servo — controls the opening and closing of the gripper. "
     "Located at the tip of the arm."),
]

TROUBLESHOOT = [
    ("No power",   "Check the power board LED is lit. The XL330 requires 5 V — "
                   "verify your supply is on and the JST power connector is seated."),
    ("No data",    "Re-seat the JST data cable at both ends until it clicks. "
                   "A half-inserted cable passes power but not signal."),
    ("Only one!",  "Disconnect every other servo. Only this single servo should be "
                   "connected to the data line when scanning."),
    ("Bad cable",  "Try a different JST-SH 3-pin cable — cables fail silently."),
    ("LED check",  "When powered, the servo LED should blink once on boot. "
                   "No blink = no power. Continuous blink = hardware error."),
    ("USB reset",  "Unplug the U2D2 USB cable, wait 3 seconds, re-plug, then retry."),
]

# ─────────────────────────────── Utilities ────────────────────────────────────

def detect_port() -> Optional[str]:
    by_id = Path("/dev/serial/by-id")
    if by_id.exists():
        for p in sorted(by_id.iterdir()):
            if "FTDI" in p.name or "U2D2" in p.name:
                return str(p)
    for fb in ["/dev/ttyUSB0", "/dev/ttyUSB1"]:
        if Path(fb).exists():
            return fb
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            desc = (port.manufacturer or "") + (port.description or "")
            if "FTDI" in desc or "U2D2" in desc:
                return port.device
        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device
    except ImportError:
        pass
    return None


def load_profile() -> dict:
    try:
        if PROFILE_PATH.exists():
            return json.loads(PROFILE_PATH.read_text())
    except Exception:
        pass
    return {}


def save_profile(data: dict):
    PROFILE_PATH.write_text(json.dumps(data, indent=2))


def update_yaml(offsets: List[float], g_open: float, g_close: float, port: str,
                config_path: Optional[Path] = None):
    cfg = config_path or CONFIG_SIM
    text = cfg.read_text()
    text = re.sub(r'port: "[^"]*"', f'port: "{port}"', text, count=1)
    block = "joint_offsets: [\n"
    for i, o in enumerate(offsets):
        comma = "," if i < len(offsets) - 1 else ""
        block += f"      {o:.4f}{comma}\n"
    block += "    ]"
    text = re.sub(r"joint_offsets: \[.*?\]", block, text, flags=re.DOTALL)
    go, gc = int(round(g_open)), int(round(g_close))
    text = re.sub(r"gripper_config: \[\d+, \d+, \d+\]", f"gripper_config: [8, {go}, {gc}]", text)
    cfg.write_text(text)


# ─────────────────────────────── Workers ──────────────────────────────────────

class ScanWorker(QThread):
    result = pyqtSignal(dict)
    def __init__(self, port):
        super().__init__(); self.port = port; self._stop = False
    def stop(self): self._stop = True
    def run(self):
        try:
            from dynamixel_sdk import PacketHandler, PortHandler
            ph = PortHandler(self.port); pk = PacketHandler(2.0)
            ph.openPort(); ph.setBaudRate(BAUD_RATE)
            data, _ = pk.broadcastPing(ph); ph.closePort()
            if not self._stop: self.result.emit(dict(data) if data else {})
        except Exception:
            if not self._stop: self.result.emit({})


class SetIDWorker(QThread):
    done = pyqtSignal(bool)
    def __init__(self, port, from_id, to_id):
        super().__init__(); self.port, self.from_id, self.to_id = port, from_id, to_id
    def run(self):
        try:
            from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
            ph = PortHandler(self.port); pk = PacketHandler(2.0)
            ph.openPort(); ph.setBaudRate(BAUD_RATE)
            r, _ = pk.write1ByteTxRx(ph, self.from_id, 7, self.to_id)
            ph.closePort(); self.done.emit(r == COMM_SUCCESS)
        except Exception:
            self.done.emit(False)


class LivePositionWorker(QThread):
    """Continuously reads joint angles for real-time display using raw SDK.
    Avoids DynamixelDriver so there is no background thread holding the port open.
    """
    update = pyqtSignal(list)
    def __init__(self, port):
        super().__init__(); self.port = port; self._running = True
    def stop(self): self._running = False
    def run(self):
        try:
            from dynamixel_sdk import (
                GroupSyncRead, PacketHandler, PortHandler, COMM_SUCCESS
            )
            ADDR_POS = 132
            LEN_POS  = 4
            ph = PortHandler(self.port)
            pk = PacketHandler(2.0)
            ph.openPort(); ph.setBaudRate(BAUD_RATE)
            gsr = GroupSyncRead(ph, pk, ADDR_POS, LEN_POS)
            for id_ in range(1, NUM_JOINTS + 1):
                gsr.addParam(id_)
            while self._running:
                if gsr.txRxPacket() == COMM_SUCCESS:
                    angles = []
                    for id_ in range(1, NUM_JOINTS + 1):
                        raw = gsr.getData(id_, ADDR_POS, LEN_POS)
                        if raw > 0x7FFFFFFF:   # two's-complement sign fix
                            raw -= 0x100000000
                        angles.append(float(np.rad2deg(raw / 2048.0 * np.pi)))
                    self.update.emit(angles)
                self.msleep(200)
            ph.closePort()          # port released immediately, no background thread
        except Exception:
            pass


class CalibrationWorker(QThread):
    done    = pyqtSignal(list, float, float, list)  # offsets, g_open, g_close, residuals_deg
    error   = pyqtSignal(str)
    def __init__(self, port):
        super().__init__(); self.port = port; self._stop = False
    def stop(self): self._stop = True
    def _read_raw(self, gsr, n_ids):
        """Return one averaged reading (radians) over 5 stable samples."""
        from dynamixel_sdk import COMM_SUCCESS
        ADDR_POS = 132; LEN_POS = 4
        samples = []
        attempts = 0
        while len(samples) < 5 and attempts < 20:
            attempts += 1
            if gsr.txRxPacket() == COMM_SUCCESS:
                row = []
                for id_ in range(1, n_ids + 1):
                    raw = gsr.getData(id_, ADDR_POS, LEN_POS)
                    if raw > 0x7FFFFFFF:
                        raw -= 0x100000000
                    row.append(raw / 2048.0 * np.pi)
                samples.append(row)
            time.sleep(0.02)
        if not samples:
            return None
        return list(np.mean(samples, axis=0))
    def run(self):
        try:
            from dynamixel_sdk import (
                GroupSyncRead, PacketHandler, PortHandler
            )
            ADDR_POS = 132; LEN_POS = 4
            n_ids = NUM_JOINTS + 1   # joints 1-7 + gripper 8
            ph = PortHandler(self.port)
            pk = PacketHandler(2.0)
            ph.openPort(); ph.setBaudRate(BAUD_RATE)
            gsr = GroupSyncRead(ph, pk, ADDR_POS, LEN_POS)
            for id_ in range(1, n_ids + 1):
                gsr.addParam(id_)
            # Warmup — flush stale data from the servo buffers
            for _ in range(15):
                if self._stop: ph.closePort(); return
                gsr.txRxPacket()
                time.sleep(0.02)
            if self._stop: ph.closePort(); return
            curr = self._read_raw(gsr, n_ids)
            ph.closePort()
            if curr is None:
                if not self._stop: self.error.emit("Failed to read servo positions — check cables.")
                return
            if self._stop: return
            # Find nearest multiple of π/2 for each joint
            best = []
            residuals_deg = []
            for i in range(NUM_JOINTS):
                best_off, best_err = 0.0, 1e9
                for off in np.linspace(-8 * np.pi, 8 * np.pi, 33):
                    err = abs(curr[i] - off)
                    if err < best_err:
                        best_err, best_off = err, off
                best.append(float(best_off))
                residuals_deg.append(float(np.rad2deg(best_err)))
            g_open  = float(np.rad2deg(curr[-1]) - 0.2)
            g_close = float(np.rad2deg(curr[-1]) - 42.0)
            if not self._stop:
                self.done.emit(best, g_open, g_close, residuals_deg)
        except Exception as e:
            if not self._stop:
                self.error.emit(str(e))


# ─────────────────────────────── Design tokens ────────────────────────────────

BG      = "#F5F5F7"      # light Apple-style gray
CARD    = "#FFFFFF"
TEXT    = "#1D1D1F"
MUTED   = "#6E6E73"
BORDER  = "#D2D2D7"
ACCENT  = "#0071E3"      # Apple blue
ACCENTh = "#0077ED"
GREEN   = "#34C759"
ORANGE  = "#FF9F0A"
RED     = "#FF3B30"

STYLE = f"""
* {{ font-family: -apple-system, 'SF Pro Text', 'Segoe UI', Arial, sans-serif; }}

QMainWindow, QWidget {{ background: {BG}; color: {TEXT}; }}

QFrame#card {{
    background: {CARD};
    border-radius: 14px;
    border: 1px solid {BORDER};
}}
QFrame#info_card {{
    background: #EBF5FB;
    border-radius: 10px;
    border: 1px solid #C6E2F5;
}}
QFrame#warn_card {{
    background: #FFF8EC;
    border-radius: 10px;
    border: 1px solid #FFD580;
}}
QFrame#err_card {{
    background: #FFF0F0;
    border-radius: 10px;
    border: 1px solid #FFB3B0;
}}
QFrame#check_row {{
    background: transparent;
}}

QPushButton {{
    background: {ACCENT};
    color: white;
    border: none;
    border-radius: 8px;
    padding: 9px 20px;
    font-size: 13px;
    font-weight: 600;
}}
QPushButton:hover   {{ background: {ACCENTh}; }}
QPushButton:pressed {{ background: #005BBF; }}
QPushButton:disabled {{ background: {BORDER}; color: {MUTED}; }}

QPushButton#ghost {{
    background: transparent;
    color: {ACCENT};
    border: 1.5px solid {ACCENT};
    padding: 8px 18px;
}}
QPushButton#ghost:hover {{ background: #EAF2FC; }}
QPushButton#ghost:disabled {{ border-color: {BORDER}; color: {MUTED}; }}

QPushButton#stop {{
    background: {RED};
    color: white;
}}
QPushButton#stop:hover {{ background: #D93025; }}
QPushButton#stop:disabled {{ background: {BORDER}; color: {MUTED}; }}

QPushButton#launch {{
    background: {GREEN};
    color: white;
    font-size: 14px;
    font-weight: 700;
    padding: 12px 32px;
    border-radius: 10px;
}}
QPushButton#launch:hover    {{ background: #2DB84C; }}
QPushButton#launch:disabled {{ background: {BORDER}; color: {MUTED}; }}

QPushButton#link {{
    background: transparent;
    color: {ACCENT};
    border: none;
    padding: 4px 0px;
    font-size: 12px;
    text-align: left;
}}
QPushButton#link:hover {{ color: {ACCENTh}; }}

QLabel#h1    {{ font-size: 24px; font-weight: 700; color: {TEXT}; }}
QLabel#h2    {{ font-size: 16px; font-weight: 600; color: {TEXT}; }}
QLabel#h3    {{ font-size: 13px; font-weight: 600; color: {TEXT}; }}
QLabel#body  {{ font-size: 13px; color: {MUTED}; line-height: 1.5; }}
QLabel#tag   {{ font-size: 11px; color: {MUTED}; font-weight: 500; }}
QLabel#ok    {{ font-size: 13px; color: {GREEN};  font-weight: 600; }}
QLabel#warn  {{ font-size: 13px; color: {ORANGE}; font-weight: 600; }}
QLabel#err   {{ font-size: 13px; color: {RED};    font-weight: 600; }}
QLabel#mono  {{ font-family: 'SF Mono', 'Consolas', monospace; font-size: 12px; color: {ACCENT}; }}

QProgressBar {{
    background: {BORDER};
    border: none;
    border-radius: 3px;
    max-height: 4px;
}}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 3px; }}
"""


# ─────────────────────────────── UI helpers ───────────────────────────────────

def shadow(widget, blur=20, offset_y=2, alpha=20):
    e = QGraphicsDropShadowEffect()
    e.setBlurRadius(blur)
    e.setOffset(0, offset_y)
    e.setColor(QColor(0, 0, 0, alpha))
    widget.setGraphicsEffect(e)
    return widget


def card_widget() -> QFrame:
    f = QFrame(); f.setObjectName("card"); shadow(f); return f


def hline() -> QFrame:
    l = QFrame(); l.setFrameShape(QFrame.Shape.HLine)
    l.setStyleSheet(f"color: {BORDER}; max-height: 1px;"); return l


def label(text, obj="body", wrap=False) -> QLabel:
    l = QLabel(text); l.setObjectName(obj)
    if wrap: l.setWordWrap(True)
    return l


def restyle(widget):
    widget.style().unpolish(widget); widget.style().polish(widget)


# ─────────────────────────────── Servo dot strip ──────────────────────────────

class ServoDots(QWidget):
    def __init__(self):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(10)
        self._dots: List[QLabel] = []
        for i in range(1, 9):
            col = QVBoxLayout(); col.setSpacing(2)
            dot = QLabel("●"); dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
            dot.setStyleSheet("font-size: 17px; color: #D2D2D7;")
            lbl = QLabel("G" if i == 8 else str(i))
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(f"font-size: 10px; color: {MUTED}; font-weight: 500;")
            col.addWidget(dot); col.addWidget(lbl)
            row.addLayout(col); self._dots.append(dot)
        row.addStretch()

    def refresh(self, found: list, highlight: int = -1):
        for i, dot in enumerate(self._dots, 1):
            if i == highlight:
                dot.setStyleSheet(f"font-size: 17px; color: {ACCENT};")
            elif i in found:
                dot.setStyleSheet(f"font-size: 17px; color: {GREEN};")
            else:
                dot.setStyleSheet("font-size: 17px; color: #D2D2D7;")


# ─────────────────────────────── Check row widget ─────────────────────────────

class CheckRow(QWidget):
    """Single preflight check item with icon + label + status."""
    def __init__(self, label_text: str):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 3, 0, 3)
        self._icon = QLabel("○")
        self._icon.setFixedWidth(20)
        self._icon.setStyleSheet(f"font-size: 14px; color: {MUTED};")
        self._lbl = QLabel(label_text)
        self._lbl.setObjectName("body")
        self._detail = QLabel("")
        self._detail.setObjectName("tag")
        self._detail.setAlignment(Qt.AlignmentFlag.AlignRight)
        row.addWidget(self._icon)
        row.addWidget(self._lbl)
        row.addStretch()
        row.addWidget(self._detail)

    def set_ok(self, detail=""):
        self._icon.setText("✓"); self._icon.setStyleSheet(f"font-size: 14px; color: {GREEN};")
        self._detail.setText(detail); self._detail.setStyleSheet(f"font-size: 11px; color: {GREEN};")

    def set_warn(self, detail=""):
        self._icon.setText("⚠"); self._icon.setStyleSheet(f"font-size: 14px; color: {ORANGE};")
        self._detail.setText(detail); self._detail.setStyleSheet(f"font-size: 11px; color: {ORANGE};")

    def set_err(self, detail=""):
        self._icon.setText("✗"); self._icon.setStyleSheet(f"font-size: 14px; color: {RED};")
        self._detail.setText(detail); self._detail.setStyleSheet(f"font-size: 11px; color: {RED};")

    def set_pending(self, detail="…"):
        self._icon.setText("○"); self._icon.setStyleSheet(f"font-size: 14px; color: {MUTED};")
        self._detail.setText(detail); self._detail.setStyleSheet(f"font-size: 11px; color: {MUTED};")


# ═════════════════════════════════════════════════════════════════════════════
# PAGE 1 — HOME
# ═════════════════════════════════════════════════════════════════════════════

class HomePage(QWidget):
    go_setup  = pyqtSignal()
    go_launch = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._scan_w: Optional[ScanWorker] = None
        self._port: Optional[str] = None
        self._servo_count = 0
        self._config_ok = False
        self._model_ok  = False
        self._static_issues: List[str] = []
        self._build()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(36, 30, 36, 30)
        root.setSpacing(18)

        # ── Title ──────────────────────────────────────────────
        title = label("GELLO  xArm7", "h1")
        sub   = label("Teleoperation Setup & Launcher", "body")
        root.addWidget(title)
        root.addWidget(sub)

        # ── Preflight checks card ──────────────────────────────
        c = card_widget()
        cl = QVBoxLayout(c)
        cl.setContentsMargins(22, 18, 22, 18)
        cl.setSpacing(2)

        hdr = label("System Checks", "h3")
        cl.addWidget(hdr)
        cl.addSpacing(6)

        self._chk_port   = CheckRow("U2D2 controller connected")
        self._chk_config = CheckRow("Configuration file exists")
        self._chk_model  = CheckRow("xArm7 MuJoCo model found")
        self._chk_setup  = CheckRow("Servo setup completed")
        self._chk_servos = CheckRow("Servos responding")

        for row in [self._chk_port, self._chk_config, self._chk_model,
                    self._chk_setup, self._chk_servos]:
            cl.addWidget(row)

        cl.addSpacing(8)
        cl.addWidget(hline())
        cl.addSpacing(8)

        # Servo dot strip
        dots_lbl = label("Servo IDs on bus", "tag")
        cl.addWidget(dots_lbl)
        cl.addSpacing(4)
        self._dots = ServoDots()
        cl.addWidget(self._dots)

        root.addWidget(c)

        # ── Guidance banner (shown when something fails) ───────
        self._guide_card = QFrame(); self._guide_card.setObjectName("warn_card")
        gl = QHBoxLayout(self._guide_card)
        gl.setContentsMargins(16, 12, 16, 12)
        gl.setSpacing(10)
        guide_icon = QLabel("ℹ"); guide_icon.setFixedWidth(18)
        guide_icon.setStyleSheet(f"font-size: 16px; color: {ORANGE};")
        self._guide_lbl = label("", "body", wrap=True)
        self._guide_lbl.setStyleSheet(f"color: #7C4F00; font-size: 12px;")
        gl.addWidget(guide_icon)
        gl.addWidget(self._guide_lbl)
        self._guide_card.setVisible(False)
        root.addWidget(self._guide_card)

        # ── Action buttons ─────────────────────────────────────
        self._setup_btn = QPushButton("⚙   Setup Servos  (first-time)")
        self._setup_btn.setObjectName("ghost")
        self._setup_btn.clicked.connect(self.go_setup)
        root.addWidget(self._setup_btn)

        self._launch_btn = QPushButton("▶   Calibrate & Launch Simulation")
        self._launch_btn.setObjectName("launch")
        self._launch_btn.setEnabled(False)
        self._launch_btn.clicked.connect(self.go_launch)
        root.addWidget(self._launch_btn)

        root.addStretch()
        foot = label("Move GELLO → simulation follows in real time", "tag")
        foot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(foot)

        QTimer.singleShot(200, self.refresh)

    def refresh(self):
        self._port = detect_port()
        config_ok  = CONFIG_SIM.exists()
        model_ok     = MODEL_PATH.exists()

        # Update checks
        if self._port:
            short = self._port.split("/")[-1][:30] if "/" in self._port else self._port
            self._chk_port.set_ok(short)
        else:
            self._chk_port.set_err("Not detected")

        self._chk_config.set_ok("Found") if config_ok else self._chk_config.set_err("Missing")
        self._chk_model.set_ok("Found")  if model_ok  else self._chk_model.set_err("Missing — run: git submodule update")
        self._chk_setup.set_pending("Scanning for IDs…")
        self._chk_servos.set_pending("Scanning…")

        self._config_ok = config_ok
        self._model_ok  = model_ok

        # Guidance (static issues only; servo count added after scan)
        self._static_issues = []
        if not self._port:   self._static_issues.append("U2D2 not found — plug in the USB cable and check drivers.")
        if not config_ok:    self._static_issues.append("Config file missing — check configs/xarm_sim_test.yaml exists.")
        if not model_ok:     self._static_issues.append("xArm7 model missing — run: git submodule update --init.")

        self._launch_btn.setEnabled(False)

        # Scan servos
        if self._port:
            if self._scan_w and self._scan_w.isRunning():
                self._scan_w.stop()
            self._scan_w = ScanWorker(self._port)
            self._scan_w.result.connect(self._on_scan)
            self._scan_w.start()
        else:
            self._guide_lbl.setText("  ·  ".join(self._static_issues))
            self._guide_card.setVisible(bool(self._static_issues))

    def _on_scan(self, data: dict):
        found = sorted(data.keys())
        n = len(found)
        self._dots.refresh(found)

        # "Servos responding" check
        if n == 8:
            self._chk_servos.set_ok("8 / 8 detected")
        elif n > 0:
            self._chk_servos.set_warn(f"{n} / 8 detected")
        else:
            self._chk_servos.set_err("None detected")

        # "Servo setup completed" check — truth is the actual ID count
        if n == 8:
            self._chk_setup.set_ok("All 8 IDs confirmed on bus")
        elif n > 0:
            missing = [i for i in range(1, 9) if i not in found]
            self._chk_setup.set_warn(f"Only {n}/8 IDs found — missing {missing}")
        else:
            self._chk_setup.set_err("No servo IDs detected — run Setup Servos")

        # Update guidance banner
        issues = list(self._static_issues)
        if n < 8:
            if n == 0:
                issues.append("No servos detected — check power and USB, then run Setup Servos.")
            else:
                missing = [i for i in range(1, 9) if i not in found]
                issues.append(f"Missing servo IDs {missing} — run Setup Servos to assign them.")
        if issues:
            self._guide_lbl.setText("  ·  ".join(issues))
            self._guide_card.setVisible(True)
        else:
            self._guide_card.setVisible(False)

        self._launch_btn.setEnabled(bool(self._port and self._config_ok and self._model_ok and n == 8))


# ═════════════════════════════════════════════════════════════════════════════
# PAGE 2 — ONBOARDING
# ═════════════════════════════════════════════════════════════════════════════

class OnboardingPage(QWidget):
    go_back = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._port: Optional[str] = None
        self._current  = 1
        self._assigned: List[int] = []
        self._scan_w: Optional[ScanWorker] = None
        self._id_w:   Optional[SetIDWorker] = None
        self._build()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(36, 26, 36, 26)
        root.setSpacing(14)

        # ── Header ─────────────────────────────────────────────
        hdr = QHBoxLayout()
        self._back_btn = QPushButton("← Back"); self._back_btn.setObjectName("ghost")
        self._back_btn.setFixedWidth(88); self._back_btn.clicked.connect(self._exit)
        self._title = label("Servo Setup", "h1")
        hdr.addWidget(self._back_btn); hdr.addSpacing(12)
        hdr.addWidget(self._title);   hdr.addStretch()
        root.addLayout(hdr)

        # ── Progress ───────────────────────────────────────────
        pr = QHBoxLayout()
        self._prog_lbl = label("Step 1 of 8", "tag")
        pr.addWidget(self._prog_lbl); pr.addStretch()
        root.addLayout(pr)
        self._progress = QProgressBar(); self._progress.setMaximum(8)
        root.addWidget(self._progress)

        # ── Overview card (shown at step 1 only) ───────────────
        self._intro_card = QFrame(); self._intro_card.setObjectName("info_card")
        il = QHBoxLayout(self._intro_card)
        il.setContentsMargins(16, 12, 16, 12); il.setSpacing(10)
        intro_icon = QLabel("ℹ"); intro_icon.setStyleSheet(f"font-size: 16px; color: {ACCENT}; font-weight: bold;")
        intro_icon.setFixedWidth(20)
        intro_txt = label(
            "You will connect each servo one at a time and assign it a unique ID. "
            "This only needs to be done once. Have all 8 servos unplugged from the data chain "
            "before you begin — you will plug in one at a time when prompted.",
            "body", wrap=True
        )
        intro_txt.setStyleSheet(f"color: #0A3D6B; font-size: 12px;")
        il.addWidget(intro_icon); il.addWidget(intro_txt)
        root.addWidget(self._intro_card)

        # ── Instruction card ───────────────────────────────────
        c = card_widget()
        cl = QVBoxLayout(c)
        cl.setContentsMargins(22, 18, 22, 18); cl.setSpacing(10)

        self._joint_badge = QLabel()
        self._joint_badge.setStyleSheet(
            f"background: {ACCENT}; color: white; border-radius: 6px; "
            f"padding: 3px 10px; font-size: 11px; font-weight: 700;"
        )
        self._joint_badge.setFixedHeight(22)
        self._joint_badge.setSizePolicy(
            self._joint_badge.sizePolicy().horizontalPolicy(),
            self._joint_badge.sizePolicy().verticalPolicy()
        )

        badge_row = QHBoxLayout()
        badge_row.addWidget(self._joint_badge); badge_row.addStretch()
        cl.addLayout(badge_row)

        self._instr_title = label("", "h2")
        self._instr_body  = label("", "body", wrap=True)
        self._status_lbl  = label("", "body", wrap=True)

        cl.addWidget(self._instr_title)
        cl.addWidget(self._instr_body)
        cl.addSpacing(4)
        cl.addWidget(self._status_lbl)
        root.addWidget(c)

        # ── Dots ───────────────────────────────────────────────
        self._dots = ServoDots()
        root.addWidget(self._dots)

        # ── Troubleshoot card (hidden) ─────────────────────────
        self._trouble = card_widget()
        self._trouble.setVisible(False)
        tc = QVBoxLayout(self._trouble)
        tc.setContentsMargins(20, 14, 20, 14); tc.setSpacing(10)
        tc.addWidget(label("Troubleshooting guide", "h3"))
        tc.addSpacing(2)
        for title_t, detail_t in TROUBLESHOOT:
            row = QHBoxLayout(); row.setSpacing(10)
            badge = QLabel(title_t)
            badge.setStyleSheet(
                f"background: #FFF0F0; color: {RED}; border-radius: 4px; "
                f"padding: 2px 8px; font-size: 11px; font-weight: 600;"
            )
            badge.setFixedWidth(100)
            detail = label(detail_t, "body", wrap=True)
            row.addWidget(badge); row.addWidget(detail)
            tc.addLayout(row)
        root.addWidget(self._trouble)

        root.addStretch()

        # ── Buttons ────────────────────────────────────────────
        btn_row = QHBoxLayout(); btn_row.setSpacing(10)
        self._stop_btn = QPushButton("✕  Stop"); self._stop_btn.setObjectName("stop")
        self._stop_btn.setFixedWidth(96); self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._do_stop)

        self._scan_btn = QPushButton("🔍  Scan & Assign")
        self._scan_btn.clicked.connect(self._do_scan)

        btn_row.addWidget(self._stop_btn); btn_row.addWidget(self._scan_btn)
        root.addLayout(btn_row)

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def start(self, port: str):
        self._port = port
        self._current = 1; self._assigned = []
        self._progress.setValue(0)
        self._trouble.setVisible(False)
        self._dots.refresh([])
        self._intro_card.setVisible(True)
        self._scan_btn.clicked.disconnect()
        self._scan_btn.clicked.connect(self._do_scan)
        self._scan_btn.setEnabled(True)
        self._scan_btn.setText("🔍  Scan & Assign")
        self._stop_btn.setEnabled(False)
        self._update_ui()

    def _update_ui(self):
        n = self._current
        jname, jrole, jdesc = JOINT_INFO[n - 1]
        self._prog_lbl.setText(f"Step {n} of 8")
        self._progress.setValue(n - 1)
        self._intro_card.setVisible(n == 1)
        self._joint_badge.setText(f"  Servo {n}  ·  {jname}  ({jrole})  ")
        self._instr_title.setText(f"Connect the {jname} servo only")
        self._instr_body.setText(
            f"Location: {jdesc}\n\n"
            "Plug this ONE servo into the power board. "
            "Make sure all other servos are disconnected from the data line. "
            "The servo should blink once when it powers on. "
            "Then click Scan."
        )
        self._status_lbl.setText("")
        self._trouble.setVisible(False)
        self._dots.refresh(self._assigned, highlight=n)
        self._scan_btn.setEnabled(True)
        self._scan_btn.setText("🔍  Scan & Assign")
        self._stop_btn.setEnabled(False)

    # ── Actions ────────────────────────────────────────────────────────────────

    def _do_scan(self):
        if not self._port:
            self._set_status("No USB port detected — check U2D2 connection.", "err"); return
        self._scan_btn.setEnabled(False); self._scan_btn.setText("Scanning…")
        self._stop_btn.setEnabled(True)
        self._trouble.setVisible(False)
        self._set_status("Scanning for servo on the bus…", "muted")
        self._scan_w = ScanWorker(self._port)
        self._scan_w.result.connect(self._on_scan); self._scan_w.start()

    def _on_scan(self, data: dict):
        self._stop_btn.setEnabled(False)
        n = len(data)

        if n == 0:
            self._set_status(
                "❌  No servo detected. Make sure it is powered and the data cable is connected.", "err")
            self._trouble.setVisible(True)
            self._reset_scan_btn(); return

        if n > 1:
            ids = ", ".join(str(i) for i in sorted(data.keys()))
            self._set_status(
                f"⚠  Multiple servos detected ({ids}). "
                "Disconnect all except this one servo, then scan again.", "warn")
            self._reset_scan_btn(); return

        found = list(data.keys())[0]
        target = self._current
        if found == target:
            self._set_status(f"✓  Servo found — already has the correct ID ({target})", "ok")
            QTimer.singleShot(700, self._advance)
        else:
            self._set_status(
                f"Found servo at ID {found}. Reassigning to ID {target}…", "warn")
            self._id_w = SetIDWorker(self._port, found, target)
            self._id_w.done.connect(self._on_id_set); self._id_w.start()

    def _on_id_set(self, ok: bool):
        if ok:
            self._set_status(f"✓  Servo successfully assigned ID {self._current}", "ok")
            QTimer.singleShot(700, self._advance)
        else:
            self._set_status("❌  Failed to set ID. Try disconnecting and reconnecting the servo.", "err")
            self._trouble.setVisible(True); self._reset_scan_btn()

    def _advance(self):
        self._assigned.append(self._current); self._current += 1
        if self._current > 8:
            self._progress.setValue(8)
            self._dots.refresh(self._assigned)
            profile = load_profile()
            profile.update({"onboarded": True, "port": self._port})
            save_profile(profile)
            self._title.setText("Setup Complete ✓")
            self._joint_badge.setText("  All 8 servos assigned  ")
            self._joint_badge.setStyleSheet(
                f"background: {GREEN}; color: white; border-radius: 6px; "
                f"padding: 3px 10px; font-size: 11px; font-weight: 700;")
            self._instr_title.setText("All servos are ready")
            self._instr_body.setText(
                "Daisy-chain all 8 servos together (OUT → IN through the chain) "
                "and connect them all to the power board.\n\n"
                "Your GELLO is now fully configured.")
            self._status_lbl.setText("")
            self._scan_btn.setEnabled(True); self._scan_btn.setText("← Back to Home")
            self._scan_btn.clicked.disconnect(); self._scan_btn.clicked.connect(self._exit)
            self._stop_btn.setEnabled(False)
        else:
            self._update_ui()

    def _do_stop(self):
        if self._scan_w: self._scan_w.stop()
        if self._id_w and self._id_w.isRunning(): self._id_w.terminate()
        self._set_status("Stopped.", "muted"); self._reset_scan_btn()
        self._stop_btn.setEnabled(False)

    def _exit(self):
        self._do_stop(); self.go_back.emit()

    def _reset_scan_btn(self):
        self._scan_btn.setEnabled(True); self._scan_btn.setText("🔍  Scan & Assign")

    def _set_status(self, text, style):
        self._status_lbl.setText(text)
        c = {
            "ok": GREEN, "warn": ORANGE, "err": RED, "muted": MUTED
        }.get(style, MUTED)
        self._status_lbl.setStyleSheet(f"font-size: 13px; color: {c};")


# ═════════════════════════════════════════════════════════════════════════════
# PAGE 3 — CALIBRATE & LAUNCH
# ═════════════════════════════════════════════════════════════════════════════

class CalibratePage(QWidget):
    go_back = pyqtSignal()
    go_sim  = pyqtSignal(object)   # emits SimulationThread

    def __init__(self):
        super().__init__()
        self._port: Optional[str] = None
        self._scan_w: Optional[ScanWorker]         = None
        self._live_w: Optional[LivePositionWorker]  = None
        self._cal_w:  Optional[CalibrationWorker]   = None
        self._proc:   Optional[subprocess.Popen]    = None
        self._countdown = 3
        self._ctimer = QTimer(); self._ctimer.timeout.connect(self._tick)
        self._build()

    # ── Build ──────────────────────────────────────────────────────────────────

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(36, 26, 36, 26); root.setSpacing(14)

        # Header
        hdr = QHBoxLayout()
        self._back_btn = QPushButton("← Back"); self._back_btn.setObjectName("ghost")
        self._back_btn.setFixedWidth(88); self._back_btn.clicked.connect(self._exit)
        title = label("Calibrate & Launch", "h1")
        hdr.addWidget(self._back_btn); hdr.addSpacing(12)
        hdr.addWidget(title); hdr.addStretch()
        root.addLayout(hdr)

        # ── Servo check card ───────────────────────────────────
        sc = card_widget()
        sl = QVBoxLayout(sc); sl.setContentsMargins(22, 16, 22, 16); sl.setSpacing(10)
        sch = QHBoxLayout()
        sch.addWidget(label("Servo Check", "h3"))
        sch.addStretch()
        self._rescan = QPushButton("↻  Re-scan"); self._rescan.setObjectName("ghost")
        self._rescan.setFixedWidth(100); self._rescan.clicked.connect(self._do_scan)
        sch.addWidget(self._rescan)
        sl.addLayout(sch)
        self._scan_lbl = label("Scanning…", "body")
        sl.addWidget(self._scan_lbl)
        self._scan_dots = ServoDots()
        sl.addWidget(self._scan_dots)
        self._scan_issue = label("", "warn", wrap=True); self._scan_issue.setVisible(False)
        sl.addWidget(self._scan_issue)
        root.addWidget(sc)

        # ── Zero-position instruction card ─────────────────────
        zc = QFrame(); zc.setObjectName("info_card")
        zl = QHBoxLayout(zc); zl.setContentsMargins(16, 12, 16, 12); zl.setSpacing(10)
        z_icon = QLabel("🦾"); z_icon.setFixedWidth(22)
        z_icon.setStyleSheet("font-size: 18px;")
        z_txt = label(
            "<b>Zero position:</b> arm pointing straight up, all joints at 0°, "
            "matching the xArm7 default upright pose. "
            "Check the live readings below — all angles should be near 0° before calibrating.",
            "body", wrap=True
        )
        z_txt.setStyleSheet(f"color: #0A3D6B; font-size: 12px;")
        zl.addWidget(z_icon); zl.addWidget(z_txt)
        root.addWidget(zc)

        # ── Live joint angles ──────────────────────────────────
        lc = card_widget()
        ll = QVBoxLayout(lc); ll.setContentsMargins(22, 14, 22, 14); ll.setSpacing(6)
        lhdr = QHBoxLayout()
        lhdr.addWidget(label("Live Joint Angles  (°)", "h3"))
        lhdr.addStretch()
        self._live_lbl = label("—", "tag")
        lhdr.addWidget(self._live_lbl)
        ll.addLayout(lhdr)
        self._angle_row = QHBoxLayout(); self._angle_row.setSpacing(6)
        self._angle_labels: List[QLabel] = []
        for i in range(NUM_JOINTS):
            col = QVBoxLayout(); col.setSpacing(2)
            val = QLabel("—"); val.setObjectName("mono")
            val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val.setStyleSheet(f"font-family: monospace; font-size: 12px; color: {ACCENT}; font-weight: 600;")
            lbl_ = label(f"J{i+1}", "tag")
            lbl_.setAlignment(Qt.AlignmentFlag.AlignCenter)
            col.addWidget(val); col.addWidget(lbl_)
            self._angle_row.addLayout(col)
            self._angle_labels.append(val)
        self._angle_row.addStretch()
        ll.addLayout(self._angle_row)
        root.addWidget(lc)

        # ── Calibration card ───────────────────────────────────
        cc = card_widget()
        cl_ = QVBoxLayout(cc); cl_.setContentsMargins(22, 16, 22, 16); cl_.setSpacing(8)
        self._cal_title  = label("Ready to calibrate", "h3")
        self._cal_body   = label(
            "When all joint angles above are near 0°, click Run Calibration.", "body", wrap=True)
        self._spinner    = QProgressBar(); self._spinner.setRange(0, 0)
        self._spinner.setVisible(False)
        self._cal_status = label("", "body", wrap=True)
        cl_.addWidget(self._cal_title); cl_.addWidget(self._cal_body)
        cl_.addWidget(self._spinner);   cl_.addWidget(self._cal_status)
        root.addWidget(cc)

        # ── Result ─────────────────────────────────────────────
        self._result_card = card_widget(); self._result_card.setVisible(False)
        rl = QVBoxLayout(self._result_card); rl.setContentsMargins(20, 12, 20, 12); rl.setSpacing(4)
        rl.addWidget(label("Saved Calibration", "tag"))
        self._result_lbl = label("", "mono", wrap=True)
        rl.addWidget(self._result_lbl)
        root.addWidget(self._result_card)

        root.addStretch()

        # ── Buttons ────────────────────────────────────────────
        btn_row = QHBoxLayout(); btn_row.setSpacing(10)
        self._stop_btn = QPushButton("✕  Stop"); self._stop_btn.setObjectName("stop")
        self._stop_btn.setFixedWidth(96); self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._do_stop)
        self._cal_btn  = QPushButton("📐  Run Calibration")
        self._cal_btn.clicked.connect(self._do_calibrate)
        self._launch_btn = QPushButton("▶  Launch Simulation")
        self._launch_btn.setObjectName("launch")
        self._launch_btn.setVisible(False)
        self._launch_btn.clicked.connect(self._do_launch)
        self._real_btn = QPushButton("🦾  Launch Real Arm")
        self._real_btn.setObjectName("ghost")
        self._real_btn.setVisible(False)
        self._real_btn.clicked.connect(self._do_launch_real)
        btn_row.addWidget(self._stop_btn)
        btn_row.addWidget(self._cal_btn)
        btn_row.addWidget(self._launch_btn)
        btn_row.addWidget(self._real_btn)
        root.addLayout(btn_row)

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def start(self, port: str):
        self._port = port
        self._ctimer.stop()
        self._last_angles: List[float] = []
        self._zero_warned = False
        # Reset
        self._cal_title.setText("Ready to calibrate")
        self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {TEXT};")
        self._cal_body.setText("When all joint angles above are near 0°, click Run Calibration.")
        self._spinner.setVisible(False); self._cal_status.setText("")
        self._result_card.setVisible(False)
        self._cal_btn.setVisible(True); self._cal_btn.setEnabled(True); self._cal_btn.setText("📐  Run Calibration")
        self._launch_btn.setVisible(False)
        self._real_btn.setVisible(False)
        self._stop_btn.setEnabled(False); self._back_btn.setEnabled(True)
        # Start live & scan
        self._do_scan()
        self._start_live()

    def _start_live(self):
        if self._live_w and self._live_w.isRunning():
            self._live_w.stop(); self._live_w.wait(300)
        self._live_w = LivePositionWorker(self._port)
        self._live_w.update.connect(self._on_live)
        self._live_w.start()
        self._live_lbl.setText("● live")
        self._live_lbl.setStyleSheet(f"font-size: 11px; color: {GREEN};")

    def _on_live(self, angles: list):
        self._last_angles = angles
        for i, (lbl_, ang) in enumerate(zip(self._angle_labels, angles)):
            lbl_.setText(f"{ang:+.1f}")
            close = abs(ang) < 15
            lbl_.setStyleSheet(
                f"font-family: monospace; font-size: 12px; font-weight: 600; "
                f"color: {GREEN if close else ORANGE};"
            )

    # ── Servo scan ─────────────────────────────────────────────────────────────

    def _do_scan(self):
        self._scan_lbl.setText("Scanning…"); self._scan_lbl.setStyleSheet(f"color: {MUTED};")
        self._scan_issue.setVisible(False); self._rescan.setEnabled(False)
        self._scan_dots.refresh([])
        if self._scan_w and self._scan_w.isRunning(): self._scan_w.stop()
        self._scan_w = ScanWorker(self._port)
        self._scan_w.result.connect(self._on_scan_result); self._scan_w.start()

    def _on_scan_result(self, data: dict):
        self._rescan.setEnabled(True)
        found = sorted(data.keys()); n = len(found)
        self._scan_dots.refresh(found)
        if n == 8:
            self._scan_lbl.setText("✓  All 8 servos detected")
            self._scan_lbl.setStyleSheet(f"font-size: 13px; color: {GREEN}; font-weight: 600;")
            self._scan_issue.setVisible(False)
            self._cal_btn.setEnabled(True)
        else:
            missing = [i for i in range(1, 9) if i not in found]
            self._scan_lbl.setText(f"⚠  {n} / 8 servos detected")
            self._scan_lbl.setStyleSheet(f"font-size: 13px; color: {ORANGE}; font-weight: 600;")
            self._scan_issue.setText(
                f"Missing servo IDs: {missing}. "
                "Check cables/power or run Setup Servos from the home screen."
            )
            self._scan_issue.setVisible(True)
            self._cal_btn.setEnabled(n > 0)

    # ── Calibration ────────────────────────────────────────────────────────────

    def _do_calibrate(self):
        # Zero-position auto-detection: check if live angles suggest the arm isn't at zero
        if hasattr(self, '_last_angles') and self._last_angles:
            far = [i + 1 for i, a in enumerate(self._last_angles) if abs(a) > 40]
            if far and not hasattr(self, '_zero_warned'):
                self._zero_warned = True
                self._cal_title.setText(f"⚠  Joints {far} appear far from zero position")
                self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 700; color: {ORANGE};")
                self._cal_body.setText(
                    "The GELLO should be in the zero position (arm straight up, all joints at 0°) "
                    "before calibrating. Adjust the arm and click Run Calibration again, or click "
                    "again to proceed anyway.")
                return
        self._zero_warned = False  # reset for next time

        self._cal_btn.setEnabled(False); self._cal_btn.setText("Preparing…")
        self._back_btn.setEnabled(False); self._stop_btn.setEnabled(True)
        self._result_card.setVisible(False); self._launch_btn.setVisible(False)
        # Release the port before calibration opens it
        if self._live_w and self._live_w.isRunning():
            self._live_w.stop(); self._live_w.wait(1000)
        if self._scan_w and self._scan_w.isRunning():
            self._scan_w.stop(); self._scan_w.wait(500)
        self._live_lbl.setText("paused"); self._live_lbl.setStyleSheet(f"font-size: 11px; color: {MUTED};")
        # Countdown
        self._countdown = 3
        self._cal_title.setText(f"⚠  Hold completely still — starting in 3…")
        self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 700; color: {ORANGE};")
        self._cal_body.setText("Do NOT move the GELLO. Calibration reads your servo positions in 3 seconds.")
        self._ctimer.start(1000)

    def _tick(self):
        self._countdown -= 1
        if self._countdown > 0:
            self._cal_title.setText(f"⚠  Hold completely still — starting in {self._countdown}…")
        else:
            self._ctimer.stop()
            self._cal_title.setText("Reading servo positions…")
            self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {TEXT};")
            self._cal_body.setText("Computing joint offsets — keep the GELLO still.")
            self._spinner.setVisible(True)
            self._cal_w = CalibrationWorker(self._port)
            self._cal_w.done.connect(self._on_cal_done)
            self._cal_w.error.connect(self._on_cal_error)
            self._cal_w.start()

    def _on_cal_done(self, offsets: list, g_open: float, g_close: float, residuals: list):
        self._spinner.setVisible(False); self._stop_btn.setEnabled(False)
        port = self._port or ""
        try:
            update_yaml(offsets, g_open, g_close, port, CONFIG_SIM)
            if CONFIG_REAL.exists():
                update_yaml(offsets, g_open, g_close, port, CONFIG_REAL)
        except Exception as e:
            self._on_cal_error(str(e)); return

        # Build result text with per-joint residuals
        lines = "Offsets: [" + ", ".join(f"{o:.3f}" for o in offsets) + "]\n"
        lines += f"Gripper — open: {g_open:.1f}°   close: {g_close:.1f}°\n"
        bad = [i + 1 for i, r in enumerate(residuals) if r > np.rad2deg(CAL_RESIDUAL_WARN)]
        if bad:
            lines += f"\n⚠  Joints {bad} are >{np.rad2deg(CAL_RESIDUAL_WARN):.0f}° from nearest π/2 — check zero pose"
        self._result_lbl.setText(lines)
        self._result_card.setVisible(True)

        if bad:
            self._cal_title.setText("⚠  Calibration complete — some joints may be off")
            self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 700; color: {ORANGE};")
            self._cal_body.setText(
                f"Joints {bad} have high residual error. Make sure the GELLO is at the exact zero "
                "position (arm straight up) before calibrating. You can retry or proceed.")
            self._cal_btn.setVisible(True); self._cal_btn.setEnabled(True)
            self._cal_btn.setText("📐  Retry Calibration")
        else:
            self._cal_title.setText("✓  Calibration complete")
            self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 700; color: {GREEN};")
            self._cal_body.setText("Configuration saved. Launch simulation or connect the real arm.")
            self._cal_btn.setVisible(False)

        self._launch_btn.setVisible(True)
        self._real_btn.setVisible(CONFIG_REAL.exists())
        self._back_btn.setEnabled(True)
        # Restart live view
        self._start_live()

    def _on_cal_error(self, msg: str):
        self._spinner.setVisible(False)
        self._cal_title.setText("Calibration failed")
        self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 700; color: {RED};")
        self._cal_status.setText(f"Error: {msg}")
        self._cal_status.setStyleSheet(f"color: {RED}; font-size: 12px;")
        self._cal_btn.setEnabled(True); self._cal_btn.setText("📐  Retry Calibration")
        self._stop_btn.setEnabled(False); self._back_btn.setEnabled(True)
        self._start_live()

    # ── Launch ─────────────────────────────────────────────────────────────────

    def _do_launch(self):
        # Release the serial port before creating the agent in-process
        if self._live_w and self._live_w.isRunning():
            self._live_w.stop(); self._live_w.wait(1000)
        if self._scan_w and self._scan_w.isRunning():
            self._scan_w.stop(); self._scan_w.wait(500)
        self._live_lbl.setText("released"); self._live_lbl.setStyleSheet(f"font-size: 11px; color: {MUTED};")
        time.sleep(0.3)  # let OS fully release serial fd

        self._launch_btn.setText("Starting…"); self._launch_btn.setEnabled(False)
        self._real_btn.setEnabled(False); self._back_btn.setEnabled(False)

        try:
            from omegaconf import OmegaConf
            cfg = OmegaConf.to_container(OmegaConf.load(str(CONFIG_SIM)), resolve=True)
            agent = _instantiate(cfg["agent"])
            xml_path = str(BASE_DIR / cfg["robot"]["xml_path"])
            sim = SimulationThread(xml_path, agent)
            self.go_sim.emit(sim)
        except Exception as e:
            self._launch_btn.setText("▶  Launch Simulation"); self._launch_btn.setEnabled(True)
            self._real_btn.setEnabled(True); self._back_btn.setEnabled(True)
            self._cal_status.setText(f"Launch failed: {e}")
            self._cal_status.setStyleSheet(f"color: {RED}; font-size: 12px;")
            self._start_live()

    def _do_launch_real(self):
        """Launch with the real xArm7 arm instead of the MuJoCo simulation."""
        if self._live_w and self._live_w.isRunning():
            self._live_w.stop(); self._live_w.wait(1000)
        if self._scan_w and self._scan_w.isRunning():
            self._scan_w.stop(); self._scan_w.wait(500)
        self._live_lbl.setText("released"); self._live_lbl.setStyleSheet(f"font-size: 11px; color: {MUTED};")

        for candidate in [
            BASE_DIR / ".venv" / "bin" / "python",
            BASE_DIR / ".venv" / "Scripts" / "python.exe",
            Path(sys.executable),
        ]:
            if Path(candidate).exists():
                python = str(candidate); break
        else:
            python = sys.executable

        self._proc = subprocess.Popen(
            [python, str(BASE_DIR / "experiments" / "launch_yaml.py"),
             "--left-config-path", str(CONFIG_REAL)],
            cwd=str(BASE_DIR),
            start_new_session=True,
        )
        self._real_btn.setText("Real arm running…")
        self._real_btn.setEnabled(False)
        self._launch_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._stop_btn.setText("✕  Stop Arm")

    # ── Stop ───────────────────────────────────────────────────────────────────

    def _kill_proc(self):
        """Kill the simulation subprocess and its entire process group."""
        if self._proc and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
        self._proc = None

    def _do_stop(self):
        self._ctimer.stop()
        if self._cal_w and self._cal_w.isRunning():
            self._cal_w.stop(); self._cal_w.wait(500)
        self._kill_proc()
        self._spinner.setVisible(False)
        self._cal_title.setText("Stopped")
        self._cal_title.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {MUTED};")
        self._cal_btn.setVisible(True); self._cal_btn.setEnabled(True)
        self._cal_btn.setText("📐  Run Calibration")
        self._launch_btn.setText("▶  Launch Simulation"); self._launch_btn.setEnabled(True)
        self._real_btn.setText("🦾  Launch Real Arm"); self._real_btn.setEnabled(True)
        self._stop_btn.setEnabled(False); self._stop_btn.setText("✕  Stop")
        self._back_btn.setEnabled(True)
        self._start_live()

    def _exit(self):
        self._ctimer.stop()
        if self._live_w: self._live_w.stop()
        if self._scan_w: self._scan_w.stop()
        if self._cal_w and self._cal_w.isRunning(): self._cal_w.stop()
        self._kill_proc()
        self.go_back.emit()


# ═════════════════════════════════════════════════════════════════════════════
# PAGE 4 — EMBEDDED SIMULATION
# ═════════════════════════════════════════════════════════════════════════════

def _instantiate(cfg):
    """Instantiate an object from a dict with _target_ key."""
    if isinstance(cfg, dict) and "_target_" in cfg:
        module_path, class_name = cfg["_target_"].rsplit(".", 1)
        cls = getattr(importlib.import_module(module_path), class_name)
        kwargs = {k: v for k, v in cfg.items() if k != "_target_"}
        return cls(**{k: _instantiate(v) for k, v in kwargs.items()})
    elif isinstance(cfg, dict):
        return {k: _instantiate(v) for k, v in cfg.items()}
    elif isinstance(cfg, list):
        return [_instantiate(v) for v in cfg]
    return cfg


class SimulationThread(QThread):
    """Runs MuJoCo physics + GelloAgent in a tight loop, emitting frames."""
    frame_ready = pyqtSignal(object)       # numpy (H,W,3) uint8
    telemetry   = pyqtSignal(dict)
    error       = pyqtSignal(str)

    def __init__(self, xml_path: str, agent, render_w=800, render_h=600):
        super().__init__()
        self._xml_path = xml_path
        self._agent = agent
        self._rw, self._rh = render_w, render_h
        self._running = True
        self._cam_lock = threading.Lock()
        self._cam_azimuth = 150.0
        self._cam_elevation = -20.0
        self._cam_distance = 1.8
        self._cam_lookat = np.array([0.0, 0.0, 0.3])

    def update_camera(self, daz=0.0, dele=0.0, ddist=0.0):
        with self._cam_lock:
            self._cam_azimuth += daz
            self._cam_elevation = np.clip(self._cam_elevation + dele, -89, 89)
            self._cam_distance = max(0.3, self._cam_distance + ddist)

    def stop(self):
        self._running = False

    def run(self):
        try:
            import mujoco
            from gello.robots.sim_robot import build_scene
            arena = build_scene(self._xml_path)
            xml_string = arena.to_xml_string()
            assets = {}
            for asset in arena.asset.all_children():
                if asset.tag == "mesh":
                    f = asset.file
                    assets[f.get_vfs_filename()] = f.contents
            model = mujoco.MjModel.from_xml_string(xml_string, assets)
            data = mujoco.MjData(model)
            num_joints = model.nu
            renderer = mujoco.Renderer(model, height=self._rh, width=self._rw)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.azimuth = self._cam_azimuth
            cam.elevation = self._cam_elevation
            cam.distance = self._cam_distance
            cam.lookat[:] = self._cam_lookat
            scene = mujoco.MjvScene(model, maxgeom=1000)
            opt = mujoco.MjvOption()
            render_interval = 1.0 / 60.0
            last_render = 0.0
            obs = {}
            while self._running:
                t0 = time.time()
                # Read GELLO servos
                try:
                    action = self._agent.act(obs)
                    if len(action) > num_joints:
                        action = action[:num_joints]
                    data.ctrl[:len(action)] = action
                except Exception:
                    pass
                mujoco.mj_step(model, data)
                now = time.time()
                if now - last_render >= render_interval:
                    with self._cam_lock:
                        cam.azimuth = self._cam_azimuth
                        cam.elevation = self._cam_elevation
                        cam.distance = self._cam_distance
                    renderer.update_scene(data, cam)
                    frame = renderer.render().copy()
                    self.frame_ready.emit(frame)
                    last_render = now
                # Telemetry at 10 Hz
                elapsed = time.time() - t0
                hz = 1.0 / elapsed if elapsed > 0 else 0
                self.telemetry.emit({
                    "joint_deg": [float(np.rad2deg(data.qpos[i])) for i in range(min(NUM_JOINTS, num_joints))],
                    "hz": hz,
                    "sim_time": float(data.time),
                })
                # Rate limit to model timestep
                remaining = model.opt.timestep - (time.time() - t0)
                if remaining > 0:
                    time.sleep(remaining)
            renderer.close()
        except Exception as e:
            self.error.emit(str(e))


class MujocoViewerWidget(QWidget):
    """Viewport with mouse orbit controls, screenshot and recording."""
    def __init__(self):
        super().__init__()
        self._sim: Optional[SimulationThread] = None
        self._last_frame: Optional[np.ndarray] = None
        self._recording = False
        self._rec_frames: List[np.ndarray] = []
        self._drag_last = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        self._viewport = QLabel()
        self._viewport.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._viewport.setStyleSheet("background: #000; border-radius: 8px;")
        self._viewport.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._viewport.setMinimumSize(400, 300)
        lay.addWidget(self._viewport)
        # Toolbar
        tb = QHBoxLayout(); tb.setSpacing(8)
        self._snap_btn = QPushButton("📷  Screenshot")
        self._snap_btn.setObjectName("ghost"); self._snap_btn.setFixedHeight(30)
        self._snap_btn.clicked.connect(self._screenshot)
        self._rec_btn = QPushButton("⏺  Record")
        self._rec_btn.setObjectName("ghost"); self._rec_btn.setFixedHeight(30)
        self._rec_btn.clicked.connect(self._toggle_record)
        tb.addWidget(self._snap_btn); tb.addWidget(self._rec_btn); tb.addStretch()
        lay.addLayout(tb)

    def attach(self, sim: SimulationThread):
        self._sim = sim
        sim.frame_ready.connect(self._on_frame)

    def _on_frame(self, frame: np.ndarray):
        self._last_frame = frame
        if self._recording:
            self._rec_frames.append(frame.copy())
        h, w, _ = frame.shape
        qimg = QImage(frame.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        vw, vh = self._viewport.width(), self._viewport.height()
        pm = QPixmap.fromImage(qimg).scaled(vw, vh, Qt.AspectRatioMode.KeepAspectRatio,
                                             Qt.TransformationMode.SmoothTransformation)
        self._viewport.setPixmap(pm)

    # ── Mouse orbit ───────────────────────────────────────────────────────────
    def mousePressEvent(self, ev):
        self._drag_last = ev.position()
    def mouseMoveEvent(self, ev):
        if self._drag_last and self._sim:
            dx = ev.position().x() - self._drag_last.x()
            dy = ev.position().y() - self._drag_last.y()
            self._sim.update_camera(daz=-dx * 0.5, dele=dy * 0.3)
            self._drag_last = ev.position()
    def mouseReleaseEvent(self, ev):
        self._drag_last = None
    def wheelEvent(self, ev):
        if self._sim:
            d = -ev.angleDelta().y() / 600.0
            self._sim.update_camera(ddist=d)

    # ── Screenshot ────────────────────────────────────────────────────────────
    def _screenshot(self):
        if self._last_frame is None: return
        path, _ = QFileDialog.getSaveFileName(self, "Save Screenshot", "screenshot.png", "PNG (*.png)")
        if path:
            from PIL import Image
            Image.fromarray(self._last_frame).save(path)

    # ── Recording ─────────────────────────────────────────────────────────────
    def _toggle_record(self):
        if not self._recording:
            self._recording = True; self._rec_frames = []
            self._rec_btn.setText("⏹  Stop Recording")
            self._rec_btn.setStyleSheet(f"color: {RED}; border-color: {RED};")
        else:
            self._recording = False
            self._rec_btn.setText("⏺  Record")
            self._rec_btn.setStyleSheet("")
            if not self._rec_frames: return
            path, _ = QFileDialog.getSaveFileName(self, "Save Recording", "recording.mp4",
                                                   "MP4 (*.mp4);;PNG sequence directory (*)")
            if not path: return
            self._save_recording(path)

    def _save_recording(self, path: str):
        try:
            import cv2
            h, w, _ = self._rec_frames[0].shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(path, fourcc, 30, (w, h))
            for f in self._rec_frames:
                writer.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
            writer.release()
        except ImportError:
            # Fallback: save as PNG sequence
            out = Path(path).with_suffix("")
            out.mkdir(parents=True, exist_ok=True)
            from PIL import Image
            for i, f in enumerate(self._rec_frames):
                Image.fromarray(f).save(str(out / f"frame_{i:05d}.png"))


class PerformanceDashboard(QWidget):
    """Side panel showing live joint angles, loop Hz, and sim time."""
    def __init__(self):
        super().__init__()
        self.setFixedWidth(220)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(6)
        # Header
        lay.addWidget(label("Dashboard", "h2"))
        lay.addSpacing(4)
        # Hz
        self._hz_lbl = QLabel("Loop: — Hz")
        self._hz_lbl.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {GREEN};")
        lay.addWidget(self._hz_lbl)
        # Sim time
        self._time_lbl = label("Sim: 0.0 s", "tag")
        lay.addWidget(self._time_lbl)
        lay.addSpacing(8)
        lay.addWidget(hline())
        lay.addSpacing(4)
        lay.addWidget(label("Joint Angles (°)", "h3"))
        lay.addSpacing(4)
        # Joint rows
        self._bars: List[QProgressBar] = []
        self._vals: List[QLabel] = []
        for i in range(NUM_JOINTS):
            row = QHBoxLayout(); row.setSpacing(6)
            nm = QLabel(f"J{i+1}"); nm.setFixedWidth(24)
            nm.setStyleSheet(f"font-size: 11px; color: {MUTED}; font-weight: 600;")
            bar = QProgressBar(); bar.setRange(-180, 180); bar.setValue(0)
            bar.setTextVisible(False); bar.setFixedHeight(10)
            bar.setStyleSheet(f"""
                QProgressBar {{ background: {BORDER}; border: none; border-radius: 4px; }}
                QProgressBar::chunk {{ background: {ACCENT}; border-radius: 4px; }}
            """)
            val = QLabel("0.0"); val.setFixedWidth(50)
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            val.setStyleSheet(f"font-family: monospace; font-size: 11px; color: {TEXT}; font-weight: 600;")
            row.addWidget(nm); row.addWidget(bar); row.addWidget(val)
            lay.addLayout(row)
            self._bars.append(bar); self._vals.append(val)
        lay.addStretch()

    def update_telemetry(self, data: dict):
        hz = data.get("hz", 0)
        self._hz_lbl.setText(f"Loop: {hz:.0f} Hz")
        c = GREEN if hz > 25 else (ORANGE if hz > 10 else RED)
        self._hz_lbl.setStyleSheet(f"font-size: 13px; font-weight: 600; color: {c};")
        self._time_lbl.setText(f"Sim: {data.get('sim_time', 0):.1f} s")
        for i, deg in enumerate(data.get("joint_deg", [])):
            if i < len(self._bars):
                self._bars[i].setValue(int(np.clip(deg, -180, 180)))
                self._vals[i].setText(f"{deg:+.1f}")
                ac = GREEN if abs(deg) < 90 else (ORANGE if abs(deg) < 150 else RED)
                self._bars[i].setStyleSheet(f"""
                    QProgressBar {{ background: {BORDER}; border: none; border-radius: 4px; }}
                    QProgressBar::chunk {{ background: {ac}; border-radius: 4px; }}
                """)


class SimulationPage(QWidget):
    go_back = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._sim: Optional[SimulationThread] = None
        self._build()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12); root.setSpacing(8)
        # Header
        hdr = QHBoxLayout()
        back = QPushButton("← Back"); back.setObjectName("ghost")
        back.setFixedWidth(88); back.clicked.connect(self._exit)
        self._back_btn = back
        hdr.addWidget(back); hdr.addSpacing(8)
        hdr.addWidget(label("Simulation", "h1")); hdr.addStretch()
        root.addLayout(hdr)
        # Body: viewer + dashboard
        body = QHBoxLayout(); body.setSpacing(10)
        self._viewer = MujocoViewerWidget()
        self._dash = PerformanceDashboard()
        dash_card = card_widget()
        dc = QVBoxLayout(dash_card); dc.setContentsMargins(0, 0, 0, 0)
        dc.addWidget(self._dash)
        body.addWidget(self._viewer, stretch=3)
        body.addWidget(dash_card, stretch=0)
        root.addLayout(body)
        # Stop button
        self._stop_btn = QPushButton("✕  Stop Simulation"); self._stop_btn.setObjectName("stop")
        self._stop_btn.clicked.connect(self._exit)
        root.addWidget(self._stop_btn)

    def start(self, sim_thread: SimulationThread):
        self._sim = sim_thread
        self._viewer.attach(sim_thread)
        sim_thread.telemetry.connect(self._dash.update_telemetry)
        sim_thread.error.connect(self._on_error)
        sim_thread.start()

    def _on_error(self, msg: str):
        self._viewer._viewport.setText(f"Error: {msg}")
        self._viewer._viewport.setStyleSheet(f"background: #000; color: {RED}; padding: 20px; font-size: 14px;")

    def _exit(self):
        if self._sim and self._sim.isRunning():
            self._sim.stop(); self._sim.wait(3000)
        self.go_back.emit()


# ═════════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GELLO xArm7 Launcher")
        self.setMinimumSize(640, 620); self.resize(660, 660)
        self._stack = QStackedWidget(); self.setCentralWidget(self._stack)

        self._home    = HomePage()
        self._onboard = OnboardingPage()
        self._cal     = CalibratePage()
        self._sim_page = SimulationPage()

        self._stack.addWidget(self._home)     # 0
        self._stack.addWidget(self._onboard)  # 1
        self._stack.addWidget(self._cal)      # 2
        self._stack.addWidget(self._sim_page) # 3

        self._home.go_setup.connect(self._show_onboarding)
        self._home.go_launch.connect(self._show_calibrate)
        self._onboard.go_back.connect(self._show_home)
        self._cal.go_back.connect(self._show_home)
        self._cal.go_sim.connect(self._show_simulation)
        self._sim_page.go_back.connect(self._show_calibrate_from_sim)

    def _show_home(self):
        self._stack.setCurrentIndex(0); self._home.refresh()

    def _show_onboarding(self):
        port = detect_port() or "/dev/ttyUSB0"
        self._onboard.start(port); self._stack.setCurrentIndex(1)

    def _show_calibrate(self):
        port = detect_port() or "/dev/ttyUSB0"
        self._cal.start(port); self._stack.setCurrentIndex(2)

    def _show_simulation(self, sim_thread: SimulationThread):
        self.resize(1100, 700)
        self._sim_page.start(sim_thread)
        self._stack.setCurrentIndex(3)

    def _show_calibrate_from_sim(self):
        self.resize(660, 660)
        self._show_calibrate()

    def closeEvent(self, event):
        self._sim_page._exit()
        event.accept()


# ─────────────────────────────── Entry ────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    app.setApplicationName("GELLO xArm7 Launcher")
    w = MainWindow(); w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
