#!/usr/bin/env python3
"""
GELLO xArm7 — Per-Joint Calibration Wizard

Step-by-step calibration with live MuJoCo preview.
For each joint: see the sim pose → match your servo → save offset → verify direction.
Then calibrate gripper open/close. Finally, test everything together.
"""

import re
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)

# ─────────────────────────────── Constants ───────────────────────────────────

BASE_DIR   = Path(__file__).parent
CONFIG_SIM = BASE_DIR / "configs" / "xarm_sim_test.yaml"
MODEL_PATH = BASE_DIR / "third_party" / "mujoco_menagerie" / "ufactory_xarm7" / "xarm7_nohand.xml"
GRIPPER_PATH = BASE_DIR / "third_party" / "mujoco_menagerie" / "ufactory_xarm7" / "hand.xml"
NUM_JOINTS = 7
BAUD_RATE  = 57600

# Colors
BG      = "#F5F5F7"
TEXT    = "#1D1D1F"
MUTED   = "#86868B"
ACCENT  = "#0071E3"
GREEN   = "#34C759"
ORANGE  = "#FF9500"
RED     = "#FF3B30"
CARD    = "#FFFFFF"
BORDER  = "#D2D2D7"

STYLE = f"""
* {{ font-family: 'Segoe UI', Arial, sans-serif; }}
QWidget {{ background: {BG}; color: {TEXT}; }}
QFrame#card {{ background: {CARD}; border-radius: 10px; border: 1px solid {BORDER}; }}
QFrame#info {{ background: #EBF5FB; border-radius: 8px; border: 1px solid #C6E2F5; }}
QFrame#warn {{ background: #FFF8EC; border-radius: 8px; border: 1px solid #FFD580; }}
QPushButton {{
    background: {ACCENT}; color: white; border: none; border-radius: 6px;
    padding: 8px 16px; font-size: 13px; font-weight: 600;
}}
QPushButton:hover {{ background: #005BBF; }}
QPushButton:disabled {{ background: {BORDER}; color: {MUTED}; }}
QPushButton#ghost {{
    background: transparent; color: {ACCENT}; border: 1.5px solid {ACCENT};
}}
QPushButton#ghost:hover {{ background: #EAF2FC; }}
QPushButton#green {{
    background: {GREEN}; color: white; font-size: 14px; font-weight: 700;
    padding: 10px 24px; border-radius: 8px;
}}
QPushButton#green:hover {{ background: #2DB84C; }}
QPushButton#green:disabled {{ background: {BORDER}; color: {MUTED}; }}
QPushButton#red {{
    background: {RED}; color: white;
}}
"""


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


# ─────────────────────────────── Servo I/O ───────────────────────────────────

class ServoIO:
    """Low-level servo read/write via dynamixel_sdk."""

    def __init__(self, port: str):
        from dynamixel_sdk import GroupSyncRead, PacketHandler, PortHandler
        self._ph = PortHandler(port)
        self._pk = PacketHandler(2.0)
        self._ph.openPort()
        self._ph.setBaudRate(BAUD_RATE)
        self._gsr = GroupSyncRead(self._ph, self._pk, 132, 4)  # ADDR_POS, LEN=4
        for i in range(1, NUM_JOINTS + 2):  # 1-8
            self._gsr.addParam(i)
        # Warmup
        for _ in range(10):
            self._gsr.txRxPacket()
            time.sleep(0.01)

    def read_raw(self) -> List[float]:
        """Read all 8 servos, return radians."""
        from dynamixel_sdk import COMM_SUCCESS
        if self._gsr.txRxPacket() != COMM_SUCCESS:
            return [0.0] * (NUM_JOINTS + 1)
        out = []
        for i in range(1, NUM_JOINTS + 2):
            raw = self._gsr.getData(i, 132, 4)
            if raw > 0x7FFFFFFF:
                raw -= 0x100000000
            out.append(raw / 2048.0 * np.pi)
        return out

    def read_avg(self, n=10) -> List[float]:
        """Average n readings."""
        samples = []
        for _ in range(n):
            samples.append(self.read_raw())
            time.sleep(0.02)
        return list(np.mean(samples, axis=0))

    def close(self):
        try:
            self._ph.closePort()
        except Exception:
            pass


class ServoReaderThread(QThread):
    """Continuously reads servo positions at ~30Hz."""
    update = pyqtSignal(list)  # list of 8 floats (radians)

    def __init__(self, port: str):
        super().__init__()
        self._port = port
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        try:
            io = ServoIO(self._port)
            while self._running:
                vals = io.read_raw()
                self.update.emit(vals)
                time.sleep(0.033)
            io.close()
        except Exception:
            pass


# ─────────────────────────────── MuJoCo Renderer ─────────────────────────────

class SimRenderer(QThread):
    """Renders MuJoCo frames with configurable joint positions."""
    frame_ready = pyqtSignal(object)

    def __init__(self, w=640, h=480):
        super().__init__()
        self._rw, self._rh = w, h
        self._running = True
        self._qpos = np.zeros(20)  # more than enough
        self._lock = threading.Lock()

    def set_qpos(self, idx: int, val: float):
        with self._lock:
            self._qpos[idx] = val

    def set_all_qpos(self, vals):
        with self._lock:
            for i, v in enumerate(vals):
                if i < len(self._qpos):
                    self._qpos[i] = v

    def stop(self):
        self._running = False

    def run(self):
        try:
            import mujoco
            from gello.robots.sim_robot import build_scene

            arena = build_scene(str(MODEL_PATH), str(GRIPPER_PATH))
            xml_string = arena.to_xml_string()
            assets = {}
            for asset in arena.asset.all_children():
                if asset.tag == "mesh":
                    f = asset.file
                    assets[f.get_vfs_filename()] = f.contents
            model = mujoco.MjModel.from_xml_string(xml_string, assets)
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, self._rw)
            model.vis.global_.offheight = max(model.vis.global_.offheight, self._rh)
            data = mujoco.MjData(model)
            renderer = mujoco.Renderer(model, height=self._rh, width=self._rw)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.azimuth = 150.0
            cam.elevation = -20.0
            cam.distance = 1.8
            cam.lookat[:] = [0.0, 0.0, 0.3]

            while self._running:
                with self._lock:
                    nq = min(len(self._qpos), model.nq)
                    data.qpos[:nq] = self._qpos[:nq]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, cam)
                frame = renderer.render().copy()
                self.frame_ready.emit(frame)
                time.sleep(1.0 / 30)
            renderer.close()
        except Exception as e:
            print(f"Renderer error: {e}")


# ─────────────────────────────── Wizard UI ───────────────────────────────────

JOINT_NAMES = [f"J{i+1}" for i in range(NUM_JOINTS)]


class CalibrationWizard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GELLO Calibration Wizard")
        self.setMinimumSize(900, 400)

        self._port = detect_port()
        self._servo_reader: Optional[ServoReaderThread] = None
        self._sim: Optional[SimRenderer] = None
        self._raw_readings = [0.0] * (NUM_JOINTS + 1)

        # Calibration state
        self._offsets = [0.0] * NUM_JOINTS
        self._signs = [1, -1, 1, 1, 1, 1, -1]  # xArm7 default signs
        self._gripper_open_deg = 198.0
        self._gripper_close_deg = 156.0
        self._current_step = 0  # 0..6 = J1..J7, 7 = gripper open, 8 = gripper close, 9 = test

        self._build()
        self._start_threads()
        self._update_step()

    def _build(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Left: MuJoCo viewer ──
        self._viewport = QLabel()
        self._viewport.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._viewport.setStyleSheet("background: #1a1a1a; border: none;")
        self._viewport.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._viewport.setMinimumWidth(500)
        root.addWidget(self._viewport, 3)

        # ── Right: controls panel ──
        panel = QWidget()
        panel.setFixedWidth(420)
        panel.setStyleSheet(f"background: {CARD};")
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(0, 0, 0, 0)
        pl.setSpacing(0)

        # Scrollable content area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"background: {CARD};")

        content_w = QWidget()
        content_w.setStyleSheet(f"background: {CARD};")
        cl = QVBoxLayout(content_w)
        cl.setContentsMargins(24, 24, 24, 12)
        cl.setSpacing(8)

        # Title
        t = QLabel("Calibration Wizard")
        t.setStyleSheet(f"font-size: 22px; font-weight: 800; color: {TEXT};")
        cl.addWidget(t)

        # Step indicator
        self._step_lbl = QLabel("Step 1 of 10")
        self._step_lbl.setStyleSheet(f"font-size: 13px; color: {MUTED}; font-weight: 600;")
        cl.addWidget(self._step_lbl)

        # Port status
        port_txt = self._port or "NOT FOUND"
        port_c = GREEN if self._port else RED
        pl_port = QLabel(f"Port: {port_txt}")
        pl_port.setStyleSheet(f"font-size: 12px; color: {port_c}; font-weight: 600;")
        cl.addWidget(pl_port)

        cl.addSpacing(4)

        # Instruction card
        inst_frame = QFrame()
        inst_frame.setObjectName("info")
        il = QVBoxLayout(inst_frame)
        il.setContentsMargins(16, 14, 16, 14)
        il.setSpacing(8)
        self._inst_title = QLabel("Position J1")
        self._inst_title.setStyleSheet(f"font-size: 17px; font-weight: 700; color: #0A3D6B;")
        self._inst_body = QLabel("Match this joint to the simulation pose.")
        self._inst_body.setStyleSheet(f"font-size: 13px; color: #0A3D6B; line-height: 1.4;")
        self._inst_body.setWordWrap(True)
        il.addWidget(self._inst_title)
        il.addWidget(self._inst_body)
        cl.addWidget(inst_frame)

        # Live reading display
        reading_frame = QFrame()
        reading_frame.setObjectName("card")
        rl = QVBoxLayout(reading_frame)
        rl.setContentsMargins(16, 12, 16, 12)
        rl.setSpacing(4)
        rh = QLabel("Live Servo Reading")
        rh.setStyleSheet(f"font-size: 12px; font-weight: 600; color: {MUTED};")
        rl.addWidget(rh)
        self._reading_lbl = QLabel("--")
        self._reading_lbl.setStyleSheet(f"font-family: Consolas, monospace; font-size: 28px; font-weight: 700; color: {ACCENT};")
        self._reading_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        rl.addWidget(self._reading_lbl)
        self._reading_detail = QLabel("")
        self._reading_detail.setStyleSheet(f"font-size: 11px; color: {MUTED};")
        self._reading_detail.setAlignment(Qt.AlignmentFlag.AlignCenter)
        rl.addWidget(self._reading_detail)
        cl.addWidget(reading_frame)

        # Direction indicator (shown during verify phase)
        self._dir_frame = QFrame()
        self._dir_frame.setObjectName("card")
        dl = QVBoxLayout(self._dir_frame)
        dl.setContentsMargins(16, 12, 16, 12)
        dl.setSpacing(6)
        dh = QLabel("Direction Check")
        dh.setStyleSheet(f"font-size: 12px; font-weight: 600; color: {MUTED};")
        dl.addWidget(dh)
        self._dir_lbl = QLabel("Move the joint — sim should follow correctly")
        self._dir_lbl.setStyleSheet(f"font-size: 13px; color: {TEXT};")
        self._dir_lbl.setWordWrap(True)
        dl.addWidget(self._dir_lbl)
        self._dir_frame.setVisible(False)
        cl.addWidget(self._dir_frame)

        cl.addStretch(1)
        scroll.setWidget(content_w)
        pl.addWidget(scroll, 1)

        # ── Buttons pinned at bottom, outside the scroll area ──
        btn_container = QWidget()
        btn_container.setStyleSheet(f"background: {CARD}; border-top: 1px solid {BORDER};")
        bl = QVBoxLayout(btn_container)
        bl.setContentsMargins(24, 10, 24, 14)
        bl.setSpacing(8)

        self._flip_btn = QPushButton("Flip Direction")
        self._flip_btn.setObjectName("ghost")
        self._flip_btn.setMinimumHeight(38)
        self._flip_btn.clicked.connect(self._on_flip)
        self._flip_btn.setVisible(False)
        bl.addWidget(self._flip_btn)

        self._action_btn = QPushButton("Save J1 Offset")
        self._action_btn.setObjectName("green")
        self._action_btn.setMinimumHeight(48)
        self._action_btn.clicked.connect(self._on_action)
        bl.addWidget(self._action_btn)

        pl.addWidget(btn_container)

        root.addWidget(panel)

        # Timer for live updates
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

    # ── Threads ───────────────────────────────────────────────────────────────

    def _start_threads(self):
        self._sim = SimRenderer()
        self._sim.frame_ready.connect(self._on_frame)
        self._sim.start()

        if self._port:
            self._servo_reader = ServoReaderThread(self._port)
            self._servo_reader.update.connect(self._on_servo)
            self._servo_reader.start()

    def _on_frame(self, frame):
        h, w, _ = frame.shape
        img = QImage(frame.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        scaled = QPixmap.fromImage(img).scaled(
            self._viewport.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation)
        self._viewport.setPixmap(scaled)

    def _on_servo(self, readings: list):
        self._raw_readings = readings

    # ── Step logic ────────────────────────────────────────────────────────────

    # Steps: 0-6 = J1-J7 (position + verify), 7 = gripper open, 8 = gripper close, 9 = full test
    @property
    def _in_verify(self):
        return hasattr(self, '_verifying') and self._verifying

    def _update_step(self):
        s = self._current_step
        self._verifying = False
        self._dir_frame.setVisible(False)
        self._flip_btn.setVisible(False)

        total = 10  # J1-J7 (7) + gripper open + gripper close + test
        self._step_lbl.setText(f"Step {s + 1} of {total}")

        if s < NUM_JOINTS:
            jname = JOINT_NAMES[s]
            self._inst_title.setText(f"Position {jname}")
            self._inst_body.setText(
                f"Look at the simulation — position servo {s+1} on your GELLO "
                f"to match the sim's zero pose for {jname}.\n\n"
                f"When it matches, click the button below.")
            self._action_btn.setText(f"Save {jname} Offset")
        elif s == NUM_JOINTS:  # gripper open
            self._inst_title.setText("Gripper — Open Position")
            self._inst_body.setText(
                "Open the gripper on your GELLO as wide as it goes.\n\n"
                "Click Save when fully open.")
            self._action_btn.setText("Save Open Position")
        elif s == NUM_JOINTS + 1:  # gripper close
            self._inst_title.setText("Gripper — Closed Position")
            self._inst_body.setText(
                "Close the gripper on your GELLO completely.\n\n"
                "Click Save when fully closed.")
            self._action_btn.setText("Save Closed Position")
        elif s == NUM_JOINTS + 2:  # full test
            self._inst_title.setText("Full Test")
            self._inst_body.setText(
                "Move your GELLO around — the simulation should follow "
                "all joints correctly in real time.\n\n"
                "If everything looks good, save the config.")
            self._action_btn.setText("Save Config & Exit")

    def _enter_verify(self):
        """After saving offset, let user verify direction."""
        s = self._current_step
        jname = JOINT_NAMES[s]
        self._verifying = True
        self._dir_frame.setVisible(True)
        self._flip_btn.setVisible(True)
        self._inst_title.setText(f"Verify {jname} Direction")
        self._inst_body.setText(
            f"Move {jname} on your GELLO — the sim should follow in the SAME direction.\n\n"
            f"If it moves opposite, click Flip Direction.")
        self._dir_lbl.setText("Wiggle the joint and watch the sim...")
        self._action_btn.setText(f"Next →")

    def _tick(self):
        """Update live reading display and sim joint positions."""
        s = self._current_step
        readings = self._raw_readings

        if s < NUM_JOINTS:
            raw_rad = readings[s]
            raw_deg = np.rad2deg(raw_rad)
            self._reading_lbl.setText(f"{raw_deg:+.1f}°")
            self._reading_detail.setText(f"Raw: {raw_rad:.4f} rad")

            if self._in_verify or s == NUM_JOINTS + 2:
                # Show calibrated angle during verify
                cal_deg = np.rad2deg((raw_rad - self._offsets[s]) * self._signs[s])
                self._dir_lbl.setText(f"Calibrated: {cal_deg:+.1f}°  —  wiggle to check")
        elif s == NUM_JOINTS or s == NUM_JOINTS + 1:
            raw_rad = readings[NUM_JOINTS]  # servo 8 = index 7
            raw_deg = np.rad2deg(raw_rad)
            self._reading_lbl.setText(f"{raw_deg:+.1f}°")
            self._reading_detail.setText(f"Gripper servo raw: {raw_rad:.4f} rad")
        elif s == NUM_JOINTS + 2:
            self._reading_lbl.setText("All joints")
            self._reading_detail.setText("Full test mode")

        # Update sim joint positions
        if self._sim:
            if self._in_verify and s < NUM_JOINTS:
                # During verify: show only this joint moving with calibration applied
                qpos_val = (readings[s] - self._offsets[s]) * self._signs[s]
                self._sim.set_qpos(s, qpos_val)
            elif s == NUM_JOINTS + 2:
                # Full test: all joints calibrated
                for i in range(NUM_JOINTS):
                    val = (readings[i] - self._offsets[i]) * self._signs[i]
                    self._sim.set_qpos(i, val)
                # Gripper
                g_open_rad = self._gripper_open_deg * np.pi / 180
                g_close_rad = self._gripper_close_deg * np.pi / 180
                g_raw = readings[NUM_JOINTS]
                if abs(g_close_rad - g_open_rad) > 0.01:
                    g_norm = (g_raw - g_open_rad) / (g_close_rad - g_open_rad)
                    g_norm = min(max(g_norm, 0), 1)
                else:
                    g_norm = 0
                # Gripper actuator expects 0-255 but we set qpos directly
                # For the sim, gripper qpos controls finger spread
                self._sim.set_qpos(NUM_JOINTS, g_norm * 0.8)
            else:
                # Default: show zero pose (all joints at 0)
                for i in range(NUM_JOINTS):
                    self._sim.set_qpos(i, 0.0)

    # ── Actions ───────────────────────────────────────────────────────────────

    def _on_action(self):
        """Single button handles save/next/finish depending on current state."""
        s = self._current_step
        readings = self._raw_readings

        # During verify phase: advance to next step
        if self._in_verify:
            self._current_step += 1
            self._update_step()
            return

        if s < NUM_JOINTS:
            # Save offset = current raw reading (nearest π/2 multiple)
            raw = readings[s]
            best_off, best_err = 0.0, 1e9
            for off in np.linspace(-8 * np.pi, 8 * np.pi, 33):
                err = abs(raw - off)
                if err < best_err:
                    best_err, best_off = err, off
            self._offsets[s] = best_off
            residual_deg = np.rad2deg(best_err)
            if residual_deg > 30:
                self._reading_detail.setText(
                    f"Warning: {residual_deg:.1f}° from nearest 90° — reposition?")
            self._enter_verify()

        elif s == NUM_JOINTS:
            # Gripper open
            raw_deg = np.rad2deg(readings[NUM_JOINTS])
            self._gripper_open_deg = raw_deg
            self._reading_detail.setText(f"Open position saved: {raw_deg:.1f}°")
            self._current_step += 1
            self._update_step()

        elif s == NUM_JOINTS + 1:
            # Gripper close
            raw_deg = np.rad2deg(readings[NUM_JOINTS])
            self._gripper_close_deg = raw_deg
            self._reading_detail.setText(f"Closed position saved: {raw_deg:.1f}°")
            self._current_step += 1
            self._update_step()

        elif s == NUM_JOINTS + 2:
            # Full test — save and exit
            self._do_finish()

    def _on_flip(self):
        s = self._current_step
        if s < NUM_JOINTS:
            self._signs[s] *= -1
            self._dir_lbl.setText(f"Sign flipped to {self._signs[s]:+d} — check again")

    def _do_finish(self):
        """Write calibration to config and exit."""
        try:
            self._save_config()
            self._inst_title.setText("Saved!")
            self._inst_body.setText(
                f"Config written to:\n{CONFIG_SIM}\n\n"
                "You can now close this window and launch the simulator.")
            self._action_btn.setText("Exit")
            self._action_btn.clicked.disconnect()
            self._action_btn.clicked.connect(self.close)
        except Exception as e:
            self._inst_title.setText("Save failed")
            self._inst_body.setText(str(e))

    def _save_config(self):
        """Write offsets, signs, and gripper config to the YAML."""
        if not CONFIG_SIM.exists():
            raise FileNotFoundError(f"Config not found: {CONFIG_SIM}")

        text = CONFIG_SIM.read_text()

        # Update port
        port = self._port or "COM9"
        text = re.sub(r'port: "[^"]*"', f'port: "{port}"', text, count=1)

        # Update joint_offsets
        block = "joint_offsets: [\n"
        for i, o in enumerate(self._offsets):
            comma = "," if i < len(self._offsets) - 1 else ""
            block += f"      {o:.4f}{comma}\n"
        block += "    ]"
        text = re.sub(r"joint_offsets: \[.*?\]", block, text, flags=re.DOTALL)

        # Update joint_signs
        signs_str = "[" + ", ".join(str(s) for s in self._signs) + "]"
        text = re.sub(r"joint_signs: \[.*?\]", f"joint_signs: {signs_str}", text)

        # Update gripper_config
        go = int(round(self._gripper_open_deg))
        gc = int(round(self._gripper_close_deg))
        text = re.sub(r"gripper_config: \[\d+, \d+, \d+\]", f"gripper_config: [8, {go}, {gc}]", text)

        CONFIG_SIM.write_text(text)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._servo_reader:
            self._servo_reader.stop()
            self._servo_reader.wait(500)
        if self._sim:
            self._sim.stop()
            self._sim.wait(500)
        event.accept()


# ─────────────────────────────── Entry ───────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    w = CalibrationWizard()
    w.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
