#!/usr/bin/env python3
"""
xArm7 — Per-Joint Calibration Wizard

Step-by-step calibration with live MuJoCo preview.
For each joint: see the sim pose -> match your servo -> save offset -> verify direction.
Then calibrate gripper open/close. Finally, test everything together.
"""

import re
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)

from gello.utils.servo_io import ServoReaderThread, detect_port, NUM_JOINTS
from gello.utils.sim_renderer import SimRenderer
from gello.utils.ui_common import (
    BG, CARD, TEXT, MUTED, ACCENT, GREEN, RED, BORDER, STYLE,
)

# ─────────────────────────────── Constants ───────────────────────────────────

BASE_DIR   = Path(__file__).parent
CONFIG_SIM = BASE_DIR / "configs" / "xarm_sim_test.yaml"

JOINT_NAMES = [f"J{i+1}" for i in range(NUM_JOINTS)]


# ─────────────────────────────── Wizard UI ───────────────────────────────────

class CalibrationWizard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("xArm7 Calibration Wizard")
        self.setMinimumSize(900, 400)

        self._port = detect_port()
        self._servo_reader: Optional[ServoReaderThread] = None
        self._sim: Optional[SimRenderer] = None
        self._raw_readings = [0.0] * (NUM_JOINTS + 1)

        self._offsets = [0.0] * NUM_JOINTS
        self._signs = [1, -1, 1, 1, 1, 1, -1]
        self._gripper_open_deg = 198.0
        self._gripper_close_deg = 156.0
        self._current_step = 0

        self._build()
        self._start_threads()
        self._update_step()

    def _build(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Left: MuJoCo viewer
        self._viewport = QLabel()
        self._viewport.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._viewport.setStyleSheet("background: #1a1a1a; border: none;")
        self._viewport.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._viewport.setMinimumWidth(500)
        root.addWidget(self._viewport, 3)

        # Right: controls panel — QFrame with object name so global QWidget rule doesn't override bg
        panel = QFrame()
        panel.setObjectName("calib_panel")
        panel.setStyleSheet(f"QFrame#calib_panel {{ background: {CARD}; }}")
        panel.setFixedWidth(420)
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(0, 0, 0, 0)
        pl.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea {{ background: {CARD}; border: none; }}"
                             f"QWidget {{ background: {CARD}; }}")

        content_w = QFrame()
        content_w.setObjectName("calib_content")
        content_w.setStyleSheet(f"QFrame#calib_content {{ background: {CARD}; }}")
        cl = QVBoxLayout(content_w)
        cl.setContentsMargins(24, 24, 24, 12)
        cl.setSpacing(8)

        t = QLabel("Calibration Wizard")
        t.setStyleSheet(f"font-size: 22px; font-weight: 800; color: {TEXT};")
        cl.addWidget(t)

        self._step_lbl = QLabel("Step 1 of 10")
        self._step_lbl.setStyleSheet(f"font-size: 13px; color: {MUTED}; font-weight: 600;")
        cl.addWidget(self._step_lbl)

        port_txt = self._port or "NOT FOUND"
        port_c = GREEN if self._port else RED
        pl_port = QLabel(f"Port: {port_txt}")
        pl_port.setStyleSheet(f"font-size: 12px; color: {port_c}; font-weight: 600;")
        cl.addWidget(pl_port)

        cl.addSpacing(4)

        inst_frame = QFrame()
        inst_frame.setObjectName("info")
        il = QVBoxLayout(inst_frame)
        il.setContentsMargins(16, 14, 16, 14)
        il.setSpacing(8)
        self._inst_title = QLabel("Position J1")
        self._inst_title.setStyleSheet(f"font-size: 17px; font-weight: 700; color: #0A3D6B;")
        self._inst_body = QLabel("Match this joint to the simulation pose.")
        self._inst_body.setStyleSheet(f"font-size: 13px; color: #0A3D6B;")
        self._inst_body.setWordWrap(True)
        self._inst_body.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
        il.addWidget(self._inst_title)
        il.addWidget(self._inst_body)
        cl.addWidget(inst_frame)

        reading_frame = QFrame()
        reading_frame.setObjectName("card")
        reading_frame.setMinimumHeight(140)
        rl = QVBoxLayout(reading_frame)
        rl.setContentsMargins(16, 16, 16, 16)
        rl.setSpacing(6)
        rh = QLabel("Live Servo Reading")
        rh.setStyleSheet(f"font-size: 12px; font-weight: 600; color: {MUTED};")
        rl.addWidget(rh)
        rl.addStretch(1)
        self._reading_lbl = QLabel("--")
        self._reading_lbl.setStyleSheet(
            f"font-family: Consolas, monospace; font-size: 28px; font-weight: 700; color: {ACCENT};")
        self._reading_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        rl.addWidget(self._reading_lbl)
        self._reading_detail = QLabel("")
        self._reading_detail.setStyleSheet(f"font-size: 11px; color: {MUTED};")
        self._reading_detail.setAlignment(Qt.AlignmentFlag.AlignCenter)
        rl.addWidget(self._reading_detail)
        rl.addStretch(1)
        cl.addWidget(reading_frame)

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

        # Buttons pinned to bottom
        btn_container = QWidget()
        btn_container.setStyleSheet(f"background: {CARD}; border-top: 1px solid {BORDER};")
        bl = QVBoxLayout(btn_container)
        bl.setContentsMargins(24, 12, 24, 16)
        bl.setSpacing(8)

        self._flip_btn = QPushButton("Flip Direction")
        self._flip_btn.setObjectName("ghost")
        self._flip_btn.setMinimumHeight(38)
        self._flip_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._flip_btn.clicked.connect(self._on_flip)
        self._flip_btn.setVisible(False)
        bl.addWidget(self._flip_btn)

        self._action_btn = QPushButton("Save J1 Offset")
        self._action_btn.setMinimumHeight(48)
        self._action_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._action_btn.setStyleSheet(
            f"QPushButton {{ background: {ACCENT}; color: white; font-size: 14px;"
            f"  font-weight: 700; padding: 12px 24px; border-radius: 8px; border: none; }}"
            f"QPushButton:hover {{ background: #005BBF; }}"
            f"QPushButton:disabled {{ background: {BORDER}; color: {MUTED}; }}"
        )
        self._action_btn.clicked.connect(self._on_action)
        bl.addWidget(self._action_btn)

        pl.addWidget(btn_container)
        root.addWidget(panel)

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

    @property
    def _in_verify(self):
        return hasattr(self, '_verifying') and self._verifying

    def _update_step(self):
        s = self._current_step
        self._verifying = False
        self._dir_frame.setVisible(False)
        self._flip_btn.setVisible(False)

        if self._sim:
            if s < NUM_JOINTS:
                self._sim.set_active_joint(s)
            elif s in (NUM_JOINTS, NUM_JOINTS + 1):
                self._sim.set_active_joint(NUM_JOINTS)
            else:
                self._sim.set_active_joint(-1)

        total = 10
        self._step_lbl.setText(f"Step {s + 1} of {total}")

        if s < NUM_JOINTS:
            jname = JOINT_NAMES[s]
            self._inst_title.setText(f"Position {jname}")
            self._inst_body.setText(
                f"Look at the simulation — position servo {s+1} on your arm "
                f"to match the sim's zero pose for {jname}.\n\n"
                f"When it matches, click the button below.")
            self._action_btn.setText(f"Save {jname} Offset")
        elif s == NUM_JOINTS:
            self._inst_title.setText("Gripper — Open Position")
            self._inst_body.setText(
                "Open the gripper as wide as it goes.\n\n"
                "Click Save when fully open.")
            self._action_btn.setText("Save Open Position")
        elif s == NUM_JOINTS + 1:
            self._inst_title.setText("Gripper — Closed Position")
            self._inst_body.setText(
                "Close the gripper completely.\n\n"
                "Click Save when fully closed.")
            self._action_btn.setText("Save Closed Position")
        elif s == NUM_JOINTS + 2:
            self._inst_title.setText("Full Test")
            self._inst_body.setText(
                "Move the arm around — the simulation should follow "
                "all joints correctly in real time.\n\n"
                "If everything looks good, save the config.")
            self._action_btn.setText("Save Config & Exit")

    def _enter_verify(self):
        s = self._current_step
        jname = JOINT_NAMES[s]
        self._verifying = True
        self._dir_frame.setVisible(True)
        self._flip_btn.setVisible(True)
        self._inst_title.setText(f"Verify {jname} Direction")
        self._inst_body.setText(
            f"Move {jname} on the arm — the sim should follow in the SAME direction.\n\n"
            f"If it moves opposite, click Flip Direction.")
        self._dir_lbl.setText("Wiggle the joint and watch the sim...")
        self._action_btn.setText("Next →")

    def _tick(self):
        s = self._current_step
        readings = self._raw_readings

        if s < NUM_JOINTS:
            raw_rad = readings[s]
            raw_deg = np.rad2deg(raw_rad)
            self._reading_lbl.setText(f"{raw_deg:+.1f}°")
            self._reading_detail.setText(f"Raw: {raw_rad:.4f} rad")

            if self._in_verify or s == NUM_JOINTS + 2:
                cal_deg = np.rad2deg((raw_rad - self._offsets[s]) * self._signs[s])
                self._dir_lbl.setText(f"Calibrated: {cal_deg:+.1f}°  —  wiggle to check")
        elif s == NUM_JOINTS or s == NUM_JOINTS + 1:
            raw_rad = readings[NUM_JOINTS]
            raw_deg = np.rad2deg(raw_rad)
            self._reading_lbl.setText(f"{raw_deg:+.1f}°")
            self._reading_detail.setText(f"Gripper servo raw: {raw_rad:.4f} rad")
        elif s == NUM_JOINTS + 2:
            self._reading_lbl.setText("All joints")
            self._reading_detail.setText("Full test mode")

        if self._sim:
            if self._in_verify and s < NUM_JOINTS:
                qpos_val = (readings[s] - self._offsets[s]) * self._signs[s]
                self._sim.set_qpos(s, qpos_val)
            elif s == NUM_JOINTS + 2:
                for i in range(NUM_JOINTS):
                    val = (readings[i] - self._offsets[i]) * self._signs[i]
                    self._sim.set_qpos(i, val)
                g_open_rad = self._gripper_open_deg * np.pi / 180
                g_close_rad = self._gripper_close_deg * np.pi / 180
                g_raw = readings[NUM_JOINTS]
                if abs(g_close_rad - g_open_rad) > 0.01:
                    g_norm = (g_raw - g_open_rad) / (g_close_rad - g_open_rad)
                    g_norm = min(max(g_norm, 0), 1)
                else:
                    g_norm = 0
                self._sim.set_qpos(NUM_JOINTS, g_norm * 0.8)
            else:
                for i in range(NUM_JOINTS):
                    self._sim.set_qpos(i, 0.0)

    # ── Actions ───────────────────────────────────────────────────────────────

    def _on_action(self):
        s = self._current_step
        readings = self._raw_readings

        if self._in_verify:
            self._current_step += 1
            self._update_step()
            return

        if s < NUM_JOINTS:
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
            raw_deg = np.rad2deg(readings[NUM_JOINTS])
            self._gripper_open_deg = raw_deg
            self._reading_detail.setText(f"Open position saved: {raw_deg:.1f}°")
            self._current_step += 1
            self._update_step()

        elif s == NUM_JOINTS + 1:
            raw_deg = np.rad2deg(readings[NUM_JOINTS])
            self._gripper_close_deg = raw_deg
            self._reading_detail.setText(f"Closed position saved: {raw_deg:.1f}°")
            self._current_step += 1
            self._update_step()

        elif s == NUM_JOINTS + 2:
            self._do_finish()

    def _on_flip(self):
        s = self._current_step
        if s < NUM_JOINTS:
            self._signs[s] *= -1
            self._dir_lbl.setText(f"Sign flipped to {self._signs[s]:+d} — check again")

    def _do_finish(self):
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
        if not CONFIG_SIM.exists():
            raise FileNotFoundError(f"Config not found: {CONFIG_SIM}")

        text = CONFIG_SIM.read_text()
        port = self._port or "COM9"
        text = re.sub(r'port: "[^"]*"', f'port: "{port}"', text, count=1)

        block = "joint_offsets: [\n"
        for i, o in enumerate(self._offsets):
            comma = "," if i < len(self._offsets) - 1 else ""
            block += f"      {o:.4f}{comma}\n"
        block += "    ]"
        text = re.sub(r"joint_offsets: \[.*?\]", block, text, flags=re.DOTALL)

        signs_str = "[" + ", ".join(str(s) for s in self._signs) + "]"
        text = re.sub(r"joint_signs: \[.*?\]", f"joint_signs: {signs_str}", text)

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
