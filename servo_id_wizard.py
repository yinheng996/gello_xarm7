#!/usr/bin/env python3
"""
xArm7 — Servo ID Assignment Wizard

Step-by-step wizard to connect one servo at a time, scan it, and assign the
correct ID (1-8).  Split-panel UI: MuJoCo preview on the left highlights
which joint you're assigning, controls on the right.
"""

import sys
from pathlib import Path
from typing import List, Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)

from gello.utils.servo_io import (
    ScanWorker, SetIDWorker, detect_port, NUM_JOINTS,
)
from gello.utils.sim_renderer import SimRenderer
from gello.utils.ui_common import (
    BG, CARD, TEXT, MUTED, ACCENT, GREEN, ORANGE, RED, BORDER,
    STYLE, ServoDots,
)

JOINT_INFO = [
    ("Joint 1 — Base",
     "The lowest servo — rotates the entire arm left/right around the vertical axis."),
    ("Joint 2 — Shoulder",
     "The shoulder servo — swings the arm forward and backward."),
    ("Joint 3 — Upper Arm",
     "The upper-arm servo — extends or retracts the upper arm segment."),
    ("Joint 4 — Elbow",
     "The elbow servo — bends the lower arm up and down."),
    ("Joint 5 — Forearm Rotation",
     "Rotates the forearm around its own axis."),
    ("Joint 6 — Wrist Bend",
     "Tilts the wrist up and down."),
    ("Joint 7 — Wrist Rotation",
     "Rotates the wrist — last joint before the gripper."),
    ("Gripper",
     "The gripper servo — controls opening and closing at the tip."),
]

TROUBLESHOOT = [
    ("No power",   "Check the power board LED is lit. XL330 requires 5 V."),
    ("No data",    "Re-seat the JST data cable at both ends until it clicks."),
    ("Only one!",  "Disconnect every other servo — only this single servo should be on the bus."),
    ("Bad cable",  "Try a different JST-SH 3-pin cable — cables fail silently."),
    ("LED check",  "Servo LED should blink once on boot. No blink = no power."),
    ("USB reset",  "Unplug U2D2, wait 3 s, re-plug, then retry."),
]


class ServoIDWizard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("xArm7 Servo ID Wizard")
        self.setMinimumSize(900, 400)

        self._port = detect_port()
        self._sim: Optional[SimRenderer] = None
        self._scan_w: Optional[ScanWorker] = None
        self._id_w: Optional[SetIDWorker] = None

        self._current_step = 1  # 1-8
        self._assigned: List[int] = []

        self._build()
        self._start_sim()
        self._update_step()

    # ── Build ─────────────────────────────────────────────────────────────────

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

        # Right: controls panel
        panel = QWidget()
        panel.setFixedWidth(420)
        panel.setStyleSheet(f"background: {CARD};")
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(0, 0, 0, 0)
        pl.setSpacing(0)

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

        t = QLabel("Servo ID Wizard")
        t.setStyleSheet(f"font-size: 22px; font-weight: 800; color: {TEXT};")
        cl.addWidget(t)

        self._step_lbl = QLabel("Step 1 of 8")
        self._step_lbl.setStyleSheet(f"font-size: 13px; color: {MUTED}; font-weight: 600;")
        cl.addWidget(self._step_lbl)

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
        self._inst_title = QLabel("")
        self._inst_title.setStyleSheet(f"font-size: 17px; font-weight: 700; color: #0A3D6B;")
        self._inst_body = QLabel("")
        self._inst_body.setStyleSheet(f"font-size: 13px; color: #0A3D6B; line-height: 1.4;")
        self._inst_body.setWordWrap(True)
        il.addWidget(self._inst_title)
        il.addWidget(self._inst_body)
        cl.addWidget(inst_frame)

        # Status label
        self._status_lbl = QLabel("")
        self._status_lbl.setWordWrap(True)
        self._status_lbl.setStyleSheet(f"font-size: 13px; color: {MUTED};")
        cl.addWidget(self._status_lbl)

        # Servo dots
        cl.addSpacing(4)
        dots_lbl = QLabel("Assigned IDs")
        dots_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        dots_lbl.setStyleSheet(f"font-size: 12px; color: {MUTED}; font-weight: 600;")
        cl.addWidget(dots_lbl)
        self._dots = ServoDots()
        cl.addWidget(self._dots)

        # Troubleshooting (hidden by default)
        cl.addSpacing(8)
        self._trouble_frame = QFrame()
        self._trouble_frame.setObjectName("card")
        self._trouble_frame.setVisible(False)
        tl = QVBoxLayout(self._trouble_frame)
        tl.setContentsMargins(16, 14, 16, 14)
        tl.setSpacing(6)
        th = QLabel("Troubleshooting")
        th.setStyleSheet(f"font-size: 14px; font-weight: 700; color: {TEXT};")
        tl.addWidget(th)
        for title_t, detail_t in TROUBLESHOOT:
            row = QHBoxLayout()
            row.setSpacing(8)
            badge = QLabel(title_t)
            badge.setStyleSheet(
                f"background: #FFF0F0; color: {RED}; border-radius: 4px; "
                f"padding: 2px 8px; font-size: 11px; font-weight: 600;")
            badge.setFixedWidth(90)
            detail = QLabel(detail_t)
            detail.setWordWrap(True)
            detail.setStyleSheet(f"font-size: 12px; color: {TEXT};")
            row.addWidget(badge)
            row.addWidget(detail)
            tl.addLayout(row)
        cl.addWidget(self._trouble_frame)

        cl.addStretch(1)
        scroll.setWidget(content_w)
        pl.addWidget(scroll, 1)

        # Buttons pinned to bottom
        btn_container = QWidget()
        btn_container.setStyleSheet(f"background: {CARD}; border-top: 1px solid {BORDER};")
        bl = QVBoxLayout(btn_container)
        bl.setContentsMargins(24, 12, 24, 16)
        bl.setSpacing(8)

        self._action_btn = QPushButton("Assign")
        self._action_btn.setMinimumHeight(48)
        self._action_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._action_btn.setStyleSheet(
            f"QPushButton {{ background: {ACCENT}; color: white; font-size: 14px;"
            f"  font-weight: 700; padding: 12px 24px; border-radius: 8px; border: none; }}"
            f"QPushButton:hover {{ background: #005BBF; }}"
            f"QPushButton:disabled {{ background: {BORDER}; color: {MUTED}; }}"
        )
        self._action_btn.clicked.connect(self._do_scan)
        bl.addWidget(self._action_btn)

        pl.addWidget(btn_container)
        root.addWidget(panel)

    # ── Sim ───────────────────────────────────────────────────────────────────

    def _start_sim(self):
        self._sim = SimRenderer()
        self._sim.frame_ready.connect(self._on_frame)
        self._sim.start()

    def _on_frame(self, frame):
        h, w, _ = frame.shape
        img = QImage(frame.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        scaled = QPixmap.fromImage(img).scaled(
            self._viewport.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation)
        self._viewport.setPixmap(scaled)

    # ── Step logic ────────────────────────────────────────────────────────────

    def _update_step(self):
        n = self._current_step
        self._step_lbl.setText(f"Step {n} of 8")
        self._trouble_frame.setVisible(False)
        self._status_lbl.setText("")
        self._dots.refresh(self._assigned, highlight=n)
        self._action_btn.setText("Scan && Assign")
        self._action_btn.setEnabled(True)

        # Highlight the joint in the sim
        if self._sim:
            joint_idx = n - 1 if n <= NUM_JOINTS else NUM_JOINTS
            self._sim.set_active_joint(joint_idx)

        if n <= 8:
            title, desc = JOINT_INFO[n - 1]
            self._inst_title.setText(f"Connect: {title}")
            self._inst_body.setText(
                f"{desc}\n\n"
                "Plug ONLY this servo into the data line. "
                "Make sure all other servos are disconnected. "
                "When ready, click Scan & Assign.")

    def _finish(self):
        self._dots.refresh(self._assigned)
        if self._sim:
            self._sim.set_active_joint(-1)
        self._inst_title.setText("All 8 servos assigned!")
        self._inst_body.setText(
            "Daisy-chain all 8 servos together (OUT → IN through the chain) "
            "and connect them all to the power board.\n\n"
            "All servos configured. You can close this window.")
        self._status_lbl.setText("")
        self._step_lbl.setText("Complete")
        self._action_btn.setText("Close")
        self._action_btn.clicked.disconnect()
        self._action_btn.clicked.connect(self.close)

    # ── Scan & assign ─────────────────────────────────────────────────────────

    def _do_scan(self):
        if not self._port:
            self._set_status("No USB port detected — check U2D2 connection.", RED)
            return
        self._action_btn.setEnabled(False)
        self._action_btn.setText("Scanning…")
        self._trouble_frame.setVisible(False)
        self._set_status("Scanning for servo on the bus…", MUTED)
        self._scan_w = ScanWorker(self._port)
        self._scan_w.result.connect(self._on_scan)
        self._scan_w.error.connect(lambda msg: self._set_status(msg, RED))
        self._scan_w.start()

    def _on_scan(self, data: dict):
        n = len(data)

        if n == 0:
            self._set_status(
                "No servo detected. Check power and data cable.", RED)
            self._trouble_frame.setVisible(True)
            self._action_btn.setEnabled(True)
            self._action_btn.setText("Retry Scan")
            return

        if n > 1:
            ids = ", ".join(str(i) for i in sorted(data.keys()))
            self._set_status(
                f"Multiple servos detected ({ids}). "
                "Disconnect all except this one servo.", ORANGE)
            self._action_btn.setEnabled(True)
            self._action_btn.setText("Retry Scan")
            return

        found = list(data.keys())[0]
        target = self._current_step
        if found == target:
            self._set_status(f"Servo already has the correct ID ({target}).", GREEN)
            QTimer.singleShot(700, self._advance)
        else:
            self._set_status(f"Found servo at ID {found}. Reassigning to ID {target}…", ORANGE)
            self._id_w = SetIDWorker(self._port, found, target)
            self._id_w.done.connect(self._on_id_set)
            self._id_w.start()

    def _on_id_set(self, ok: bool):
        if ok:
            self._set_status(f"Servo assigned ID {self._current_step}.", GREEN)
            QTimer.singleShot(700, self._advance)
        else:
            self._set_status("Failed to set ID. Try disconnecting and reconnecting.", RED)
            self._trouble_frame.setVisible(True)
            self._action_btn.setEnabled(True)
            self._action_btn.setText("Retry Scan")

    def _advance(self):
        self._assigned.append(self._current_step)
        self._current_step += 1
        if self._current_step > 8:
            self._finish()
        else:
            self._update_step()

    def _set_status(self, text: str, color: str):
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(f"font-size: 13px; color: {color};")

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._sim:
            self._sim.stop()
            self._sim.wait(500)
        event.accept()


# ─────────────────────────────── Entry ───────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    w = ServoIDWizard()
    w.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
