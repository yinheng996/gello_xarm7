#!/usr/bin/env python3
"""
xArm7 Launcher — Central Hub

Home page with system checks and three navigation paths:
  1. Servo ID Wizard    — assign servo IDs one by one
  2. Calibration Wizard — per-joint calibration with sim preview
  3. Launch             — run sim or real arm
"""

import importlib
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QFileDialog, QFrame, QHBoxLayout,
    QLabel, QMainWindow, QProgressBar, QPushButton, QSizePolicy, QStackedWidget,
    QVBoxLayout, QWidget,
)

from gello.utils.servo_io import (
    ScanWorker, detect_port, NUM_JOINTS, BAUD_RATE,
)
from gello.utils.ui_common import (
    BG, CARD, TEXT, MUTED, ACCENT, GREEN, ORANGE, RED, BORDER,
    STYLE, ServoDots, card_widget, hline, label, shadow,
)

# ─────────────────────────────── Constants ────────────────────────────────────

BASE_DIR    = Path(__file__).parent
CONFIG_SIM  = BASE_DIR / "configs" / "xarm_sim_test.yaml"
CONFIG_REAL = BASE_DIR / "configs" / "xarm_real.yaml"
MODEL_PATH  = BASE_DIR / "third_party" / "mujoco_menagerie" / "ufactory_xarm7" / "xarm7.xml"


# ─────────────────────────────── Helpers ──────────────────────────────────────

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


# ─────────────────────────────── Simulation Thread ────────────────────────────

class SimulationThread(QThread):
    """Runs MuJoCo physics + servo agent in a tight loop, emitting frames.

    All heavy initialisation (model load, port open) happens inside run() on
    the worker thread so the GUI never blocks or crashes.
    """
    frame_ready = pyqtSignal(object)
    telemetry   = pyqtSignal(dict)
    error       = pyqtSignal(str)
    ready       = pyqtSignal()       # emitted once the sim is up

    def __init__(self, agent_cfg: dict, xml_path: str,
                 gripper_xml_path: Optional[str] = None,
                 render_w=800, render_h=600):
        super().__init__()
        self._agent_cfg        = agent_cfg
        self._xml_path         = xml_path
        self._gripper_xml_path = gripper_xml_path
        self._rw, self._rh     = render_w, render_h
        self._running          = True
        self._cam_lock         = threading.Lock()
        self._cam_azimuth      = 150.0
        self._cam_elevation    = -20.0
        self._cam_distance     = 1.8
        self._cam_lookat       = np.array([0.0, 0.0, 0.3])

    def update_camera(self, daz=0.0, dele=0.0, ddist=0.0):
        with self._cam_lock:
            self._cam_azimuth   += daz
            self._cam_elevation  = np.clip(self._cam_elevation + dele, -89, 89)
            self._cam_distance   = max(0.3, self._cam_distance + ddist)

    def stop(self):
        self._running = False

    def run(self):
        try:
            import mujoco
            from gello.robots.sim_robot import build_scene

            # ── Build agent (opens serial port) on this thread ────────────────
            agent = _instantiate(self._agent_cfg)

            # ── Build MuJoCo model ────────────────────────────────────────────
            arena      = build_scene(self._xml_path, self._gripper_xml_path)
            xml_string = arena.to_xml_string()
            assets     = {}
            for asset in arena.asset.all_children():
                if asset.tag == "mesh":
                    f = asset.file
                    assets[f.get_vfs_filename()] = f.contents

            model = mujoco.MjModel.from_xml_string(xml_string, assets)
            model.vis.global_.offwidth  = max(model.vis.global_.offwidth,  self._rw)
            model.vis.global_.offheight = max(model.vis.global_.offheight, self._rh)
            data       = mujoco.MjData(model)
            num_joints = model.nu
            renderer   = mujoco.Renderer(model, height=self._rh, width=self._rw)

            cam             = mujoco.MjvCamera()
            cam.type        = mujoco.mjtCamera.mjCAMERA_FREE
            cam.azimuth     = self._cam_azimuth
            cam.elevation   = self._cam_elevation
            cam.distance    = self._cam_distance
            cam.lookat[:]   = self._cam_lookat

            self.ready.emit()

            # Run at 30 Hz: compute how many physics substeps fit in one frame
            TARGET_HZ  = 30
            dt_target  = 1.0 / TARGET_HZ
            n_substeps = max(1, round(dt_target / model.opt.timestep))
            obs        = {}

            while self._running:
                t0 = time.perf_counter()

                # --- read servo ---
                try:
                    action = agent.act(obs)
                    if len(action) > num_joints:
                        action = action[:num_joints]
                    if len(action) == num_joints and num_joints >= 8:
                        action     = action.copy()
                        action[7] *= 255
                    data.ctrl[:len(action)] = action
                except Exception:
                    pass

                # --- step physics ---
                for _ in range(n_substeps):
                    mujoco.mj_step(model, data)

                # --- render every frame (we're only at 30 Hz, so cost is amortised) ---
                with self._cam_lock:
                    cam.azimuth   = self._cam_azimuth
                    cam.elevation = self._cam_elevation
                    cam.distance  = self._cam_distance
                renderer.update_scene(data, cam)
                frame = renderer.render().copy()
                self.frame_ready.emit(frame)

                # --- telemetry ---
                elapsed = time.perf_counter() - t0
                hz = 1.0 / elapsed if elapsed > 0 else 0
                self.telemetry.emit({
                    "joint_deg": [float(np.rad2deg(data.qpos[i]))
                                  for i in range(min(NUM_JOINTS, num_joints))],
                    "hz":      hz,
                    "sim_time": float(data.time),
                })

                # --- sleep remainder so we hold the target rate ---
                remaining = dt_target - (time.perf_counter() - t0)
                if remaining > 0:
                    time.sleep(remaining)

            renderer.close()
        except Exception as e:
            self.error.emit(str(e))


# ─────────────────────────────── Sim Viewer Widget ────────────────────────────

class MujocoViewerWidget(QWidget):
    """Viewport with mouse orbit controls."""

    def __init__(self):
        super().__init__()
        self._sim: Optional[SimulationThread] = None
        self._last_frame: Optional[np.ndarray] = None
        self._drag_last = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self._viewport = QLabel()
        self._viewport.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._viewport.setStyleSheet("background: #1a1a1a; border: none;")
        self._viewport.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._viewport.setMinimumSize(400, 300)
        lay.addWidget(self._viewport)

    def attach(self, sim: SimulationThread):
        self._sim = sim
        sim.frame_ready.connect(self._on_frame)

    def _on_frame(self, frame: np.ndarray):
        self._last_frame = frame
        h, w, _ = frame.shape
        qimg = QImage(frame.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        vw, vh = self._viewport.width(), self._viewport.height()
        pm = QPixmap.fromImage(qimg).scaled(
            vw, vh, Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation)
        self._viewport.setPixmap(pm)

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

    def screenshot(self):
        if self._last_frame is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Screenshot", "screenshot.png", "PNG (*.png)")
        if path:
            from PIL import Image
            Image.fromarray(self._last_frame).save(path)


# ─────────────────────────────── Dashboard ────────────────────────────────────

class PerformanceDashboard(QFrame):
    """Right panel: live joint angles, loop Hz, sim time, stop button."""
    stop_clicked = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setObjectName("dash_panel")
        self.setFixedWidth(380)
        self.setStyleSheet(f"QFrame#dash_panel {{ background: {CARD}; }}")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(32, 36, 32, 32)
        lay.setSpacing(0)

        lay.addStretch(1)

        title = QLabel("Simulation")
        title.setStyleSheet(
            f"font-size: 22px; font-weight: 800; color: {TEXT}; background: transparent;")
        lay.addWidget(title)
        lay.addSpacing(4)

        self._hz_lbl = QLabel("Connecting…")
        self._hz_lbl.setStyleSheet(
            f"font-size: 13px; color: {MUTED}; background: transparent;")
        lay.addWidget(self._hz_lbl)

        lay.addSpacing(28)

        jt_hdr = QLabel("JOINT ANGLES")
        jt_hdr.setStyleSheet(
            f"font-size: 10px; font-weight: 700; color: {MUTED};"
            f" letter-spacing: 0.8px; background: transparent;")
        lay.addWidget(jt_hdr)
        lay.addSpacing(10)

        self._bars: List[QProgressBar] = []
        self._vals: List[QLabel] = []
        for i in range(NUM_JOINTS):
            row = QHBoxLayout()
            row.setSpacing(8)
            nm = QLabel(f"J{i+1}")
            nm.setFixedWidth(22)
            nm.setStyleSheet(
                f"font-size: 12px; color: {MUTED}; font-weight: 600;"
                f" background: transparent;")
            bar = QProgressBar()
            bar.setRange(-180, 180)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFixedHeight(6)
            bar.setStyleSheet(f"""
                QProgressBar {{ background: {BORDER}; border: none; border-radius: 3px; }}
                QProgressBar::chunk {{ background: {ACCENT}; border-radius: 3px; }}
            """)
            val = QLabel("  0.0°")
            val.setFixedWidth(54)
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            val.setStyleSheet(
                f"font-family: 'Consolas', monospace; font-size: 12px;"
                f" color: {TEXT}; font-weight: 600; background: transparent;")
            row.addWidget(nm)
            row.addWidget(bar, 1)
            row.addWidget(val)
            lay.addLayout(row)
            lay.addSpacing(6)
            self._bars.append(bar)
            self._vals.append(val)

        lay.addStretch(1)

        div = QFrame()
        div.setFixedHeight(1)
        div.setStyleSheet(f"background: {BORDER};")
        lay.addWidget(div)
        lay.addSpacing(20)

        self._stop_btn = QPushButton("Stop Simulation")
        self._stop_btn.setMinimumHeight(46)
        self._stop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._stop_btn.setStyleSheet(f"""
            QPushButton {{
                background: {RED}; color: white;
                border: none; border-radius: 10px;
                font-size: 14px; font-weight: 700;
            }}
            QPushButton:hover {{ background: #D93025; }}
        """)
        self._stop_btn.clicked.connect(self.stop_clicked)
        lay.addWidget(self._stop_btn)

    def update_telemetry(self, data: dict):
        hz = data.get("hz", 0)
        t  = data.get("sim_time", 0)
        c  = GREEN if hz > 25 else (ORANGE if hz > 10 else RED)
        self._hz_lbl.setText(f"{hz:.0f} Hz  ·  {t:.1f} s")
        self._hz_lbl.setStyleSheet(
            f"font-size: 13px; color: {c}; background: transparent;")
        for i, deg in enumerate(data.get("joint_deg", [])):
            if i < len(self._bars):
                self._bars[i].setValue(int(np.clip(deg, -180, 180)))
                self._vals[i].setText(f"{deg:+.1f}°")
                ac = GREEN if abs(deg) < 90 else (ORANGE if abs(deg) < 150 else RED)
                self._bars[i].setStyleSheet(f"""
                    QProgressBar {{ background: {BORDER}; border: none; border-radius: 3px; }}
                    QProgressBar::chunk {{ background: {ac}; border-radius: 3px; }}
                """)


# ═════════════════════════════════════════════════════════════════════════════
# PAGE 1 — HOME (system checks + 3 navigation buttons)
# ═════════════════════════════════════════════════════════════════════════════

class HomePage(QWidget):
    go_pkg_wizard   = pyqtSignal()
    go_servo_wizard = pyqtSignal()
    go_calib_wizard = pyqtSignal()
    go_launch       = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._scan_w: Optional[ScanWorker] = None
        self._port: Optional[str] = None
        self._servo_count = 0
        self._build()

    # ── dark-panel helper labels ─────────────────────────────────────────────
    @staticmethod
    def _dark_label(text, size=13, bold=False, color="rgba(255,255,255,0.85)",
                    spacing=None):
        lbl = QLabel(text)
        weight = "700" if bold else "400"
        extra = f" letter-spacing: {spacing}px;" if spacing else ""
        lbl.setStyleSheet(
            f"font-size: {size}px; font-weight: {weight}; color: {color};{extra}"
            f" background: transparent; border: none;")
        return lbl

    def _build(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ══════════════════════════════════════════════════════════════════════
        # LEFT — dark panel: branding + system status
        # ══════════════════════════════════════════════════════════════════════
        dark = QFrame()
        dark.setObjectName("home_dark")
        dark.setStyleSheet("QFrame#home_dark { background: #1a1a1a; }")
        dark.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        dl = QVBoxLayout(dark)
        dl.setContentsMargins(80, 40, 80, 40)
        dl.setSpacing(0)

        dl.addStretch(1)
        dl.addWidget(self._dark_label("xArm7", size=32, bold=True, color="white"))
        dl.addSpacing(4)
        dl.addWidget(self._dark_label("Controller", size=32, bold=True,
                                       color="rgba(255,255,255,0.45)"))
        dl.addSpacing(32)

        # Status rows
        dl.addWidget(self._dark_label("SYSTEM STATUS", size=10, bold=True,
                                       color="rgba(255,255,255,0.35)", spacing=1.2))
        dl.addSpacing(16)

        self._chk_port   = self._dark_check_row("U2D2 controller")
        self._chk_config = self._dark_check_row("Configuration file")
        self._chk_model  = self._dark_check_row("MuJoCo model")
        self._chk_servos = self._dark_check_row("Servos on bus")

        for i, (row, _, _) in enumerate([self._chk_port, self._chk_config,
                                          self._chk_model, self._chk_servos]):
            dl.addWidget(row)
            if i < 3:
                div = QFrame()
                div.setFixedHeight(1)
                div.setStyleSheet("background: rgba(255,255,255,0.08);")
                dl.addWidget(div)

        dl.addSpacing(28)

        dl.addWidget(self._dark_label("SERVO IDS ON BUS", size=10, bold=True,
                                       color="rgba(255,255,255,0.35)", spacing=1.2))
        dl.addSpacing(12)
        self._dots = ServoDots()
        self._dots.setStyleSheet("background: transparent;")
        dl.addWidget(self._dots)

        dl.addSpacing(24)

        # Guidance banner
        self._guide_card = QFrame()
        self._guide_card.setStyleSheet(
            "background: rgba(255, 159, 10, 0.12); border-radius: 8px;")
        gl = QHBoxLayout(self._guide_card)
        gl.setContentsMargins(14, 10, 14, 10)
        gl.setSpacing(10)
        gi = QLabel("⚠")
        gi.setStyleSheet(f"font-size: 12px; color: {ORANGE}; background: transparent;")
        gi.setFixedWidth(16)
        self._guide_lbl = QLabel("")
        self._guide_lbl.setWordWrap(True)
        self._guide_lbl.setStyleSheet(
            f"font-size: 12px; color: {ORANGE}; background: transparent;")
        gl.addWidget(gi)
        gl.addWidget(self._guide_lbl)
        self._guide_card.setVisible(False)
        dl.addWidget(self._guide_card)

        dl.addStretch(1)

        root.addWidget(dark, 3)

        # ══════════════════════════════════════════════════════════════════════
        # RIGHT — light panel: navigation tiles
        # ══════════════════════════════════════════════════════════════════════
        panel = QFrame()
        panel.setObjectName("home_light")
        panel.setStyleSheet(f"QFrame#home_light {{ background: {CARD}; }}")
        panel.setFixedWidth(420)
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(48, 40, 48, 40)
        pl.setSpacing(0)

        pl.addStretch(1)
        pl.addWidget(self._section_label("SETUP"))
        pl.addSpacing(12)

        self._pkg_btn = self._nav_tile(
            "Package Wizard",
            "Check & install required Python packages",
        )
        self._pkg_btn.clicked.connect(self.go_pkg_wizard)
        pl.addWidget(self._pkg_btn)
        pl.addSpacing(10)

        self._servo_btn = self._nav_tile(
            "Servo ID Wizard",
            "Assign IDs to servos one at a time",
        )
        self._servo_btn.clicked.connect(self.go_servo_wizard)
        pl.addWidget(self._servo_btn)
        pl.addSpacing(10)

        self._calib_btn = self._nav_tile(
            "Calibration Wizard",
            "Set joint offsets & verify directions",
        )
        self._calib_btn.clicked.connect(self.go_calib_wizard)
        pl.addWidget(self._calib_btn)

        pl.addSpacing(32)
        pl.addWidget(self._section_label("RUN"))
        pl.addSpacing(12)

        self._launch_tile = self._nav_tile(
            "Launch",
            "Run simulation or connect to the real arm",
            primary=True,
            enabled=False,
        )
        self._launch_tile.clicked.connect(self.go_launch)
        pl.addWidget(self._launch_tile)

        pl.addStretch(1)

        root.addWidget(panel)

        QTimer.singleShot(200, self.refresh)

    # ── dark-panel check row (icon · label · detail) ──────────────────────
    def _dark_check_row(self, text: str):
        row = QWidget()
        row.setFixedHeight(36)
        row.setStyleSheet("background: transparent;")
        rl = QHBoxLayout(row)
        rl.setContentsMargins(0, 0, 0, 0)
        icon = QLabel("○")
        icon.setFixedWidth(20)
        icon.setStyleSheet(f"font-size: 13px; color: rgba(255,255,255,0.25);"
                           f" background: transparent;")
        lbl = QLabel(text)
        lbl.setStyleSheet("font-size: 13px; color: rgba(255,255,255,0.7);"
                          " background: transparent;")
        detail = QLabel("")
        detail.setAlignment(Qt.AlignmentFlag.AlignRight)
        detail.setStyleSheet("font-size: 11px; color: rgba(255,255,255,0.35);"
                             " background: transparent;")
        rl.addWidget(icon)
        rl.addWidget(lbl)
        rl.addStretch()
        rl.addWidget(detail)
        return row, icon, detail

    @staticmethod
    def _section_label(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet(
            f"font-size: 11px; font-weight: 700; color: {MUTED};"
            f" letter-spacing: 1px; background: transparent;")
        return lbl

    def _nav_tile(self, title: str, subtitle: str,
                  primary: bool = False, enabled: bool = True) -> QPushButton:
        btn = QPushButton("")
        btn.setEnabled(enabled)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        btn.setMinimumHeight(72)
        btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        if primary:
            bg, bg_h, bg_dis = ACCENT, "#005BBF", "#C8C8CC"
            border     = "none"
            border_h   = "none"
            border_dis = "none"
            fg, fg_dis = "white", "#888"
            sub_c  = "rgba(255,255,255,0.6)"
            chev_c = "rgba(255,255,255,0.45)"
        else:
            bg, bg_h, bg_dis = CARD, "#F5F7FF", CARD
            border     = f"1.5px solid {BORDER}"
            border_h   = f"1.5px solid {ACCENT}"
            border_dis = f"1.5px solid {BORDER}"
            fg, fg_dis = TEXT, MUTED
            sub_c  = MUTED
            chev_c = "#C8C8CC"

        _t = "transparent"
        btn.setStyleSheet(f"""
            QPushButton {{
                background: {bg};
                border: {border};
                border-radius: 12px;
                padding: 0px;
                text-align: left;
            }}
            QPushButton:hover:enabled {{ background: {bg_h}; border: {border_h}; }}
            QPushButton:disabled {{ background: {bg_dis}; border: {border_dis}; }}
        """)

        row = QHBoxLayout(btn)
        row.setContentsMargins(20, 16, 20, 16)
        row.setSpacing(0)

        text_col = QVBoxLayout()
        text_col.setSpacing(1)
        title_lbl = QLabel(title, btn)
        title_lbl.setStyleSheet(
            f"font-size: 15px; font-weight: 700; color: {fg if enabled else fg_dis};"
            f" background: {_t}; border: none;")
        title_lbl.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        sub_lbl = QLabel(subtitle, btn)
        sub_lbl.setStyleSheet(
            f"font-size: 12px; color: {sub_c}; background: {_t}; border: none;"
            f" font-weight: 400;")
        sub_lbl.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        text_col.addWidget(title_lbl)
        text_col.addWidget(sub_lbl)

        chev = QLabel("›", btn)
        chev.setStyleSheet(
            f"font-size: 20px; color: {chev_c}; background: {_t}; border: none;")
        chev.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

        row.addLayout(text_col, 1)
        row.addWidget(chev)

        btn._title_lbl = title_lbl
        btn._fg        = fg
        btn._fg_dis    = fg_dis

        return btn

    # ── dark check-row state helpers ─────────────────────────────────────────
    def _dchk_ok(self, chk_tuple, detail=""):
        _, icon, det = chk_tuple
        icon.setText("✓")
        icon.setStyleSheet(f"font-size: 13px; color: {GREEN}; background: transparent;")
        det.setText(detail)
        det.setStyleSheet(f"font-size: 11px; color: {GREEN}; background: transparent;")

    def _dchk_warn(self, chk_tuple, detail=""):
        _, icon, det = chk_tuple
        icon.setText("⚠")
        icon.setStyleSheet(f"font-size: 13px; color: {ORANGE}; background: transparent;")
        det.setText(detail)
        det.setStyleSheet(f"font-size: 11px; color: {ORANGE}; background: transparent;")

    def _dchk_err(self, chk_tuple, detail=""):
        _, icon, det = chk_tuple
        icon.setText("✗")
        icon.setStyleSheet(f"font-size: 13px; color: {RED}; background: transparent;")
        det.setText(detail)
        det.setStyleSheet(f"font-size: 11px; color: {RED}; background: transparent;")

    def _dchk_pending(self, chk_tuple, detail="…"):
        _, icon, det = chk_tuple
        icon.setText("○")
        icon.setStyleSheet("font-size: 13px; color: rgba(255,255,255,0.25);"
                           " background: transparent;")
        det.setText(detail)
        det.setStyleSheet("font-size: 11px; color: rgba(255,255,255,0.35);"
                          " background: transparent;")

    def refresh(self):
        self._port = detect_port()
        config_ok = CONFIG_SIM.exists()
        model_ok = MODEL_PATH.exists()

        if self._port:
            short = self._port.split("/")[-1][:30] if "/" in self._port else self._port
            self._dchk_ok(self._chk_port, short)
        else:
            self._dchk_err(self._chk_port, "Not detected")

        if config_ok:
            self._dchk_ok(self._chk_config, "Found")
        else:
            self._dchk_err(self._chk_config, "Missing")

        if model_ok:
            self._dchk_ok(self._chk_model, "Found")
        else:
            self._dchk_err(self._chk_model, "Missing")

        self._dchk_pending(self._chk_servos, "Scanning…")

        issues = []
        if not self._port:
            issues.append("U2D2 not found — plug in USB cable.")
        if not config_ok:
            issues.append("Config missing — check configs/xarm_sim_test.yaml.")
        if not model_ok:
            issues.append("Model missing — run: git submodule update --init.")

        self._set_launch_enabled(False)

        if self._port:
            if self._scan_w and self._scan_w.isRunning():
                self._scan_w.stop()
            self._scan_w = ScanWorker(self._port)
            self._scan_w.result.connect(
                lambda data: self._on_scan(data, config_ok, model_ok, issues))
            self._scan_w.start()
        else:
            if issues:
                self._guide_lbl.setText("  ·  ".join(issues))
                self._guide_card.setVisible(True)
            else:
                self._guide_card.setVisible(False)

    def _on_scan(self, data: dict, config_ok: bool, model_ok: bool, issues: list):
        found = sorted(data.keys())
        n = len(found)
        self._dots.refresh(found)
        self._servo_count = n

        if n == 8:
            self._dchk_ok(self._chk_servos, "8 / 8")
        elif n > 0:
            self._dchk_warn(self._chk_servos, f"{n} / 8")
        else:
            self._dchk_err(self._chk_servos, "None")

        if n < 8:
            if n == 0:
                issues.append("No servos detected — check power/USB, run Servo ID Wizard.")
            else:
                missing = [i for i in range(1, 9) if i not in found]
                issues.append(f"Missing servo IDs {missing} — run Servo ID Wizard.")

        if issues:
            self._guide_lbl.setText("  ·  ".join(issues))
            self._guide_card.setVisible(True)
        else:
            self._guide_card.setVisible(False)

        self._set_launch_enabled(bool(self._port and config_ok and model_ok and n == 8))

    def _set_launch_enabled(self, ok: bool):
        self._launch_tile.setEnabled(ok)
        fg = self._launch_tile._fg if ok else self._launch_tile._fg_dis
        self._launch_tile._title_lbl.setStyleSheet(
            f"font-size: 15px; font-weight: 700; color: {fg};"
            f" background: transparent; border: none;")


# ─── shared dark-panel helpers used by LaunchPage ────────────────────────────

def _mk_dark_lbl(text, size=13, bold=False,
                 color="rgba(255,255,255,0.8)", wrap=False) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"font-size: {size}px; font-weight: {'700' if bold else '400'}; color: {color};"
        f" background: transparent; border: none;")
    if wrap:
        lbl.setWordWrap(True)
    return lbl


def _mk_section_lbl(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"font-size: 11px; font-weight: 700; color: {MUTED};"
        f" letter-spacing: 1px; background: transparent;")
    return lbl


# ═════════════════════════════════════════════════════════════════════════════
# PAGE 2 — LAUNCH (choose sim or real, clean sim viewer)
# ═════════════════════════════════════════════════════════════════════════════

class LaunchPage(QWidget):
    go_back = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._sim_thread: Optional[SimulationThread] = None
        self._proc: Optional[subprocess.Popen] = None
        self._mode: Optional[str] = None  # "sim" or "real"
        self._build()

    def _build(self):
        self._stack = QStackedWidget()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._stack)

        # Sub-page 0: mode picker
        self._picker = QWidget()
        self._build_picker()
        self._stack.addWidget(self._picker)

        # Sub-page 1: simulation viewer
        self._sim_page = QWidget()
        self._build_sim_viewer()
        self._stack.addWidget(self._sim_page)

    def _build_picker(self):
        root = QHBoxLayout(self._picker)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Dark left panel — use QFrame with object name so it wins over global QWidget rule
        dark = QFrame()
        dark.setObjectName("dark_panel")
        dark.setStyleSheet("QFrame#dark_panel { background: #1a1a1a; }")
        dark.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        dl = QVBoxLayout(dark)
        dl.setContentsMargins(80, 40, 80, 40)
        dl.setSpacing(0)

        dl.addStretch(2)
        dl.addWidget(_mk_dark_lbl("Launch", 32, bold=True, color="white"))
        dl.addSpacing(2)
        dl.addWidget(_mk_dark_lbl("xArm7", 32, bold=True,
                                   color="rgba(255,255,255,0.35)"))
        dl.addSpacing(20)
        dl.addWidget(_mk_dark_lbl(
            "Choose how to run the controller.\nSimulation lets you test without the physical arm.",
            13, color="rgba(255,255,255,0.4)", wrap=True))
        dl.addStretch(3)

        root.addWidget(dark, 3)

        # Light right panel
        panel = QFrame()
        panel.setObjectName("light_panel")
        panel.setStyleSheet(f"QFrame#light_panel {{ background: {CARD}; }}")
        panel.setFixedWidth(420)
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(40, 32, 40, 40)
        pl.setSpacing(0)

        # Back button at the top of the right panel
        back = QPushButton("← Back")
        back.setCursor(Qt.CursorShape.PointingHandCursor)
        back.setFixedWidth(88)
        back.setFixedHeight(34)
        back.setStyleSheet(f"""
            QPushButton {{
                background: transparent;
                color: {ACCENT};
                border: 1.5px solid {BORDER};
                border-radius: 8px;
                font-size: 13px; font-weight: 600;
                padding: 0px 12px;
            }}
            QPushButton:hover {{ background: #F0F4FF; border-color: {ACCENT}; }}
        """)
        back.clicked.connect(self._do_back)
        pl.addWidget(back)

        pl.addStretch(1)

        pl.addWidget(_mk_section_lbl("MODE"))
        pl.addSpacing(14)

        self._sim_btn = self._launch_tile(
            "Simulation",
            "Virtual MuJoCo arm follows your servo input",
            primary=True,
        )
        self._sim_btn.clicked.connect(self._launch_sim)
        pl.addWidget(self._sim_btn)
        pl.addSpacing(10)

        real_ok = CONFIG_REAL.exists()
        self._real_btn = self._launch_tile(
            "Real xArm7",
            "Servo input drives the physical arm directly" if real_ok
            else "Config not found — configs/xarm_real.yaml",
            primary=False,
            enabled=real_ok,
        )
        self._real_btn.clicked.connect(self._launch_real)
        pl.addWidget(self._real_btn)

        pl.addSpacing(20)
        self._picker_status = QLabel("")
        self._picker_status.setWordWrap(True)
        self._picker_status.setStyleSheet(
            f"font-size: 12px; color: {MUTED}; background: transparent;")
        pl.addWidget(self._picker_status)

        pl.addStretch(1)

        root.addWidget(panel)

    @staticmethod
    def _launch_tile(title, subtitle, primary=True, enabled=True):
        btn = QPushButton("")
        btn.setEnabled(enabled)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        btn.setMinimumHeight(72)
        btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        _t = "transparent"
        if primary:
            bg, bg_h = ACCENT, "#005BBF"
            fg, sub_c, chev_c = "white", "rgba(255,255,255,0.6)", "rgba(255,255,255,0.45)"
            border = "none"
        else:
            bg, bg_h = CARD, "#F5F7FF"
            fg = TEXT if enabled else MUTED
            sub_c, chev_c = MUTED, "#C8C8CC"
            border = f"1.5px solid {BORDER}"

        btn.setStyleSheet(f"""
            QPushButton {{
                background: {bg}; border: {border};
                border-radius: 12px; padding: 0px;
            }}
            QPushButton:hover:enabled {{ background: {bg_h}; }}
            QPushButton:disabled {{ background: {CARD if not primary else BORDER};
                                    border: 1.5px solid {BORDER}; }}
        """)
        row = QHBoxLayout(btn)
        row.setContentsMargins(20, 16, 20, 16)
        row.setSpacing(0)
        col = QVBoxLayout()
        col.setSpacing(1)
        t = QLabel(title, btn)
        t.setStyleSheet(
            f"font-size: 15px; font-weight: 700; color: {fg};"
            f" background: {_t}; border: none;")
        t.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        s = QLabel(subtitle, btn)
        s.setStyleSheet(
            f"font-size: 12px; color: {sub_c}; background: {_t}; border: none;")
        s.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        col.addWidget(t)
        col.addWidget(s)
        chev = QLabel("›", btn)
        chev.setStyleSheet(
            f"font-size: 20px; color: {chev_c}; background: {_t}; border: none;")
        chev.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        row.addLayout(col, 1)
        row.addWidget(chev)
        return btn

    def _build_sim_viewer(self):
        root = QHBoxLayout(self._sim_page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Left: MuJoCo viewport (dark, edge-to-edge)
        viewer_wrap = QFrame()
        viewer_wrap.setObjectName("sim_dark")
        viewer_wrap.setStyleSheet("QFrame#sim_dark { background: #1a1a1a; }")
        viewer_wrap.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        vl = QVBoxLayout(viewer_wrap)
        vl.setContentsMargins(0, 0, 0, 0)
        self._viewer = MujocoViewerWidget()
        vl.addWidget(self._viewer)
        root.addWidget(viewer_wrap, 3)

        # Right: white control panel (same width as home-page right panel)
        self._dash = PerformanceDashboard()
        self._dash.stop_clicked.connect(self._stop_sim)
        root.addWidget(self._dash)

    # ── Launch actions ────────────────────────────────────────────────────────

    def _launch_sim(self):
        self._sim_btn.setEnabled(False)
        self._real_btn.setEnabled(False)
        self._picker_status.setText("")
        self._stack.setCurrentIndex(1)   # show viewer immediately
        self._dash._hz_lbl.setText("Initialising…")

        try:
            from omegaconf import OmegaConf
            cfg          = OmegaConf.to_container(OmegaConf.load(str(CONFIG_SIM)), resolve=True)
            agent_cfg    = cfg["agent"]
            xml_path     = str(BASE_DIR / cfg["robot"]["xml_path"])
            gripper_path = cfg["robot"].get("gripper_xml_path")
            if gripper_path:
                gripper_path = str(BASE_DIR / gripper_path)

            self._sim_thread = SimulationThread(
                agent_cfg    = agent_cfg,
                xml_path     = xml_path,
                gripper_xml_path = gripper_path,
            )
            self._viewer.attach(self._sim_thread)
            self._sim_thread.ready.connect(
                lambda: self._dash._hz_lbl.setText("Running"))
            self._sim_thread.telemetry.connect(self._dash.update_telemetry)
            self._sim_thread.error.connect(self._on_sim_error)
            self._sim_thread.start()
            self._mode = "sim"
        except Exception as e:
            self._on_sim_error(str(e))

    def _launch_real(self):
        self._real_btn.setText("Starting…")
        self._real_btn.setEnabled(False)
        self._sim_btn.setEnabled(False)

        for candidate in [
            BASE_DIR / ".venv" / "bin" / "python",
            BASE_DIR / ".venv" / "Scripts" / "python.exe",
            Path(sys.executable),
        ]:
            if Path(candidate).exists():
                python = str(candidate)
                break
        else:
            python = sys.executable

        try:
            self._proc = subprocess.Popen(
                [python, str(BASE_DIR / "experiments" / "launch_yaml.py"),
                 "--left-config-path", str(CONFIG_REAL)],
                cwd=str(BASE_DIR),
                start_new_session=True,
            )
            self._mode = "real"
            self._real_btn.setText("🦾  Running…")
        except Exception as e:
            self._real_btn.setText("🦾  Launch Real Arm")
            self._real_btn.setEnabled(True)
            self._sim_btn.setEnabled(True)
            self._picker_status.setText(f"Launch failed: {e}")
            self._picker_status.setStyleSheet(f"font-size: 13px; color: {RED};")

    def _on_sim_error(self, msg: str):
        # Show error in the viewport and reset the panel so user can go back
        self._viewer._viewport.setPixmap(QPixmap())
        self._viewer._viewport.setText(f"⚠  {msg}")
        self._viewer._viewport.setStyleSheet(
            f"background: #1a1a1a; color: {RED};"
            f" padding: 32px; font-size: 13px; font-weight: 600;")
        self._dash._hz_lbl.setText("Error — see left panel")
        self._dash._hz_lbl.setStyleSheet(f"font-size: 13px; color: {RED}; background: transparent;")

    def _stop_sim(self):
        if self._sim_thread and self._sim_thread.isRunning():
            self._sim_thread.stop()
            self._sim_thread.wait(3000)
        self._sim_thread = None
        self._reset_picker()
        self._stack.setCurrentIndex(0)   # reset to picker for next time
        self.go_back.emit()              # go all the way back to home

    def _kill_proc(self):
        if self._proc and self._proc.poll() is None:
            try:
                if sys.platform == "win32":
                    self._proc.kill()
                else:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
        self._proc = None

    def _reset_picker(self):
        self._sim_btn.setText("▶  Launch Simulation")
        self._sim_btn.setEnabled(True)
        self._real_btn.setText("🦾  Launch Real Arm")
        self._real_btn.setEnabled(CONFIG_REAL.exists())
        self._picker_status.setText("")

    def _do_back(self):
        self._kill_proc()
        if self._sim_thread and self._sim_thread.isRunning():
            self._sim_thread.stop()
            self._sim_thread.wait(3000)
        self._sim_thread = None
        self._reset_picker()
        self.go_back.emit()

    def cleanup(self):
        self._kill_proc()
        if self._sim_thread and self._sim_thread.isRunning():
            self._sim_thread.stop()
            self._sim_thread.wait(3000)


# ═════════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("xArm7 Controller")
        self.setMinimumSize(800, 500)

        self._stack = QStackedWidget()
        self.setCentralWidget(self._stack)

        self._home = HomePage()
        self._launch_page = LaunchPage()

        self._stack.addWidget(self._home)        # 0
        self._stack.addWidget(self._launch_page)  # 1

        self._home.go_pkg_wizard.connect(self._open_pkg_wizard)
        self._home.go_servo_wizard.connect(self._open_servo_wizard)
        self._home.go_calib_wizard.connect(self._open_calib_wizard)
        self._home.go_launch.connect(self._show_launch)
        self._launch_page.go_back.connect(self._show_home)

    def _show_home(self):
        self._stack.setCurrentIndex(0)
        self._home.refresh()

    def _show_launch(self):
        self._stack.setCurrentIndex(1)

    def _open_pkg_wizard(self):
        """Launch the Package Wizard in a separate process."""
        subprocess.Popen(
            [sys.executable, str(BASE_DIR / "package_wizard.py")],
            cwd=str(BASE_DIR),
        )

    def _open_servo_wizard(self):
        """Launch the Servo ID Wizard in a separate process."""
        python = sys.executable
        subprocess.Popen(
            [python, str(BASE_DIR / "servo_id_wizard.py")],
            cwd=str(BASE_DIR),
        )

    def _open_calib_wizard(self):
        """Launch the Calibration Wizard in a separate process."""
        python = sys.executable
        subprocess.Popen(
            [python, str(BASE_DIR / "calibration_wizard.py")],
            cwd=str(BASE_DIR),
        )

    def closeEvent(self, event):
        self._launch_page.cleanup()
        event.accept()


# ─────────────────────────────── Entry ────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    app.setApplicationName("xArm7 Controller")
    w = MainWindow()
    w.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
