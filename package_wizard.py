#!/usr/bin/env python3
"""
xArm7 — Package Installation Wizard

Checks which required Python packages are installed and lets the user
install any missing ones with a single click.
"""

import importlib.util
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Tuple

from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)

from gello.utils.ui_common import (
    ACCENT, BG, CARD, BORDER, GREEN, MUTED, ORANGE, RED, STYLE, TEXT,
)

# ─────────────────────────── Package registry ─────────────────────────────────
# Each entry: (display_name, import_name, pip_name, description)
PACKAGES: List[Tuple[str, str, str, str]] = [
    ("PyQt6",            "PyQt6",           "PyQt6",               "GUI framework"),
    ("NumPy",            "numpy",            "numpy",               "Numerical computing"),
    ("MuJoCo",           "mujoco",           "mujoco",              "Physics simulation"),
    ("dm_control",       "dm_control",       "dm_control",          "DeepMind MuJoCo bindings"),
    ("OmegaConf",        "omegaconf",        "omegaconf==2.3.0",    "Configuration management"),
    ("Tyro",             "tyro",             "tyro",                "CLI argument parsing"),
    ("xArm Python SDK",  "xarm",             "xarm-python-sdk",     "xArm robot control"),
    ("Dynamixel SDK",    "dynamixel_sdk",    "dynamixel-sdk",       "Servo communication"),
    ("Pillow",           "PIL",              "Pillow",              "Image processing"),
    ("pyzmq",            "zmq",              "pyzmq",               "ZeroMQ messaging"),
    ("quaternion",       "quaternion",       "numpy-quaternion",    "Quaternion math"),
    ("termcolor",        "termcolor",        "termcolor",           "Coloured terminal output"),
]

BASE_DIR = Path(__file__).parent


# ─────────────────────────── Worker threads ───────────────────────────────────

class CheckWorker(QThread):
    """Check which packages are importable."""
    result = pyqtSignal(list)   # list of bools, one per PACKAGES entry

    def run(self):
        statuses = []
        for _, import_name, _, _ in PACKAGES:
            spec = importlib.util.find_spec(import_name)
            statuses.append(spec is not None)
        self.result.emit(statuses)


class InstallWorker(QThread):
    """pip-install a list of package specs, emitting one log line at a time."""
    log_line  = pyqtSignal(str)
    pkg_done  = pyqtSignal(str, bool)   # (pip_name, success)
    finished  = pyqtSignal(bool)        # overall success

    def __init__(self, pip_names: List[str]):
        super().__init__()
        self._pip_names = pip_names

    def run(self):
        all_ok = True
        for pkg in self._pip_names:
            self.log_line.emit(f"Installing {pkg}…")
            try:
                proc = subprocess.run(
                    [sys.executable, "-m", "pip", "install", pkg,
                     "--quiet", "--no-warn-script-location"],
                    capture_output=True, text=True,
                )
                ok = proc.returncode == 0
                if not ok:
                    all_ok = False
                    err = proc.stderr.strip().splitlines()
                    for line in err[-3:]:
                        self.log_line.emit(f"  {line}")
                self.pkg_done.emit(pkg, ok)
            except Exception as exc:
                self.log_line.emit(f"  Error: {exc}")
                self.pkg_done.emit(pkg, False)
                all_ok = False
        self.finished.emit(all_ok)


# ─────────────────────────── Wizard UI ────────────────────────────────────────

_DARK     = "#1a1a1a"
_T        = "transparent"


def _dark_lbl(text, size=13, bold=False,
               color="rgba(255,255,255,0.8)", wrap=False) -> QLabel:
    lbl = QLabel(text)
    weight = "700" if bold else "400"
    lbl.setStyleSheet(
        f"font-size: {size}px; font-weight: {weight}; color: {color};"
        f" background: {_T}; border: none;")
    if wrap:
        lbl.setWordWrap(True)
    return lbl


class PackageWizard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("xArm7 Package Wizard")
        self.setMinimumSize(860, 480)

        self._statuses: List[Optional[bool]] = [None] * len(PACKAGES)
        self._check_w:   Optional[CheckWorker]   = None
        self._install_w: Optional[InstallWorker] = None

        self._build()
        # Auto-check on open
        QTimer.singleShot(200, self._run_check)

    # ── layout ────────────────────────────────────────────────────────────────
    def _build(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── LEFT: dark — package list ─────────────────────────────────────────
        dark = QWidget()
        dark.setStyleSheet(f"background: {_DARK};")
        dark.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        dl = QVBoxLayout(dark)
        dl.setContentsMargins(60, 40, 60, 40)
        dl.setSpacing(0)

        dl.addStretch(1)
        dl.addWidget(_dark_lbl("Package", size=28, bold=True, color="white"))
        dl.addSpacing(2)
        dl.addWidget(_dark_lbl("Setup", size=28, bold=True,
                               color="rgba(255,255,255,0.35)"))
        dl.addSpacing(8)
        dl.addWidget(_dark_lbl(
            "Checks all Python dependencies required to run the xArm7 software.",
            size=12, color="rgba(255,255,255,0.4)", wrap=True))
        dl.addSpacing(32)

        dl.addWidget(_dark_lbl("REQUIRED PACKAGES", size=10, bold=True,
                               color="rgba(255,255,255,0.3)"))
        dl.addSpacing(14)

        # Scrollable package list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setStyleSheet(f"""
            QScrollArea {{ background: {_T}; border: none; }}
            QScrollBar:vertical {{
                background: rgba(255,255,255,0.05);
                width: 4px; border-radius: 2px;
            }}
            QScrollBar::handle:vertical {{
                background: rgba(255,255,255,0.15); border-radius: 2px;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}
        """)

        pkg_container = QWidget()
        pkg_container.setStyleSheet(f"background: {_T};")
        pkg_layout = QVBoxLayout(pkg_container)
        pkg_layout.setContentsMargins(0, 0, 0, 0)
        pkg_layout.setSpacing(0)

        self._pkg_rows: List[Tuple[QLabel, QLabel]] = []  # (icon_lbl, detail_lbl)
        for i, (display, _, _, desc) in enumerate(PACKAGES):
            row_w = QWidget()
            row_w.setFixedHeight(38)
            row_w.setStyleSheet(f"background: {_T};")
            rl = QHBoxLayout(row_w)
            rl.setContentsMargins(0, 0, 4, 0)
            rl.setSpacing(10)

            icon = QLabel("○")
            icon.setFixedWidth(18)
            icon.setStyleSheet(
                f"font-size: 12px; color: rgba(255,255,255,0.2); background: {_T};")

            name_lbl = QLabel(display)
            name_lbl.setStyleSheet(
                f"font-size: 13px; color: rgba(255,255,255,0.75); background: {_T};")

            desc_lbl = QLabel(desc)
            desc_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            desc_lbl.setStyleSheet(
                f"font-size: 11px; color: rgba(255,255,255,0.25); background: {_T};")

            rl.addWidget(icon)
            rl.addWidget(name_lbl)
            rl.addStretch()
            rl.addWidget(desc_lbl)

            pkg_layout.addWidget(row_w)
            if i < len(PACKAGES) - 1:
                div = QFrame()
                div.setFixedHeight(1)
                div.setStyleSheet(f"background: rgba(255,255,255,0.06);")
                pkg_layout.addWidget(div)

            self._pkg_rows.append((icon, desc_lbl))

        scroll.setWidget(pkg_container)
        dl.addWidget(scroll)
        dl.addStretch(1)

        root.addWidget(dark, 3)

        # ── RIGHT: light — actions ────────────────────────────────────────────
        panel = QWidget()
        panel.setFixedWidth(380)
        panel.setStyleSheet(f"background: {CARD};")
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(40, 40, 40, 40)
        pl.setSpacing(0)

        pl.addStretch(1)

        # Summary badge
        self._summary_icon = QLabel("○")
        self._summary_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._summary_icon.setStyleSheet(
            f"font-size: 48px; color: {BORDER}; background: {_T}; border: none;")
        pl.addWidget(self._summary_icon)
        pl.addSpacing(16)

        self._summary_title = QLabel("Checking…")
        self._summary_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._summary_title.setStyleSheet(
            f"font-size: 18px; font-weight: 700; color: {TEXT};"
            f" background: {_T}; border: none;")
        pl.addWidget(self._summary_title)
        pl.addSpacing(6)

        self._summary_sub = QLabel("")
        self._summary_sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._summary_sub.setWordWrap(True)
        self._summary_sub.setStyleSheet(
            f"font-size: 12px; color: {MUTED}; background: {_T}; border: none;")
        pl.addWidget(self._summary_sub)

        pl.addSpacing(32)

        # Install button
        self._install_btn = QPushButton("Install Missing")
        self._install_btn.setEnabled(False)
        self._install_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._install_btn.setMinimumHeight(48)
        self._install_btn.setStyleSheet(f"""
            QPushButton {{
                background: {ACCENT}; color: white;
                border: none; border-radius: 10px;
                font-size: 15px; font-weight: 700;
                padding: 0px;
            }}
            QPushButton:hover:enabled {{ background: #005BBF; }}
            QPushButton:disabled {{ background: {BORDER}; color: {MUTED}; }}
        """)
        self._install_btn.clicked.connect(self._run_install)
        pl.addWidget(self._install_btn)

        pl.addSpacing(10)

        # Re-check button
        self._recheck_btn = QPushButton("Re-check")
        self._recheck_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._recheck_btn.setMinimumHeight(38)
        self._recheck_btn.setStyleSheet(f"""
            QPushButton {{
                background: {_T}; color: {ACCENT};
                border: 1.5px solid {BORDER}; border-radius: 10px;
                font-size: 13px; font-weight: 600;
                padding: 0px;
            }}
            QPushButton:hover:enabled {{ background: #F0F4FF; border-color: {ACCENT}; }}
            QPushButton:disabled {{ color: {MUTED}; border-color: {BORDER}; }}
        """)
        self._recheck_btn.clicked.connect(self._run_check)
        pl.addWidget(self._recheck_btn)

        pl.addSpacing(24)

        # Log area
        log_hdr = QLabel("INSTALL LOG")
        log_hdr.setStyleSheet(
            f"font-size: 10px; font-weight: 700; color: {MUTED};"
            f" letter-spacing: 1px; background: {_T};")
        pl.addWidget(log_hdr)
        pl.addSpacing(6)

        log_frame = QFrame()
        log_frame.setStyleSheet(
            f"background: {BG}; border-radius: 8px; border: 1px solid {BORDER};")
        log_frame.setMinimumHeight(100)
        lfl = QVBoxLayout(log_frame)
        lfl.setContentsMargins(12, 10, 12, 10)

        self._log_lbl = QLabel("—")
        self._log_lbl.setWordWrap(True)
        self._log_lbl.setAlignment(Qt.AlignmentFlag.AlignTop)
        self._log_lbl.setStyleSheet(
            f"font-size: 11px; color: {MUTED}; background: {_T};"
            f" font-family: 'Consolas', 'SF Mono', monospace;")
        lfl.addWidget(self._log_lbl)

        pl.addWidget(log_frame)

        pl.addStretch(1)

        root.addWidget(panel)

    # ── check logic ───────────────────────────────────────────────────────────
    def _run_check(self):
        self._set_buttons_busy(True)
        self._summary_title.setText("Checking…")
        self._summary_sub.setText("")
        self._summary_icon.setText("○")
        self._summary_icon.setStyleSheet(
            f"font-size: 48px; color: {BORDER}; background: {_T}; border: none;")
        for icon, _ in self._pkg_rows:
            icon.setText("○")
            icon.setStyleSheet(
                f"font-size: 12px; color: rgba(255,255,255,0.2); background: {_T};")

        self._check_w = CheckWorker()
        self._check_w.result.connect(self._on_check_done)
        self._check_w.start()

    def _on_check_done(self, statuses: list):
        self._statuses = statuses
        missing = []

        for i, (ok) in enumerate(statuses):
            icon, _ = self._pkg_rows[i]
            if ok:
                icon.setText("✓")
                icon.setStyleSheet(f"font-size: 12px; color: {GREEN}; background: {_T};")
            else:
                icon.setText("✗")
                icon.setStyleSheet(f"font-size: 12px; color: {RED}; background: {_T};")
                missing.append(PACKAGES[i][2])  # pip name

        n_ok      = sum(statuses)
        n_total   = len(PACKAGES)
        n_missing = n_total - n_ok

        if n_missing == 0:
            self._summary_icon.setText("✓")
            self._summary_icon.setStyleSheet(
                f"font-size: 48px; color: {GREEN}; background: {_T}; border: none;")
            self._summary_title.setText("All packages installed")
            self._summary_sub.setText(
                f"All {n_total} required packages are available.")
            self._install_btn.setEnabled(False)
            self._install_btn.setText("All installed")
        else:
            self._summary_icon.setText("✗")
            self._summary_icon.setStyleSheet(
                f"font-size: 48px; color: {RED}; background: {_T}; border: none;")
            self._summary_title.setText(f"{n_missing} missing")
            self._summary_sub.setText(
                f"{n_ok} of {n_total} packages found. "
                f"Click below to install the missing ones.")
            self._install_btn.setEnabled(True)
            self._install_btn.setText(
                f"Install {n_missing} Missing Package{'s' if n_missing != 1 else ''}")

        self._set_buttons_busy(False)
        self._log("Check complete.")

    # ── install logic ─────────────────────────────────────────────────────────
    def _run_install(self):
        missing_pips = [
            PACKAGES[i][2]
            for i, ok in enumerate(self._statuses)
            if not ok
        ]
        if not missing_pips:
            return

        self._set_buttons_busy(True)
        self._install_btn.setText("Installing…")
        self._log_lbl.setText("")
        self._summary_icon.setText("↓")
        self._summary_icon.setStyleSheet(
            f"font-size: 48px; color: {ACCENT}; background: {_T}; border: none;")
        self._summary_title.setText("Installing…")
        self._summary_sub.setText("Please wait, do not close this window.")

        self._install_w = InstallWorker(missing_pips)
        self._install_w.log_line.connect(self._log)
        self._install_w.pkg_done.connect(self._on_pkg_done)
        self._install_w.finished.connect(self._on_install_done)
        self._install_w.start()

    def _on_pkg_done(self, pip_name: str, success: bool):
        status = "✓" if success else "✗"
        self._log(f"{status} {pip_name}")
        # Update the matching row icon
        for i, (_, _, pn, _) in enumerate(PACKAGES):
            if pn == pip_name:
                icon, _ = self._pkg_rows[i]
                if success:
                    self._statuses[i] = True
                    icon.setText("✓")
                    icon.setStyleSheet(
                        f"font-size: 12px; color: {GREEN}; background: {_T};")
                else:
                    icon.setText("✗")
                    icon.setStyleSheet(
                        f"font-size: 12px; color: {RED}; background: {_T};")

    def _on_install_done(self, all_ok: bool):
        self._set_buttons_busy(False)
        if all_ok:
            self._summary_icon.setText("✓")
            self._summary_icon.setStyleSheet(
                f"font-size: 48px; color: {GREEN}; background: {_T}; border: none;")
            self._summary_title.setText("Installation complete")
            self._summary_sub.setText("All packages installed successfully.")
            self._install_btn.setEnabled(False)
            self._install_btn.setText("All installed")
            self._log("Done.")
        else:
            self._summary_icon.setText("⚠")
            self._summary_icon.setStyleSheet(
                f"font-size: 48px; color: {ORANGE}; background: {_T}; border: none;")
            self._summary_title.setText("Some failed")
            self._summary_sub.setText(
                "One or more packages could not be installed. Check the log.")
            self._install_btn.setEnabled(True)
            self._install_btn.setText("Retry Failed")
            self._log("Finished with errors.")

    # ── helpers ───────────────────────────────────────────────────────────────
    def _set_buttons_busy(self, busy: bool):
        self._install_btn.setEnabled(not busy)
        self._recheck_btn.setEnabled(not busy)

    def _log(self, line: str):
        current = self._log_lbl.text()
        if current == "—":
            current = ""
        lines = (current + "\n" + line).strip().splitlines()
        self._log_lbl.setText("\n".join(lines[-8:]))

    def closeEvent(self, event):
        for w in (self._check_w, self._install_w):
            if w and w.isRunning():
                w.quit()
                w.wait(500)
        event.accept()


# ─────────────────────────── Entry point ──────────────────────────────────────

if __name__ == "__main__":
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    w = PackageWizard()
    w.showMaximized()
    sys.exit(app.exec())
