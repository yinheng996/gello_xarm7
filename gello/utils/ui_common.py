"""
Shared UI constants and helper widgets for xArm7 GUIs.

Provides:
  - Color constants (BG, CARD, TEXT, MUTED, ACCENT, GREEN, ORANGE, RED, BORDER)
  - STYLE — global stylesheet for all xArm7 Qt apps
  - Helper functions: shadow(), card_widget(), hline(), label()
  - ServoDots — compact widget showing 8 servo status dots
  - CheckRow  — single preflight check item (icon + label + status)
"""

from typing import List

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QFrame, QGraphicsDropShadowEffect, QHBoxLayout, QLabel,
    QVBoxLayout, QWidget,
)

# ─────────────────────────────── Colors ───────────────────────────────────────

BG      = "#F5F5F7"
CARD    = "#FFFFFF"
TEXT    = "#1D1D1F"
MUTED   = "#6E6E73"
BORDER  = "#D2D2D7"
ACCENT  = "#0071E3"
ACCENTh = "#0077ED"
GREEN   = "#34C759"
ORANGE  = "#FF9F0A"
RED     = "#FF3B30"

# ─────────────────────────────── Stylesheet ───────────────────────────────────

STYLE = f"""
* {{ font-family: -apple-system, 'SF Pro Text', 'Segoe UI', Arial, sans-serif; }}

QMainWindow, QWidget {{ background: {BG}; color: {TEXT}; }}

QFrame#card {{
    background: {CARD};
    border-radius: 14px;
    border: 1px solid {BORDER};
}}
QFrame#info {{ background: #EBF5FB; border-radius: 8px; border: 1px solid #C6E2F5; }}
QFrame#info_card {{
    background: #EBF5FB;
    border-radius: 10px;
    border: 1px solid #C6E2F5;
}}
QFrame#warn {{ background: #FFF8EC; border-radius: 8px; border: 1px solid #FFD580; }}
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

QPushButton {{
    background: {ACCENT};
    color: white;
    border: none;
    border-radius: 8px;
    padding: 9px 20px;
    font-size: 14px;
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

QPushButton#green {{
    background: {GREEN}; color: white; font-size: 14px; font-weight: 700;
    padding: 12px 24px; border-radius: 8px;
}}
QPushButton#green:hover {{ background: #2DB84C; }}
QPushButton#green:disabled {{ background: {BORDER}; color: {MUTED}; }}

QPushButton#link {{
    background: transparent;
    color: {ACCENT};
    border: none;
    padding: 4px 0px;
    font-size: 12px;
    text-align: left;
}}
QPushButton#link:hover {{ color: {ACCENTh}; }}

QLabel#h1    {{ font-size: 17px; font-weight: 700; color: {TEXT}; }}
QLabel#h2    {{ font-size: 12px; font-weight: 600; color: {TEXT}; }}
QLabel#h3    {{ font-size: 14px; font-weight: 600; color: {TEXT}; }}
QLabel#body  {{ font-size: 14px; color: {MUTED}; padding: 2px 0px; }}
QLabel#tag   {{ font-size: 12px; color: {MUTED}; font-weight: 500; }}
QLabel#ok    {{ font-size: 14px; color: {GREEN};  font-weight: 600; }}
QLabel#warn  {{ font-size: 14px; color: {ORANGE}; font-weight: 600; }}
QLabel#err   {{ font-size: 14px; color: {RED};    font-weight: 600; }}
QLabel#mono  {{ font-family: 'SF Mono', 'Consolas', monospace; font-size: 13px; color: {ACCENT}; }}

QProgressBar {{
    background: {BORDER};
    border: none;
    border-radius: 3px;
    max-height: 4px;
}}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 3px; }}
"""


# ─────────────────────────────── Helpers ──────────────────────────────────────

def shadow(widget, blur=20, offset_y=2, alpha=20):
    e = QGraphicsDropShadowEffect()
    e.setBlurRadius(blur)
    e.setOffset(0, offset_y)
    e.setColor(QColor(0, 0, 0, alpha))
    widget.setGraphicsEffect(e)
    return widget


def card_widget() -> QFrame:
    f = QFrame()
    f.setObjectName("card")
    shadow(f)
    return f


def hline() -> QFrame:
    ln = QFrame()
    ln.setFrameShape(QFrame.Shape.HLine)
    ln.setStyleSheet(f"color: {BORDER}; max-height: 1px;")
    return ln


def label(text: str, obj: str = "body", wrap: bool = False) -> QLabel:
    lbl = QLabel(text)
    lbl.setObjectName(obj)
    if wrap:
        lbl.setWordWrap(True)
    _min_h = {
        "h1": 36, "h2": 26, "h3": 22, "body": 20, "tag": 18,
        "ok": 20, "warn": 20, "err": 20, "mono": 20,
    }
    if obj in _min_h:
        lbl.setMinimumHeight(_min_h[obj])
    return lbl


# ─────────────────────────────── ServoDots ────────────────────────────────────

class ServoDots(QWidget):
    """Compact row of 8 status dots for servo IDs 1-8."""

    def __init__(self):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(10)
        row.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self._dots: List[QLabel] = []
        for i in range(1, 9):
            col = QVBoxLayout()
            col.setSpacing(2)
            col.setAlignment(Qt.AlignmentFlag.AlignHCenter)
            dot = QLabel("●")
            dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
            dot.setStyleSheet("font-size: 17px; color: #D2D2D7;")
            lbl = QLabel("G" if i == 8 else str(i))
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(f"font-size: 10px; color: {MUTED}; font-weight: 500;")
            col.addWidget(dot)
            col.addWidget(lbl)
            row.addLayout(col)
            self._dots.append(dot)

    def refresh(self, found: list, highlight: int = -1):
        for i, dot in enumerate(self._dots, 1):
            if i == highlight:
                dot.setStyleSheet(f"font-size: 17px; color: {ACCENT};")
            elif i in found:
                dot.setStyleSheet(f"font-size: 17px; color: {GREEN};")
            else:
                dot.setStyleSheet("font-size: 17px; color: #D2D2D7;")


# ─────────────────────────────── CheckRow ─────────────────────────────────────

class CheckRow(QWidget):
    """Single preflight check item with icon + label + status."""

    def __init__(self, label_text: str):
        super().__init__()
        self.setMinimumHeight(24)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 2, 0, 2)
        self._icon = QLabel("○")
        self._icon.setFixedWidth(18)
        self._icon.setFixedHeight(20)
        self._icon.setStyleSheet(f"font-size: 14px; color: {MUTED};")
        self._lbl = QLabel(label_text)
        self._lbl.setObjectName("body")
        self._lbl.setMinimumHeight(20)
        self._detail = QLabel("")
        self._detail.setObjectName("tag")
        self._detail.setMinimumHeight(20)
        self._detail.setAlignment(Qt.AlignmentFlag.AlignRight)
        row.addWidget(self._icon)
        row.addWidget(self._lbl)
        row.addStretch()
        row.addWidget(self._detail)

    def set_ok(self, detail=""):
        self._icon.setText("✓")
        self._icon.setStyleSheet(f"font-size: 14px; color: {GREEN};")
        self._detail.setText(detail)
        self._detail.setStyleSheet(f"font-size: 11px; color: {GREEN};")

    def set_warn(self, detail=""):
        self._icon.setText("⚠")
        self._icon.setStyleSheet(f"font-size: 14px; color: {ORANGE};")
        self._detail.setText(detail)
        self._detail.setStyleSheet(f"font-size: 11px; color: {ORANGE};")

    def set_err(self, detail=""):
        self._icon.setText("✗")
        self._icon.setStyleSheet(f"font-size: 14px; color: {RED};")
        self._detail.setText(detail)
        self._detail.setStyleSheet(f"font-size: 11px; color: {RED};")

    def set_pending(self, detail="…"):
        self._icon.setText("○")
        self._icon.setStyleSheet(f"font-size: 14px; color: {MUTED};")
        self._detail.setText(detail)
        self._detail.setStyleSheet(f"font-size: 11px; color: {MUTED};")
