# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec for GELLO xArm7 Launcher.
Build on Linux:  .venv/bin/pyinstaller gello_launcher.spec
Build on Windows: .venv\Scripts\pyinstaller gello_launcher.spec
"""

import sys
from pathlib import Path

block_cipher = None
base = Path(SPECPATH)

# Data files to bundle alongside the executable
datas = [
    (str(base / 'configs'), 'configs'),
    (str(base / 'third_party' / 'mujoco_menagerie' / 'ufactory_xarm7'), 'third_party/mujoco_menagerie/ufactory_xarm7'),
    (str(base / 'experiments'), 'experiments'),
    (str(base / 'gello'), 'gello'),
]

# Hidden imports that PyInstaller can't detect via static analysis
hiddenimports = [
    'dynamixel_sdk',
    'dynamixel_sdk.port_handler',
    'dynamixel_sdk.packet_handler',
    'dynamixel_sdk.group_sync_read',
    'dynamixel_sdk.group_sync_write',
    'dynamixel_sdk.robotis_def',
    'mujoco',
    'mujoco.viewer',
    'dm_control',
    'dm_control.mjcf',
    'numpy',
    'zmq',
    'yaml',
    'hydra',
    'omegaconf',
    'serial',
    'serial.tools',
    'serial.tools.list_ports',
    'gello',
    'gello.agents',
    'gello.agents.agent',
    'gello.agents.gello_agent',
    'gello.robots',
    'gello.robots.robot',
    'gello.robots.sim_robot',
    'gello.robots.xarm_robot',
    'gello.robots.dynamixel',
    'gello.dynamixel',
    'gello.dynamixel.driver',
    'PyQt6',
    'PyQt6.QtCore',
    'PyQt6.QtGui',
    'PyQt6.QtWidgets',
]

a = Analysis(
    [str(base / 'gello_launcher.py')],
    pathex=[str(base)],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='GELLO_Launcher',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,        # no terminal window on Windows
    icon=None,            # add an .ico file here later if desired
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='GELLO_Launcher',
)
