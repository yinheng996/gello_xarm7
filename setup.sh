#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# GELLO xArm7 — One-click setup (Linux / macOS)
# ═══════════════════════════════════════════════════════════════════
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔══════════════════════════════════════╗"
echo "║   GELLO xArm7 Setup                 ║"
echo "╚══════════════════════════════════════╝"
echo ""

# ── 1. Check Python ───────────────────────────────────────────────
echo "[1/6] Checking Python..."
if command -v python3 &>/dev/null; then
    PY=python3
elif command -v python &>/dev/null; then
    PY=python
else
    echo "ERROR: Python 3.10+ is required. Install from https://python.org"
    exit 1
fi
PY_VER=$($PY -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "       Found Python $PY_VER"

# ── 2. Install uv if needed ──────────────────────────────────────
echo "[2/6] Checking uv package manager..."
if ! command -v uv &>/dev/null; then
    echo "       Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
fi
echo "       uv $(uv --version)"

# ── 3. Create venv ────────────────────────────────────────────────
echo "[3/6] Creating virtual environment..."
if [ ! -d ".venv" ]; then
    uv venv .venv
fi
echo "       .venv ready"

# ── 4. Clone submodules ──────────────────────────────────────────
echo "[4/6] Initialising git submodules..."
git submodule update --init --recursive 2>/dev/null || true
# If mujoco_menagerie is empty, clone it
if [ ! -f "third_party/mujoco_menagerie/ufactory_xarm7/xarm7.xml" ]; then
    echo "       Cloning mujoco_menagerie..."
    rm -rf third_party/mujoco_menagerie
    git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git third_party/mujoco_menagerie
fi
# DynamixelSDK
if [ ! -f "third_party/DynamixelSDK/python/setup.py" ]; then
    echo "       Cloning DynamixelSDK..."
    rm -rf third_party/DynamixelSDK
    git clone --depth 1 https://github.com/ROBOTIS-GIT/DynamixelSDK.git third_party/DynamixelSDK
fi
echo "       Submodules ready"

# ── 5. Install dependencies ──────────────────────────────────────
echo "[5/6] Installing Python packages..."
uv pip install -e . --python .venv/bin/python 2>/dev/null || true
uv pip install -e third_party/DynamixelSDK/python --python .venv/bin/python
uv pip install PyQt6 mujoco numpy Pillow pyserial --python .venv/bin/python
echo "       Packages installed"

# ── 6. USB permissions (Linux only) ──────────────────────────────
echo "[6/6] Setting up USB permissions..."
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if ! groups | grep -q dialout; then
        echo "       Adding $USER to dialout group (needs sudo)..."
        sudo usermod -aG dialout "$USER"
        echo "       You may need to log out and back in for this to take effect."
        echo "       Or run: newgrp dialout"
    else
        echo "       dialout group OK"
    fi
else
    echo "       Skipped (not Linux)"
fi

echo ""
echo "══════════════════════════════════════════"
echo "  Setup complete!"
echo ""
echo "  Launch the GUI:"
echo "    .venv/bin/python gello_launcher.py"
echo ""
echo "══════════════════════════════════════════"
