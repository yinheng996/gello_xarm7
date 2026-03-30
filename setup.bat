@echo off
REM ═══════════════════════════════════════════════════════════════
REM  GELLO xArm7 — One-click setup (Windows)
REM ═══════════════════════════════════════════════════════════════
setlocal

echo ======================================
echo   GELLO xArm7 Setup
echo ======================================
echo.

cd /d "%~dp0"

REM ── 1. Check Python ───────────────────────────────────────────
echo [1/5] Checking Python...
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python 3.10+ is required. Install from https://python.org
    pause
    exit /b 1
)
for /f "tokens=2" %%V in ('python --version') do echo        Found Python %%V

REM ── 2. Create venv ────────────────────────────────────────────
echo [2/5] Creating virtual environment...
if not exist ".venv" (
    python -m venv .venv
)
echo        .venv ready

REM ── 3. Clone submodules ───────────────────────────────────────
echo [3/5] Initialising git submodules...
git submodule update --init --recursive 2>nul
if not exist "third_party\mujoco_menagerie\ufactory_xarm7\xarm7.xml" (
    echo        Cloning mujoco_menagerie...
    git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git third_party\mujoco_menagerie
)
if not exist "third_party\DynamixelSDK\python\setup.py" (
    echo        Cloning DynamixelSDK...
    git clone --depth 1 https://github.com/ROBOTIS-GIT/DynamixelSDK.git third_party\DynamixelSDK
)
echo        Submodules ready

REM ── 4. Install dependencies ───────────────────────────────────
echo [4/5] Installing Python packages...
.venv\Scripts\pip install -e . 2>nul
.venv\Scripts\pip install -e third_party\DynamixelSDK\python
.venv\Scripts\pip install PyQt6 mujoco numpy Pillow pyserial
echo        Packages installed

REM ── 5. Done ───────────────────────────────────────────────────
echo [5/5] Setup complete!
echo.
echo ======================================
echo   Launch the GUI:
echo     .venv\Scripts\python gello_launcher.py
echo ======================================
pause
