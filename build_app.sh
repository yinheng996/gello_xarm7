#!/bin/bash
# Build GELLO Launcher as a standalone application.
# On Linux:  produces dist/GELLO_Launcher/GELLO_Launcher
# On Windows: produces dist/GELLO_Launcher/GELLO_Launcher.exe
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Use venv pyinstaller
if [ -f ".venv/bin/pyinstaller" ]; then
    PYINSTALLER=".venv/bin/pyinstaller"
elif [ -f ".venv/Scripts/pyinstaller.exe" ]; then
    PYINSTALLER=".venv/Scripts/pyinstaller.exe"
else
    echo "PyInstaller not found. Install with: uv pip install pyinstaller"
    exit 1
fi

echo "Building GELLO Launcher..."
$PYINSTALLER gello_launcher.spec --noconfirm
echo ""
echo "Done! Output: dist/GELLO_Launcher/"
