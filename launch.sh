#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

find_python() {
  for cmd in python3.12 python3.11 python3.10 python3 python; do
    if command -v "$cmd" >/dev/null 2>&1; then
      "$cmd" - <<'PY' >/dev/null 2>&1 && { echo "$cmd"; return 0; }
import sys
raise SystemExit(0 if sys.version_info >= (3, 10) else 1)
PY
    fi
  done
  return 1
}

PYEXE="$(find_python || true)"

if [ -z "${PYEXE}" ]; then
  echo "Python 3.10+ not found."
  case "$(uname -s)" in
    Darwin)
      echo "Install it with: brew install python"
      ;;
    Linux)
      echo "Install it with your package manager, for example:"
      echo "  Ubuntu/Debian: sudo apt install python3 python3-pip"
      echo "  Fedora: sudo dnf install python3 python3-pip"
      echo "  Arch: sudo pacman -S python python-pip"
      ;;
    *)
      echo "Unsupported operating system."
      ;;
  esac
  exit 1
fi

echo "Using ${PYEXE}"

if ! "${PYEXE}" -m pip --version >/dev/null 2>&1; then
  echo "pip not found, attempting to install it..."
  "${PYEXE}" -m ensurepip --upgrade || true
fi

echo "Installing requirements..."
"${PYEXE}" -m pip install -r requirements.txt

echo "Installing local package..."
"${PYEXE}" -m pip install -e .

echo "Launching xArm7 Controller..."
exec "${PYEXE}" -X utf8 gello_launcher.py
