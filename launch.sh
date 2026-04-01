#!/bin/bash
# Launch the GELLO xArm7 application
# Always use this instead of running python3 directly

cd "$(dirname "$0")"

# Kill any stale processes holding the serial port
pkill -f "gello_launcher" 2>/dev/null
sleep 0.5

# Use venv if available, otherwise system python
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
fi

python3 gello_launcher.py
