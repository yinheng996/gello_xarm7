@echo off
setlocal enabledelayedexpansion
title xArm7 Controller — Startup
cd /d "%~dp0"

echo.
echo  ================================================
echo   xArm7 Controller
echo  ================================================
echo.

:: ── 1. Find a suitable Python (3.10+) ────────────────────────────────────────
set PYEXE=
set PYVER_OK=0

:: Try candidates in order of preference
for %%C in ("py -3.12" "py -3.11" "py -3.10" "py -3" "python3" "python") do (
    if "!PYEXE!"=="" (
        set _TRY=%%~C
        !_TRY! --version >nul 2>&1
        if !errorlevel!==0 (
            :: Read the version string
            for /f "tokens=2 delims= " %%V in ('!_TRY! --version 2^>^&1') do (
                for /f "tokens=1,2 delims=." %%A in ("%%V") do (
                    set _MAJ=%%A
                    set _MIN=%%B
                )
            )
            if !_MAJ! GEQ 3 if !_MIN! GEQ 10 (
                set PYEXE=!_TRY!
                set PYVER_OK=1
                echo  [OK]  Python !_MAJ!.!_MIN! found  ^(!_TRY!^)
            )
        )
    )
)

if "!PYEXE!"=="" (
    echo  [!!]  Python 3.10 or newer was not found on this machine.
    echo.
    echo        Attempting automatic install via winget...
    echo.
    winget install --id Python.Python.3.11 --source winget --silent --accept-package-agreements --accept-source-agreements
    if !errorlevel!==0 (
        echo  [OK]  Python installed. Please re-run this script.
    ) else (
        echo  [!!]  winget install failed. Please install Python manually:
        echo.
        echo        https://www.python.org/downloads/
        echo.
        echo        Make sure to tick "Add Python to PATH" during install.
    )
    echo.
    pause
    exit /b 1
)

:: ── 2. Check / install pip ────────────────────────────────────────────────────
echo.
echo  Checking pip...
!PYEXE! -m pip --version >nul 2>&1
if !errorlevel! neq 0 (
    echo  [!!]  pip not found — installing...
    !PYEXE! -m ensurepip --upgrade
)

:: ── 3. Install / upgrade required packages ────────────────────────────────────
echo  Installing requirements (this may take a minute on first run)...
echo.
!PYEXE! -m pip install -r requirements.txt --quiet --no-warn-script-location
if !errorlevel! neq 0 (
    echo.
    echo  [WARN] Some packages failed to install. The app may still work.
    echo         Run the Package Wizard inside the app for details.
)

:: Install the local gello package in editable mode if not already installed
!PYEXE! -m pip show gello >nul 2>&1
if !errorlevel! neq 0 (
    echo  Installing local gello package...
    !PYEXE! -m pip install -e . --quiet
)

:: ── 4. Launch ─────────────────────────────────────────────────────────────────
echo.
echo  [>>]  Launching xArm7 Controller...
echo.
!PYEXE! -X utf8 gello_launcher.py
if !errorlevel! neq 0 (
    echo.
    echo  [!!]  The app exited with an error (code !errorlevel!).
    echo        Check the output above for details.
    echo.
    pause
)
endlocal
