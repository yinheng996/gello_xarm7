# Individual Contribution — xArm7 Controller

## Overview

I took an existing open-source teleoperation framework (GELLO) that supported multiple robot platforms and simplified it into a single-purpose desktop application for the xArm7 robot arm. The original codebase required terminal commands, third-party desktop tools, manual configuration file editing, and familiarity with the codebase structure to get a working setup. My contribution was to script and automate the entire bring-up process, from first launch to real-arm control, and package it behind a GUI that guides the user through every step.

## Problem

The original GELLO software was built as a research framework. It supported Franka, UR, YAM, xArm, bimanual setups, ROS 2, ZMQ communication, camera integration, and data collection. For an operator who only needed to set up one xArm7, this meant:

- Navigating a large codebase to find the relevant xArm files among dozens of unrelated modules.
- Manually installing Python packages from the terminal.
- Downloading and learning the Dynamixel Wizard 2.0 desktop application just to assign servo IDs.
- Running terminal scripts with specific flags to detect joint offsets, then copying the output values into YAML configuration files by hand.
- Remembering which joint signs to use and manually flipping them when direction was wrong.
- Launching simulation and real-arm control from separate scripts with different command-line arguments.

None of these steps were scripted or connected to each other. Each one assumed the user was a developer comfortable with terminals, Python packaging, and the internal structure of the codebase. There were no checks, no feedback, and no visual guidance during setup.

## What I Built

I built a complete desktop application that replaces all of those manual steps with a single guided workflow.

### Startup Script (launch.bat)

A double-clickable Windows batch script that handles the entire environment setup. It detects Python 3.10+, attempts to install it via winget if missing, installs all pip dependencies from requirements.txt, installs the local package, and launches the application. The user does not need to open a terminal at any point.

The consideration here was that the target user may not have Python installed and may not know what pip is. The script needed to handle every failure case gracefully — missing Python, missing pip, failed package installs — and either fix the problem automatically or tell the user exactly what went wrong.

### Central Launcher (gello_launcher.py)

A PyQt6 desktop application that serves as the home screen for the entire workflow. It shows system status checks (Python version, Dynamixel port detection, servo connectivity) and provides navigation to three tools: the Servo ID Wizard, the Calibration Wizard, and the Launch page.

The consideration was that the user should never need to remember which script to run or in what order. The launcher presents the tools in the correct workflow order and shows live hardware status so the user knows immediately if something is disconnected.

### Package Wizard (package_wizard.py)

A visual dependency checker and installer. It lists every required Python package with a clear installed/missing status indicator and provides a single Install button to install everything that is missing. The button disables during installation to prevent double-clicks, pip output is shown in a live log, and a re-check button lets the user verify the result.

The consideration was that pip install failures in a terminal are confusing for non-developers. The wizard makes the status of every package visible at a glance, and the log output means the user can see exactly what is happening during installation without needing to understand pip's command-line interface.

### Servo ID Wizard (servo_id_wizard.py)

A guided tool for assigning Dynamixel servo IDs. The user connects one servo at a time, presses a button, and the wizard scans the bus, detects the servo, and writes the correct ID for the current joint. A live MuJoCo 3D preview highlights the joint being assigned with a pulsating dot so the user can see exactly which physical servo they should be connecting.

The consideration was that the previous method — using the separate Dynamixel Wizard 2.0 application — required the user to download another tool, learn its interface, and manually track which servo corresponds to which joint number. On a 7-DOF arm plus gripper, this is easy to get wrong. By integrating servo ID assignment directly into the workflow with a 3D visual reference, the chance of assigning the wrong ID to the wrong servo is significantly reduced. A built-in troubleshooting panel covers the most common hardware issues (no power, bad cable, LED not blinking) so the user does not need to search online when a servo does not respond.

### Calibration Wizard (calibration_wizard.py)

A step-by-step calibration tool for saving per-joint offsets and verifying joint directions. For each of the 7 joints and the gripper, the wizard shows the expected simulation pose, displays the live servo reading in real time, and lets the user save the offset with one button. Immediately after saving, a direction check lets the user move the joint and see in real time whether the simulation follows correctly. If the direction is wrong, a Flip Direction button inverts the sign on the spot.

The consideration was that the previous method required running a terminal script with flags like --start-joints and --joint-signs, reading offset values from the output, and pasting them into a YAML file. If a direction was wrong, the user had to re-run the script with a different sign and try again. This loop was slow and error-prone. The wizard eliminates file editing entirely for basic calibration and gives immediate visual feedback so the user can verify each joint before moving to the next one.

### Live MuJoCo Preview

The MuJoCo simulation preview is not just for the launch phase. It is embedded in both the Servo ID Wizard and the Calibration Wizard. During servo ID assignment, it highlights the joint being assigned so the user can visually confirm they are wiring the correct servo. During calibration, it shows the expected pose and responds to live servo input so the user can immediately see whether the offset and direction are right.

The consideration was that joint numbers and radian values are abstract. Seeing the arm in 3D and watching it respond to physical input removes the ambiguity. The pulsating dot that highlights the active joint was carefully positioned for each joint — for example, joint 1 shows the dot at the base housing rather than the shoulder pivot, and joints 3 and 5 show the dot at the midpoint between adjacent pivots, because those are more visually intuitive locations on the physical arm.

### Consistent UI Design

All three wizards share the same split-panel layout: dark left panel for the 3D preview, light right panel for controls and information. Navigation, button placement, and styling are consistent across every screen.

The consideration was that each wizard serves a different function but the user should not feel like they are switching between different applications. A consistent layout means the user always knows where to look — the arm is always on the left, the action is always on the right. Shared UI components (colours, card styles, button styles, servo dot indicators) are defined in one module and reused everywhere.

### Simulation-First Launch Flow

The launcher offers Simulation and Real xArm7 as separate launch options. The workflow is designed so that simulation is always the recommended first step after calibration.

The consideration was safety. If offsets or signs are wrong, the real arm will move in unexpected ways. By running the same servo input through MuJoCo first, errors are caught visually before any commands are sent to real hardware. The simulation uses the same 30 Hz loop and the same servo-reading path as the real-arm mode, so if it works in simulation, it will work on hardware.

### Codebase Cleanup

I removed all code unrelated to the xArm7 single-arm workflow. This included support for Franka, UR, YAM, bimanual setups, ROS 2, ZMQ communication, RealSense cameras, spacemouse input, data collection, FACTR gravity compensation, Docker configuration, CI workflows, and linter configs for the old multi-robot repo. I also extracted shared logic (servo communication, simulation rendering, UI styles) into reusable modules so the remaining code is not duplicated across tools.

The consideration was that a large multi-purpose codebase makes it harder to understand what is relevant, harder to modify without breaking unrelated features, and harder for a new user to navigate. By removing everything that is not part of the xArm7 workflow, the codebase is smaller, more readable, and more maintainable.

## Summary

My contribution was taking a general-purpose research framework and turning it into a focused, operator-friendly desktop application for the xArm7. The core idea was that every manual step in the original workflow — installing packages, assigning servo IDs, calibrating offsets, verifying joint directions, launching simulation and hardware control — could and should be scripted, guided, and visual. The result is a workflow where a user can go from downloading the repository to controlling the arm without ever opening a terminal, editing a config file, or using a separate tool.
