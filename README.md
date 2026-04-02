# xArm7 Controller

A desktop application for setting up and controlling an xArm7 robot arm using Dynamixel servos, with a built-in MuJoCo simulation preview.

## Quick Start

1. Build the physical arm and wire the Dynamixel servos to the U2D2 adapter.
2. Download this repository and extract it.
3. Double-click `launch.bat`.

Everything else happens from inside the app. Python installation, package installation, servo setup, calibration, and launching are all handled for you.

## Before vs. After

Previously, bringing up an xArm7 with Dynamixel servos required:

- manually installing Python and pip packages from the command line
- downloading and using the Dynamixel Wizard desktop app to set servo IDs one at a time
- running separate terminal scripts to detect joint offsets
- hand-editing configuration files with the correct offsets, signs, and port paths
- launching simulation and real-arm control from different scripts with different flags
- navigating a codebase full of unrelated robot support (Franka, UR, YAM, bimanual, ROS, ZMQ)

None of that was scripted. Every step was manual and error-prone.

Now, the entire flow is scripted and runs from one GUI:

```
Build the arm
  └─ launch.bat
       └─ xArm7 Launcher
            ├─ Package Wizard ......... check and install Python dependencies
            ├─ Servo ID Wizard ........ identify and assign servo IDs with live 3D preview
            ├─ Calibration Wizard ..... save offsets and verify joint directions with live 3D preview
            └─ Launch
                 ├─ Simulation ........ MuJoCo sim driven by the physical controller
                 └─ Real xArm7 ........ connect to and control the actual arm
```

Every step that used to require a separate tool, a terminal, or manual file editing is now a guided wizard inside the app.

## Workflow

### 0. Build the Hardware

Assemble the xArm7 master arm, install the Dynamixel servos (7 joints + gripper), and connect the servo chain to the U2D2 USB adapter. This is the only step that happens outside of software.

### 1. Install

Double-click `launch.bat`. It checks for Python 3.10+, installs it via `winget` if missing, installs all required packages, and opens the launcher. If you want to verify or fix individual packages later, use the Package Wizard inside the app.

### 2. Assign Servo IDs

Open the Servo ID Wizard. Connect one servo at a time. The wizard scans the bus, shows which joint you are assigning in the 3D preview, and writes the correct ID. Repeat for all 7 joints and the gripper.

### 3. Calibrate

Open the Calibration Wizard. For each joint, match the physical servo to the simulation pose, save the offset, and verify the direction is correct. The wizard walks through all joints and the gripper step by step.

### 4. Launch

Choose Simulation or Real xArm7 from the launcher. Always test in simulation first. If the simulation tracks the physical controller correctly, you are ready for the real arm.

## Design Considerations

Every part of the application was designed to reduce the time and knowledge required to bring up a new arm. Below is a breakdown of the thinking behind each component.

### launch.bat — Zero-Knowledge Entry Point

The startup script exists so that a user who has never opened a terminal can still get the software running. It detects whether Python is installed, attempts to install it automatically if not, installs all pip dependencies, and launches the app. The user does not need to know what pip is, what a virtual environment is, or what packages are required. The script also installs the local package in editable mode so that internal imports resolve correctly without the user needing to understand Python packaging.

### Package Wizard — Visual Dependency Management

Checking packages in a terminal is fast for developers but opaque for operators. The Package Wizard shows every required package with a clear installed/missing indicator, and the Install button handles all of them in one click. The button disables itself during installation to prevent double-clicks, and a live log shows pip output so the user is never staring at a frozen screen wondering if something is happening. A re-check button lets the user verify the result without restarting the app. This replaces the need to ever type `pip install` manually.

### Servo ID Wizard — Replacing the Dynamixel Wizard

Setting servo IDs previously required downloading a separate desktop application (Dynamixel Wizard 2.0), scanning for servos, navigating its interface, and changing the ID field for each servo individually. Users had to remember which physical servo corresponds to which joint number, which is easy to get wrong on a 7-DOF arm. The Servo ID Wizard removes all of that. It shows a 3D model of the arm with the current joint highlighted by a pulsating dot, tells the user exactly which servo to connect, scans the bus automatically, and assigns the correct ID. The user only needs to plug in one servo at a time and press one button. A built-in troubleshooting panel covers common hardware issues (no power, bad cable, LED not blinking) so the user does not need to search online when something does not respond.

### Calibration Wizard — Guided Offset and Direction Verification

Calibration previously required running a terminal script with flags like `--start-joints` and `--joint-signs`, reading the output, and copying numbers into a YAML file by hand. If a sign was wrong, the user had to re-run the script, flip the sign, and try again. The Calibration Wizard turns this into a visual, step-by-step process. For each joint, the wizard shows the expected simulation pose, displays the live servo reading, and lets the user save the offset with one button. A direction check immediately follows: the user moves the joint and can see in real time whether the simulation follows correctly. If the direction is wrong, a Flip Direction button inverts the sign on the spot. The user never needs to open or edit a config file for basic calibration.

### Consistent Split-Panel UI

All three wizards share the same visual layout: a dark left panel for the MuJoCo 3D preview, and a light right panel for controls and information. This was a deliberate choice so that moving between wizards feels familiar rather than like switching to a different application. The consistent layout also means the user always knows where to look — the arm is always on the left, and the action they need to take is always on the right.

### Live MuJoCo Preview During Setup

The 3D preview is not just for the launch phase. It is embedded in both the Servo ID Wizard and the Calibration Wizard. During servo ID assignment, it highlights the joint being assigned so the user can visually confirm they are wiring the correct servo. During calibration, it shows the expected pose and responds to live servo input so the user can immediately see whether the offset and direction are right. This eliminates the guesswork that comes from working with joint numbers and radian values in a terminal.

### Pulsating Dot for Joint Identification

Each wizard highlights the active joint in the 3D preview with a pulsating dot. The dot position is computed by projecting the joint's 3D world coordinates onto the 2D rendered frame. For joints where the pivot point is not visually obvious (like joint 1 at the base, or joints 3 and 5 which sit between links), the dot is placed at a more intuitive location — the base housing for joint 1, and the midpoint between adjacent pivots for joints 3 and 5. This makes it immediately clear which physical servo the wizard is referring to without needing to count joints or read labels.

### Simulation-First Launch

The launcher offers both Simulation and Real xArm7 modes, but the workflow is designed so that simulation is always the first thing you try after calibration. This catches offset errors, sign errors, and wiring mistakes before power is sent to a real robot arm. The simulation runs at 30 Hz with the same servo input path as the real arm, so if it tracks correctly in simulation, it will track correctly on hardware.

### Reusable Internal Modules

Servo communication (`servo_io.py`), MuJoCo rendering (`sim_renderer.py`), and shared UI styles (`ui_common.py`) were extracted into standalone modules. This means the Servo ID Wizard, Calibration Wizard, and Launcher all share the same servo-reading and rendering code instead of each having their own copy. When a bug is fixed or behaviour is improved in one place, it applies everywhere.

### Codebase Cleanup

The original upstream repository supported Franka, UR, YAM, bimanual setups, ROS 2, ZMQ-based communication, RealSense cameras, spacemouse input, data collection utilities, and FACTR gravity compensation. None of that is relevant to the xArm7 single-arm workflow. All of it was removed so that the codebase is smaller, easier to navigate, and does not confuse users with files and imports that have nothing to do with their setup.

## Important Files

| File | Purpose |
|------|---------|
| `launch.bat` | Double-click startup script for Windows |
| `gello_launcher.py` | Central launcher with system checks and navigation |
| `package_wizard.py` | Dependency checker and installer |
| `servo_id_wizard.py` | Guided Dynamixel ID assignment |
| `calibration_wizard.py` | Per-joint offset and direction calibration |
| `configs/xarm_real.yaml` | Configuration for the real xArm7 |
| `configs/xarm_sim_test.yaml` | Configuration for simulation |
| `gello/utils/servo_io.py` | Reusable Dynamixel communication helpers |
| `gello/utils/sim_renderer.py` | Reusable MuJoCo rendering with joint highlighting |
| `gello/utils/ui_common.py` | Shared UI styles, colours, and widgets |

## Configuration

The two config files you may need to edit are:

- `configs/xarm_real.yaml` — real arm (xArm IP, COM port, offsets, signs, gripper)
- `configs/xarm_sim_test.yaml` — simulation (COM port, offsets, signs, gripper)

If you move to a different machine or USB adapter, update the `port` field. If you rebuild the arm or re-calibrate, the Calibration Wizard updates the offsets for you.

## Requirements

Handled automatically by `launch.bat` and the Package Wizard. For reference, the core dependencies are:

`PyQt6` · `numpy` · `mujoco` · `dm_control` · `omegaconf` · `tyro` · `xarm-python-sdk` · `dynamixel-sdk` · `Pillow` · `termcolor`

## Notes

- Always verify in simulation before connecting the real arm.
- If simulation does not match the physical controller, check joint IDs, offsets, signs, and gripper settings in the config files.
- The launcher is the recommended way to access all tools once the app is open.
