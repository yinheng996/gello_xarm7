## 3.X Software Development Methodology

### 3.X.1 Software Objective

The software component of the project serves as the interface between the human operator and the xArm7 robotic arm. Its purpose is to translate the operator's physical manipulation of the teleoperation device into corresponding joint-level commands, either within a simulated environment or, in future work, on the physical robot. Within the broader project aim of developing a low-cost and intuitive teleoperation manipulator, the software is responsible for minimising the time cost of system setup, reducing the technical knowledge required to operate the system, and providing a guided, visually informative workflow that enables a new user to proceed from installation to operation with minimal friction.

### 3.X.2 Starting Point and Scope of Adaptation

The software was developed from a forked repository originally based on GELLO, an open-source teleoperation framework. However, the contribution of this project is not a reproduction of GELLO. The original codebase was designed as a general-purpose research tool supporting multiple robot platforms, and its workflow assumed familiarity with command-line configuration, manual dependency management, and direct editing of configuration files. The adaptation undertaken in this project restructured the software into an xArm7-specific application with a streamlined, GUI-centred workflow. Non-essential code paths and configurations relating to other robot platforms were removed or bypassed, and the remaining functionality was reorganised around a guided setup process designed for the xArm7 use case.

### 3.X.3 Rationale for a GUI-Centred Workflow

The original GELLO workflow required the user to interact with the system primarily through command-line operations: installing dependencies manually via pip, editing YAML configuration files, running calibration scripts with terminal arguments, and launching simulation or control processes from the command line. Each of these steps introduced a potential point of failure, particularly for users without significant prior experience with Python environments or robotics software.

To address this, the adapted software replaces the majority of command-line interactions with a graphical user interface built using PyQt6. The rationale for this design choice is grounded in the project's definition of intuitiveness: the system should reduce onboarding difficulty by presenting information visually, guiding the user through sequential steps, surfacing the current system state at each stage, and requiring user input only through clearly labelled interface elements rather than typed commands. This approach does not eliminate the underlying technical operations but abstracts them behind a guided interface, thereby reducing the likelihood of user error and lowering the time cost of each setup stage.

### 3.X.4 System Entry Point

The application is initiated via a batch script (`launch.bat`) that serves as the single entry point for the entire system. When executed, the script performs the following pre-launch checks:

1. **Python version detection.** The script searches for a compatible Python installation (version 3.10 or above) by testing multiple common Python executable names in order of preference. If no suitable version is found, it attempts an automatic installation via `winget` and, if that also fails, directs the user to install Python manually.
2. **Pip availability check.** The script verifies that `pip` is available and, if absent, bootstraps it using `ensurepip`.
3. **Dependency installation.** Required Python packages listed in `requirements.txt` are installed automatically. The local project package is also installed in editable mode if not already present.
4. **Application launch.** Once the environment is verified, the script launches the main GUI application.

This approach ensures that the user is not required to configure a Python environment manually, create a virtual environment, or run installation commands individually. The entire pre-launch sequence is handled by a single script execution, which directly supports the project goal of reducing time cost during initial setup.

### 3.X.5 The Launcher Interface

Upon successful execution of the batch script, the application opens a central launcher window that acts as a navigation hub. The launcher is structured as a two-panel layout. The left panel displays real-time system status information, including:

- whether the U2D2 servo controller is detected on a serial port,
- whether the required configuration file exists,
- whether the MuJoCo simulation model is present,
- the number and identity of servos detected on the communication bus.

This status panel provides the user with immediate visibility into the system's readiness without requiring any diagnostic commands. The right panel presents navigation tiles for the three setup stages and the launch function. A contextual guidance banner also appears when the system detects an issue that the user should address before proceeding, such as missing servos or an absent configuration file.

### 3.X.6 Three-Stage Setup Process

The setup process is divided into three sequential stages, each implemented as a separate GUI page accessible from the launcher.

#### 3.X.6.1 Package Installation

The Package Installation page lists all required Python packages alongside their current installation status. On opening, the page automatically scans for each dependency by checking whether the corresponding Python module can be imported. Packages that are already installed are marked accordingly, while missing packages are flagged. The user can then install all missing dependencies with a single button click, which triggers a background thread that invokes `pip install` for each missing package and reports progress in real time.

This stage is necessary because the system depends on several external libraries — including PyQt6, MuJoCo, the Dynamixel SDK, and the xArm Python SDK — and manual installation of these packages is both time-consuming and error-prone for users unfamiliar with Python package management. Automating this step removes a significant source of onboarding friction.

#### 3.X.6.2 Servo ID Registration

The Servo ID Registration page provides a guided interface for assigning a unique ID to each Dynamixel servo motor in the teleoperation device. Since all servos ship from the manufacturer with an identical default ID, each must be individually connected and re-addressed before the full chain can be assembled and operated. The page walks the user through this process one servo at a time, detecting the currently connected servo, displaying its ID, and allowing the user to assign the correct ID for its intended joint position.

This step is necessary because incorrect or duplicate servo IDs would prevent the system from distinguishing between joints, leading to communication failures during calibration and operation.

#### 3.X.6.3 Calibration Wizard

The Calibration Wizard guides the user through a per-joint calibration procedure that determines the offset and rotational direction for each of the seven arm joints and the gripper. The wizard is structured as a ten-step sequential process:

1. For each of the seven joints, the user is shown a live MuJoCo simulation displaying the target zero-position pose. The user physically positions the corresponding servo on the teleoperation device to match this pose and saves the offset. The wizard then enters a verification sub-step in which the user moves the joint to confirm that the simulation follows in the correct direction; if the direction is inverted, a "Flip Direction" button allows immediate correction.
2. The gripper is calibrated in two steps: the user records the fully open position and the fully closed position, establishing the range of motion.
3. A final full-test step allows the user to move all joints simultaneously and observe the simulation response in real time before saving the complete configuration.

Calibration is essential because manufacturing tolerances, assembly variations, and servo mounting orientation mean that raw servo readings do not correspond directly to the joint angles expected by the simulation model. Without calibration, the simulated arm would not accurately mirror the physical teleoperation device.

The calibration data — joint offsets, direction signs, and gripper range — is written directly to a YAML configuration file used by the simulation, eliminating any need for the user to manually edit configuration files.

### 3.X.7 Reduction of Onboarding Friction

Collectively, the three-stage setup process addresses the main sources of onboarding friction identified in the original workflow:

| Source of friction | Original workflow | Adapted workflow |
|---|---|---|
| Dependency installation | Manual pip commands | Automated detection and one-click installation |
| Servo addressing | Command-line SDK utilities | Guided per-servo wizard |
| Calibration | Script execution with manual config editing | Step-by-step wizard with live simulation preview |
| System status visibility | Terminal output and file inspection | Persistent status panel in launcher |
| Configuration file editing | Direct YAML editing | Automated file writes from GUI inputs |

Each stage reduces the number of manual steps, the amount of technical knowledge assumed, and the time required before the system is operational. This directly supports both the low time cost and intuitiveness objectives of the project.

### 3.X.8 Simulation and Real-Arm Modes

The launcher provides two operational pathways: simulation mode and real xArm7 mode. In simulation mode, the system reads servo data from the teleoperation device and drives a MuJoCo physics simulation of the xArm7 arm in real time at approximately 30 Hz. The simulation renders each frame and displays it in an interactive viewport within the application, alongside a performance dashboard showing live joint angles, loop frequency, and simulation time.

The real-arm mode pathway is present in the interface as a planned feature but was not completed within the scope of this project. The project requirement specified successful operation up to and including simulation-based teleoperation, and this milestone was achieved. Extension to real-arm control would require additional network communication with the physical xArm7 via its Python SDK, along with appropriate safety protocols, but the software architecture does not preclude this addition.

### 3.X.9 Contribution to Project Objectives

The software methodology contributes to the three defining criteria of the project as follows:

**Low monetary cost.** The software is built entirely from open-source libraries and frameworks (PyQt6, MuJoCo, Dynamixel SDK, xArm Python SDK). No proprietary software licences are required. The choice of Python as the development language ensures broad compatibility with existing hardware and avoids platform-specific toolchain costs.

**Low time cost.** The batch script entry point, automated dependency management, guided servo registration, and step-by-step calibration wizard collectively reduce the time required to bring the system from a fresh repository clone to operational status. Tasks that previously required multiple separate terminal sessions, manual file edits, and familiarity with the underlying libraries are consolidated into a single guided workflow.

**Intuitiveness.** Intuitiveness is operationally defined in this project as reduced dependence on command-line usage, fewer manual configuration steps, improved visibility of system state, guided sequential workflows, and clearer mapping between user actions and system responses. The GUI-centred design addresses each of these criteria: the launcher surfaces system readiness at a glance, each setup stage presents exactly the information and controls relevant to the current task, and the calibration wizard provides immediate visual feedback through the live simulation preview, allowing the user to verify correctness at every step without interpreting raw numerical output.
