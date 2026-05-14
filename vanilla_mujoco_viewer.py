#!/usr/bin/env python3
"""
Launch the stock MuJoCo passive viewer for the xArm7.

This script mirrors the implementation already used in the app:
1. Load the xArm7 + gripper MJCF scene
2. Read raw Dynamixel encoder values from the master arm
3. Apply the saved calibration offsets and signs from configs/xarm_sim_test.yaml
4. Write calibrated joint values into data.qpos
5. Call mj_forward() and viewer.sync() in a loop

Use this when you want a presentation-friendly example of the MuJoCo
implementation without the embedded Qt renderer.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Optional

import numpy as np
from omegaconf import OmegaConf

import mujoco
import mujoco.viewer

from gello.robots.sim_robot import build_scene
from gello.utils.servo_io import NUM_JOINTS, ServoIO, detect_port

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = BASE_DIR / "configs" / "xarm_sim_test.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch the stock MuJoCo passive viewer for the xArm7."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Path to the simulation config YAML.",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Override the Dynamixel port. Defaults to config port, then auto-detect.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=30.0,
        help="Viewer update rate in Hz.",
    )
    parser.add_argument(
        "--no-servo",
        action="store_true",
        help="Launch the viewer without reading the master arm.",
    )
    return parser.parse_args()


def load_config(config_path: Path) -> tuple[dict, str, Optional[str]]:
    cfg = OmegaConf.to_container(OmegaConf.load(config_path), resolve=True)
    robot_cfg = cfg["robot"]
    agent_cfg = cfg["agent"]
    xml_path = BASE_DIR / robot_cfg["xml_path"]
    gripper_xml_path = robot_cfg.get("gripper_xml_path")
    if gripper_xml_path:
        gripper_xml_path = str(BASE_DIR / gripper_xml_path)
    return agent_cfg, str(xml_path), gripper_xml_path


def build_model(xml_path: str, gripper_xml_path: Optional[str]) -> tuple[mujoco.MjModel, mujoco.MjData]:
    arena = build_scene(xml_path, gripper_xml_path)
    xml_string = arena.to_xml_string()
    assets = {}
    for asset in arena.asset.all_children():
        if asset.tag == "mesh":
            mesh_file = asset.file
            assets[mesh_file.get_vfs_filename()] = mesh_file.contents
    model = mujoco.MjModel.from_xml_string(xml_string, assets)
    data = mujoco.MjData(model)
    return model, data


def calibrated_qpos(raw_readings: list[float], dynamixel_cfg: dict, model_nq: int) -> np.ndarray:
    qpos = np.zeros(model_nq, dtype=np.float64)

    offsets = np.asarray(dynamixel_cfg["joint_offsets"], dtype=np.float64)
    signs = np.asarray(dynamixel_cfg["joint_signs"], dtype=np.float64)
    calibrated = (np.asarray(raw_readings[:NUM_JOINTS]) - offsets) * signs

    nq_arm = min(NUM_JOINTS, model_nq)
    qpos[:nq_arm] = calibrated[:nq_arm]

    gripper_cfg = dynamixel_cfg.get("gripper_config")
    if gripper_cfg and model_nq > NUM_JOINTS and len(raw_readings) > NUM_JOINTS:
        _, open_deg, close_deg = gripper_cfg
        open_rad = np.deg2rad(open_deg)
        close_rad = np.deg2rad(close_deg)
        raw_gripper = raw_readings[NUM_JOINTS]

        if abs(close_rad - open_rad) > 1e-6:
            normalized = (raw_gripper - open_rad) / (close_rad - open_rad)
            normalized = float(np.clip(normalized, 0.0, 1.0))
        else:
            normalized = 0.0

        # Match the calibration preview path, which drives the gripper qpos directly.
        qpos[NUM_JOINTS] = normalized * 0.8

    return qpos


def main() -> int:
    args = parse_args()
    agent_cfg, xml_path, gripper_xml_path = load_config(args.config)
    model, data = build_model(xml_path, gripper_xml_path)

    port = args.port or agent_cfg.get("port") or detect_port()
    use_servo = not args.no_servo and bool(port)
    servo: Optional[ServoIO] = None

    if use_servo:
        servo = ServoIO(port, NUM_JOINTS + 1)
        print(f"Reading master arm from {port}")
    else:
        print("Launching viewer without master-arm input")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.azimuth = 150.0
            viewer.cam.elevation = -20.0
            viewer.cam.distance = 1.8
            viewer.cam.lookat[:] = [0.0, 0.0, 0.0]

            while viewer.is_running():
                if servo is not None:
                    raw_readings = servo.read_raw()
                    qpos = calibrated_qpos(
                        raw_readings,
                        agent_cfg["dynamixel_config"],
                        model.nq,
                    )
                    data.qpos[: len(qpos)] = qpos

                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(max(0.0, 1.0 / args.hz))
    finally:
        if servo is not None:
            servo.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
