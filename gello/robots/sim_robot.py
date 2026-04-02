"""MuJoCo scene helpers for the xArm7 simulation."""
from typing import Optional

import numpy as np
from dm_control import mjcf


def attach_hand_to_arm(
    arm_mjcf: mjcf.RootElement,
    hand_mjcf: mjcf.RootElement,
) -> None:
    """Attach a hand MJCF to an arm MJCF at its 'attachment_site'."""
    physics = mjcf.Physics.from_mjcf_model(hand_mjcf)
    attachment_site = arm_mjcf.find("site", "attachment_site")
    if attachment_site is None:
        raise ValueError("No attachment site found in the arm model.")

    arm_key = arm_mjcf.find("key", "home")
    if arm_key is not None:
        hand_key = hand_mjcf.find("key", "home")
        if hand_key is None:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
            arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
        else:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, hand_key.ctrl])
            arm_key.qpos = np.concatenate([arm_key.qpos, hand_key.qpos])

    attachment_site.attach(hand_mjcf)


def build_scene(robot_xml_path: str,
                gripper_xml_path: Optional[str] = None) -> mjcf.RootElement:
    """Load arm (and optional gripper) MJCF and return a combined arena."""
    arena = mjcf.RootElement()
    arm = mjcf.from_path(robot_xml_path)
    if gripper_xml_path is not None:
        gripper = mjcf.from_path(gripper_xml_path)
        attach_hand_to_arm(arm, gripper)
    arena.worldbody.attach(arm)
    return arena
