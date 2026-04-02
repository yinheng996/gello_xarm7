"""
Shared MuJoCo offline renderer for xArm7.

Provides:
  - SimRenderer  — QThread that renders MuJoCo frames at ~30 fps,
                   supports qpos control and pulsing-dot joint highlighting.
"""

import math
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

NUM_JOINTS = 7
BASE_DIR = Path(__file__).resolve().parent.parent.parent
MODEL_PATH = BASE_DIR / "third_party" / "mujoco_menagerie" / "ufactory_xarm7" / "xarm7_nohand.xml"
GRIPPER_PATH = BASE_DIR / "third_party" / "mujoco_menagerie" / "ufactory_xarm7" / "hand.xml"


class SimRenderer(QThread):
    """Renders MuJoCo frames with configurable joint positions and pulsing dot overlay."""
    frame_ready = pyqtSignal(object)

    def __init__(self, w: int = 640, h: int = 480,
                 model_path: Optional[str] = None,
                 gripper_path: Optional[str] = None):
        super().__init__()
        self._rw, self._rh = w, h
        self._model_path = model_path or str(MODEL_PATH)
        self._gripper_path = gripper_path or str(GRIPPER_PATH)
        self._running = True
        self._qpos = np.zeros(20)
        self._lock = threading.Lock()
        self._active_joint = -1

    def set_qpos(self, idx: int, val: float):
        with self._lock:
            self._qpos[idx] = val

    def set_all_qpos(self, vals):
        with self._lock:
            for i, v in enumerate(vals):
                if i < len(self._qpos):
                    self._qpos[i] = v

    def set_active_joint(self, idx: int):
        """Set which joint gets a pulsing dot overlay (-1 = none)."""
        self._active_joint = idx

    def stop(self):
        self._running = False

    @staticmethod
    def _project_to_2d(point_3d, gl_cam, fovy_deg, width, height):
        cam_pos = np.array(gl_cam.pos)
        forward = np.array(gl_cam.forward)
        up = np.array(gl_cam.up)
        right = np.cross(forward, up)

        p = np.array(point_3d) - cam_pos
        x_cam = np.dot(p, right)
        y_cam = np.dot(p, up)
        z_cam = np.dot(p, forward)
        if z_cam <= 0:
            return None

        f = 1.0 / np.tan(np.deg2rad(fovy_deg) / 2.0)
        px = int(width / 2 + x_cam / z_cam * f * height / 2)
        py = int(height / 2 - y_cam / z_cam * f * height / 2)
        return (px, py)

    @staticmethod
    def _draw_pulsing_dot(frame, cx, cy, t):
        h, w, _ = frame.shape
        pulse = math.sin(t * 5.0) * 0.5 + 0.5

        r_inner = 5 + 4 * pulse
        r_outer = 14 + 10 * pulse
        r_max = int(r_outer) + 2

        y0, y1 = max(0, cy - r_max), min(h, cy + r_max + 1)
        x0, x1 = max(0, cx - r_max), min(w, cx + r_max + 1)
        if y1 <= y0 or x1 <= x0:
            return

        yy, xx = np.mgrid[y0:y1, x0:x1]
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2).astype(np.float32)

        color = np.array([0, 113, 227], dtype=np.float32)
        region = frame[y0:y1, x0:x1]

        outer_mask = (dist > r_inner) & (dist <= r_outer)
        if outer_mask.any():
            frac = (dist[outer_mask] - r_inner) / (r_outer - r_inner)
            alpha = ((0.35 + 0.2 * pulse) * (1.0 - frac))[:, np.newaxis]
            region[outer_mask] = (
                alpha * color + (1 - alpha) * region[outer_mask].astype(np.float32)
            ).astype(np.uint8)

        inner_mask = dist <= r_inner
        region[inner_mask] = color.astype(np.uint8)

        center_mask = dist <= r_inner * 0.35
        region[center_mask] = [255, 255, 255]

    def run(self):
        try:
            import mujoco
            from gello.robots.sim_robot import build_scene

            arena = build_scene(self._model_path, self._gripper_path)
            xml_string = arena.to_xml_string()
            assets = {}
            for asset in arena.asset.all_children():
                if asset.tag == "mesh":
                    f = asset.file
                    assets[f.get_vfs_filename()] = f.contents
            model = mujoco.MjModel.from_xml_string(xml_string, assets)
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, self._rw)
            model.vis.global_.offheight = max(model.vis.global_.offheight, self._rh)
            data = mujoco.MjData(model)
            renderer = mujoco.Renderer(model, height=self._rh, width=self._rw)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.azimuth = 150.0
            cam.elevation = -20.0
            cam.distance = 1.8
            cam.lookat[:] = [0.0, 0.0, 0.3]

            # Build name→id maps for joints and bodies.
            # dm_control's attach() namespaces names (e.g. "xarm7_nohand/joint1"),
            # so we match by the final segment after any "/" separator.
            jnt_name_map: dict = {}
            for j in range(model.njnt):
                full = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
                suffix = full.split("/")[-1]
                jnt_name_map[suffix] = j
                jnt_name_map[full] = j

            body_name_map: dict = {}
            for b in range(model.nbody):
                full = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
                suffix = full.split("/")[-1]
                body_name_map[suffix] = b
                body_name_map[full] = b

            arm_jnt_ids = [jnt_name_map.get(f"joint{i}", -1) for i in range(1, NUM_JOINTS + 1)]

            # Servo 1 (joint1) lives in the base housing — show dot there instead of
            # at joint1's pivot which sits at shoulder height.
            link_base_id = body_name_map.get("link_base", -1)

            gripper_jnt_id = next(
                (jnt_name_map[k] for k in ("gripper", "finger1", "left_driver_joint")
                 if k in jnt_name_map),
                arm_jnt_ids[-1]
            )

            fovy = model.vis.global_.fovy

            while self._running:
                with self._lock:
                    nq = min(len(self._qpos), model.nq)
                    data.qpos[:nq] = self._qpos[:nq]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, cam)
                frame = renderer.render().copy()

                aj = self._active_joint
                pos_3d = None
                if 0 <= aj < NUM_JOINTS:
                    jid = arm_jnt_ids[aj]
                    if jid >= 0:
                        if aj == 0 and link_base_id >= 0:
                            # Servo 1: base rotation motor lives in link_base
                            pos_3d = data.xpos[link_base_id]
                        elif aj == 2 and arm_jnt_ids[1] >= 0:
                            # Servo 3: midpoint between joint2 and joint3 pivots
                            pos_3d = (data.xanchor[arm_jnt_ids[1]] + data.xanchor[jid]) * 0.5
                        elif aj == 4 and arm_jnt_ids[3] >= 0:
                            # Servo 5: midpoint between joint4 and joint5 pivots
                            pos_3d = (data.xanchor[arm_jnt_ids[3]] + data.xanchor[jid]) * 0.5
                        else:
                            pos_3d = data.xanchor[jid]
                elif aj == NUM_JOINTS and gripper_jnt_id >= 0:
                    pos_3d = data.xanchor[gripper_jnt_id]

                if pos_3d is not None:
                    gl_cam = renderer._scene.camera[0]
                    pt = self._project_to_2d(pos_3d, gl_cam, fovy, self._rw, self._rh)
                    if pt is not None:
                        self._draw_pulsing_dot(frame, pt[0], pt[1], time.time())

                self.frame_ready.emit(frame)
                time.sleep(1.0 / 30)
            renderer.close()
        except Exception as e:
            print(f"SimRenderer error: {e}")
