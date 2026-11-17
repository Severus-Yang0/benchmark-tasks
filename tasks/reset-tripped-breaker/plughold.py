"""The receptacle's hold on the heater's plug, applied before every physics step (240 Hz), and whether it is plugged in.

A NEMA 5-15R receptacle grips the plug's blades: UL 498 requires it to hold a test plug against a 3 lbf (13 N) pull and to
let it go at no more than 15 lbf (67 N). The simulated slots are plain holes with clearance, so this adds the grip: while
any blade or the ground pin is still in a slot, a downward force on the plug of HOLD_N, falling to 0 over the last
HOLD_RAMP of the blades' travel out. Pulling the plug out takes HOLD_N plus its weight; pushed in or left alone, the
force only presses it onto the face.

Plugged in: the plug's centre within SEAT_XY of either outlet's seat sideways, and the lowest blade or pin tip below the
receptacle's face (computed from the plug's pose, so a tilted plug counts while any tip is still in).
Isaac Lab writes every articulation's data, including its external wrenches, to PhysX before each physics substep
(``write_data_to_sim``); this wraps that call on the cord.
"""

from __future__ import annotations

import numpy as np

HOLD_N = 15.0
HOLD_RAMP = 0.004
SEAT_XY = 0.005


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64)


def _rot_xyzw(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class PlugHold:
    def __init__(self, env, cord_name: str, seats, face_z: float, tips):
        """seats: each outlet's seated plug centre (world x, y); face_z: the receptacle's face (world z); tips: the
        blade and pin tips in the plug's frame, (n, 3)."""
        self.env = env
        self.cord = env.unwrapped.scene[cord_name]
        self.plug = list(self.cord.body_names).index("end_plug")
        self.seats = [np.asarray(s, float) for s in seats]
        self.face_z = face_z
        self.tips = np.asarray(tips, float)
        write = self.cord.write_data_to_sim

        def write_with_hold():
            self.update()
            write()

        self.cord.write_data_to_sim = write_with_hold

    def pose(self) -> tuple[np.ndarray, np.ndarray]:
        """The plug's centre (station frame) and rotation."""
        origin = _np(self.env.unwrapped.scene.env_origins[0])
        pos = _np(self.cord.data.body_pos_w)[0, self.plug] - origin
        quat = _np(self.cord.data.body_quat_w)[0, self.plug]
        return pos, _rot_xyzw(quat)

    def depth(self) -> float:
        """How far the deepest tip is below the face, if the plug sits over an outlet; else -1."""
        pos, rot = self.pose()
        if min(float(np.hypot(*(pos[:2] - s))) for s in self.seats) > SEAT_XY:
            return -1.0
        tips = pos + self.tips @ rot.T
        return float(self.face_z - tips[:, 2].min())

    def plugged(self) -> bool:
        return self.depth() > 0.0

    def update(self) -> None:
        import torch

        d = self.depth()
        f = HOLD_N * min(max(d / HOLD_RAMP, 0.0), 1.0) if d > 0.0 else 0.0
        forces = torch.tensor([[[0.0, 0.0, -f]]], device=self.cord.device)
        self.cord.permanent_wrench_composer.set_forces_and_torques_index(forces=forces, body_ids=[self.plug],
                                                                       is_global=True)
