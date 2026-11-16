"""The x16 slot's latch spring, applied before every physics step (240 Hz). (This board has no Q-Release button.)

The latch (board joint pcie0_pos_joint, 0 rad = shut, +30 degrees = open) has an over-centre spring: within SNAP of
shut it is pulled shut and held stiffly; otherwise it is pulled open, firmly enough that the card's weight doesn't
tread its foot shut. Here it starts shut, holding a seated card.
Values as work/board/pcie_slot.py authors them. Isaac Lab writes every articulation's joint targets to PhysX before
each physics substep (``write_data_to_sim``); the spring wraps that call on the board, so it decides on the joint
angles PhysX reported after the previous substep.
"""

from __future__ import annotations

import math

LATCH = "pcie0_pos_joint"
OPEN = math.radians(30.0)
SNAP = math.radians(26.0)
PER_DEG = 180.0 / math.pi  # N m per degree -> N m per radian
STIFF_OPEN = 0.0018 * PER_DEG
STIFF_SHUT = 0.005 * PER_DEG
DAMPING = 1e-6 * PER_DEG


class LatchSpring:
    def __init__(self, board):
        import torch

        self.board = board
        self.latch = board.find_joints([LATCH], preserve_order=True)[0][0]
        self.shut = True
        dev = board.device
        board.write_joint_damping_to_sim_index(damping=torch.tensor([[DAMPING]], device=dev), joint_ids=[self.latch])
        self._write(True)
        write = board.write_data_to_sim

        def write_with_spring():
            self.update()
            write()

        board.write_data_to_sim = write_with_spring

    def angle(self) -> float:
        """The latch's angle from shut (rad)."""
        q = self.board.data.joint_pos
        return float(getattr(q, "torch", q)[0, self.latch])

    def update(self) -> None:
        now = abs(self.angle()) < SNAP
        if now != self.shut:
            self._write(now)

    def reset(self) -> None:
        self._write(True)

    def _write(self, shut: bool) -> None:
        import torch

        dev = self.board.device
        self.board.set_joint_position_target_index(target=torch.tensor([[0.0 if shut else OPEN]], device=dev),
                                                   joint_ids=[self.latch])
        self.board.write_joint_stiffness_to_sim_index(
            stiffness=torch.tensor([[STIFF_SHUT if shut else STIFF_OPEN]], device=dev), joint_ids=[self.latch])
        self.shut = shut
