"""The cooler's clip springs and lever, applied before every physics step (240 Hz).

The cooler (assets/cooler.usda) is a floating-base articulation: per clip leg a slide along z (``leg_*_slide_joint``,
0 = loose, negative = pushed down) sprung up, and a flex about x at the leg's top (``leg_*_flex_joint``, 0 = upright,
outwards positive on the +y leg and negative on the -y leg) sprung upright; and the lever (``lever_joint``, 0 =
unlocked, LEVER_SPAN = locked; it swings in the plane of the +y leg's face) with an over-centre spring: past straight
up (within LEVER_SNAP of locked) it is pulled locked and held stiffly, otherwise it is pulled back to unlocked. Its cam raises the +y leg's spring target by up to CAM_LIFT as it
goes down, clamping the cooler harder. Values as work/cooler/make_cooler.py authors them.
Isaac Lab writes every articulation's joint targets to PhysX before each physics substep (``write_data_to_sim``);
this wraps that call on the cooler, so it decides on the joint positions PhysX reported after the previous substep.
"""

from __future__ import annotations

import math

PER_DEG = 180.0 / math.pi  # N m per degree -> N m per radian
LEGS = ("neg", "pos")
SIGN = {"neg": -1.0, "pos": 1.0}
LEG_STIFFNESS = 600.0  # N/m: hooked, a leg pulls up about 3.6 N
LEG_DAMPING = 20.0
FLEX_STIFFNESS = 0.008 * PER_DEG
FLEX_DAMPING = 0.0005 * PER_DEG
FLEX_PRELOAD = math.radians(1.0)  # held this far inwards, against the upright stop
LEVER = "lever_joint"
LEVER_SPAN = math.radians(115.0)
LEVER_SNAP = math.radians(90.0)
LEVER_STIFFNESS = 0.002 * PER_DEG  # unlocked; ten times this locked
LEVER_DAMPING = 0.0002 * PER_DEG
CAM_LIFT = 0.004
HOOK_DROP = 0.0060625  # a leg's travel from loose to hooked (its window's bottom edge under the lug)
HOOK_TOL = (0.0006, 0.0005)  # hooked: travel within this below and above HOOK_DROP...
FLEX_TOL = math.radians(1.5)  # ...and hanging within this of upright (the lug in its window)


class ClipSpring:
    def __init__(self, cooler):
        import torch

        self.cooler = cooler
        self.dev = cooler.device
        names = [f"leg_{n}_slide_joint" for n in LEGS] + [f"leg_{n}_flex_joint" for n in LEGS] + [LEVER]
        self.ids = dict(zip(names, cooler.find_joints(names, preserve_order=True)[0]))
        self.lever = self.ids[LEVER]
        self.cam = self.ids["leg_pos_slide_joint"]
        stiff = [LEG_STIFFNESS] * 2 + [FLEX_STIFFNESS] * 2 + [LEVER_STIFFNESS]
        damp = [LEG_DAMPING] * 2 + [FLEX_DAMPING] * 2 + [LEVER_DAMPING]
        ids = [self.ids[n] for n in names]
        cooler.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([stiff], device=self.dev), joint_ids=ids)
        cooler.write_joint_damping_to_sim_index(damping=torch.tensor([damp], device=self.dev), joint_ids=ids)
        self.locked = False
        self.reset()
        write = cooler.write_data_to_sim

        def write_with_spring():
            self.update()
            write()

        cooler.write_data_to_sim = write_with_spring

    def q(self) -> dict:
        q = self.cooler.data.joint_pos
        q = getattr(q, "torch", q)[0]
        return {n: float(q[i]) for n, i in self.ids.items()}

    def _targets(self, lever: float, cam: float) -> None:
        import torch

        ids = [self.ids[f"leg_{n}_flex_joint"] for n in LEGS] + [self.lever, self.cam]
        t = [-SIGN[n] * FLEX_PRELOAD for n in LEGS] + [lever, cam]
        self.cooler.set_joint_position_target_index(target=torch.tensor([t], device=self.dev), joint_ids=ids)

    def _stiff_lever(self, locked: bool) -> None:
        import torch

        k = LEVER_STIFFNESS * (10.0 if locked else 1.0)
        self.cooler.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]], device=self.dev),
                                                       joint_ids=[self.lever])
        self.locked = locked

    def update(self) -> None:
        lever = self.q()[LEVER]
        locked = lever > LEVER_SPAN - LEVER_SNAP
        if locked != self.locked:
            self._stiff_lever(locked)
        self._targets(LEVER_SPAN if locked else 0.0, CAM_LIFT * min(max(lever / LEVER_SPAN, 0.0), 1.0))

    def reset(self) -> None:
        self._stiff_lever(False)
        self._targets(0.0, 0.0)

    def state(self) -> dict:
        """{'lever': travel from unlocked (rad), 'locked': bool, 'neg'/'pos': (travel down m, flex out rad, hooked)}."""
        q = self.q()
        out = {"lever": q[LEVER], "locked": q[LEVER] > LEVER_SPAN - LEVER_SNAP}
        for n in LEGS:
            drop = -q[f"leg_{n}_slide_joint"]
            flex = SIGN[n] * q[f"leg_{n}_flex_joint"]
            out[n] = (drop, flex, HOOK_DROP - HOOK_TOL[1] < drop < HOOK_DROP + HOOK_TOL[0] and flex < FLEX_TOL)
        return out
