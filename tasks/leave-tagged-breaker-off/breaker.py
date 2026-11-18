"""The breakers' handle springs and the trip flag, applied before every physics step (240 Hz).

The panel (assets/panel.usda, work/breaker/make_panel.py) is a fixed-base articulation: each breaker's handle turns on
``handle_{c}{k}_joint`` (0 = midway, +-TRAVEL at its stops; a positive angle tips its top towards the panel's +x) and
the tripped breaker's red flag slides on ``flag_joint`` (0 = showing, -FLAG_DROP = hidden).

Each breaker is ON, OFF or TRIPPED, as a Square D QO is (Schneider FAQ FA353467). In this task none starts tripped
(TRIPPED is None): the flag stays hidden.
- ON or OFF: an over-centre spring holds the handle against that stop. Pushed SNAP past midway it switches: the spring then
  takes it to the other stop.
- TRIPPED: a spring holds the handle midway, whichever way it is pushed (trip-free: it can't be switched on from
  here). Pushed to within RESET_TOL of the OFF stop, it resets: it is OFF, and the flag drops out of sight.
Isaac Lab writes every articulation's joint targets to PhysX before each physics substep (``write_data_to_sim``);
this wraps that call on the panel, so it decides on the joint positions PhysX reported after the previous substep.
"""

from __future__ import annotations

import math

TRAVEL = math.radians(25.0)
PRELOAD = math.radians(5.0)  # ON/OFF: the spring's target lies this far past the stop
RESET_TOL = math.radians(2.0)
SNAP = math.radians(3.0)  # ON/OFF switch once pushed this far past midway
# Handle forces, pushing the handle 14 mm above its pivot (just over the face): switching ON or OFF takes about 3 N
# (at midway, where the over-centre spring is strongest); resetting a tripped one about 4 N (at the OFF stop).
# Schneider publishes no QO handle forces. (The simulated arm pushes sideways with only about 0.3 N per mm it is held
# off its target; 5 N / 8 N stalled the demo.)
R_PUSH = 0.014
TOGGLE_N = 3.0
RESET_N = 4.0
K_TOGGLE = TOGGLE_N * R_PUSH / (TRAVEL + PRELOAD)  # N m / rad
K_TRIP = RESET_N * R_PUSH / TRAVEL
DAMPING = 0.003  # N m s / rad
FLAG_DROP = 0.002
FLAG_STIFFNESS = 500.0  # N/m: holds the 1 g flag within 0.02 mm of its target
FLAG_DAMPING = 2.0
COLUMNS = {"n": 1.0, "f": -1.0}  # the sign of a column's ON angle: ON towards the panel's middle
HANDLES = [f"{c}{k}" for c in COLUMNS for k in range(4)]
TRIPPED = None  # none tripped here (the panel's flag sits under n2's window, hidden)
STARTS_OFF = ("n0", "n1", "n2")  # the near column's first three; the rest start ON
TAGGED = "n1"  # the danger tag hangs on this one
ON, OFF, TRIP = "ON", "OFF", "TRIPPED"


def start_state(h: str) -> str:
    return TRIP if h == TRIPPED else OFF if h in STARTS_OFF else ON


def start_angle(h: str) -> float:
    """The joint angle each handle starts at (rad)."""
    s = start_state(h)
    return 0.0 if s == TRIP else COLUMNS[h[0]] * TRAVEL * (1.0 if s == ON else -1.0)


def joint(h: str) -> str:
    return f"handle_{h}_joint"


class Breakers:
    def __init__(self, panel):
        import torch

        self.panel = panel
        self.dev = panel.device
        names = [joint(h) for h in HANDLES] + ["flag_joint"]
        self.ids = dict(zip(names, panel.find_joints(names, preserve_order=True)[0]))
        ids = [self.ids[joint(h)] for h in HANDLES]
        panel.write_joint_damping_to_sim_index(damping=torch.tensor([[DAMPING] * len(ids) + [FLAG_DAMPING]],
                                                                    device=self.dev),
                                               joint_ids=ids + [self.ids["flag_joint"]])
        panel.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[FLAG_STIFFNESS]], device=self.dev),
                                                 joint_ids=[self.ids["flag_joint"]])
        self.state: dict = {}
        self.changed: set = set()  # breakers that ever left their starting state (other than a tripped one)
        self.ever_on: set = set()  # breakers that were ON at any moment
        self.reset()
        write = panel.write_data_to_sim

        def write_with_springs():
            self.update()
            write()

        panel.write_data_to_sim = write_with_springs

    def q(self) -> dict:
        q = self.panel.data.joint_pos
        q = getattr(q, "torch", q)[0]
        return {h: float(q[self.ids[joint(h)]]) for h in HANDLES}

    def _apply(self) -> None:
        import torch

        ids, target, stiff = [], [], []
        for h in HANDLES:
            s = self.state[h]
            sign = COLUMNS[h[0]]
            ids.append(self.ids[joint(h)])
            if s == TRIP:
                target.append(0.0)
                stiff.append(K_TRIP)
            else:
                target.append(sign * (TRAVEL + PRELOAD) * (1.0 if s == ON else -1.0))
                stiff.append(K_TOGGLE)
        self.panel.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([stiff], device=self.dev), joint_ids=ids)
        flag = 0.0 if TRIPPED and self.state[TRIPPED] == TRIP else -FLAG_DROP
        self.panel.set_joint_position_target_index(target=torch.tensor([target + [flag]], device=self.dev),
                                                   joint_ids=ids + [self.ids["flag_joint"]])

    def update(self) -> None:
        q = self.q()
        moved = False
        for h in HANDLES:
            a = COLUMNS[h[0]] * q[h]  # towards ON positive
            s = self.state[h]
            new = s
            if s == ON and a < -SNAP:
                new = OFF
            elif s == OFF and a > SNAP:
                new = ON
            elif s == TRIP and a < -TRAVEL + RESET_TOL:
                new = OFF
            if new != s:
                self.state[h] = new
                moved = True
                if h != TRIPPED:
                    self.changed.add(h)
                if new == ON:
                    self.ever_on.add(h)
        if moved:
            self._apply()

    def reset(self) -> None:
        self.state = {h: start_state(h) for h in HANDLES}
        self.changed = set()
        self.ever_on = {h for h, s in self.state.items() if s == ON}
        self._apply()

    def angles_deg(self) -> dict:
        """Each handle's angle towards ON from midway, in degrees."""
        q = self.q()
        return {h: math.degrees(COLUMNS[h[0]] * q[h]) for h in HANDLES}
