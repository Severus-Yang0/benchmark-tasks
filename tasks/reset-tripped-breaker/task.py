"""On the station table, straight ahead between the arms: a home's breaker panel (a load center, 226 x 320 x 100 mm)
lying face up, its cover on, its door off. Eight single-pole breakers stand in two columns of four, their handles
sticking up through the cover; each column's handles point towards the panel's middle when ON. One breaker in the near
column has tripped: its handle rests midway and a red flag shows in the window beside it. One other is switched OFF.
To the panel's right, a floor box with a duplex receptacle face up; a small space heater beyond it, its cord's
right-angle plug in the receptacle's near outlet. The heater tripped the breaker.

Scene, scoring and scripted demo (see README.md).
Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import UsdBody, on_reset, on_step  # noqa: E402

import breaker  # noqa: E402
import cablekit  # noqa: E402,F401
import plughold  # noqa: E402
import touch  # noqa: E402
from cablekit import Plug, cable  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("The space heater has tripped its breaker in this panel. Unplug the heater and reset that breaker so that "
               "its circuit has power again. Leave the other breakers as they are.")

MM = 0.001
TABLE = 0.7475

# -- the panel (assets/panel.usda: origin at the box's bottom centre, x across, y along, z up; make_panel.py) ------
PANEL_POS = np.array([0.300, 0.040, TABLE])  # the breaker rows' middle straight ahead, at world y 0
COL_X = {"n": -38 * MM, "f": 38 * MM}
ROW_Y0 = -40 * MM
PITCH = 19.05 * MM
PIVOT_Z = 88 * MM
COVER_TOP = 100 * MM
HANDLE_TOP = 110 * MM  # midway
CAP = (7 * MM, 10 * MM)

PANEL = UsdBody("panel", usd="assets/panel.usda", pos=tuple(PANEL_POS), size=(0.226, 0.320, 0.100), mass=None,
                articulation=True, material=False, solver_iters=32, joint_damping=breaker.DAMPING,
                joint_pos=tuple((breaker.joint(h), breaker.start_angle(h)) for h in breaker.HANDLES)
                + (("flag_joint", 0.0),))


def handle_xy(h: str) -> np.ndarray:
    """A handle's pivot line, world x and y."""
    return PANEL_POS[:2] + np.array([COL_X[h[0]], ROW_Y0 + (int(h[1]) - 1.5) * PITCH])


# -- the floor box and heater (assets/floor.usda, origin at the floor box's bottom centre; make_panel.py) ----------
FLOOR_POS = np.array([0.300, -0.220, TABLE])
FLOOR = UsdBody("outlet_box", usd="assets/floor.usda", pos=tuple(FLOOR_POS), size=(0.16, 0.37, 0.22), mass=None,
                material=False)
FACE_Z = 25 * MM  # the receptacle's face above the table
OUTLETS = [FLOOR_POS[:2] + np.array([dx, 0.0]) for dx in (-22 * MM, 22 * MM)]  # the duplex's two outlets, one circuit
OUTLET = OUTLETS[0]  # the heater is plugged into the near one
HEATER_FACE_Y = FLOOR_POS[1] - 0.250 + 0.075  # the heater's side towards the floor box

# The heater's cord: 7 mm round, from the heater's side (anchored there) along +y to its right-angle NEMA 5-15P plug
# (38 mm along the cord, 32 mm across, 14 mm thick), lying on the receptacle's face with its blades in the slots. In
# the plug's frame (x along the cord, z up): two 6.35 x 1.5 mm blades 16 mm long, 12.7 mm apart, 2 mm past the plug's
# middle; a 4.8 mm ground pin 18.8 mm long, 9.5 mm before it.
PLUG = (38 * MM, 32 * MM, 14 * MM)
PLUG_AT = np.array([OUTLET[0], OUTLET[1] - 2 * MM, TABLE + FACE_Z + PLUG[2] / 2])  # seated
BLADE_Z = -PLUG[2] / 2 - 8 * MM
CORD = cable("heater_cord", start=(PLUG_AT[0], HEATER_FACE_Y, PLUG_AT[2]), yaw=math.pi / 2,
             length=float(PLUG_AT[1] - PLUG[0] / 2 - HEATER_FACE_Y), radius=3.5 * MM, color=(0.10, 0.10, 0.11),
             cable_mass=0.03, bend_stiffness=0.008, anchor_start=True, friction=1.2,  # PVC/rubber
             end_plug=Plug(size=PLUG, mass=0.05, color=(0.09, 0.09, 0.10),
                           blades=tuple(((2 * MM, s * 6.35 * MM, BLADE_Z), (6.35 * MM, 1.5 * MM, 16 * MM))
                                        for s in (-1, 1))
                           + (((-9.5 * MM, 0.0, -PLUG[2] / 2 - 9.4 * MM), (4.8 * MM, 4.8 * MM, 18.8 * MM)),)))
OBJECTS = [PANEL, FLOOR, CORD]
SEATS = [o + np.array([0.0, PLUG_AT[1] - OUTLET[1]]) for o in OUTLETS]  # the plug's centre seated in either outlet
TIPS = [(2 * MM, s * 6.35 * MM, BLADE_Z - 8 * MM) for s in (-1, 1)] + [(-9.5 * MM, 0.0, -PLUG[2] / 2 - 18.8 * MM)]


# -- the springs, the plug's hold, contacts and the overload rule ------------------------------------------------
_STATE: dict = {"springs": None, "hold": None, "free": True, "plugged": True, "overload": False}


@on_reset
def _reset(environment) -> None:
    panel = environment.unwrapped.scene["panel"]
    if _STATE["springs"] is None or _STATE["springs"].panel is not panel:
        _STATE["springs"] = breaker.Breakers(panel)
    else:
        _STATE["springs"].reset()
    cord = environment.unwrapped.scene["heater_cord"]
    if _STATE["hold"] is None or _STATE["hold"].cord is not cord:
        _STATE["hold"] = plughold.PlugHold(environment, "heater_cord", SEATS, TABLE + FACE_Z, TIPS)
    touch.reset()
    _STATE.update(free=True, plugged=True, overload=False)


@on_step
def _watch(environment) -> None:
    """Whether the tripped breaker's handle is released (no robot link touches it: PhysX contacts, touch.py), whether
    the heater is plugged in (into either outlet: plughold.py), and the overload rule: ON while the heater is plugged
    in, the breaker would trip again."""
    name = f"panel/handle_{breaker.TRIPPED}"
    _STATE["free"] = not touch.touching(environment, [name])[name]
    _STATE["plugged"] = _STATE["hold"].plugged()
    if _STATE["plugged"] and states()[breaker.TRIPPED] == breaker.ON:
        _STATE["overload"] = True


# -- scoring ------------------------------------------------------------------------------------------------------


def states() -> dict:
    s = _STATE["springs"]
    return dict(s.state) if s else {h: breaker.start_state(h) for h in breaker.HANDLES}


def score(objects) -> float:
    """0 for the trial if any other breaker was ever switched, or the tripped one was switched ON while the heater was
    plugged in (it would trip again). Otherwise 0.25 for each of: the heater unplugged, the breaker reset (pushed all
    the way to OFF). 0.75: and the breaker ON. 1.0: ON and its handle released."""
    s = _STATE["springs"]
    if (s is not None and s.changed) or _STATE["overload"]:
        return 0.0
    st = states()[breaker.TRIPPED]
    out = not _STATE["plugged"]
    if st == breaker.ON:  # only reachable with the heater out (else the overload rule fired)
        return 1.0 if _STATE["free"] else 0.75
    return 0.25 * out + 0.25 * (st == breaker.OFF)


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo --------------------------------------------------------------------------------------------
PAD_HALF = 13.2 * MM  # closed pads, fingers closing along world y: 6.3 mm along y, 26.4 mm along x
PAD_Z = COVER_TOP - 1 * MM  # the pads' tips commanded this high (the arm sags with them; they ride on the cover)
RESET_PUSH = 30 * MM  # past the handle's face at its OFF stop: the arm is soft, about 0.3 N per mm
ON_PUSH = 22 * MM
CLEAR = 6 * MM  # the pads this far from a handle's cap while coming down beside it
PLUG_SQUEEZE = 3 * MM  # each finger commanded this far into the plug's side: enough grip to pull it against the
# receptacle's 15 N hold (4 mm jammed the blades against the slot walls and the light cord's solver blew up)
PLUG_PAD_Z = FACE_Z + 3 * MM  # the pads' tips this high while gripping the plug (over the receptacle's face)
PLUG_DROP = (0.215, -0.300)  # where the plug is laid down, clear of the floor box and the heater


def unplug_steps(hover):
    """Right arm: grip the plug across its sides (fingers closing along world x), pull it straight up out of the
    receptacle, lay it on the table aside and let go."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    YAW = -math.pi / 2
    C = -(PLUG[1] / 2 - PLUG_SQUEEZE)
    px, py = PLUG_AT[:2]
    z = TABLE + PLUG_PAD_Z + TIP
    dx, dy = PLUG_DROP
    return [
        W(2.0, (px, py, hover), O, "over the heater's plug", YAW),
        W(1.5, (px, py, z + 0.03), O, "down to it, open", YAW),
        W(1.0, (px, py, z), O, "round its sides", YAW),
        W(1.0, (px, py, z), C, "grip", YAW),
        W(2.0, (px, py, z + 0.07), C, "pull it straight up out of the outlet (the arm lags: about 0.5 N per mm)", YAW),
        W(1.5, (dx, dy, z + 0.07), C, "aside", YAW),
        W(1.0, (dx, dy, TABLE + 0.012 + TIP), C, "down to the table", YAW),
        W(0.8, (dx, dy, TABLE + 0.012 + TIP), O, "let go", YAW),
        W(1.0, (dx, dy, hover), O, "up", YAW),
    ]


def reset_steps(hover):
    """Right arm, closed fingertips: down beside the tripped handle on its ON side (towards the panel's middle), push it
    all the way to OFF until it resets, lift over it, come down on its OFF side and push it over to ON, then back off
    and leave."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    h = breaker.TRIPPED
    hx, hy = handle_xy(h)
    on = breaker.COLUMNS[h[0]]  # +1: ON towards world +x
    z0 = TABLE + PAD_Z + TIP
    above = TABLE + HANDLE_TOP + 0.012 + TIP
    lean = math.sin(breaker.TRAVEL)
    # The handle's face (half the cap's 7 mm) at the pads' height (12 mm over its pivot), at either stop.
    face_at_stop = (PAD_Z + 3 * MM - PIVOT_Z) * lean - CAP[0] / 2  # past its pivot line, the pushing side
    on_side = hx + on * (CAP[0] / 2 + CLEAR + PAD_HALF)  # midway: the pads on its ON side
    reset_end = hx - on * (face_at_stop + RESET_PUSH - PAD_HALF)
    off_side = hx - on * ((HANDLE_TOP - PIVOT_Z) * lean + CAP[0] / 2 + CLEAR + PAD_HALF)  # at OFF, on its OFF side
    on_end = hx + on * (face_at_stop + ON_PUSH - PAD_HALF)
    return [
        W(2.0, (on_side, hy, hover), 0.0, "over the tripped breaker, on its ON side", 0.0),
        W(1.5, (on_side, hy, above), 0.0, "down, clear of the handle", 0.0),
        W(1.0, (on_side, hy, z0), 0.0, "down to the cover", 0.0),
        W(2.5, (reset_end, hy, z0), 0.0, "push it all the way to OFF", 0.0),
        W(1.0, (reset_end, hy, z0), 0.0, "hold: it resets", 0.0),
        W(1.0, (reset_end, hy, above), 0.0, "up over it", 0.0),
        W(1.5, (off_side, hy, above), 0.0, "over to its OFF side", 0.0),
        W(1.0, (off_side, hy, z0), 0.0, "down to the cover", 0.0),
        W(2.0, (on_end, hy, z0), 0.0, "push it over to ON", 0.0),
        W(1.0, (on_end, hy, z0), 0.0, "hold", 0.0),
        W(1.0, (off_side, hy, z0), 0.0, "back off", 0.0),
        W(1.0, (off_side, hy, hover), 0.0, "up", 0.0),
        W(1.5, (0.25, -0.25, hover), O, "away", 0.0),
    ]


def demo():
    """The right arm unplugs the heater, then resets the breaker and switches it ON. The left arm stays put."""
    from yam_arena.demo import Waypoint as W

    hover = TABLE + 0.15
    right = unplug_steps(hover) + reset_steps(hover)
    left = [W(sum(w.duration for w in right), (0.25, 0.25, hover), 0.0, "wait", 0.0)]
    return {"left": left, "right": right}
