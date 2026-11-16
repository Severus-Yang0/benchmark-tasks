"""On the station table, between the arms: an ATX motherboard (305 x 244 mm) lying flat, its CPU socket straight
ahead between the arms, a processor already in it. The socket's retention frame has a lug at each end (its left and
right ends, each towards one arm). On the left, a boxed-style CPU cooler (88 x 84 x 65 mm, 0.36 kg: aluminium fins, two
copper heat pipes, a fan under a black cover) stands on a foam block, its steel clip loose: a leg with a window hangs at
each end, and a black cam lever stands up at its right end.

Scene, scoring and scripted demo (see README.md).
Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402

import clip  # noqa: E402
import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Install the CPU cooler on the processor. Lower it straight onto the processor and, once its thermal "
               "paste touches, don't lift it off again or slide it more than 10 mm. Then press each end of its clip "
               "down until it hooks over the lug, and push the lever over and down until it locks.")

MM = 0.001
TABLE = 0.7475
GREY = (0.45, 0.47, 0.50)
FOAM = (0.20, 0.21, 0.22)

# -- the board (assets/board.usda: origin at the board's centre, bottom face) -----------------------------------
BOARD_YAW = 0.0  # board +y along world +y: the clip runs left-right, each tab towards one arm
SOCKET = (8 * MM, 64 * MM)  # the socket's centre, board frame
CPU_TOP = 9.7 * MM  # the processor's top above the board's bottom face
BOARD_POS = np.array([0.290 - SOCKET[0], -SOCKET[1], TABLE])  # the socket at world (0.29, 0)


def yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def board_to_world(p) -> np.ndarray:
    c, s = math.cos(BOARD_YAW), math.sin(BOARD_YAW)
    x, y, z = p
    return BOARD_POS + np.array([c * x - s * y, s * x + c * y, z])


# -- the cooler (assets/cooler.usda, origin at the base's bottom centre, z up, its clip across its y, the lever on its
# +y side; work/cooler/make_cooler.py). On the CPU it is turned 180 degrees from the board, its lever towards the right
# arm (world -y).
COOLER_YAW = BOARD_YAW + math.pi
SEATED_POS = board_to_world((*SOCKET, CPU_TOP))
COOLER_MASS = 0.36
COVER = (76 * MM, 76 * MM, (40 * MM, 62 * MM))  # fan cover: x, y, z range
CLIP_Y = 50.5 * MM  # each leg's plane, from the cooler's centre
CLIP_T = 1.2 * MM
LEG_TOP = 30 * MM  # the legs' press tabs' top
TAB = 9 * MM  # the tabs reach this far out from the legs
LEVER_PIVOT = np.array([0.0, CLIP_Y + 21 * MM, 38 * MM])
LEVER_LEN = 32 * MM
LEVER_T = 4 * MM
LEVER_UP = math.radians(-25.0)  # unlocked: leaning this far from straight up, to the cooler's -x (it swings about the
# clip's line, in the plane of the leg's face, over the top to lie along the cooler's +x, locked)


def cooler_to_world(p, origin=SEATED_POS) -> np.ndarray:
    c, s = math.cos(COOLER_YAW), math.sin(COOLER_YAW)
    x, y, z = p
    return np.asarray(origin) + np.array([c * x - s * y, s * x + c * y, z])


# -- the foam block it starts on (left of the board): the base on it, the legs hanging clear beside it -------------
FOAM_SIZE = (0.040, 0.040, 0.015)
FOAM_XY = np.array([0.290, 0.200])
START_POS = np.array([*FOAM_XY, TABLE + FOAM_SIZE[2] + 0.0003])
FOAM_BLOCK = Shape("foam", pos=(*FOAM_XY, TABLE + FOAM_SIZE[2] / 2), size=FOAM_SIZE, color=FOAM)

BOARD = UsdBody("board", usd="assets/board.usda", pos=tuple(BOARD_POS), rot_xyzw=yaw_xyzw(BOARD_YAW),
                size=(0.244, 0.305, 0.04), mass=None, material=False)
COOLER = UsdBody("cooler", usd="assets/cooler.usda", pos=tuple(START_POS), rot_xyzw=yaw_xyzw(COOLER_YAW),
                 size=(0.088, 0.084, 0.065), articulation=True, material=False, solver_iters=32,
                 joint_pos=tuple((n, 0.0) for n in ("leg_neg_slide_joint", "leg_pos_slide_joint", "leg_neg_flex_joint",
                                                    "leg_pos_flex_joint", "lever_joint")))
OBJECTS = [BOARD, COOLER, FOAM_BLOCK]

# -- the clip springs ---------------------------------------------------------------------------------------------
_STATE: dict = {"spring": None, "free": True, "anchor": None, "slid": 0.0, "lifted": False}


@on_reset
def _reset(environment) -> None:
    cooler = environment.unwrapped.scene["cooler"]
    if _STATE["spring"] is None or _STATE["spring"].cooler is not cooler:
        _STATE["spring"] = clip.ClipSpring(cooler)
    else:
        _STATE["spring"].reset()
    touch.reset()
    _STATE.update(free=True, anchor=None, slid=0.0, lifted=False)


@on_step
def _watch(environment) -> None:
    """Whether the cooler is released (no robot link touches its body, from PhysX contacts: touch.py), and the paste
    rule: from the moment the base first touches the processor, how far its centre has moved from there while touching,
    and whether it was lifted off again."""
    from yam_arena.scene import object_states

    _STATE["free"] = not touch.touching(environment, ["cooler"])["cooler"]
    paste_watch(object_states(environment, ["cooler"])["cooler"])


# -- the paste rule ---------------------------------------------------------------------------------------------
# The cooler's base (40 x 40 mm) carries pre-applied thermal paste, as AMD's boxed coolers do. AMD's instructions:
# once the heatsink has been installed it may not be lifted off and put back without cleaning both surfaces and
# applying new material, and the material must not be scratched or have gaps (FM2+ installation guide; Builder's Guide
# 30925, p. 23). So after the base touches the processor (40 x 40 mm, AMD's AM4/AM5 lid) it must not lift off, and it
# may not slide so far that the paste is wiped off part of the processor (a small shuffle into place is normal).
BASE_HALF = 20 * MM
CPU_HALF = 20 * MM
PASTE_TOUCH = 0.7 * MM  # the base's lowest corner within this of the processor's top, over it: touching
PASTE_LIFT = 3.0 * MM  # after touching, the base's lowest corner this far above the processor's top: lifted off
SLIDE_MAX = 10.0 * MM  # the base's centre may move this far (sideways) from where it first touched, while touching


def base_low(c) -> float:
    """The base's lowest corner above the processor's top (m)."""
    rot = c.rotation()
    corners = [c.pos + rot @ np.array([sx * BASE_HALF, sy * BASE_HALF, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    return min(p[2] for p in corners) - SEATED_POS[2]


def paste_watch(c) -> None:
    off = c.pos[:2] - SEATED_POS[:2]
    over = abs(off[0]) < BASE_HALF + CPU_HALF and abs(off[1]) < BASE_HALF + CPU_HALF  # the base overlaps the processor
    low = base_low(c)
    if over and low < PASTE_TOUCH:
        if _STATE["anchor"] is None:
            _STATE["anchor"] = c.pos[:2].copy()
        _STATE["slid"] = max(_STATE["slid"], float(np.hypot(*(c.pos[:2] - _STATE["anchor"]))))
    elif _STATE["anchor"] is not None and (not over or low > PASTE_LIFT):
        _STATE["lifted"] = True


def paste_broken() -> bool:
    return _STATE.get("lifted", False) or _STATE.get("slid", 0.0) > SLIDE_MAX


# -- scoring ----------------------------------------------------------------------------------------------------
ON_XY = 3.0 * MM  # the base's centre within this of the processor's centre
ON_Z = 1.5 * MM  # and its bottom within this of the processor's top
ON_YAW = math.radians(5.0)  # turned within this of square with the socket (lever towards the robot)
ON_TILT = math.radians(3.0)


def measure(objects) -> dict:
    """The cooler against its seated pose, each leg's travel, flex and whether it is hooked, and the lever."""
    c = objects["cooler"]
    rot = c.rotation()
    off = c.pos - SEATED_POS
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    j = c.joints
    legs = {}
    for n in clip.LEGS:
        drop = -j.get(f"leg_{n}_slide_joint", 0.0)
        flex = clip.SIGN[n] * j.get(f"leg_{n}_flex_joint", 0.0)
        legs[n] = (clip.HOOK_DROP - clip.HOOK_TOL[1] < drop < clip.HOOK_DROP + clip.HOOK_TOL[0]
                   and flex < clip.FLEX_TOL)
    lever = j.get(clip.LEVER, 0.0)
    return {
        "xy": float(np.hypot(off[0], off[1])),
        "z": float(off[2]),
        "yaw_err": abs((yaw - COOLER_YAW + math.pi) % (2 * math.pi) - math.pi),
        "tilt": math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))),
        "hooked": legs,
        "locked": lever > clip.LEVER_SPAN - clip.LEVER_SNAP,
    }


def score(objects) -> float:
    """0 if the paste was torn (lifted off after touching) or wiped (the base slid more than 10 mm on the processor),
    or unless the cooler sits square on the processor. 0.25: it does, released. 0.5: and one clip leg hooks its lug.
    0.75: both legs hook. 1.0: and the lever is down, locked."""
    m = measure(objects)
    on = m["xy"] < ON_XY and abs(m["z"]) < ON_Z and m["yaw_err"] < ON_YAW and m["tilt"] < ON_TILT
    if paste_broken() or not (on and _STATE["free"]):
        return 0.0
    hooks = sum(m["hooked"].values())
    if hooks < 2:
        return 0.25 + 0.25 * hooks
    return 1.0 if m["locked"] else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo ----------------------------------------------------------------------------------------
GRIP_Z = 43 * MM  # the pads' tips this high on the fan cover (3 mm over the fins): the pads take its lower 19 mm
GRIP_SQUEEZE = 6.0 * MM  # each finger commanded this far into the cover
CARRY = 0.07  # the base this high over the table while carrying
SET_ABOVE = 0.004  # lower to this over the processor, then set it down slowly
SET_CORRECTION = (0.0059, -0.0050)  # aim the base this far past the processor (world x, y): the arm sags with the cooler and
# it hangs in the hand (measured)
TAB_CORRECTION = {"left": (0.0, 0.0), "right": (0.0, 0.0)}  # aim each arm's TCP this far past its tab (measured)
TAB_PRESS = 40.0 * MM  # the fingertips commanded this far below a tab's top: the arm presses about 20 N (the leg stops
# at 6.8 mm; 9 mm only reached the tab)
# Where the closed pads press each tab, out from the cooler's centre, the fingers closing along world y (the clip's
# line): the pads' 6.3 mm on the tab, clear of the leg's own plate (a pad rubbing it holds the leg upright) and of the
# lever (its inner face 13 mm out); their 26 mm across the tab's 18 mm.
TAB_OUT = CLIP_Y + 5.6 * MM  # the pads 1.7 mm clear of the leg's plate and 2.4 mm of the lever's arm
ARC_R = 18 * MM  # the fingertips run round the lever's pivot this far out (low on the arm: the arm reaches here a few
# mm high, and at 24 mm the fingertips passed over the lever's end)
ARC_GAP = 5.6 * MM  # the pads' centre this far behind the lever's face (half the pads' 6.3 mm, half the arm, 0.3 mm)
ARC_END = 25.0  # degrees past straight up: the over-centre spring takes it the rest of the way down


def demo():
    """The left arm takes the cooler off its foam block by its fan cover, carries it over the processor, lowers it
    onto it square and lets go. Each arm then presses the clip's tab on its own side down with closed fingertips (the
    left one first), the leg riding out over its lug and hooking it; then the right arm pushes the lever over the top
    from behind until it snaps down, locked."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    # -- left arm: the cooler
    YAW_L = -math.pi / 2  # the fingers close along world x, across the cover, clear of the clip's legs and the lever
    C = -(COVER[0] / 2 - GRIP_SQUEEZE)
    up = GRIP_Z + TIP  # the TCP above the cooler's origin
    sx, sy = START_POS[:2]
    px, py = SEATED_POS[0] + SET_CORRECTION[0], SEATED_POS[1] + SET_CORRECTION[1]
    carry = TABLE + CARRY + up
    hover = TABLE + 0.15
    cover_top = SEATED_POS[2] + COVER[2][1]
    left = [
        W(2.0, (sx, sy, START_POS[2] + up + 0.04), O, "over the cooler", YAW_L),
        W(2.0, (sx, sy, START_POS[2] + up), O, "down round the fan cover", YAW_L),
        W(1.5, (sx, sy, START_POS[2] + up), C, "grip", YAW_L),
        W(1.0, (sx, sy, START_POS[2] + up), C, "settle", YAW_L),
        W(1.5, (sx, sy, carry), C, "lift", YAW_L),
        W(3.0, (px, py, carry), C, "over the processor", YAW_L),
        W(2.0, (px, py, SEATED_POS[2] + SET_ABOVE + up), C, "down to it", YAW_L),
        W(1.5, (px, py, SEATED_POS[2] + up - 0.001), C, "set it on", YAW_L),
        W(1.5, (px, py, SEATED_POS[2] + up - 0.001), O, "let go", YAW_L),
        W(1.0, (px, py, cover_top + 0.025 + TIP), O, "up, open, the fingertips clear of the cover", YAW_L),
        W(1.0, (px, py + 0.03, hover), 0.0, "up and towards its own side, closing the fingers", YAW_L),
    ]
    right = [W(sum(w.duration for w in left), (0.25, -0.20, hover), 0.0, "wait for the cooler", 0.0)]

    def sync():
        """Pad the arm that is ahead with a wait, so both start the next step together."""
        tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
        if tl < tr:
            left.append(W(tr - tl, left[-1].pos, left[-1].finger, "wait", left[-1].yaw))
        elif tr < tl:
            right.append(W(tl - tr, right[-1].pos, right[-1].finger, "wait", right[-1].yaw))

    def press(side, s):
        """Closed fingertips (closing along world y) pressing a tab down over its lug, and back up."""
        t = cooler_to_world((0.0, s * TAB_OUT, LEG_TOP)) + (*TAB_CORRECTION[side], 0.0)
        return [
            W(2.0, (t[0], t[1], hover), 0.0, f"over the {side} tab, high", 0.0),
            W(1.5, (t[0], t[1], t[2] + 0.03 + TIP), 0.0, "down over it", 0.0),
            W(1.0, (t[0], t[1], t[2] + 0.002 + TIP), 0.0, "down to it", 0.0),
            W(2.0, (t[0], t[1], t[2] - TAB_PRESS + TIP), 0.0, "press it down over the lug", 0.0),
            W(1.0, (t[0], t[1], t[2] - TAB_PRESS + TIP), 0.0, "hold", 0.0),
            W(1.0, (t[0], t[1], t[2] + 0.03 + TIP), 0.0, "up", 0.0),
            W(1.0, (t[0], t[1], hover), 0.0, "up, clear of the fan cover", 0.0),
        ]

    # Each arm presses the tab on its own side, the left one first (the cooler's -y end, world +y). Pressed alone, a
    # tab tips the cooler until its fins rest on the retention frame (about 4 degrees), then the leg goes down; the
    # hooked leg then holds its end down for the other.
    left += press("left", -1.0)
    left += [W(2.0, (0.25, 0.25, TABLE + 0.20), O, "away", 0.0)]  # clear of the right arm before it comes in
    sync()
    right += press("right", 1.0)
    # The right arm then pushes the lever over the top, along the cooler's +x (world -x, towards the robot), from behind
    # it: the fingers close along world x, the pads' 6.3 mm along the push.
    pivot = cooler_to_world(LEVER_PIVOT)
    ex = cooler_to_world((1.0, 0.0, 0.0)) - cooler_to_world((0.0, 0.0, 0.0))  # the way the lever goes over (world)

    def on_arc(deg):
        a = math.radians(deg)
        d = np.sin(a) * ex + np.array([0.0, 0.0, math.cos(a)])  # along the lever, from the pivot
        n = np.cos(a) * -ex + np.array([0.0, 0.0, math.sin(a)])  # its back's normal, turning with it
        p = pivot + ARC_R * d + ARC_GAP * n
        return (p[0], p[1], p[2] + TIP)

    YAW_R = -math.pi / 2
    start = math.degrees(LEVER_UP) - 25.0
    s0 = np.array(on_arc(start))
    beside = s0 - 0.015 * ex  # further behind, clear of the lever's end, to come down beside it
    right += [
        W(1.5, (beside[0], beside[1], hover), 0.0, "behind the lever, high", YAW_R),
        W(1.5, tuple(beside), 0.0, "down behind it, clear of its end", YAW_R),
        W(1.0, tuple(s0), 0.0, "in against its back, low", YAW_R),
    ]
    for deg in [*np.arange(start + 10.0, ARC_END, 10.0), ARC_END]:
        right.append(W(0.5, on_arc(deg), 0.0, f"push it over ({deg:.0f} deg)", YAW_R))
    end = np.array(on_arc(ARC_END))
    right += [
        W(1.0, tuple(end), 0.0, "hold, the spring takes it down", YAW_R),
        W(1.5, (end[0], end[1], hover), 0.0, "up", YAW_R),
        W(1.5, (0.25, -0.25, hover), O, "away", YAW_R),
    ]
    sync()
    return {"left": left, "right": right}
