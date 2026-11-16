"""On the station table, between the arms: an ATX motherboard (305 x 244 mm) lying flat, its x16 PCIe slot running
left-right straight ahead between the arms. The old graphics card (RTX 5060 Ti: 229 x 121 x 49 mm, 0.7 kg, two fans
facing the robot) is seated in the slot, held by the slot's retention latch at its right end; the card's cooler covers
the latch, and only the end of the latch's enlarged press tab shows, in a 7 mm gap beside the card's backplate. On the
right, a long flat-head screwdriver (200 mm) stands tip down in a grey rack, with an empty grey cup beside it. On the
left, a new card (RTX 5060 ITX: 145 x 121 x 45 mm, 0.55 kg, one fan facing the robot) stands in a
grey holder, and an empty holder for the old card stands beyond it.

Draft: scene, scoring and scripted demo (see PLAN.md).
Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402

import latches  # noqa: E402
import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Replace the graphics card with the new one on the left. The card's cooler covers the slot's latch: "
               "press the latch's tab down with the screwdriver, stand the old card in the empty holder, then install "
               "the new card and press it down until the latch clicks shut.")

MM = 0.001
TABLE = 0.7475
GREY = (0.45, 0.47, 0.50)

# -- the board (assets/board.usda: origin at the board's centre, bottom face; no Q-Release button) ----------------
BOARD_POS = np.array([0.262, -0.050, TABLE])  # the x16 slot centred between the arms
BOARD_YAW = -math.pi / 2  # board +y along world +x, board +x along world -y


def yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def board_to_world(p) -> np.ndarray:
    c, s = math.cos(BOARD_YAW), math.sin(BOARD_YAW)
    x, y, z = p
    return BOARD_POS + np.array([c * x - s * y, s * x + c * y, z])


# The x16 slot and its latch (work/board/pcie_slot.py), board frame.
SLOT_Y = -32 * MM
FINGERS = (-72.5 * MM, 16.5 * MM)  # a seated card's edge connector along the slot
FLOOR = 2.9 * MM  # a seated card's connector edge above the board's bottom face
SLOT_TOP = 9.8 * MM
LATCH_TOP = 11.0 * MM  # the press tab's top (shut)
TAB_X = (FINGERS[1] + 5.0 * MM, FINGERS[1] + 11.0 * MM)  # the press tab along the slot
TAB_SIDE = (2.0 * MM, 9.0 * MM)  # its enlarged part, to the backplate side of the slot's centre line
BACKPLATE_OUT = 4.1 * MM  # the card's backplate face, from the slot's centre line (card y)

# -- the card (assets/gpu_rtx5060ti.usda: origin at the edge connector's bottom edge, x along the card towards the
# latch end, -y the fan side, z up; turned like the board's frame when seated) ------------------------------------
CARD_MASS = 0.70
CARD_TOP = 120.5 * MM
CONNECTOR = (-64.5 * MM, 24.5 * MM)
FACES = (-45.0 * MM, 4.1 * MM)  # shroud and backplate faces, card y
CARD_X = FINGERS[0] - CONNECTOR[0]  # the seated card's origin along the slot
SEATED_POS = board_to_world((CARD_X, SLOT_Y, FLOOR))
GRIP_DX = (FACES[0] + FACES[1]) / 2  # world x of the thickness's middle from the card origin

# -- the new card (assets/gpu_rtx5060.usda, an RTX 5060 ITX: 145 x 121 x 45 mm, 0.55 kg, one fan; same frame) -------
NEW_MASS = 0.55
NEW_LENGTH = 145.7 * MM
NEW_CONNECTOR = (-22.5 * MM, 66.5 * MM)
NEW_FACES = (-40.9 * MM, 4.1 * MM)
NEW_SEATED = board_to_world((FINGERS[0] - NEW_CONNECTOR[0], SLOT_Y, FLOOR))
NEW_GRIP_DX = (NEW_FACES[0] + NEW_FACES[1]) / 2

# -- the card holders (left of the board): a plate with a wall along each face of a standing card ------------------
HOLDER_BASE = 0.004
WALL_T = 0.006
HOLDER_LEN = 0.120


def holder(name, pos, faces, wall_h, gap):
    x0, x1 = pos[0] + faces[0] - gap - WALL_T, pos[0] + faces[1] + gap + WALL_T
    return [
        Shape(f"{name}_base", pos=((x0 + x1) / 2, pos[1], TABLE + HOLDER_BASE / 2), size=(x1 - x0, HOLDER_LEN, HOLDER_BASE),
              color=GREY),
        *(Shape(f"{name}_wall{i}", pos=(x + WALL_T / 2, pos[1], TABLE + HOLDER_BASE + wall_h / 2),
                size=(WALL_T, HOLDER_LEN, wall_h), color=GREY) for i, x in enumerate((x0, x1 - WALL_T))),
    ]


# the new card's: tall close walls hold it within 1 degree of upright, so a hand takes it that way (it must go into
# the slot upright); the old card's, 10 cm further from the robot: low walls with 5 mm to spare on each side
NEW_START = np.array([NEW_SEATED[0], 0.28, TABLE + HOLDER_BASE + 0.0003])  # the new card's origin, standing in it
HOLDER_POS = np.array([SEATED_POS[0] + 0.10, 0.30, TABLE + HOLDER_BASE + 0.0003])  # the old card's, standing in its own
HOLDERS = (holder("new_holder", NEW_START, NEW_FACES, 0.040, 0.00025)
           + holder("old_holder", HOLDER_POS, FACES, 0.025, 0.005))

# -- the screwdriver (assets/screwdriver.usda: origin at its tip, z along it; tip 5 x 1 mm, shaft 4 mm square to
# 135 mm, handle 22 mm square to 200 mm) in its rack (right of the board): a 30 mm block with a 4.6 mm square hole ---
TOOL_MASS = 0.06
TOOL_HANDLE = (135.0 * MM, 200.0 * MM)
RACK_FLOOR, RACK_H, RACK_HOLE, RACK_W = 0.003, 0.030, 0.0046, 0.030
RACK_XY = np.array([0.24, -0.25])
TOOL_START = np.array([*RACK_XY, TABLE + RACK_FLOOR + 0.0003])  # the tip, resting on the rack's floor
TOOL_YAW = math.pi / 2  # the tip's 5 mm width along world y (along the slot), its 1 mm across
_side = (RACK_W - RACK_HOLE) / 2
RACK = [
    Shape("rack_floor", pos=(*RACK_XY, TABLE + RACK_FLOOR / 2), size=(RACK_W, RACK_W, RACK_FLOOR), color=GREY),
    *(Shape(f"rack_wall{i}", pos=(RACK_XY[0] + sx * (RACK_HOLE + _side) / 2, RACK_XY[1] + sy * (RACK_HOLE + _side) / 2,
                                  TABLE + RACK_FLOOR + RACK_H / 2),
            size=(_side if sx else RACK_W, _side if sy else RACK_HOLE, RACK_H), color=GREY)
      for i, (sx, sy) in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1)))),
]
# the cup the tool goes back into: wide, so a tool that has turned a little in the hand still drops in and leans there
CUP_FLOOR, CUP_H, CUP_IN, CUP_T = 0.003, 0.060, 0.024, 0.010
CUP_XY = np.array([0.32, -0.25])
_cw = CUP_IN + 2 * CUP_T
CUP = [
    Shape("cup_floor", pos=(*CUP_XY, TABLE + CUP_FLOOR / 2), size=(_cw, _cw, CUP_FLOOR), color=GREY),
    *(Shape(f"cup_wall{i}", pos=(CUP_XY[0] + sx * (CUP_IN + CUP_T) / 2, CUP_XY[1] + sy * (CUP_IN + CUP_T) / 2,
                                 TABLE + CUP_FLOOR + CUP_H / 2),
            size=(CUP_T if sx else _cw, CUP_T if sy else CUP_IN, CUP_H), color=GREY)
      for i, (sx, sy) in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1)))),
]

BOARD = UsdBody("board", usd="assets/board.usda", pos=tuple(BOARD_POS), rot_xyzw=yaw_xyzw(BOARD_YAW),
                size=(0.244, 0.305, 0.04), articulation=True, material=False,
                joint_pos=((latches.LATCH, 0.0),), joint_damping=latches.DAMPING)
OLD = UsdBody("old_card", usd="assets/gpu_rtx5060ti.usda", pos=tuple(SEATED_POS), rot_xyzw=yaw_xyzw(BOARD_YAW),
              size=(0.230, FACES[1] - FACES[0], CARD_TOP), mass=CARD_MASS, material=False, torsional_patch=0.01)
NEW = UsdBody("new_card", usd="assets/gpu_rtx5060.usda", pos=tuple(NEW_START), rot_xyzw=yaw_xyzw(BOARD_YAW),
              size=(NEW_LENGTH, NEW_FACES[1] - NEW_FACES[0], CARD_TOP), mass=NEW_MASS, material=False,
              torsional_patch=0.01)
TOOL = UsdBody("screwdriver", usd="assets/screwdriver.usda", pos=tuple(TOOL_START), rot_xyzw=yaw_xyzw(TOOL_YAW),
               size=(0.022, 0.022, 0.2), mass=TOOL_MASS, material=False, torsional_patch=0.011)
OBJECTS = [BOARD, OLD, NEW, TOOL, *HOLDERS, *RACK, *CUP]

# -- the latch spring ---------------------------------------------------------------------------------------------
_STATE: dict = {"spring": None, "opened": False, "torn": False, "old_free": True, "new_free": True}


@on_reset
def _reset(environment) -> None:
    board = environment.unwrapped.scene["board"]
    if _STATE["spring"] is None or _STATE["spring"].board is not board:
        _STATE["spring"] = latches.LatchSpring(board)
    else:
        _STATE["spring"].reset()
    touch.reset()
    _STATE.update(opened=False, torn=False, old_free=True, new_free=True)


def tcps(environment) -> dict:
    """Each arm's TCP (station frame), finger gap (m), grip yaw (rad, within +-90 degrees) and finger positions."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    out = {}
    for side in ("left", "right"):
        robot = environment.unwrapped.scene[f"{side}_robot"]
        q = getattr(robot.data.joint_pos, "torch", robot.data.joint_pos)[0]
        arm = q[robot.find_joints(ARM_JOINTS, preserve_order=True)[0]].cpu().numpy().astype(float)
        fingers = q[robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy().astype(float)
        pos, rot = ik.tcp_pose(arm, station.BASES[side])
        yaw = math.atan2(-rot[0, 0], rot[1, 0])
        out[side] = (pos, float(-fingers.sum()), (yaw + math.pi / 2) % math.pi - math.pi / 2, fingers)
    return out


TORN = 2.5 * MM  # the old card is torn out if either connector end rises this far before the latch was opened (pressing
# the tab lifts the latch end about 1.3 mm)


@on_step
def _watch(environment) -> None:
    """Whether the latch has been tipped open, whether the old card left the slot before that (torn out), and whether
    each card is released (no robot link touches it, from PhysX contacts: touch.py)."""
    from yam_arena.scene import object_states

    st = object_states(environment, ["board", "old_card", "new_card"])
    _STATE["opened"] = _STATE["opened"] or abs(st["board"].joints.get(latches.LATCH, 0.0)) > latches.SNAP
    if not _STATE["opened"] and max(_card(st["old_card"], SEATED_POS, CONNECTOR)["ends"]) > TORN:
        _STATE["torn"] = True
    touched = touch.touching(environment, ["old_card", "new_card"])
    _STATE.update(old_free=not touched["old_card"], new_free=not touched["new_card"])


# -- scoring (draft) ----------------------------------------------------------------------------------------------
SEATED = 0.5 * MM  # both connector ends within this of the slot floor
LATCH_SHUT = math.radians(5.0)
ALIGN_XY = 1.5 * MM
ALIGN_YAW = math.radians(5.0)
UPRIGHT = math.radians(5.0)


def _tilt(rot) -> float:
    return math.acos(float(np.clip(rot[2, 2], -1.0, 1.0)))


def _card(d, seated, connector) -> dict:
    """A card relative to the seated pose: offsets across (world x) and along (world y) the slot, yaw error, tilt and
    each connector end's height above the slot floor."""
    rot = d.rotation()
    off = d.pos - seated
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return {
        "across": float(off[0]),
        "along": float(off[1]),
        "yaw_err": abs((yaw - BOARD_YAW + math.pi) % (2 * math.pi) - math.pi),
        "tilt": _tilt(rot),
        "ends": [float(off[2] + x * rot[2, 0]) for x in connector],
    }


def _in_slot(c) -> bool:
    return (abs(c["across"]) < ALIGN_XY and abs(c["along"]) < ALIGN_XY and c["yaw_err"] < ALIGN_YAW
            and c["tilt"] < UPRIGHT and max(c["ends"]) < SLOT_TOP - FLOOR)


def measure(objects) -> dict:
    """Both cards (see _card), whether the old card stands in its holder, the latch angle."""
    old, new = objects["old_card"], objects["new_card"]
    hold = old.pos - HOLDER_POS
    return {
        "old": _card(old, SEATED_POS, CONNECTOR),
        "new": _card(new, NEW_SEATED, NEW_CONNECTOR),
        "in_holder": bool(abs(hold[0]) < 7 * MM and abs(hold[1]) < 0.06 and abs(hold[2]) < 3 * MM
                          and _tilt(old.rotation()) < math.radians(10)),
        "latch": abs(objects["board"].joints.get(latches.LATCH, 0.0)),
    }


def score(objects) -> float:
    """0 if the old card was torn out (it left the slot before the latch was tipped open). 0.25: the latch was tipped
    open. 0.5: and the old card stands in its holder, released. 0.75: and the new card stands in the slot, released.
    1.0: and the new card is fully seated with the latch shut."""
    m = measure(objects)
    if _STATE["torn"] or not _STATE["opened"]:
        return 0.0
    if not (m["in_holder"] and _STATE["old_free"]):
        return 0.25
    n = m["new"]
    if not (_in_slot(n) and _STATE["new_free"]):
        return 0.5
    return 1.0 if max(n["ends"]) < SEATED and m["latch"] < LATCH_SHUT else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo ----------------------------------------------------------------------------------------
TOOL_GRIP = 140.0 * MM  # the right fingertips' bottom this far above the tool's tip, on its handle
TOOL_LIFT = 0.055  # lift the tool this far to clear its rack (the arm sags about 7 mm with it)
TOOL_CARRY_TIP = 0.025  # the tool's tip this high over the table while it travels beside the card
TOOL_CORRECTION = (0.0, 0.0)  # aim the tool's tip this far past the tab (the arm's sag)
TAB_PRESS = 40.0 * MM  # the tip is commanded this far below the tab's top (it stops where the latch does; deeper, the arm
# presses harder: the shut latch takes about 12 N on the tab to tip past its snap angle)
CUP_CORRECTION = (0.0, 0.010)  # the hand this far from the cup middle: the tool turns 1-6 deg in the hand during the press, tip towards -y
TOOL_YAW_R = math.pi / 2  # the right fingers close along world x: they square the handle across the 7 mm gap
CARD_GRIP_BELOW_TOP = 0.040
CARD_GRIP_ALONG = -20.0 * MM  # near the card's centre of mass
CARD_SQUEEZE = 6.0 * MM
CARD_YAW_L = -math.pi / 2
CARD_LIFT = 0.060
CARD_SET_CORRECTION = (0.009, 0.0)  # the card hangs towards -x in the hand and the arm drifts that way going down

# installing the new card (as in tasks/install-graphics-card, one hand, the left)
GRIP_SQUEEZE = 6.0 * MM  # each finger commanded this far into the card: the two fingers centre it between them
YAW = -math.pi / 2  # the left hand's fingers close across the card (along world x)
CARRY_EDGE = 0.05  # the connector edge's height over the table while carrying
SET_ABOVE = 0.001  # set it with the connector edge this far above the slot top
SET_CORRECTION = (0.0065, -0.0068)  # aim this far past the slot in x and y (the arm's sag with the card, and how
# the card hangs in the hand; measured on this path, from the old card's holder)
YAW_CORRECTION = math.radians(-2.4)  # and turn the wrist this much more
SET_IN = 4.0 * MM  # set the connector this far into the slot, so the slot steers it, then push it home
PUSH_DEPTH = 0.06  # gripping: the connector edge commanded this far below the slot floor (about 30 N)
PRESS_BRACKET = NEW_CONNECTOR[0] + 12.5 * MM  # then, let go, press the card's top over the connector's bracket end
PRESS_LATCH = NEW_CONNECTOR[1] - 12.5 * MM  # and then over its latch end, which the shut latch leaves up to 0.5 mm high
PRESS_DX = -6.0 * MM  # world x of the pressing TCP from the edge connector
PRESS_SIDE = 0.0  # and this far further towards the backplate
NEW_GRIP_ALONG = 0.0  # grip the new card at its middle (its centre of mass)
PRESS_DEPTH = 0.06  # the pressing TCP is commanded this far into the card's top (about 25 N)
PRESS_HOLD = 2.0


def demo():
    """The right arm takes the screwdriver from its rack by the handle, carries it beside the card's backplate with
    its tip low, lowers the tip into the gap onto the latch's tab and presses the tab down until the latch tips open,
    then lifts the tool clear and drops it in the cup. The left arm then grips the old card across its thickness, lifts
    it straight out and stands it in its holder; it takes the new card from the other holder, sets its connector a few
    millimetres into the slot and pushes it home gripping it, until the latch shuts, lets go and presses the top of the
    card's bracket end home with closed fingers."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.constants import PAD_TIP_PAST_TCP as TIP
    from yam_arena.demo import Waypoint as W

    # -- right arm: the screwdriver
    tool_tcp = TOOL_GRIP + TIP  # the gripping TCP above the tool's tip
    C_tool = -(22.0 * MM / 2 - 6.0 * MM)
    tx, ty = RACK_XY
    grip_z = TOOL_START[2] + tool_tcp
    tab = board_to_world(((TAB_X[0] + TAB_X[1]) / 2, SLOT_Y + (TAB_SIDE[1] + BACKPLATE_OUT + 2 * MM) / 2, LATCH_TOP))
    px, py = tab[0] + TOOL_CORRECTION[0], tab[1] + TOOL_CORRECTION[1]
    carry_z = TABLE + TOOL_CARRY_TIP + tool_tcp
    ux, uy = CUP_XY + np.array(CUP_CORRECTION)
    cup_z = TABLE + CUP_FLOOR + 0.003 + tool_tcp  # the tip 3 mm over the cup's floor
    over_cup = TABLE + CUP_FLOOR + CUP_H + 0.015 + tool_tcp  # the tip 15 mm over the cup's walls
    beside_y = SEATED_POS[1] - 0.13  # beside the card's latch end, clear of the M.2 heatsink, before sliding in
    right = [
        # the fingers close along world x, so the open hand comes in along y, the handle between its fingers
        W(2.0, (tx, ty - 0.06, grip_z), O, "beside the screwdriver's handle", TOOL_YAW_R),
        W(2.0, (tx, ty, grip_z), O, "around the handle", TOOL_YAW_R),
        W(2.0, (tx, ty, grip_z), C_tool, "grip", TOOL_YAW_R),
        W(1.0, (tx, ty, grip_z), C_tool, "settle", TOOL_YAW_R),
        W(1.5, (tx, ty, grip_z + TOOL_LIFT), C_tool, "lift it out of the rack", TOOL_YAW_R),
        W(2.5, (px, beside_y, carry_z), C_tool, "beside the card", TOOL_YAW_R),
        W(2.0, (px, py, carry_z), C_tool, "along the backplate, over the tab", TOOL_YAW_R),
        W(1.5, (px, py, tab[2] + 0.004 + tool_tcp), C_tool, "tip down into the gap", TOOL_YAW_R),
        W(1.5, (px, py, tab[2] - TAB_PRESS + tool_tcp), C_tool, "press the tab", TOOL_YAW_R),
        W(1.0, (px, py, tab[2] - TAB_PRESS + tool_tcp), C_tool, "hold", TOOL_YAW_R),
        W(1.0, (px, py, carry_z), C_tool, "tip up", TOOL_YAW_R),
        W(2.0, (px, beside_y, carry_z), C_tool, "back out", TOOL_YAW_R),
        W(1.0, (px, beside_y, over_cup), C_tool, "up, the tip clear of the cup's walls", TOOL_YAW_R),
        W(2.5, (ux, uy, over_cup), C_tool, "over the cup", TOOL_YAW_R),
        W(1.0, (ux, uy, over_cup), C_tool, "settle over it", TOOL_YAW_R),
        W(1.5, (ux, uy, cup_z + 0.02), C_tool, "tip into the cup", TOOL_YAW_R),
        W(1.5, (ux, uy, cup_z), C_tool, "tip down into the cup", TOOL_YAW_R),
        W(1.0, (ux, uy, cup_z), O, "let go", TOOL_YAW_R),
        W(2.0, (ux, uy, cup_z + 0.09), O, "straight up off the handle", TOOL_YAW_R),
        W(2.0, (ux, uy - 0.06, cup_z + 0.09), O, "away", TOOL_YAW_R),
    ]
    # -- left arm: the card, once the latch is open
    C_card = -((FACES[1] - FACES[0]) / 2 - CARD_SQUEEZE)
    grip_up = CARD_TOP - CARD_GRIP_BELOW_TOP + TIP
    cx, cy = SEATED_POS[0] + GRIP_DX, SEATED_POS[1] - CARD_GRIP_ALONG
    hx = HOLDER_POS[0] + GRIP_DX + CARD_SET_CORRECTION[0]
    hy = HOLDER_POS[1] - CARD_GRIP_ALONG + CARD_SET_CORRECTION[1]
    seated_z = SEATED_POS[2] + grip_up
    lifted = seated_z + CARD_LIFT
    pressed = sum(w.duration for w in right[:11])
    left = [
        W(pressed, (0.25, 0.20, TABLE + 0.18), O, "wait for the latch", CARD_YAW_L),
        W(2.0, (cx, cy, seated_z + 0.03), O, "over the card", CARD_YAW_L),
        W(1.5, (cx, cy, seated_z), O, "down around it", CARD_YAW_L),
        W(2.0, (cx, cy, seated_z), C_card, "grip", CARD_YAW_L),
        W(1.0, (cx, cy, seated_z), C_card, "settle", CARD_YAW_L),
        W(3.0, (cx, cy, lifted), C_card, "lift it straight out", CARD_YAW_L),
        W(2.0, (hx, cy, lifted), C_card, "aside, clear of the new card", CARD_YAW_L),
        W(3.0, (hx, hy, lifted), C_card, "over the holder", CARD_YAW_L),
        W(3.0, (hx, hy, HOLDER_POS[2] + grip_up + 0.002), C_card, "into the holder", CARD_YAW_L),
        W(2.0, (hx, hy, HOLDER_POS[2] + grip_up + 0.002), O, "let go", CARD_YAW_L),
        W(2.5, (hx, hy, TABLE + 0.18), O, "up, the fingers clear of the old card", CARD_YAW_L),
        W(1.0, (hx, hy, TABLE + 0.18), O, "settle", CARD_YAW_L),
    ]
    # -- left arm: the new card
    C = -((NEW_FACES[1] - NEW_FACES[0]) / 2 - GRIP_SQUEEZE)
    nx, ny = NEW_START[0] + NEW_GRIP_DX, NEW_START[1] - NEW_GRIP_ALONG
    sx = NEW_SEATED[0] + NEW_GRIP_DX + SET_CORRECTION[0]
    sy = NEW_SEATED[1] - NEW_GRIP_ALONG + SET_CORRECTION[1]
    hover = TABLE + 0.15
    carry = TABLE + CARRY_EDGE + grip_up
    set_z = TABLE + SLOT_TOP + SET_ABOVE + grip_up
    new_seated_z = NEW_SEATED[2] + grip_up
    in_z = new_seated_z + (SLOT_TOP - FLOOR) - SET_IN
    grip_z = NEW_START[2] + grip_up
    set_yaw = YAW + YAW_CORRECTION
    top = SEATED_POS[2] + CARD_TOP + TIP  # TCP with closed fingers resting on a seated card's top
    bx, by = NEW_SEATED[0] + PRESS_DX, NEW_SEATED[1] + SET_CORRECTION[1] - PRESS_BRACKET
    lx, ly = bx, NEW_SEATED[1] + SET_CORRECTION[1] - PRESS_LATCH
    left += [
        W(2.0, (nx, ny, hover), O, "over the new card", YAW),
        W(2.0, (nx, ny, grip_z), O, "descend", YAW),
        W(2.0, (nx, ny, grip_z), C, "grip", YAW),
        W(1.0, (nx, ny, grip_z), C, "settle", YAW),
        W(1.5, (nx, ny, carry), C, "lift", YAW),
        W(4.0, (sx, sy, carry), C, "carry", set_yaw),
        W(2.0, (sx, sy, set_z), C, "into the slot's mouth", set_yaw),
        W(2.0, (sx, sy, in_z), C, "into the slot", set_yaw),
        W(4.0, (sx, sy, new_seated_z - PUSH_DEPTH), C, "push it home", set_yaw),
        W(1.0, (sx, sy, new_seated_z - PUSH_DEPTH), C, "hold", set_yaw),
        W(0.8, (sx, sy, new_seated_z), C, "ease off", set_yaw),
        W(2.0, (sx, sy, new_seated_z), O, "let go", set_yaw),
        W(1.5, (sx, sy, top + 0.03), O, "up", YAW),
        W(1.5, (lx, ly, top + 0.015), 0.0, "over the latch end", YAW),
        W(1.5, (lx + PRESS_SIDE, ly, top - PRESS_DEPTH), 0.0, "press it home", YAW),
        W(PRESS_HOLD, (lx + PRESS_SIDE, ly, top - PRESS_DEPTH), 0.0, "hold", YAW),
        W(1.0, (lx, ly, top + 0.015), 0.0, "lift", YAW),
        W(1.5, (bx, by, top + 0.015), 0.0, "over the bracket end", YAW),
        W(1.5, (bx + PRESS_SIDE, by, top - PRESS_DEPTH), 0.0, "press it home", YAW),
        W(PRESS_HOLD, (bx + PRESS_SIDE, by, top - PRESS_DEPTH), 0.0, "hold", YAW),
        W(1.0, (bx, by, top + 0.015), 0.0, "lift", YAW),
        W(2.5, (0.30, 0.30, TABLE + 0.22), O, "away", YAW),
    ]
    return {"right": right, "left": left}
