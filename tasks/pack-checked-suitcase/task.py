"""Pack the last travel items into a suitcase that will be checked in, keeping the spare lithium batteries out of it.

On the station table, packing for a flight. On the right lies the main half of an open hard-side carry-on spinner
(Samsonite Freeform 21 in, navy polypropylene; inside 20.5 x 14.5 x 10 in, so each half 5 in deep), on its back, its
long side pointing away from the robot and its hinge side at the table's right edge; the lid half hangs open down over
that edge. Its far part already holds two stacks of folded clothes; its near part is empty. On the left lies a
backpack (JanSport SuperBreak Plus, red, 16.5 x 12.5 x 5.5 in) on
its back, its main compartment unzipped, the front panel folded open behind it. Between them lie five things, each
flat on the table: a Samsung T7 portable SSD (85 x 57 x 8 mm, grey), a deck of Bicycle playing cards in its red tuck
box (3.5 x 2.6 x 0.75 in), an EPICKA TA-105 universal travel adapter (2.8 x 2.05 x 1.97 in, black; no battery), an
Anker PowerCore 10000 power bank (92 x 60 x 22 mm, black) and a spare Canon LP-E6NH camera battery (56.8 x 38.4 x
21 mm, black, its terminal cover on). The suitcase will be checked in. FAA and TSA rules: power banks and spare lithium
batteries must travel in the cabin, never in checked baggage, where a battery fire in the hold can't be reached.

Scene, scoring and scripted demo (see README.md).
Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import partkit  # noqa: E402,F401
from partkit import part  # noqa: E402
from scenekit import Look, Shape, on_reset, on_step  # noqa: E402

import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Pack the things on the table into the suitcase; it will be checked in at the airport."

MM = 0.001
IN = 0.0254
ASSETS = pathlib.Path(__file__).resolve().parent / "assets"
TOP = TABLE_TOP[SCENE]

# -- the suitcase: fixed. Samsonite Freeform carry-on spinner, inside 20.5 x 14.5 x 10 in (shop.samsonite.com): the
# main half 5 in deep inside, lying on its back with its 20.5 in side along x, its hinge side at the table's right edge;
# the lid half hangs open down over that edge. 6 mm polypropylene shell.
SHELL = 6 * MM
CASE_IN = (20.5 * IN, 14.5 * IN)  # inside, x by y: 521 x 368 mm
CASE_DEPTH = 5 * IN  # each half, inside: 127 mm
CASE_X0 = 0.135  # the inside's near edge
CASE_Y0 = -0.644  # the inside's right edge (-y); the shell's outside at the table's right edge, y -0.65
CASE_XY = np.array([CASE_X0 + CASE_IN[0] / 2, CASE_Y0 + CASE_IN[1] / 2])
CASE_FLOOR = TOP + SHELL
CASE_RIM = CASE_FLOOR + CASE_DEPTH
CASE_OUT_Y1 = CASE_Y0 + CASE_IN[1] + SHELL  # the shell's left side (+y)
NAVY = (0.12, 0.17, 0.32)
NAVY_IN = (0.20, 0.24, 0.36)  # the lining


def _tray(prefix, centre, inner, floor_t, wall_t, depth, color, floor_color):
    """An open-top tray of fixed boxes: a floor and four walls round an ``inner`` (x, y) space ``depth`` deep."""
    cx, cy = centre
    ox, oy = inner[0] + 2 * wall_t, inner[1] + 2 * wall_t
    h = floor_t + depth
    return [
        Shape(f"{prefix}_floor", pos=(cx, cy, TOP + floor_t / 2), size=(ox, oy, floor_t), color=floor_color),
        *[Shape(f"{prefix}_wall_{n}", pos=(cx + s * (inner[0] + wall_t) / 2, cy, TOP + h / 2), size=(wall_t, oy, h),
                color=color) for n, s in (("near", -1), ("far", 1))],
        *[Shape(f"{prefix}_wall_{n}", pos=(cx, cy + s * (inner[1] + wall_t) / 2, TOP + h / 2),
                size=(inner[0], wall_t, h), color=color) for n, s in (("right", -1), ("left", 1))],
    ]


# the lid half, turned 270 degrees about the hinge (the main half's right top edge): hanging down outside the table's
# edge, its inside facing away from the table
_HINGE_Y, _HINGE_Z = CASE_Y0 - SHELL, CASE_RIM
_LID_W = CASE_IN[1] + 2 * SHELL  # its width, now vertical
_LID_D = CASE_DEPTH + SHELL  # its depth, now along -y
_LID_X = CASE_IN[0] + 2 * SHELL
LID = [
    Shape("lid_back", pos=(CASE_XY[0], _HINGE_Y - SHELL / 2, _HINGE_Z - _LID_W / 2), size=(_LID_X, SHELL, _LID_W),
          color=NAVY),
    *[Shape(f"lid_wall_{n}", pos=(CASE_XY[0], _HINGE_Y - _LID_D / 2, z), size=(_LID_X, _LID_D, SHELL), color=NAVY)
      for n, z in (("top", _HINGE_Z - SHELL / 2), ("bottom", _HINGE_Z - _LID_W + SHELL / 2))],
    *[Shape(f"lid_wall_{n}", pos=(CASE_XY[0] + s * (CASE_IN[0] + SHELL) / 2, _HINGE_Y - _LID_D / 2, _HINGE_Z - _LID_W / 2),
            size=(SHELL, _LID_D, _LID_W), color=NAVY) for n, s in (("near", -1), ("far", 1))],
    Shape("lid_lining", pos=(CASE_XY[0], _HINGE_Y - SHELL - 1 * MM, _HINGE_Z - _LID_W / 2),
          size=(CASE_IN[0], 2 * MM, CASE_IN[1]), color=NAVY_IN),
]
# folded clothes already packed in the far part of the main half: two stacks of T-shirts folded to about 25 x 18 cm
CLOTHES_X = (0.400, CASE_X0 + CASE_IN[0])
CLOTHES_H = 80 * MM
CLOTHES = [
    Shape(f"clothes_{n}", pos=((CLOTHES_X[0] + CLOTHES_X[1]) / 2, y, CASE_FLOOR + CLOTHES_H / 2),
          size=(CLOTHES_X[1] - CLOTHES_X[0] - 6 * MM, 0.175, CLOTHES_H), color=c)
    for n, y, c in (("right", CASE_XY[1] - 0.092, (0.55, 0.62, 0.70)), ("left", CASE_XY[1] + 0.092, (0.86, 0.84, 0.78)))
]
SUITCASE = [*_tray("case", CASE_XY, CASE_IN, SHELL, SHELL, CASE_DEPTH, NAVY, NAVY_IN), *LID, *CLOTHES]

# -- the backpack: fixed. JanSport SuperBreak Plus, 16.5 x 12.5 x 5.5 in (Public Lands listing), on its back with its
# long side along x, its bottom away from the robot; the main compartment unzipped and its front panel folded open
# beyond its bottom and doubled back on itself (the table ends 0.91 m out). Nylon walls and padded back panel modelled 10 and 15 mm thick.
BAG_OUT = (16.5 * IN, 12.5 * IN, 5.5 * IN)  # 419 x 318 x 140 mm
BAG_WALL = 10 * MM
BAG_BACK = 15 * MM
BAG_IN = (BAG_OUT[0] - 2 * BAG_WALL, BAG_OUT[1] - 2 * BAG_WALL)
BAG_XY = np.array([0.130 + BAG_OUT[0] / 2, 0.545 - BAG_OUT[1] / 2])
BAG_FLOOR = TOP + BAG_BACK
BAG_RIM = TOP + BAG_OUT[2]
BLACK_NYLON = (0.08, 0.08, 0.09)
BACKPACK = [
    *_tray("bag", BAG_XY, BAG_IN, BAG_BACK, BAG_WALL, BAG_OUT[2] - BAG_BACK, BLACK_NYLON, (0.16, 0.16, 0.18)),
    Shape("bag_front_panel", pos=(BAG_XY[0] + BAG_OUT[0] / 2 + BAG_OUT[0] / 4, BAG_XY[1], TOP + 20 * MM),
          size=(BAG_OUT[0] / 2, BAG_OUT[1], 40 * MM), color=BLACK_NYLON),
    Shape("bag_handle", pos=(BAG_XY[0] - BAG_OUT[0] / 2 - 15 * MM, BAG_XY[1], TOP + 12 * MM),
          size=(30 * MM, 60 * MM, 8 * MM), color=BLACK_NYLON),
]

# -- the items: loose, each flat on the table, its long side along x (the fingers close along y, across it).
# name: (size x, y, z; mass kg; packed where; source)
ITEMS = {
    "ssd": ((85 * MM, 57 * MM, 8 * MM), 0.058, "suitcase"),  # Samsung T7 datasheet: 85 x 57 x 8.0 mm, 58 g
    "cards": ((3.5 * IN, 2.6 * IN, 0.75 * IN), 0.090, "suitcase"),  # Bicycle poker deck in its tuck box, ~3 oz
    "adapter": ((2.8 * IN, 2.05 * IN, 1.97 * IN), 0.145, "suitcase"),  # EPICKA TA-105: 2.8 x 2.05 x 1.97 in, 5.1 oz
    "power_bank": ((92 * MM, 60 * MM, 22 * MM), 0.180, "backpack"),  # Anker PowerCore 10000 (A1263): 180 g
    "camera_battery": ((56.8 * MM, 38.4 * MM, 21 * MM), 0.080, "backpack"),  # Canon LP-E6NH: about 80 g
}
ALLOWED = [n for n, v in ITEMS.items() if v[2] == "suitcase"]
BATTERIES = [n for n, v in ITEMS.items() if v[2] == "backpack"]
START = {  # right-arm items at least 14 cm from the suitcase wall (or far out in x), so the forearm clears its rim
    "adapter": (0.310, -0.090),
    "cards": (0.200, -0.025),
    "ssd": (0.310, 0.000),
    "power_bank": (0.200, 0.085),
    "camera_battery": (0.310, 0.100),
}
GREY = (0.42, 0.43, 0.45)
DARK = (0.06, 0.06, 0.07)


def _item(name):
    (sx, sy, sz), mass, _ = ITEMS[name]
    x, y = START[name]
    body = ((0, 0, 0), (sx, sy, sz))
    if name == "ssd":  # titan grey aluminium, a black rubber end with the USB-C port
        boxes = [(*body, GREY), ((sx / 2 - 6 * MM, 0, 0), (12 * MM, sy + 0.4 * MM, sz + 0.4 * MM), DARK)]
    elif name == "cards":  # the red tuck box, a white face panel on top
        boxes = [(*body, (0.70, 0.10, 0.12)), ((0, 0, sz / 2), (sx - 14 * MM, sy - 14 * MM, 0.6 * MM),
                                              (0.93, 0.92, 0.90))]
    elif name == "adapter":  # black body, the red plug-selector slider on top
        boxes = [(*body, DARK), ((0, 0, sz / 2 + 1 * MM), (30 * MM, 10 * MM, 2 * MM), (0.75, 0.12, 0.10))]
    elif name == "power_bank":  # black, the ports and the blue charge LEDs on one end
        boxes = [(*body, DARK), ((sx / 2, 0, 0), (1 * MM, 30 * MM, 8 * MM), (0.25, 0.25, 0.28)),
                 ((sx / 2 - 10 * MM, 0, sz / 2), (6 * MM, 3 * MM, 0.6 * MM), (0.20, 0.45, 0.95))]
    else:  # the LP-E6NH: black, a grey label on top, the terminal cover on its contact end
        boxes = [(*body, DARK), ((0, 0, sz / 2), (sx - 16 * MM, sy - 10 * MM, 0.6 * MM), (0.55, 0.55, 0.56)),
                 ((sx / 2 + 1 * MM, 0, 0), (2 * MM, sy - 6 * MM, sz - 6 * MM), (0.20, 0.20, 0.22))]
    return part(name, (x, y, TOP + sz / 2 + 0.5 * MM), boxes, mass, friction=0.6, torsional_patch=5 * MM,
                visual=str(ASSETS / f"{name}.usda"))


ITEM_PARTS = [_item(n) for n in ITEMS]
# what the cameras see: textured models (assets/, from work/pack-checked-suitcase/make_looks.py) over the colliders above,
# which are hidden; the models are in world coordinates, the items' in their own frames
LOOKS = [Look(f"{n}_look", pos=(0.0, 0.0, 0.0), usd=f"assets/{n}.usda") for n in ("suitcase", "clothes", "backpack")]
OBJECTS = [*ITEM_PARTS, *(replace(o, visible=False) for o in (*SUITCASE, *BACKPACK)), *LOOKS]

# -- contacts and the whole-trial rule -------------------------------------------------------------------------------
_STATE: dict = {"free": {n: True for n in ITEMS}, "battery_in_case": False}


@on_reset
def _reset(environment) -> None:
    touch.reset()
    _STATE.update(free={n: True for n in ITEMS}, battery_in_case=False)


@on_step
def _watch(environment) -> None:
    """Whether each item is released: no robot link touches it (PhysX contacts, touch.py)."""
    t = touch.touching(environment, list(ITEMS))
    _STATE["free"] = {n: not v for n, v in t.items()}


# -- scoring --------------------------------------------------------------------------------------------------------
SIGNS = np.array([(sx, sy, sz) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def corners(state, name) -> np.ndarray:
    """The item's eight corners (its main box) in the world."""
    return np.asarray(state.pos, float) + (SIGNS * np.asarray(ITEMS[name][0]) / 2) @ _rot(state.quat_xyzw).T


def _over(points, centre, inner) -> bool:
    d = np.abs(np.atleast_2d(points)[:, :2] - centre)
    return bool(np.all(d[:, 0] <= inner[0] / 2) and np.all(d[:, 1] <= inner[1] / 2))


def in_suitcase(objects, name) -> bool:
    """Wholly over the main half's inside, every corner below its rim, at rest."""
    c = corners(objects[name], name)
    return _over(c, CASE_XY, CASE_IN) and float(c[:, 2].max()) <= CASE_RIM and objects[name].resting()


def in_backpack(objects, name) -> bool:
    """Wholly over the backpack's main compartment, every corner below its rim, at rest."""
    c = corners(objects[name], name)
    return _over(c, BAG_XY, BAG_IN) and float(c[:, 2].max()) <= BAG_RIM and objects[name].resting()


def in_lid(point) -> bool:
    """A point inside the hanging lid half's walls."""
    x, y, z = (float(v) for v in point)
    return (abs(x - CASE_XY[0]) <= CASE_IN[0] / 2 and _HINGE_Y - _LID_D <= y <= _HINGE_Y
            and _HINGE_Z - _LID_W <= z <= _HINGE_Z)


def battery_entering_case(objects) -> bool:
    """A battery's centre over the main half's inside with its lowest corner below the rim, or inside the lid half."""
    for n in BATTERIES:
        c = corners(objects[n], n)
        centre = np.asarray(objects[n].pos)
        if (_over(centre, CASE_XY, CASE_IN) and float(c[:, 2].min()) < CASE_RIM) or in_lid(centre):
            return True
    return False


def score(objects) -> float:
    """0 for the whole trial once a power bank or spare battery has been inside the suitcase (either half). Otherwise
    one fifth for each of the SSD, the cards and the adapter in the suitcase, and for each battery in the backpack,
    each counted once no robot link touches it."""
    if battery_entering_case(objects):
        _STATE["battery_in_case"] = True
    if _STATE["battery_in_case"]:
        return 0.0
    free = _STATE["free"]
    n = sum(in_suitcase(objects, a) and free[a] for a in ALLOWED)
    n += sum(in_backpack(objects, b) and free[b] for b in BATTERIES)
    return n / len(ITEMS)


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------------
PAD_BELOW = 13.2 * MM  # the pads reach this far below the TCP
HOVER = TOP + 0.175  # carry height over the suitcase or backpack: an item's bottom 14+ mm above its rim
PICK_HOVER = TOP + 0.090  # lower over the items, where the arms reach less high
VIA_Y = {"suitcase": -0.140, "backpack": 0.180}  # rise to HOVER here, before crossing the wall
PARK_X = 0.100  # each arm waits here, in front of the backpack or suitcase, clear of the top camera's view of them
RELEASE_GAP = 10 * MM  # each finger opens this much wider than the item to let go
OPEN_GAP = 10 * MM  # and this much wider while coming down beside it
FINGER_T = 8 * MM  # a finger's thickness outside its pad face (for clearances)
FINGER_W = 30 * MM  # a finger's width along the item (x)
PALM_W = 102 * MM  # the palm's length along the closing direction, 83.5 mm above the pad tips
PLACES = {  # where each item is set down (centre x, y): the suitcase's empty near part, the backpack's near end
    "adapter": (0.235, -0.360),
    "cards": (0.235, -0.460),
    "ssd": (0.235, -0.560),
    "power_bank": (0.245, 0.330),
    "camera_battery": (0.245, 0.440),
}


def grip_z(name) -> float:
    """TCP height to grip a flat item: at its middle, but the pad tips at least 1.5 mm above the table."""
    return TOP + max(ITEMS[name][0][2] / 2, PAD_BELOW + 1.5 * MM)


def _carry(name, floor, via=None):
    """Down beside an item (fingers closing along y, across it), grip, lift, rise to HOVER on the way, carry over its place, lower
    until it is 3 mm above ``floor``, open and go up. 13.3 s."""
    from yam_arena.demo import Waypoint as W

    (sx, sy, sz), _, _ = ITEMS[name]
    (x, y), (dx, dy) = START[name], PLACES[name]
    gz = grip_z(name)
    open_ = -(sy / 2 + OPEN_GAP)
    grip = -(sy / 2 - 5 * MM)  # 5 mm past contact each side: firmly held
    release = -(sy / 2 + RELEASE_GAP)
    place_z = floor + 3 * MM + (gz - TOP)
    via_y = VIA_Y[via or ("suitcase" if name in ALLOWED else "backpack")]
    return [
        W(1.5, (x, via_y, HOVER), open_, f"towards the {name}, high over the wall", 0.0),
        W(1.0, (x, y, PICK_HOVER), open_, f"over the {name}", 0.0),
        W(1.2, (x, y, gz), open_, "down beside it", 0.0),
        W(0.5, (x, y, gz), open_, "settle", 0.0),
        W(1.2, (x, y, gz), grip, "grip", 0.0),
        W(1.0, (x, y, PICK_HOVER), grip, "lift", 0.0),
        W(1.2, ((x + dx) / 2, via_y, HOVER), grip, "rise towards it", 0.0),
        W(1.2, (dx, dy, HOVER), grip, "carry", 0.0),
        W(1.5, (dx, dy, place_z), grip, "lower in", 0.0),
        W(0.5, (dx, dy, place_z), grip, "settle", 0.0),
        W(1.0, (dx, dy, place_z), release, "let go", 0.0),
        W(1.2, (dx, dy, HOVER), release, "up", 0.0),
    ]


def demo():
    """The left arm puts the camera battery and the power bank in the backpack; then the right arm packs the adapter,
    the cards and the SSD into the suitcase. Each arm waits at its side while the other works."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    left = _carry("camera_battery", BAG_FLOOR) + _carry("power_bank", BAG_FLOOR)
    left.append(W(1.5, (PARK_X, 0.30, HOVER), O, "back to its side", 0.0))
    right_work = _carry("adapter", CASE_FLOOR) + _carry("cards", CASE_FLOOR) + _carry("ssd", CASE_FLOOR)
    right_work.append(W(1.5, (PARK_X, -0.30, HOVER), O, "back to its side", 0.0))
    t_left = sum(w.duration for w in left)
    right = [W(t_left, (PARK_X, -0.30, HOVER), O, "wait", 0.0)] + right_work
    left.append(W(sum(w.duration for w in right_work), (PARK_X, 0.30, HOVER), O, "wait", 0.0))
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states, and the starting layout against itself."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def st(name, x, y, bottom, yaw=0.0, vel=0.0, roll=0.0):
        cy, sy_, cr, sr = np.cos(yaw / 2), np.sin(yaw / 2), np.cos(roll / 2), np.sin(roll / 2)
        q = np.array([sr * cy, sr * sy_, cr * sy_, cr * cy])  # yaw about z, then roll about the item's x
        half = np.abs(_rot(q)) @ (np.asarray(ITEMS[name][0]) / 2)
        centre = np.array([x, y, bottom + half[2]])
        return ObjectState(centre, q, np.array([vel, 0.0, 0.0]), np.zeros(3), centre - half, centre + half)

    start = {n: st(n, *START[n], TOP) for n in ITEMS}
    packed = {n: st(n, *PLACES[n], CASE_FLOOR if n in ALLOWED else BAG_FLOOR) for n in ITEMS}

    def run(free=None, history=(), **moved):
        _STATE.update(free={n: True for n in ITEMS} if free is None else free, battery_in_case=False)
        for h in history:
            score({**start, **h})
        s = {**start, **moved}
        return score(s), success(s)

    allowed_in = {n: packed[n] for n in ALLOWED}
    batteries_in = {n: packed[n] for n in BATTERIES}
    held = {n: True for n in ITEMS}
    held["power_bank"] = False
    cases = {
        "start": (run(), (0.0, False)),
        "all packed right, released": (run(**packed), (1.0, True)),
        "only the three allowed things in the suitcase": (run(**allowed_in), (0.6, False)),
        "only the batteries in the backpack": (run(**batteries_in), (0.4, False)),
        "all packed right, the power bank still held": (run(free=held, **packed), (0.8, False)),
        "everything in the suitcase": (
            run(**allowed_in, power_bank=st("power_bank", 0.30, -0.35, CASE_FLOOR),
                camera_battery=st("camera_battery", 0.30, -0.40, CASE_FLOOR)), (0.0, False)),
        "the power bank on the clothes in the suitcase": (
            run(**{**packed, "power_bank": st("power_bank", 0.50, -0.30, CASE_FLOOR + CLOTHES_H)}), (0.0, False)),
        "the camera battery in the open lid half": (
            run(**{**packed, "camera_battery": st("camera_battery", CASE_XY[0], _HINGE_Y - 0.06, TOP - 0.10)}),
            (0.0, False)),
        "the power bank put in the suitcase, then moved to the backpack": (
            run(history=[{"power_bank": st("power_bank", 0.24, -0.3, CASE_FLOOR)}], **packed), (0.0, False)),
        "the power bank carried over the suitcase, never below its rim": (
            run(history=[{"power_bank": st("power_bank", 0.24, -0.3, CASE_RIM + 0.01)}], **packed), (1.0, True)),
        "batteries left on the table, the rest packed": (run(**allowed_in), (0.6, False)),
        "everything in the backpack": (
            run(**batteries_in, ssd=st("ssd", 0.40, 0.33, BAG_FLOOR), cards=st("cards", 0.40, 0.44, BAG_FLOOR),
                adapter=st("adapter", 0.30, 0.38, BAG_FLOOR)), (0.4, False)),
        "the SSD still sliding in the suitcase": (run(**{**packed, "ssd": st("ssd", *PLACES["ssd"], CASE_FLOOR, vel=0.05)}),
                                                  (0.8, False)),
        "the cards resting on the suitcase's near wall, half out": (
            run(**{**packed, "cards": st("cards", CASE_X0 - 0.01, PLACES["cards"][1], CASE_RIM)}), (0.8, False)),
        "the SSD on the clothes, below the rim": (
            run(**{**packed, "ssd": st("ssd", 0.50, -0.45, CASE_FLOOR + CLOTHES_H)}), (1.0, True)),
        "the adapter on the clothes, sticking out above the rim": (
            run(**{**packed, "adapter": st("adapter", 0.50, -0.45, CASE_FLOOR + CLOTHES_H)}), (0.8, False)),
        "the SSD standing on edge in the suitcase": (
            run(**{**packed, "ssd": st("ssd", *PLACES["ssd"], CASE_FLOOR, roll=np.pi / 2)}), (1.0, True)),
        "the power bank on the backpack's front panel, outside the compartment": (
            run(**{**packed, "power_bank": st("power_bank", BAG_XY[0] + BAG_OUT[0] * 0.8, BAG_XY[1], TOP + 20 * MM)}),
            (0.8, False)),
    }
    for name, (got, want) in cases.items():
        assert abs(got[0] - want[0]) < 1e-9 and got[1] == want[1], (name, got, want)
    _STATE.update(free={n: True for n in ITEMS}, battery_in_case=False)

    # the layout: each item's open fingers (FINGER_W along it, FINGER_T thick, OPEN_GAP from its sides) clear every
    # other item and the suitcase and backpack walls by 5 mm; places clear of the clothes and inside the walls
    def rect(n):
        (x, y), (sx, sy) = START[n], ITEMS[n][0][:2]
        return np.array([x - sx / 2, y - sy / 2]), np.array([x + sx / 2, y + sy / 2])

    def fingers(n):
        (x, y), sy = START[n], ITEMS[n][0][1]
        inner = sy / 2 + OPEN_GAP
        return [(np.array([x - FINGER_W / 2, y - inner - FINGER_T]), np.array([x + FINGER_W / 2, y - inner])),
                (np.array([x - FINGER_W / 2, y + inner]), np.array([x + FINGER_W / 2, y + inner + FINGER_T]))]

    def gap(a, b):
        return float(np.max(np.maximum(a[0] - b[1], b[0] - a[1])))

    walls = [(np.array([0.0, -1.0]), np.array([1.0, CASE_OUT_Y1])),
             (np.array([0.0, BAG_XY[1] - BAG_OUT[1] / 2]), np.array([1.0, 1.0]))]
    for a in ITEMS:
        for b in ITEMS:
            if a != b:
                assert gap(rect(a), rect(b)) >= 0.01, (a, b)
                for f in fingers(a):
                    assert gap(f, rect(b)) >= 0.005, (a, "fingers", b, gap(f, rect(b)))
        (x, y) = START[a]
        palm = (np.array([x - 0.03, y - PALM_W / 2]), np.array([x + 0.03, y + PALM_W / 2]))
        for w in walls:
            assert gap(rect(a), w) >= 0.005, (a, "wall")
            assert gap(palm, w) >= 0.02, (a, "palm at a wall", gap(palm, w))  # the palm comes down below the rims
            for f in fingers(a):
                assert gap(f, w) >= 0.005, (a, "fingers at a wall", gap(f, w))
        assert ITEMS[a][0][1] + 2 * OPEN_GAP <= 0.094, a  # within the gripper's 94 mm opening
        rim = CASE_RIM if a in ALLOWED else BAG_RIM
        assert HOVER - (grip_z(a) - TOP) - 0.5 * MM > rim + 0.01, a  # carried over its destination's rim
    for n in ITEMS:  # the palm clears the destination's walls by 15 mm when it lets go (it is then below the rims)
        cy_, inner = (CASE_XY[1], CASE_IN[1]) if n in ALLOWED else (BAG_XY[1], BAG_IN[1])
        assert abs(PLACES[n][1] - cy_) + PALM_W / 2 <= inner / 2 - 0.015, (n, "palm at a wall when placing")
    for n in ALLOWED:
        assert _over(corners(packed[n], n), CASE_XY, CASE_IN), n
        assert PLACES[n][0] + ITEMS[n][0][0] / 2 < CLOTHES_X[0] - 0.01, n
    for n in BATTERIES:
        assert _over(corners(packed[n], n), BAG_XY, BAG_IN), n
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
