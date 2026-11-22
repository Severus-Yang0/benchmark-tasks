"""Lock a castle nut with a cotter pin: push the pin through the nut's slot and the stud's cross-hole, then spread its
two prongs apart round the nut with a flat-head screwdriver.

On a steel stand on the station table is a car's steering joint as installed with the wheel off: the knuckle body with its brake
dust shield beyond it, the steering arm coming off it, and through the arm an upright 3/8-24 tie-rod-end stud (the
ball housing and its rubber boot under the arm, the tie rod running off to one side) with a slotted hex nut on top,
one slot lined up over the stud's cross-hole and running along x (front to back). A new standard 3/32 x 1 in steel
cotter pin (ASME B18.8.1, its half-round prongs together, the longer one running on 4 mm) lies flat in a groove
along the top of an aluminium block in front of the right arm, prongs towards the robot, its eye sticking out past
the block's far end; a slim flat-head screwdriver stands in a rack in front of the left arm.
Mechanics finish a castle nut on a steering linkage this way: a new cotter pin through the slot and the hole, its
prongs bent apart round the nut so it can't come out (Suzuki RA410 service manual: "bend new
split pin"; Meritor: tighten to the next slot, never back the nut off).

What makes it hard: the pin (2.2 mm across) has to go in square through a 3.0 mm hole hidden behind the slot, between
the arm and the parts round it; its two prongs, 1.1 mm thick each, lie together, so one has to be caught by the end
the longer one runs on past the other, and each takes about 12 N, 10 mm from its root, to bend (cotterkit.py); a
parallel gripper can't take hold of one, so a screwdriver's blade pushes each round. Meanwhile the pin's head has
to be held home.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import cotterkit  # noqa: E402
import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Lock the castle nut with the cotter pin: push it through the nut's slot and the stud's hole, then spread its prongs apart."

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- sizes (assets/fixture.usda, assets/cotter.usda) ---------------------------------------------------------------
NUT_AF, NUT_H = 14.29 * MM, 8.33 * MM  # 3/8 in slotted hex nut, flats facing +-x
SLOT_W, SLOT_D = 3.2 * MM, 3.2 * MM
STUD_R = 9.53 * MM / 2
HOLE = 3.0 * MM
ARM_TOP = 0.040
NUT_TOP = ARM_TOP + NUT_H
SLOT_FLOOR = NUT_TOP - SLOT_D
HOLE_Z = SLOT_FLOOR + HOLE / 2  # above the table
HOME_X = NUT_AF / 2  # the prongs' roots (the cotter's origin) with its head against the near flat
PRONG_T, PRONG_W = 1.1 * MM, 2.2 * MM  # each half-round prong's thickness (y, across the split) and width (z)
PRONG_GAP = 0.0  # the prongs lie flat face to flat face, as a new pin's do
PRONG_YC = PRONG_GAP / 2 + PRONG_T / 2  # each prong's centre line from the pin's axis
PIN_W = PRONG_GAP + 2 * PRONG_T  # the pin's width: 2.2 mm
INSIDE = NUT_AF  # the shank from under the head to the prongs' roots
EYE_D = 6.0 * MM  # the eye lies flat: this long (x) and wide (y), 3.5 mm thick
PRONG_L = 25.4 * MM - INSIDE  # the short prong beyond its root (in the file +y, in the scene -y)...
EXTRA = 4.0 * MM  # ... the long one runs on this much further

# -- the scene --------------------------------------------------------------------------------------------------
NUT_XY = (0.300, 0.0)
NX, NY = NUT_XY
RAISE = 50.0 * MM  # the steering joint stands on a steel stand this tall
BASE = TOP + RAISE  # the fixture's floor (the stand's top)
PIN_DIR = -1.0  # the pin goes in pointing along -x (in from the far side, prongs towards the robot)
TURNED = (0.0, 0.0, 1.0, 0.0)  # the fixture and the pin, turned half round about z from their files (both along +x)
FIXTURE = UsdBody("fixture", usd="assets/fixture.usda", pos=(NX, NY, BASE), rot_xyzw=TURNED, size=(0.08, 0.04, 0.05),
                  mass=None, material=False)
COTTER_XY = (0.300, -0.180)  # its origin (the prongs' roots); it lies flat, pointing along -x, head towards +x

# the new pin waits in a groove along the top of a 20 x 20 x 60 mm aluminium block, its eye sticking out past the
# block's end, so a hand can close round the eye from above
BLOCK_H, BLOCK_W, BLOCK_L = 0.020, 0.020, 0.060
GROOVE_W, GROOVE_D = 3.0 * MM, 1.0 * MM
BLOCK_END = COTTER_XY[0] - PIN_DIR * 0.006  # the block's far (+x) end: the eye and 8.3 mm of shank stick out past it
ALU = (0.75, 0.76, 0.78)
_bx = BLOCK_END + PIN_DIR * BLOCK_L / 2
_rail = (BLOCK_W - GROOVE_W) / 2
BLOCK = [
    Shape("block_base", pos=(_bx, COTTER_XY[1], TOP + (BLOCK_H - GROOVE_D) / 2), size=(BLOCK_L, BLOCK_W, BLOCK_H - GROOVE_D),
          color=ALU),
    *(Shape(f"block_rail{i}", pos=(_bx, COTTER_XY[1] + s * (GROOVE_W + _rail) / 2, TOP + BLOCK_H - GROOVE_D / 2),
            size=(BLOCK_L, _rail, GROOVE_D), color=ALU) for i, s in enumerate((-1, 1))),
]
PIN_FLAT = 0.6  # lying flat, the pin's centre line sits this share of its half-height (1.1 mm) above what it rests on
COTTER_START = (COTTER_XY[0], COTTER_XY[1], TOP + BLOCK_H - GROOVE_D + PIN_FLAT * PRONG_W / 2 + 0.00005)  # in the groove
COTTER = UsdBody("cotter", usd="assets/cotter.usda", pos=COTTER_START, rot_xyzw=TURNED, size=(0.03, 0.005, 0.002),
                 articulation=True, material=False, solver_iters=64, joint_stiffness=cotterkit.K,
                 joint_damping=cotterkit.DAMPING, joint_pos=tuple((j, 0.0) for j in cotterkit.JOINTS))

# the screwdriver (assets/screwdriver.usda: a slim precision flat-head, 120 mm overall: a 40 mm blade, as thin as its
# 2.5 x 0.4 mm edge for 8 mm, then tapering to 4 x 3 mm, then an 18 mm square handle 80 mm long; origin at the edge's
# middle, z along it), standing tip down in a rack (a 30 mm block with a 4 mm square hole) in front of the left arm,
# its edge along x
GREY = (0.35, 0.36, 0.38)
TOOL_MASS = 0.03
RACK_FLOOR, RACK_H, RACK_HOLE, RACK_W = 0.003, 0.025, 0.004, 0.030
RACK_XY = (0.220, 0.180)
TOOL_START = (RACK_XY[0], RACK_XY[1], TOP + RACK_FLOOR + 0.0003)
_side = (RACK_W - RACK_HOLE) / 2
RACK = [
    Shape("rack_floor", pos=(*RACK_XY, TOP + RACK_FLOOR / 2), size=(RACK_W, RACK_W, RACK_FLOOR), color=GREY),
    *(Shape(f"rack_wall{i}", pos=(RACK_XY[0] + sx * (RACK_HOLE + _side) / 2, RACK_XY[1] + sy * (RACK_HOLE + _side) / 2,
                                  TOP + RACK_FLOOR + RACK_H / 2),
            size=(_side if sx else RACK_W, _side if sy else RACK_HOLE, RACK_H), color=GREY)
      for i, (sx, sy) in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1)))),
]
TOOL = UsdBody("screwdriver", usd="assets/screwdriver.usda", pos=TOOL_START, size=(0.018, 0.018, 0.12),
               mass=TOOL_MASS, material=False, torsional_patch=0.009)

# the steel stand under the steering joint (x from 2.5 cm in front of the stud to past the dust shield)
STAND_X, STAND_Y = (NX - 0.025, NX + 0.140), (-0.085, 0.115)
STAND = Shape("stand", pos=((STAND_X[0] + STAND_X[1]) / 2, (STAND_Y[0] + STAND_Y[1]) / 2, TOP + RAISE / 2),
              size=(STAND_X[1] - STAND_X[0], STAND_Y[1] - STAND_Y[0], RAISE), color=GREY)

OBJECTS = [FIXTURE, *BLOCK, COTTER, TOOL, *RACK, STAND]

# -- scoring ----------------------------------------------------------------------------------------------------
# the prongs' spread, from an overhead photo: each prong's angle outwards from the pin's axis, and the angle between them
LOCKED = math.radians(90.0)  # the pin counts as locked with its prongs at least this far apart...
EACH = math.radians(30.0)  # ... each of them bent at least this far out (both spread, not one)
PART = math.radians(45.0)  # partly spread: at least this far apart
HOME_GAP = 1.0 * MM  # the head counts as home within this of the nut's near flat (a 1 mm feeler gauge won't go in)
AXIS_TOL = 1.5 * MM  # the pin's axis within this of the hole's (it is in the hole: it can't be further)
TOOL_CLEAR = 0.020  # at the end the screwdriver's tip at least this far from the prongs' roots
_STATE: dict = {"plastic": None, "free": True, "t": 0.0}


def root_rel(c) -> np.ndarray:
    """The cotter's origin (its prongs' roots) relative to the stud's axis at the cross-hole's height: along the way
    the pin goes in (PIN_DIR * x), y, z."""
    r = np.asarray(c.pos) - np.array([NX, NY, BASE + HOLE_Z])
    return np.array([PIN_DIR * r[0], r[1], r[2]])


def through(c) -> bool:
    """The pin passes through the stud's cross-hole: pointing the way it goes in (-x), on the hole's axis, its shank
    spanning the stud (prong roots beyond the stud's robot side, the head on the far side)."""
    r = root_rel(c)
    along = c.rotation()[:, 0]
    return bool(PIN_DIR * along[0] > math.cos(math.radians(15)) and abs(r[1]) < AXIS_TOL and abs(r[2]) < AXIS_TOL
                and STUD_R < r[0] < INSIDE - STUD_R)


def spread(c) -> tuple[float, float]:
    """Each prong's bend outwards (rad): the -y prong's (prong_p, the short one), the +y prong's (prong_n, the long one)."""
    return tuple(cotterkit.outward(j, c.joints.get(j, 0.0)) for j in cotterkit.JOINTS)


def score(objects) -> float:
    """0: the pin is not through the stud's cross-hole.
    0.25: through the hole; its prongs less than 45 degrees apart.
    0.5: the prongs at least 45 degrees apart, but less than 90, or one of them bent out less than 30 degrees.
    0.75: the prongs at least 90 degrees apart, each bent out at least 30, but the head more than 1 mm from the nut,
    or the robot touching the pin, or the screwdriver's tip within 20 mm of the prongs' roots.
    1.0: spread so, the head within 1 mm of the nut, the robot off the pin and the screwdriver away."""
    c = objects["cotter"]
    if not through(c):
        return 0.0
    p, n = spread(c)
    if p + n < PART:
        return 0.25
    if p + n < LOCKED or min(p, n) < EACH:
        return 0.5
    home = HOME_X - root_rel(c)[0] < HOME_GAP  # the eye can't go past home: it is wider than the slot
    tool = objects.get("screwdriver")
    clear = tool is None or float(np.linalg.norm(np.asarray(tool.pos) - np.asarray(c.pos))) > TOOL_CLEAR
    return 1.0 if home and clear and _STATE["free"] else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


@on_reset
def _reset(environment) -> None:
    cotter = environment.unwrapped.scene["cotter"]
    if _STATE["plastic"] is None or _STATE["plastic"].cotter is not cotter:
        _STATE["plastic"] = cotterkit.Plastic(cotter)
    else:
        _STATE["plastic"].reset()
    touch.reset()
    _STATE.update(free=True, t=0.0)


@on_step
def _watch(environment) -> None:
    """Released: no robot link touches the pin's body or either prong (PhysX contacts, touch.py)."""
    t = touch.touching(environment, ["cotter", "cotter/prong_p", "cotter/prong_n"])
    _STATE["free"] = not any(t.values())
    _STATE["t"] += environment.unwrapped.step_dt


def self_test(ObjectState=None):
    """The scoring rules and the plastic rule on hand-built states."""
    from scenekit import BodyState

    def pin(dx=HOME_X, dy=0.0, dz=0.0, yaw=0.0, p=0.0, n=0.0):
        pos = np.array([NX + PIN_DIR * dx, NY + dy, BASE + HOLE_Z + dz])
        q = np.array([0.0, 0.0, math.sin((math.pi + yaw) / 2), math.cos((math.pi + yaw) / 2)])
        return BodyState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos,
                         joints={"prong_p_joint": p, "prong_n_joint": -n})

    def tool(dist):
        pos = np.array([NX + PIN_DIR * (HOME_X + dist), NY, BASE + HOLE_Z])
        return BodyState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)

    d = math.radians
    far = tool(0.10)
    cases = {
        "on the block": ({"cotter": pin(dx=-0.07, dz=-0.02), "screwdriver": far}, True, 0.0),
        "in the slot short of the stud": ({"cotter": pin(dx=STUD_R - 1 * MM), "screwdriver": far}, True, 0.0),
        "through, prongs straight": ({"cotter": pin(), "screwdriver": far}, True, 0.25),
        "through, prongs opened 20 degrees": ({"cotter": pin(p=d(20), n=d(20)), "screwdriver": far}, True, 0.25),
        "through, one prong spread": ({"cotter": pin(p=d(75)), "screwdriver": far}, True, 0.5),
        "one prong bent 90, the other straight": ({"cotter": pin(p=d(90)), "screwdriver": far}, True, 0.5),
        "spread 85 degrees apart": ({"cotter": pin(p=d(50), n=d(35)), "screwdriver": far}, True, 0.5),
        "spread 99 degrees apart (64 and 35)": ({"cotter": pin(p=d(64), n=d(35)), "screwdriver": far}, True, 1.0),
        "spread 95 degrees apart, one only 25": ({"cotter": pin(p=d(70), n=d(25)), "screwdriver": far}, True, 0.5),
        "through, both spread, released": ({"cotter": pin(p=d(75), n=d(70)), "screwdriver": far}, True, 1.0),
        "both spread, still held": ({"cotter": pin(p=d(75), n=d(70)), "screwdriver": far}, False, 0.75),
        "both spread, head 1.5 mm out": ({"cotter": pin(dx=HOME_X - 1.5 * MM, p=d(75), n=d(70)), "screwdriver": far}, True, 0.75),
        "both spread, screwdriver left between them": ({"cotter": pin(p=d(75), n=d(70)), "screwdriver": tool(0.008)}, True, 0.75),
        "bent into each other": ({"cotter": pin(p=-d(70), n=-d(70)), "screwdriver": far}, True, 0.25),
        "turned across the slot": ({"cotter": pin(yaw=d(30), p=d(75), n=d(70)), "screwdriver": far}, True, 0.0),
        "pulled out after bending": ({"cotter": pin(dx=INSIDE + 1 * MM, p=d(75), n=d(70)), "screwdriver": far}, True, 0.0),
    }
    for name, (objs, free, want) in cases.items():
        _STATE["free"] = free
        got = score(objs)
        assert abs(got - want) < 1e-9, (name, got, want)
    _STATE["free"] = True
    # the plastic rule: below yield a prong springs back; past it, it keeps the bend less the springback; it never
    # sets inwards past INWARD
    j = "prong_p_joint"
    s = cotterkit.plastic_step(0.0, cotterkit.SPRINGBACK * 0.9, j)
    assert s == 0.0, s
    for a in np.linspace(0.0, d(80), 200):
        s = cotterkit.plastic_step(s, a, j)
    assert abs(s - (d(80) - cotterkit.SPRINGBACK)) < 1e-9, s
    s = 0.0
    for a in np.linspace(0.0, -d(40), 200):
        s = cotterkit.plastic_step(s, a, j)
    assert abs(s + cotterkit.INWARD) < 1e-9, s
    # sizes against each other: the pin fits the hole, the slot and the groove; the prongs reach past the nut; the
    # force is within the arm's push
    assert PIN_W < SLOT_W and PRONG_W < HOLE and PIN_W < HOLE and PIN_W < GROOVE_W
    assert cotterkit.MY / (8 * MM) < 20.0, cotterkit.MY / (8 * MM)
    return len(cases) + 3


if __name__ == "__main__":
    # Offline check, no GPU: the scoring rules on hand-built states.
    print(f"self-test: {self_test()} cases pass")
