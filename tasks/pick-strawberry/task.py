"""On the station table, in front of the right arm, a strawberry plant grows in a black 6 in azalea pot (152 mm
across the top, 114 mm tall). One fruit truss rises from the crown among the leaf stalks, arches towards the robot
over the pot's near rim and hangs down beyond it; a ripe strawberry hangs from it on its own stalk (peduncle, 1.8 mm
thick), tip down, its centre 59 mm above the table and 35 mm beyond the rim. To the robot's right stands an empty
square molded-fiber quart berry basket (140 mm square, 83 mm tall, as Pactiv's 1 qt). The berry must be picked and
put in the basket.

How it comes off (published measurements):
- Pulled straight along the stalk, a berry needs 13.94 N to come off; bent against the stalk and pulled, 3.17 N
  (Dimeas et al. 2013, RAAD; quoted in Rajendran et al. 2022, arXiv 2207.12552). Pulled straight, the stalk can also
  break midway; tilted 90 degrees, the berry separates cleanly at its abscission layer with little force (Yamamoto et
  al. 2014, JARQ 48(3):261-269).
- Gripping the berry's body is safe below 10 N (Dimeas et al., same quote); harder, it bruises.
- With finger pads of friction 0.5, a grip under 10 N holds at most 10 N of pull: a straight pull slips. The berry
  comes off only if it is tilted against its stalk first.

Simulation: the berry is one rigid body (the YCB strawberry scan, 15 g); the truss and stalk one chain of 17
bending 1 cm links anchored at the crown, at rest along the truss's curve; the berry hangs from the chain's end on a
stiff spring (3000 N/m). From the first second on, every control step the spring's stretch gives the pull on the berry
and its angle to the berry's axis; once the pull reaches 13.94 N / (1 + 3.40 sin(angle)) (13.94 N straight, 4.10 N at
45 degrees, 3.17 N square), the spring is switched off and the berry is free. Pulled off within 45 degrees of
straight, the stalk is taken to have broken midway, leaving a piece on the berry (Yamamoto et al.: straight pulls
sometimes broke the stalk midway, 90-degree pulls rarely did): such a berry scores at most 0.5.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import stalkkit  # noqa: E402  (hooks yam_arena so the stalk below is simulated)
from scenekit import UsdBody, on_reset, on_step  # noqa: E402
from stalkkit import shaped  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Pick the strawberry and put it in the basket."

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- the plant (assets/plant.usdc, work/strawberry/make_plant.py): one fixed body, its origin at the pot's bottom
# centre. A black 6 in azalea pot (152 mm across the top, 114 mm at the bottom, 114 mm tall, a 4 mm lip; collider: the
# convex hull of its outside), soil 4 mm below the rim, a strawberry plant (seven trifoliate leaves and a flower on the
# far half; no colliders).
POT_XY = np.array([0.380, -0.100])
POT_W, POT_H = 152 * MM, 114 * MM
POT_R = POT_W / 2
STEM = (0.35, 0.55, 0.22)
PX, PY = POT_XY
PLANT = UsdBody("plant", usd="assets/plant.usdc", pos=(PX, PY, TOP), size=(POT_W, POT_W, POT_H), material=False)

# -- the fruit truss with the ripe berry: one bending chain anchored at the crown. At rest (unloaded) it rises among
# the leaf stalks, arches over the near rim on the pot's centre line and turns down 30 mm beyond it, its last 30 mm
# (the berry's own stalk, the peduncle) hanging nearly straight down to the calyx. Under the berry's weight it droops a
# little and settles before the arm reaches it. Points: (forward from the pot's centre, height above the table), mm.
TRUSS_REST = [(8, 118), (5, 138), (-10, 150), (-35, 152), (-60, 141), (-82, 128), (-102, 117), (-115, 103),
              (-119, 88)]
STALK_R = 1.0 * MM  # the truss about 2 mm thick, the peduncle 1.78 mm (Dimeas et al.)
BERRY_HALF = 18 * MM  # the berry's centre below its calyx
BERRY_R = 14 * MM
PULL_STRAIGHT, PULL_BENT = 13.94, 3.17  # N (Dimeas et al.): along the stalk, and bent against it
WELD_K = 3000.0  # N/m, the berry-to-stalk spring whose stretch gives the pull
STALK = shaped("stalk", [(PX + x * MM, PY, TOP + z * MM) for x, z in TRUSS_REST], radius=STALK_R, segment=10 * MM,
               color=STEM, cable_mass=0.012, bend_stiffness=1.5, bend_damping=0.02, armature=5e-5, max_bend=1.4,
               anchor_start=True, self_collision=False, weld_body="berry", weld_offset=BERRY_HALF, weld_k=WELD_K)
BERRY_SPAWN = STALK.end_tip() + STALK.end_direction() * BERRY_HALF  # where it is welded, before the truss droops
# where the berry hangs once the truss has settled under its weight (measured in the simulation: 8.7 mm out, 11.9 mm
# down)
BERRY_START = BERRY_SPAWN + np.array([0.0087, 0.0, -0.0119])
BERRY = UsdBody("berry", usd="assets/berry.usdc", pos=tuple(BERRY_SPAWN), size=(2 * BERRY_R, 2 * BERRY_R, 36 * MM),
                mass=0.015, friction=(0.5, 0.5), friction_min=True, compliance=(2000.0, 20.0), solver_iters=32,
                torsional_patch=4 * MM)

# -- the basket (assets/basket.usdc): a green molded-fiber quart berry basket (Pactiv 1 qt: 5.5 in square at the top,
# 4.25 in at the bottom, 3.25 in tall), fixed; colliders: its floor and four tilted walls
BASKET_XY = np.array([0.230, -0.250])
BASKET_OUT, BASKET_BOT, BASKET_H, WALL = 140 * MM, 108 * MM, 83 * MM, 3 * MM  # WALL: the floor's thickness
BASKET_IN = BASKET_BOT - 5 * MM  # inside, at the floor
BX, BY = BASKET_XY
BASKET = UsdBody("basket", usd="assets/basket.usdc", pos=(BX, BY, TOP), size=(BASKET_OUT, BASKET_OUT, BASKET_H),
                 material=False)

OBJECTS = [BERRY, STALK, PLANT, BASKET]

# -- the whole-trial watch: grip force and release ----------------------------------------------------------------
CRUSH = 10.0  # N, the safe grip on the berry's body (Dimeas et al.)
TORN_BELOW = math.radians(45)  # pulled off closer than this to straight: the stalk breaks midway (Yamamoto et al.)
SETTLE_S = 1.0  # the scene settles (the truss takes the berry's weight) before the detachment rule runs
_STATE: dict = {"t": 0.0, "free": True, "grip": 0.0, "max_grip": 0.0, "detached": False, "torn": False, "pull": 0.0,
                "angle": 0.0}


def pull_needed(angle: float) -> float:
    """The pull (N) that takes the berry off at ``angle`` (rad) between the pull and the berry's axis: 13.94 N straight
    (0), 3.17 N square to it (pi/2), falling smoothly between (Yamamoto et al.: less force the more it is tilted)."""
    return PULL_STRAIGHT / (1.0 + (PULL_STRAIGHT / PULL_BENT - 1.0) * math.sin(min(abs(angle), math.pi / 2)))


def pull_on_berry(berry, stalk) -> tuple[float, float]:
    """The stalk's pull on the berry (N, from the weld spring's stretch) and its angle (rad) to the berry's axis
    (calyx up: a pull straight away from the stalk is 0)."""
    d = np.asarray(berry.pos, float) - weld_point(stalk)
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        return 0.0, 0.0
    x, y, z, w = (float(v) for v in berry.quat_xyzw)
    axis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
    return WELD_K * n, math.acos(max(-1.0, min(1.0, float(np.dot(-d / n, axis)))))


@on_reset
def _reset(environment) -> None:
    touch.reset()
    _STATE.update(t=0.0, free=True, grip=0.0, max_grip=0.0, detached=False, torn=False, pull=0.0, angle=0.0)
    stalkkit.enable_weld(environment, "stalk")


@on_step
def _watch(environment) -> None:
    """Released: no robot link touches the berry. Grip: the largest force one robot link presses on it."""
    _STATE["free"] = not touch.touching(environment, ["berry"])["berry"]
    _STATE["grip"] = touch.grip_force(environment, ["berry"])["berry"]
    _STATE["max_grip"] = max(_STATE["max_grip"], _STATE["grip"])
    _STATE["t"] += environment.unwrapped.step_dt
    if _STATE["detached"] or _STATE["t"] < SETTLE_S:
        return
    from yam_arena.scene import object_states

    st = object_states(environment, ["berry", "stalk"])
    _STATE["pull"], _STATE["angle"] = pull_on_berry(st["berry"], st["stalk"])
    if _STATE["pull"] >= pull_needed(_STATE["angle"]):
        _STATE["detached"] = True
        _STATE["torn"] = _STATE["angle"] < TORN_BELOW
        print(f"[berry] off at {_STATE['pull']:.2f} N, {math.degrees(_STATE['angle']):.0f} deg"
              f"{', stalk torn midway' if _STATE['torn'] else ', clean at the calyx'}", flush=True)
        stalkkit.release_weld(environment, "stalk")


# -- scoring -----------------------------------------------------------------------------------------------------
REST_TOL = 4 * MM


def weld_point(stalk) -> np.ndarray:
    """Where the weld holds the berry's centre: beyond the stalk's last segment, along that segment's own axis."""
    axis = np.asarray(stalk.end_axis if getattr(stalk, "end_axis", None) is not None else stalk.end_dir)
    return np.asarray(stalk.points[-1]) + axis * (stalk.seg_len / 2 + BERRY_HALF)


def detached(objects) -> bool:
    """Off the stalk: the pull reached what its angle needs (checked every control step)."""
    return _STATE["detached"]


def in_basket(objects) -> bool:
    """The berry's centre inside the basket's walls seen from above (by at least half its radius), no higher above
    the floor than its centre stands when it sits upright (18 mm) plus REST_TOL, whichever way it lies (the basket is
    empty, so it is on the floor), and still."""
    b = objects["berry"]
    p = np.asarray(b.pos, float)
    inside = bool(np.all(np.abs(p[:2] - BASKET_XY) <= BASKET_IN / 2 - BERRY_R / 2))
    return inside and p[2] <= TOP + WALL + BERRY_HALF + REST_TOL and b.resting()


def score(objects) -> float:
    """0 for the whole trial once a robot link has pressed the berry with more than 10 N. Otherwise 0.5 once the
    berry is off its stalk, 0.75 when it rests in the basket, 1.0 when it rests there and no robot link touches it.
    At most 0.5 if it was pulled off within 45 degrees of straight: the stalk broke midway and a piece stays on the
    berry (on a real trial: a stalk piece longer than 10 mm above the calyx, measured on a photo)."""
    if _STATE["max_grip"] > CRUSH:
        return 0.0
    if not detached(objects):
        return 0.0
    if not in_basket(objects):
        return 0.5
    if _STATE["torn"]:
        return 0.5
    return 1.0 if _STATE["free"] else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo: the right arm -----------------------------------------------------------------------------
SQUEEZE = 6.0 * MM
GRIP_UP = 5 * MM  # the pads 5 mm above the berry's centre, on its wide shoulders (and the palm 5 mm higher)  # the close commanded this much narrower than the berry
HOVER = TOP + 0.150


def demo():
    """Down round the berry (fingers closing along y, the stalk between them), grip it gently, tilt it against its
    stalk by drawing it 60 mm towards the robot and 20 mm up (the stalk pulls across the berry's axis: it comes off
    at about 3.3 N), carry it over the basket, lower it in and let go."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    x, y, z = BERRY_START
    z += GRIP_UP
    grip = -(2 * BERRY_R - SQUEEZE) / 2
    near = -0.030  # pads 60 mm apart round the 28 mm berry
    right = [
        W(2.0, (x - 0.02, y, HOVER), near, "over the berry, clear of the pot", 0.0),
        W(1.5, (x, y, z), near, "down round it", 0.0),
        W(0.6, (x, y, z), near, "settle", 0.0),
        W(1.2, (x, y, z), grip, "grip gently", 0.0),
        W(2.5, (x - 0.060, y, z + 0.020), grip, "tilt it against the stalk and pull it off", 0.0),
        W(0.6, (x - 0.060, y, z + 0.020), grip, "settle", 0.0),
        W(2.0, (BX, BY, HOVER), grip, "carry over the basket", 0.0),
        W(1.2, (BX, BY, TOP + WALL + 0.030), grip, "lower it in", 0.0),
        W(1.0, (BX, BY, TOP + WALL + 0.030), near, "let go", 0.0),
        W(1.2, (BX, BY, HOVER), near, "up", 0.0),
        W(1.5, (0.20, -0.30, HOVER), O, "back to its side", 0.0),
    ]
    t = sum(w.duration for w in right)
    left = [W(t, (0.20, 0.30, HOVER), O, "wait", 0.0)]
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState
    from stalkkit import CableState

    top = BERRY_START + np.array([0.0, 0.0, BERRY_HALF + 0.04])  # a straight-down stalk ending at the berry's calyx
    pts = np.array([top - np.array([0.0, 0.0, (i + 0.5) * 0.01]) for i in range(4)])
    stalk = CableState(pos=pts.mean(0), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pts.min(0), high=pts.max(0), points=pts, start_dir=np.array([0, 0, 1.0]),
                       end_dir=np.array([0, 0, -1.0]), seg_len=0.01)

    def berry(p, vel=0.0, lying=False):
        q = np.array([math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)]) if lying else np.array([0, 0, 0, 1.0])
        p = np.asarray(p, float)
        return ObjectState(p, q, np.array([vel, 0, 0]), np.zeros(3), p - 0.018, p + 0.018)

    floor = TOP + WALL
    on_floor = (BX, BY, floor + BERRY_HALF)
    lying = (BX + 0.02, BY - 0.02, floor + BERRY_R)

    def run(p, free=True, grip=5.0, history=(), torn=False, **kw):
        _STATE.update(free=free, grip=grip, max_grip=grip, detached=bool(history) or
                      float(np.linalg.norm(np.asarray(p, float) - BERRY_START)) > 1e-6, torn=torn)
        for h in history:
            score({"berry": berry(h), "stalk": stalk})
        s = {"berry": berry(p, **kw), "stalk": stalk}
        return score(s), success(s)

    cases = {
        "start": (run(BERRY_START, grip=0.0), (0.0, False)),
        "held on the stalk": (run(BERRY_START, free=False), (0.0, False)),
        "pulled off, held in the air": (run(BERRY_START + np.array([-0.06, 0, 0.02]), free=False), (0.5, False)),
        "dropped on the table": (run((0.20, -0.18, TOP + BERRY_R), grip=0.0), (0.5, False)),
        "in the basket, still held": (run(on_floor, free=False), (0.75, False)),
        "in the basket, released, upright": (run(on_floor), (1.0, True)),
        "in the basket, released, lying on its side": (run(lying, lying=True), (1.0, True)),
        "in the basket, still rolling": (run(on_floor, vel=0.05), (0.5, False)),
        "held above the basket": (run((BX, BY, floor + 0.05)), (0.5, False)),
        "on the basket's rim": (run((BX + 0.06, BY, TOP + BASKET_H + BERRY_HALF)), (0.5, False)),
        "in the basket, but squeezed with 12 N on the way": (run(on_floor, grip=12.0), (0.0, False)),
        "squeezed with 9.9 N": (run(on_floor, grip=9.9), (1.0, True)),
        "pulled off straight (stalk torn), in the basket, released": (run(on_floor, torn=True), (0.5, False)),
        "pulled off straight (stalk torn), held in the air": (run(BERRY_START + np.array([0, 0, -0.04]), free=False,
                                                                 torn=True), (0.5, False)),
        "pulled off, then back where it hung": (run(BERRY_START, history=[(0.2, -0.1, TOP + 0.1)]), (0.5, False)),
    }
    for name, (got, want) in cases.items():
        assert abs(got[0] - want[0]) < 1e-9 and got[1] == want[1], (name, got, want)
    for deg, want in ((0, 13.94), (90, 3.17), (120, 3.17)):
        assert abs(pull_needed(math.radians(deg)) - want) < 1e-9, deg
    f, a = pull_on_berry(berry(BERRY_START - np.array([0, 0, 0.002])), stalk)
    assert abs(f - WELD_K * 0.002) < 1e-6 and a < 1e-6, (f, a)  # pulled straight down: along the axis
    f, a = pull_on_berry(berry(BERRY_START - np.array([0.001, 0, 0])), stalk)
    assert abs(a - math.pi / 2) < 1e-6, a  # pulled sideways: square to it
    _STATE.update(free=True, grip=0.0, max_grip=0.0, detached=False, torn=False)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    print(f"stalk {STALK.segments} links; off at {pull_needed(0):.2f} N straight, {pull_needed(math.pi / 4):.2f} N at "
          f"45 deg, {pull_needed(math.pi / 2):.2f} N square")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
