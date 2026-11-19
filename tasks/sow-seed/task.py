"""Sow a bean seed in a pot of loose potting mix, one inch (2.5 cm) deep, and leave it covered.

On the station table, in front of the right arm, stands a black square plastic nursery pot, 3.5 in (89 mm) across
and 3.5 in (89 mm) tall (as Greenhouse Megastore's 3.5" square form pot), fixed in place, filled with loose potting mix
to 10 mm below its rim. One red kidney bean (16.5 x 8.5 x 5.0 mm, 0.6 g) lies flat on the table beside it. Bean
seeds are sown one inch deep (University of Illinois Extension, "Snap Beans": "Plant seeds of all varieties one
inch deep.").

What makes it hard: the seed has to go 2.5 cm into soil that gives way. Pushed down with a fingertip, the small
smooth seed slips out from under it unless the push stays square on it; a hole dug first fills back in as soon as the
fingers leave (loose mix holds no wall), so the seed has to go in while the hole is open, or be pushed in.

Simulation: the mix is a few thousand rigid spheres (soilkit), the seed one rigid body (its convex hull), the pot
four walls and a floor (fixed boxes). Real seed sizes and mass (Aviara and Ma'aji 2023; USDA GRIN PI 527771).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step, robot_links  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import soilkit  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Sow the bean in the middle of the pot, one inch deep, and leave it covered with soil."

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- the pot: 3.5 in square, 3.5 in tall, 1.5 mm walls, a 2 mm floor; fixed. Straight walls (the real one tapers
# a little towards its base).
POT_XY = (0.360, -0.120)
PX, PY = POT_XY
POT_OUT, POT_H, WALL_T, FLOOR_T = 89 * MM, 89 * MM, 1.5 * MM, 2 * MM
POT_IN = POT_OUT - 2 * WALL_T  # 86 mm
POT_COLOR = (0.10, 0.10, 0.10)
_o = POT_IN / 2 + WALL_T / 2
POT = [
    Shape("pot_floor", pos=(PX, PY, TOP + FLOOR_T / 2), size=(POT_OUT, POT_OUT, FLOOR_T), color=POT_COLOR),
    Shape("pot_wall_far", pos=(PX + _o, PY, TOP + POT_H / 2), size=(WALL_T, POT_OUT, POT_H), color=POT_COLOR),
    Shape("pot_wall_near", pos=(PX - _o, PY, TOP + POT_H / 2), size=(WALL_T, POT_OUT, POT_H), color=POT_COLOR),
    Shape("pot_wall_left", pos=(PX, PY + _o, TOP + POT_H / 2), size=(POT_IN, WALL_T, POT_H), color=POT_COLOR),
    Shape("pot_wall_right", pos=(PX, PY - _o, TOP + POT_H / 2), size=(POT_IN, WALL_T, POT_H), color=POT_COLOR),
]
RIM = TOP + POT_H

# -- the potting mix: loose, levelled 10 mm below the rim. Grains 3 to 5 mm across. Bulk density of peat-based mixes
# 0.09-0.16 g/cm^3 (Greenhouse Management Online, Unit 7): with the grains packed about 60 %, a grain is 250 kg/m^3.
LEVEL = RIM - 10 * MM  # the soil's starting surface; depth is measured from here
R_MIN, R_MAX = 0.0015, 0.0025
# the loose lattice it starts from (before work/sow-seed/prepare_soil.sh settles and levels it) stands 2.0 times as
# tall as the settled soil, above the rim
_FILL = 2.0 * (LEVEL - TOP - FLOOR_T)
SOIL = soilkit.Soil("soil", pos=(PX, PY, TOP + FLOOR_T + _FILL / 2), size=(POT_IN - 1 * MM, POT_IN - 1 * MM, _FILL),
                    r_min=R_MIN, r_max=R_MAX, density=float(os.environ.get("SOIL_DENSITY", "250")),
                    friction=(1.0, 0.9), angular_damping=8.0,
                    start_file=os.environ.get("SOIL_START", "assets/soil_start.npz"))

# -- the seed: one red kidney bean lying flat on the table, its length along x
SEED_L, SEED_W, SEED_T, SEED_MASS = 16.5 * MM, 8.5 * MM, 5.0 * MM, 0.0006
SEED_XY = (0.300, -0.220)
SEED = UsdBody("seed", usd="assets/seed.usdc", pos=(SEED_XY[0], SEED_XY[1], TOP + SEED_T / 2),
               size=(SEED_L, SEED_W, SEED_T), mass=SEED_MASS, friction=(0.5, 0.4), solver_iters=32)

OBJECTS = [*POT, SOIL, SEED]

# -- scoring -------------------------------------------------------------------------------------------------------
DEPTH_MIN, DEPTH_MAX = 15 * MM, 35 * MM  # one inch, give or take 1 cm: the seed's top this far below LEVEL
MIDDLE = 25 * MM  # the seed's centre at most this far from the pot's centre (seen from above)
SPILL_MAX = 0.10  # at most this fraction of the soil's mass may end up outside the pot
CLEAR = 10 * MM  # at the end no fingertip (nor any other link) is over the pot's opening lower than this above its rim
FINGER_REACH = 88 * MM  # the fingertips' ends below the tip links' origins (measured in the demo: tip origins 137 mm
# above the table while the fingertips pressed the seed's top at 49 mm)
_STATE: dict = {"clear": True, "t": 0.0, "last": -1.0}


def _axes_half(seed) -> np.ndarray:
    return np.abs(seed.rotation()) @ np.array([SEED_L, SEED_W, SEED_T]) / 2  # half extents along x, y, z


def seed_top(seed) -> float:
    return float(np.asarray(seed.pos)[2] + _axes_half(seed)[2])


def _seed_points(seed) -> np.ndarray:
    """Points over the seed's upper side, seen from above: a 5 x 3 grid over its footprint, at 80 % of its height."""
    R = seed.rotation()
    pts = np.array([[u * SEED_L, v * SEED_W, 0.0] for u in np.linspace(-0.4, 0.4, 5)
                    for v in np.linspace(-0.35, 0.35, 3)])
    p = np.asarray(seed.pos) + pts @ R.T
    p[:, 2] = np.asarray(seed.pos)[2] + 0.8 * _axes_half(seed)[2]
    return p


def covered(seed, grains=None) -> bool:
    """Not visible from above: over each point of the seed's upper side lies a grain (a grain whose centre is higher
    than that point and within its own radius of it, seen from above)."""
    p, r = grains if grains is not None else soilkit.grains("soil")
    for q in _seed_points(seed):
        if not ((p[:, 2] > q[2]) & (np.linalg.norm(p[:, :2] - q[:2], axis=1) < r)).any():
            return False
    return True


def in_pot(xy) -> bool:
    return bool(np.all(np.abs(np.asarray(xy)[:2] - np.asarray(POT_XY)) < POT_IN / 2))


def spilled(grains=None) -> float:
    """The fraction of the soil's mass outside the pot's walls (seen from above)."""
    p, r = grains if grains is not None else soilkit.grains("soil")
    out = np.any(np.abs(p[:, :2] - np.asarray(POT_XY)) > POT_IN / 2, axis=1)
    m = r ** 3
    return float(m[out].sum() / m.sum())


def score(objects, grains=None) -> float:
    """0 if the seed is not in the pot (below its rim, within its walls), or over 10 % of the soil (by mass) is
    outside the pot.
    0.25: the seed in the pot, not covered (some of it visible from above).
    0.5: covered, but its top less than 15 or more than 35 mm below the soil's starting level, or its centre more
    than 25 mm from the pot's centre.
    0.75: covered, 15-35 mm deep, in the middle, but a fingertip still over the pot's opening lower than 10 mm
    above its rim. 1.0: the same with the arms clear."""
    seed = objects["seed"]
    if not in_pot(seed.pos) or seed_top(seed) > RIM or spilled(grains) > SPILL_MAX:
        return 0.0
    if not covered(seed, grains):
        return 0.25
    depth = LEVEL - seed_top(seed)
    middle = float(np.linalg.norm(np.asarray(seed.pos)[:2] - np.asarray(POT_XY))) <= MIDDLE
    if not (DEPTH_MIN <= depth <= DEPTH_MAX and middle):
        return 0.5
    return 1.0 if _STATE["clear"] else 0.75


def success(objects) -> bool:
    return score(objects) == 1.0


def fingertips(side: dict) -> np.ndarray:
    """Where one gripper's fingertips are: midway between its two tip links' origins (which sit 4.5 cm to either side,
    at the fingers' roots), FINGER_REACH lower."""
    return (side["tip_left"] + side["tip_right"]) / 2 - np.array([0.0, 0.0, FINGER_REACH])


def low_over_pot(side: dict) -> bool:
    """The gripper's fingertips, or another link's origin (but the base's), over the pot's opening lower than CLEAR
    above its rim."""
    points = [fingertips(side)] + [p for name, p in side.items() if name not in ("base", "tip_left", "tip_right")]
    return any(in_pot(p) and p[2] < RIM + CLEAR for p in points)


@on_reset
def _reset(environment) -> None:
    _STATE.update(clear=True, t=0.0, last=-1.0)


@on_step
def _watch(environment) -> None:
    """The arms are clear when no fingertip (nor other link origin, but the bases) is over the pot's opening lower
    than CLEAR above its rim."""
    links = robot_links(environment)
    _STATE["clear"] = not any(low_over_pot(side) for side in links.values())
    _STATE["t"] += environment.unwrapped.step_dt
    if os.environ.get("SOW_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
        _STATE["last"] = _STATE["t"]
        from yam_arena.scene import object_states

        seed = object_states(environment, ["seed"])["seed"]
        print(f"LOG t={_STATE['t']:.1f} seed_top={(seed_top(seed) - TOP) * 1000:.1f}mm "
              f"depth={(LEVEL - seed_top(seed)) * 1000:.1f}mm covered={covered(seed)} "
              f"off_centre={np.linalg.norm(np.asarray(seed.pos)[:2] - np.asarray(POT_XY)) * 1000:.1f}mm "
              f"spill={spilled() * 100:.1f}% clear={_STATE['clear']} score={score({'seed': seed})} "
              f"fingertips={(fingertips(links['right'])[2] - TOP) * 1000:.0f}mm "
              f"lifted={int((soilkit.grains('soil')[0][:, 2] > RIM + 5 * MM).sum())}",
              flush=True)


# -- the scripted demo: the right arm -----------------------------------------------------------------------------
HOVER = TOP + 0.150
TIP = float(os.environ.get("TIP", "0.0"))  # where the closed fingertips end, below the TCP (m)
LAY_DX = float(os.environ.get("LAY_DX", "0.009"))  # where the laid seed ends up from where it was let go (x, m)
PUSH = float(os.environ.get("PUSH", "0.020"))  # how far the TCP goes below the soil's level (the fingertips then press
# the seed's top about 30 mm down)


def demo():
    """Pick the seed by its middle (fingers closing across its width), lay it flat on the soil at the pot's centre,
    close the fingers and push it straight down with their tips, hold, pull them out: the loose mix falls back over
    the seed. Open the fingers just above the pot, so grains caught between them drop back in, then go back clear."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    sx, sy = SEED_XY
    sz = TOP + SEED_T / 2
    qx = PX + LAY_DX
    right = [
        W(2.0, (0.20, -0.30, HOVER), O, "wait for the soil", 0.0),
        W(1.5, (sx, sy, HOVER), -0.012, "over the seed", 0.0),
        W(1.5, (sx, sy, sz), -0.012, "down round it", 0.0),
        W(0.8, (sx, sy, sz), -0.002, "grip", 0.0),
        W(1.5, (sx, sy, HOVER), -0.002, "lift", 0.0),
        W(1.5, (PX, PY, HOVER), -0.002, "over the pot's centre", 0.0),
        W(1.5, (PX, PY, LEVEL + 0.010), -0.002, "down to the soil", 0.0),
        W(0.8, (PX, PY, LEVEL + 0.010), -0.012, "lay it down", 0.0),
        W(1.0, (qx, PY, LEVEL + 0.030), C, "up, fingers closed", 0.0),
        W(1.0, (qx, PY, LEVEL + TIP + 0.006), C, "onto the seed", 0.0),
        W(2.5, (qx, PY, LEVEL + TIP - PUSH), C, "push it down", 0.0),
        W(0.8, (qx, PY, LEVEL + TIP - PUSH), C, "hold", 0.0),
        W(1.2, (qx, PY, RIM + 0.040), C, "out", 0.0),
        W(0.6, (qx, PY, RIM + 0.040), -0.015, "open over the pot: grains caught between the fingers drop back", 0.0),
        W(1.0, (qx, PY, HOVER), -0.015, "up", 0.0),
        W(1.5, (0.20, -0.30, HOVER), O, "back clear", 0.0),
        W(3.0, (0.20, -0.30, HOVER), O, "wait", 0.0),
    ]
    t = sum(w.duration for w in right)
    return {"left": [W(t, (0.20, 0.30, HOVER), O, "wait", 0.0)], "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states."""
    from scenekit import BodyState

    def seed(x=PX, y=PY, top=LEVEL - 25 * MM, upright=False):
        q = np.array([0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4)]) if upright else np.array([0, 0, 0, 1.0])
        p = np.array([x, y, top - (SEED_L if upright else SEED_T) / 2])
        return BodyState(pos=p, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)

    def bed(hole=0.0, hole_floor=0.0, out=0):
        """2.2 mm grains on a 3 mm grid up to LEVEL; ``hole``: an empty shaft of that radius above ``hole_floor``;
        ``out``: that many grains moved outside the pot."""
        g = np.arange(-POT_IN / 2 + 2 * MM, POT_IN / 2 - 1 * MM, 3 * MM)
        zs = np.arange(TOP + FLOOR_T + 2 * MM, LEVEL - 1 * MM, 3 * MM)
        X, Y, Z = np.meshgrid(PX + g, PY + g, zs, indexing="ij")
        p = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
        if hole:
            p = p[~((np.linalg.norm(p[:, :2] - np.asarray(POT_XY), axis=1) < hole) & (p[:, 2] > hole_floor))]
        p[:out, 0] += POT_OUT
        return p, np.full(len(p), 2.2 * MM)

    full = bed()
    n = len(full[0])
    cases = {
        "seed on the table": (seed(*SEED_XY, top=TOP + SEED_T), full, True, 0.0),
        "seed lying on the soil": (seed(top=LEVEL + SEED_T), full, True, 0.25),
        "seed 25 mm down an open hole": (seed(), bed(hole=12 * MM, hole_floor=LEVEL - 40 * MM), True, 0.25),
        "seed 25 mm deep, covered, arms clear": (seed(), full, True, 1.0),
        "seed 25 mm deep, covered, a finger still in the pot": (seed(), full, False, 0.75),
        "seed 16 mm deep": (seed(top=LEVEL - 16 * MM), full, True, 1.0),
        "seed 10 mm deep": (seed(top=LEVEL - 10 * MM), full, True, 0.5),
        "seed 50 mm deep": (seed(top=LEVEL - 50 * MM), full, True, 0.5),
        "seed 25 mm deep by a wall": (seed(x=PX + 30 * MM), full, True, 0.5),
        "seed on end, its top 25 mm deep": (seed(upright=True), full, True, 1.0),
        "seed sown, 5 % of the soil spilled": (seed(), bed(out=n // 20), True, 1.0),
        "seed sown, 15 % of the soil spilled": (seed(), bed(out=n * 15 // 100), True, 0.0),
        "seed thrown out of the pot": (seed(x=PX + POT_OUT, top=TOP + SEED_T), full, True, 0.0),
    }
    for name, (s, grains, clear, want) in cases.items():
        _STATE["clear"] = clear
        got = score({"seed": s}, grains)
        assert abs(got - want) < 1e-9, (name, got, want)
    _STATE["clear"] = True

    def arm(x, y, tip_z):  # a straight-down gripper whose fingertips are at (x, y, tip_z)
        o = tip_z + FINGER_REACH
        return {"base": np.array([0.0, -0.3, TOP]), "link5": np.array([x, y, o + 0.12]),
                "tip_left": np.array([x, y + 0.045, o]), "tip_right": np.array([x, y - 0.045, o])}

    arms = {"fingertips pushing in the soil": (arm(PX, PY, LEVEL - 20 * MM), True),
            "fingertips 5 mm above the rim": (arm(PX, PY, RIM + 5 * MM), True),
            "fingertips 15 mm above the rim": (arm(PX, PY, RIM + 15 * MM), False),
            "fingertips low but beside the pot": (arm(PX, PY - 0.08, TOP + 0.02), False),
            "back at the side": (arm(0.20, -0.30, TOP + 0.15), False)}
    for name, (side, want) in arms.items():
        assert low_over_pot(side) == want, name
    return len(cases) + len(arms)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    p, r = soilkit.start_layout(SOIL)
    print(f"soil: {len(p)} grains, {np.sum(SOIL.density * 4 / 3 * np.pi * r ** 3) * 1000:.0f} g")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
