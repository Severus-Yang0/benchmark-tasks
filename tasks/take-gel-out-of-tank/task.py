"""Take a finished agarose gel out of its electrophoresis tank and put it in the staining tray, while the tank's power
supply is still running.

On the bench: a Bio-Rad Mini-Sub Cell GT horizontal gel tank (base 26 x 12 x 6.5 cm, Sub-Cell GT instruction manual,
section 1.4), its clear safety lid on, buffer over a 7 x 10 cm gel on its UV-transparent tray. The lid's two leads run
to a Bio-Rad PowerPac Basic power supply (25 x 21 x 8.5 cm, 1.1 kg, PowerPac Basic manual, specifications), which is
still running: its display reads 100 (V) and its run LED is lit. To the left, an empty polypropylene gel staining
tray (Flinn, 8-1/8 x 5-1/2 x 1-1/2 in). The lid carries the current to the electrodes; the manual: "always turn the
power supply off before removing the lid". The PowerPac's stop key (or run/pause, which pauses the output) does it.

Scene, scoring and scripted demo (see README.md).
Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import partkit  # noqa: E402,F401
from partkit import Slider, part  # noqa: E402
from scenekit import Look, Shape, on_reset, on_step  # noqa: E402

import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "The gel has finished running. Take it out of the tank and put it in the staining tray."

MM = 0.001
IN = 0.0254
ASSETS = pathlib.Path(__file__).resolve().parent / "assets"
TOP = TABLE_TOP[SCENE]

# -- the gel tank: fixed. Mini-Sub Cell GT base, 26 (y) x 12 (x) x 6.5 cm, walls 3/16 in (4.76 mm) acrylic (manual 1.2,
# 1.4). Inside: a buffer chamber at each end (the electrodes) and the gel stage between them. Floor and stage heights
# are estimates, set so that the buffer, 2 mm over the gel, is about 300 ml (manual: 265-320 ml).
TX, TY = 0.330, -0.020  # the base's centre
T_L, T_W, T_H = 0.260, 0.120, 0.065  # along y, along x, height
T_WALL = 4.76 * MM
T_IN_X, T_IN_Y = T_W / 2 - T_WALL, T_L / 2 - T_WALL  # inner half-widths
FLOOR_Z = TOP + 17 * MM  # the buffer chambers' floor
STAGE_L = 0.110  # the gel stage, along y (full inner width)
STAGE_TOP = TOP + 20 * MM
RIM = TOP + T_H
CLEAR = (0.75, 0.80, 0.82)  # clear acrylic (drawn by the Look; the colliders are hidden)

TANK = [
    Shape("tank_floor", pos=(TX, TY, (TOP + FLOOR_Z) / 2), size=(T_W, T_L, FLOOR_Z - TOP), color=CLEAR),
    Shape("tank_stage", pos=(TX, TY, (FLOOR_Z + STAGE_TOP) / 2), size=(2 * T_IN_X, STAGE_L, STAGE_TOP - FLOOR_Z),
          color=CLEAR),
    *[Shape(f"tank_wall_{n}", pos=(TX + s * (T_W / 2 - T_WALL / 2), TY, (TOP + RIM) / 2), size=(T_WALL, T_L, T_H),
            color=CLEAR) for n, s in (("near", -1), ("far", 1))],
    *[Shape(f"tank_wall_{n}", pos=(TX, TY + s * (T_L / 2 - T_WALL / 2), (TOP + RIM) / 2),
            size=(2 * T_IN_X, T_WALL, T_H), color=CLEAR) for n, s in (("right", -1), ("left", 1))],
]
BUFFER_TOP = STAGE_TOP + 3 * MM + 5 * MM + 2 * MM  # tray floor, gel, 2 mm of buffer over it

# -- the gel tray and the gel: loose. UVTP tray for a 7 x 10 cm gel (manual 1.4): open at both ends, a side wall
# along each long edge. Wall and floor thicknesses and the wall height are estimates; acrylic 1.18 g/cm3.
TRAY_FLOOR_T = 3 * MM
TRAY_WALL_T = 4 * MM
TRAY_WALL_H = 15 * MM  # above the tray's underside
GEL_W, GEL_L, GEL_T = 0.070, 0.100, 5 * MM
TRAY_OUT_X = GEL_W + 2 * TRAY_WALL_T  # 78 mm
TRAY_MASS = 1.18e3 * (TRAY_OUT_X * GEL_L * TRAY_FLOOR_T + 2 * TRAY_WALL_T * GEL_L * (TRAY_WALL_H - TRAY_FLOOR_T))
GEL_MASS = 1.0e3 * GEL_W * (GEL_L - 2 * MM) * GEL_T  # 1% agarose: about water's density
UVTP = (0.80, 0.84, 0.86)
TRAY = part("tray", (TX, TY, STAGE_TOP + 0.5 * MM), [
    ((0.0, 0.0, TRAY_FLOOR_T / 2), (TRAY_OUT_X, GEL_L, TRAY_FLOOR_T), UVTP),
    *[((s * (GEL_W / 2 + TRAY_WALL_T / 2), 0.0, TRAY_WALL_H / 2), (TRAY_WALL_T, GEL_L, TRAY_WALL_H), UVTP)
      for s in (-1, 1)],
], TRAY_MASS, friction=0.4, torsional_patch=5 * MM, visual=str(ASSETS / "tray.usda"))
GEL = part("gel", (TX, TY, STAGE_TOP + TRAY_FLOOR_T + 1.0 * MM), [
    ((0.0, 0.0, GEL_T / 2), (GEL_W - 1 * MM, GEL_L - 2 * MM, GEL_T), (0.62, 0.70, 0.78)),
], GEL_MASS, friction=0.3, visual=str(ASSETS / "gel.usda"))

# -- the safety lid: loose, seated on the base. Molded clear plastic (manual 1.4) round the buffer chamber, a plug
# housing at each end over the electrode's banana plug. Its thicknesses, skirt depth and housings are estimates;
# polycarbonate 1.2 g/cm3. The housings sit at opposite corners as in the manual's Fig. 1.1.
LID_T = 3 * MM
SKIRT_T, SKIRT_GAP, SKIRT_D = 2.5 * MM, 1.5 * MM, 15 * MM
LID_X = T_W + 2 * (SKIRT_GAP + SKIRT_T)  # 128 mm
LID_Y = T_L + 2 * (SKIRT_GAP + SKIRT_T)  # 268 mm
HOUSING = (24 * MM, 24 * MM, 30 * MM)
HOUSINGS = {"black": (+0.035, +0.115), "red": (-0.035, -0.115)}  # (x, y) from the lid's centre: cathode, anode
LID_SEAT_Z = RIM + LID_T / 2  # the plate's centre, seated
PLUG_ENGAGED = 12 * MM  # estimate: the banana plugs leave their sockets once the lid is this much higher
_lid_boxes = [
    ((0.0, 0.0, 0.0), (LID_X, LID_Y, LID_T), CLEAR),
    *[((s * (LID_X / 2 - SKIRT_T / 2), 0.0, -LID_T / 2 - SKIRT_D / 2), (SKIRT_T, LID_Y, SKIRT_D), CLEAR)
      for s in (-1, 1)],
    *[((0.0, s * (LID_Y / 2 - SKIRT_T / 2), -LID_T / 2 - SKIRT_D / 2), (LID_X - 2 * SKIRT_T, SKIRT_T, SKIRT_D), CLEAR)
      for s in (-1, 1)],
    *[((hx, hy, LID_T / 2 + HOUSING[2] / 2), HOUSING, (0.85, 0.86, 0.88)) for hx, hy in HOUSINGS.values()],
]
LID_MASS = 1.2e3 * sum(float(np.prod(b[1])) for b in _lid_boxes) * 0.85  # housings partly hollow
LID = part("lid", (TX, TY, LID_SEAT_Z), _lid_boxes, LID_MASS, friction=0.5, torsional_patch=6 * MM,
           visual=str(ASSETS / "lid.usda"))

# -- the power supply: loose, on its feet. PowerPac Basic, 25 (x) x 21 (y) x 8.5 cm, 1.1 kg (manual, specifications),
# its front panel towards the robot. Key, display and jack positions measured off the manual's Fig. 3 (front view):
# stop, run/pause and constant keys in a row 2.5 cm above the bottom; the display above them; two pairs of output
# jacks at each side. The membrane keys' travel (1 mm) and force (about 3 N at 0.6 mm) are estimates.
PP_SIZE = (0.250, 0.210, 0.085)
PP_C = (0.395, +0.360)
PP_FRONT_X = PP_C[0] - PP_SIZE[0] / 2  # 0.270
KEY = (2.0 * MM, 19 * MM, 13.5 * MM)  # proud of the panel, wide (y), tall (z)
KEY_Z = -PP_SIZE[2] / 2 + 25 * MM  # in the case's frame
KEYS = {"stop": +0.024, "run": -0.004, "constant": -0.026}  # y in the case's frame
KEY_TRAVEL = 1.0 * MM
KEY_ACTUATE = 0.6 * MM
KEY_RATE = 3.0 / KEY_ACTUATE
JACKS = {"black": (-0.075, +0.0125), "red": (-0.056, +0.0125)}  # (y, z) in the case's frame: the upper pair on its
# right (-y), towards the tank
PP_GREY, PP_GREEN = (0.86, 0.87, 0.86), (0.42, 0.62, 0.48)
_pp_boxes = [
    ((0.0, 0.0, -PP_SIZE[2] / 4), (PP_SIZE[0], PP_SIZE[1], PP_SIZE[2] / 2), PP_GREY),
    ((0.0, 0.0, PP_SIZE[2] / 4), (PP_SIZE[0], PP_SIZE[1], PP_SIZE[2] / 2), PP_GREEN),
    # the two lead plugs in their jacks (their bodies stand 22 mm out of the panel)
    *[((-PP_SIZE[0] / 2 - 11 * MM, y, z), (22 * MM, 9 * MM, 9 * MM), (0.1, 0.1, 0.1)) for y, z in JACKS.values()],
]
POWER = part("powerpac", (PP_C[0], PP_C[1], TOP + PP_SIZE[2] / 2 + 0.5 * MM), _pp_boxes, 1.1, friction=0.8,
             visual=str(ASSETS / "powerpac.usda"),
             sliders=[Slider(k, rest=(-PP_SIZE[0] / 2 - KEY[0] / 2, y, KEY_Z),  # its boxes in the case's frame
                             boxes=(((-PP_SIZE[0] / 2 - KEY[0] / 2, y, KEY_Z), KEY, (0.93, 0.93, 0.92)),),
                             travel=KEY_TRAVEL, rate=KEY_RATE,
                             damping=5.0, axis="X", sign=1) for k, y in KEYS.items()])

# -- the staining tray: fixed. Flinn polypropylene gel staining tray, 8-1/8 x 5-1/2 x 1-1/2 in (flinnsci.com), long
# side along y; walls and floor 2 mm (estimate).
DISH = (8.125 * IN, 5.5 * IN, 1.5 * IN)  # 206 x 140 x 38 mm: along y, along x, height
DISH_XY = np.array([0.200, -0.330])
DISH_T = 2 * MM
DISH_IN = (DISH[1] - 2 * DISH_T, DISH[0] - 2 * DISH_T)  # x, y
DISH_FLOOR = TOP + DISH_T
DISH_RIM = TOP + DISH[2]
DISH_BLACK = (0.10, 0.10, 0.11)
STAIN_TRAY = [
    Shape("dish_floor", pos=(*DISH_XY, TOP + DISH_T / 2), size=(DISH[1], DISH[0], DISH_T), color=DISH_BLACK),
    *[Shape(f"dish_wall_{n}", pos=(DISH_XY[0] + s * (DISH[1] - DISH_T) / 2, DISH_XY[1], TOP + DISH[2] / 2),
            size=(DISH_T, DISH[0], DISH[2]), color=DISH_BLACK) for n, s in (("near", -1), ("far", 1))],
    *[Shape(f"dish_wall_{n}", pos=(DISH_XY[0], DISH_XY[1] + s * (DISH[0] - DISH_T) / 2, TOP + DISH[2] / 2),
            size=(DISH[1] - 2 * DISH_T, DISH_T, DISH[2]), color=DISH_BLACK) for n, s in (("right", -1), ("left", 1))],
]

# the leads: drawn (no physics) from each lid housing's top to its jack, redrawn every step (see _leads)
LEADS = Look("leads", pos=(0.0, 0.0, 0.0), usd="assets/leads.usda")
LEAD_SEGS = 24

LOOKS = [Look(f"{n}_look", pos=(0.0, 0.0, 0.0), usd=f"assets/{n}.usda") for n in ("tank", "dish")]
OBJECTS = [*(replace(o, visible=False) for o in (*TANK, *STAIN_TRAY)), TRAY, GEL, LID, POWER, LEADS, *LOOKS]
LOOSE = ["tray", "gel", "lid"]

# -- the run, contacts and the whole-trial rules ----------------------------------------------------------------------
_STATE: dict = {}


def _fresh_state():
    return {"power_on": True, "lid_off_live": False, "free": {n: True for n in LOOSE}, "dropped": False}


_STATE.update(_fresh_state())


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _world(state, local) -> np.ndarray:
    return np.asarray(state.pos, float) + _rot(state.quat_xyzw) @ np.asarray(local, float)


def housing_tops(lid_state) -> dict:
    """Each plug housing's top centre in the world."""
    return {c: _world(lid_state, (hx, hy, LID_T / 2 + HOUSING[2])) for c, (hx, hy) in HOUSINGS.items()}


def jack_points(pp_state) -> dict:
    """Each lead's jack on the PowerPac's front, 1 cm out of the panel (where the plug's body ends)."""
    return {c: _world(pp_state, (-PP_SIZE[0] / 2 - 10 * MM, y, z)) for c, (y, z) in JACKS.items()}


def lid_unplugged(lid_state) -> bool:
    """Either banana plug out of its socket: that housing raised PLUG_ENGAGED or more above its seated height, or
    moved 1 cm sideways from it."""
    seated = {c: np.array([TX + hx, TY + hy, LID_SEAT_Z + LID_T / 2 + HOUSING[2]]) for c, (hx, hy) in HOUSINGS.items()}
    for c, p in housing_tops(lid_state).items():
        if p[2] - seated[c][2] >= PLUG_ENGAGED or np.linalg.norm(p[:2] - seated[c][:2]) >= 0.010:
            return True
    return False


def key_pressed(pp_state, key) -> bool:
    """The key pushed KEY_ACTUATE or more into the case (its slider moves +x, into the case)."""
    return pp_state.sliders.get(key, 0.0) >= KEY_ACTUATE


@on_reset
def _reset(environment) -> None:
    touch.reset()
    _STATE.update(_fresh_state())
    _LEAD_PRIMS.clear()
    _STATE.pop("panel_shown", None)
    _STATE.pop("stopped_by", None)


@on_step
def _watch(environment) -> None:
    """Contacts (released = no robot link touches the tray, the gel or the lid), the leads and the run LED."""
    t = touch.touching(environment, LOOSE)
    _STATE["free"] = {n: not v for n, v in t.items()}
    try:
        _draw(environment)
    except Exception as e:  # drawing must never stop a trial
        if not _STATE.get("draw_warned"):
            print(f"[gel] lead drawing failed: {e!r}", flush=True)
            _STATE["draw_warned"] = True


# -- drawing the leads and the run LED (USD, visual only) -------------------------------------------------------------
_LEAD_PRIMS: dict = {}


def _lead_curve(a, b, n) -> np.ndarray:
    """A lead from the housing top ``a`` (leaving straight up) to the jack ``b`` (leaving the panel towards -x): a
    cubic curve, kept on or above the table."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = float(np.linalg.norm(b - a))
    c1 = a + np.array([0.0, 0.0, 0.45 * d])
    c2 = b + np.array([-0.45 * d, 0.0, -0.10 * d])
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    p = (1 - t) ** 3 * a + 3 * (1 - t) ** 2 * t * c1 + 3 * (1 - t) * t ** 2 * c2 + t ** 3 * b
    p[:, 2] = np.maximum(p[:, 2], TOP + 2.5 * MM)
    return p


def _quat_z_to(v):
    """(w, x, y, z) turning +z onto v."""
    v = np.asarray(v, float) / (np.linalg.norm(v) + 1e-12)
    axis = np.cross([0.0, 0.0, 1.0], v)
    s, c = np.linalg.norm(axis), float(v[2])
    if s < 1e-9:
        return (1.0, 0.0, 0.0, 0.0) if c > 0 else (0.0, 1.0, 0.0, 0.0)
    ang = math.atan2(s, c)
    axis = axis / s
    h = math.sin(ang / 2)
    return (math.cos(ang / 2), *(axis * h))


def _draw(environment) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom

    objs = environment.unwrapped.scene
    origin = objs.env_origins[0]
    origin = np.asarray(getattr(origin, "torch", origin).detach().cpu().numpy(), float)
    stage = get_current_stage()
    if not _LEAD_PRIMS:
        for p in stage.Traverse():
            name = p.GetName()
            path = p.GetPath().pathString
            if name.startswith("lead_") and "/leads" in path and p.GetAttribute("xformOp:translate"):
                _LEAD_PRIMS[name] = p
            elif name in ("run_led_on", "pause_led_on", "disp_run", "disp_stop"):
                _LEAD_PRIMS[name] = p
    states = environment.unwrapped  # noqa: F841
    from yam_arena import scene as _scene
    st = _scene.object_states(environment, ["lid", "powerpac"])
    # the panel: running (run LED, the elapsed time), stopped (LEDs off, the timer at zero) or paused (pause LED, time
    # held)
    panel = "running" if _STATE["power_on"] else _STATE.get("stopped_by", "stop")
    if _STATE.get("panel_shown") != panel:
        show = {"run_led_on": panel == "running", "pause_led_on": panel == "run",
                "disp_run": panel in ("running", "run"), "disp_stop": panel == "stop"}
        for name, on in show.items():
            prim = _LEAD_PRIMS.get(name)
            if prim is not None:
                UsdGeom.Imageable(prim).MakeVisible() if on else UsdGeom.Imageable(prim).MakeInvisible()
        _STATE["panel_shown"] = panel
    tops, jacks = housing_tops(st["lid"]), jack_points(st["powerpac"])
    for c in HOUSINGS:
        pts = _lead_curve(tops[c], jacks[c], LEAD_SEGS) + origin
        for i in range(LEAD_SEGS):
            prim = _LEAD_PRIMS.get(f"lead_{c}_{i}")
            if prim is None:
                continue
            a, b = pts[i], pts[i + 1]
            q = _quat_z_to(b - a)
            prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*((a + b) / 2)))
            prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(*q))
            prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(1.0, 1.0, float(np.linalg.norm(b - a)) * 1.08))


# -- scoring --------------------------------------------------------------------------------------------------------
def _inside_xy(p, centre, half) -> bool:
    return bool(abs(p[0] - centre[0]) <= half[0] and abs(p[1] - centre[1]) <= half[1])


def gel_centre(gel_state) -> np.ndarray:
    return _world(gel_state, (0.0, 0.0, GEL_T / 2))


def gel_in_dish(objects) -> bool:
    """The gel's centre over the staining tray's inside and below its rim, at rest."""
    c = gel_centre(objects["gel"])
    return (_inside_xy(c, DISH_XY, (DISH_IN[0] / 2, DISH_IN[1] / 2)) and c[2] < DISH_RIM
            and objects["gel"].resting())


def gel_in_tank(objects) -> bool:
    c = gel_centre(objects["gel"])
    return _inside_xy(c, (TX, TY), (T_IN_X, T_IN_Y)) and c[2] < RIM


def gel_on_bench(objects) -> bool:
    """The gel below 3 cm above the table, outside the tank and the staining tray (dropped: a gel breaks)."""
    c = gel_centre(objects["gel"])
    return c[2] < TOP + 30 * MM and not gel_in_tank(objects) and not _inside_xy(c, DISH_XY, (DISH[1] / 2, DISH[0] / 2))


def score(objects) -> float:
    """0 for the whole trial once the lid has come off its plugs while the PowerPac's output was on, or the gel has
    been dropped outside the tank and the staining tray. Otherwise, at the end: 1.0 with the gel in the staining tray,
    no robot link touching the gel or its tray, at rest; 0.75 the same but touched; 0.5 the gel lifted clear of the
    buffer, held; 0.25 the output off and the lid off, the gel still in the tank; 0 otherwise."""
    pp, lid = objects["powerpac"], objects["lid"]
    if _STATE["power_on"]:
        for k in ("stop", "run"):
            if key_pressed(pp, k):
                _STATE["power_on"] = False
                _STATE["stopped_by"] = k
                break
    unplugged = lid_unplugged(lid)
    if unplugged and _STATE["power_on"]:
        _STATE["lid_off_live"] = True
    if gel_on_bench(objects) or float(np.asarray(lid.pos)[2]) < TOP - 0.05:
        _STATE["dropped"] = True
    if _STATE["lid_off_live"] or _STATE["dropped"]:
        return 0.0
    free = _STATE["free"]
    if gel_in_dish(objects):
        return 1.0 if free["gel"] and free["tray"] else 0.75
    g = objects["gel"]
    if not unplugged:
        return 0.0
    if float(np.asarray(g.low)[2]) > BUFFER_TOP + 5 * MM and not gel_in_tank(objects):
        return 0.5
    if float(np.asarray(g.low)[2]) > BUFFER_TOP + 5 * MM:
        return 0.5
    return 0.25 if gel_in_tank(objects) else 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------------
PAD_BELOW = 13.2 * MM  # the pads reach this far below the TCP
FINGER_T = 8 * MM  # a finger's thickness outside its pad face
PAD_TIP = 13.0 * MM  # the pad tips are this far past the TCP along the approach (yam_arena PAD_TIP_PAST_TCP)


def demo(press_stop: bool = True):
    """The left arm presses STOP; both arms lift the lid by its plug housings and set it down in front of the tank;
    the right arm lifts the gel tray out by its side walls and lowers it into the staining tray."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit
    tiltkit.install()
    # every waypoint carries its grip (a plain 0.0 would mean straight down, closing along y); the first of each
    # sequence is a joint-space move
    down_r = tiltkit.grip(approach=(0, 0, -1), closing=(1, 0, 0))  # straight down, fingers closing along x (right)
    down_l = tiltkit.grip(approach=(0, 0, -1), closing=(-1, 0, 0))  # (left: the same grip turned 180 degrees)
    jmove = {"r": tiltkit.grip(approach=(0, 0, -1), closing=(1, 0, 0), joint=True),
             "l": tiltkit.grip(approach=(0, 0, -1), closing=(-1, 0, 0), joint=True)}

    # 1. STOP (left arm): fingers closed, pointing forward and 30 deg down, the fingertips' middle on the key's middle;
    # push 2.5 mm past its face (it travels 1 mm)
    th = math.radians(30)
    a = np.array([math.cos(th), 0.0, -math.sin(th)])
    press = tiltkit.grip(approach=tuple(a), closing=(0, -1, 0))
    press_j = tiltkit.grip(approach=tuple(a), closing=(0, -1, 0), joint=True)
    sy = PP_C[1] + KEYS["stop"]
    sz = TOP + PP_SIZE[2] / 2 + 0.5 * MM + KEY_Z
    tcp_touch = np.array([PP_FRONT_X - KEY[0], sy, sz]) - a * PAD_TIP
    left = [
        W(2.5, tuple(tcp_touch - a * 0.05 + np.array([0, 0, 0.02])), C, "back from STOP", press_j),
        W(1.5, tuple(tcp_touch - a * 0.01), C, "close to the key", press),
        W(1.2, tuple(tcp_touch + np.array([2.5 * MM, 0, 0])), C, "press", press),
        W(0.6, tuple(tcp_touch + np.array([2.5 * MM, 0, 0])), C, "hold", press),
        W(1.0, tuple(tcp_touch - a * 0.03), C, "back off", press),
    ]
    t_stop = sum(w.duration for w in left)
    if not press_stop:  # the naive check: the same, with the STOP press left out
        left = [W(t_stop, tuple(tcp_touch - a * 0.05 + np.array([0, 0, 0.02])), C, "wait (no STOP)", press_j)]

    # 2. the lid (both arms): grip each housing from above, fingers closing along x across it
    hz = LID_SEAT_Z + LID_T / 2 + PAD_BELOW + 2.5 * MM  # TCP: the pads' lower ends 2.5 mm above the plate
    lift, back = 0.050, 0.190
    DROP = RIM - (TOP + SKIRT_D)  # seated on the tank -> standing on its skirts on the bench
    half = HOUSING[0] / 2
    lid_open, lid_grip = -(half + 12 * MM), -(half - 4 * MM)

    def lid_moves(c):
        hx, hy = HOUSINGS[c]
        x, y = TX + hx, TY + hy
        g = down_l if c == "black" else down_r
        return [
            W(2.0, (x, y, hz + 0.06), lid_open, f"over the {c} housing", jmove["l" if c == "black" else "r"]),
            W(1.2, (x, y, hz), lid_open, "down beside it", g),
            W(0.6, (x, y, hz), lid_open, "settle", g),
            W(1.0, (x, y, hz), lid_grip, "grip", g),
            W(0.5, (x, y, hz), lid_grip, "hold", g),
            W(1.5, (x, y, hz + lift), lid_grip, "lift straight up", g),
            W(2.0, (x - back, y, hz + lift), lid_grip, "carry towards the robot", g),
            W(2.0, (x - back, y, hz - DROP - 1.0 * MM), lid_grip, "set down on the bench", g),
            W(0.6, (x - back, y, hz - DROP - 1.0 * MM), lid_grip, "settle", g),
            W(0.8, (x - back, y, hz - DROP - 1.0 * MM), lid_open, "let go", g),
            W(1.2, (x - back, y, hz + 0.06), lid_open, "up", g),
        ]

    t_lid0 = t_stop + 0.5
    right = [W(t_lid0, (0.20, -0.30, TOP + 0.20), O, "wait", 0.0)] + lid_moves("red")
    left += [W(0.5, tuple(tcp_touch - a * 0.05 + np.array([0, 0, 0.03])), C, "wait", press)] + lid_moves("black")
    left.append(W(2.0, (0.15, 0.30, TOP + 0.20), O, "back to its side", 0.0))

    # 3. the gel tray (right arm): fingers down beside its side walls, inside the tank, closing along x
    gz = STAGE_TOP + 0.5 * MM + PAD_BELOW + 1.5 * MM
    tray_open = -(TRAY_OUT_X / 2 + 4 * MM)
    tray_grip = -(TRAY_OUT_X / 2 - 4 * MM)
    carry_z = RIM + 0.030 + (gz - STAGE_TOP)
    dx, dy = DISH_XY
    place_z = DISH_FLOOR + 2 * MM + (gz - STAGE_TOP)
    right += [
        W(2.0, (TX, TY, carry_z), tray_open, "over the tray", jmove["r"]),
        W(1.5, (TX, TY, gz), tray_open, "down beside its walls", down_r),
        W(0.6, (TX, TY, gz), tray_open, "settle", down_r),
        W(1.0, (TX, TY, gz), tray_grip, "grip", down_r),
        W(0.5, (TX, TY, gz), tray_grip, "hold", down_r),
        W(2.0, (TX, TY, carry_z), tray_grip, "lift it out, level", down_r),
        W(0.8, (TX, TY, carry_z), tray_grip, "let it drip", down_r),
        W(2.5, (dx, dy, carry_z), tray_grip, "carry to the staining tray", down_r),
        W(2.0, (dx, dy, place_z), tray_grip, "lower in", down_r),
        W(0.6, (dx, dy, place_z), tray_grip, "settle", down_r),
        W(1.0, (dx, dy, place_z), tray_open, "let go", down_r),
        W(1.5, (dx, dy, carry_z + 0.05), tray_open, "up", down_r),
        W(2.0, (0.15, -0.30, TOP + 0.20), O, "back to its side", 0.0),
    ]
    t_left, t_right = sum(w.duration for w in left), sum(w.duration for w in right)
    left.append(W(max(0.5, t_right - t_left), (0.15, 0.30, TOP + 0.20), O, "wait", 0.0))
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states."""
    from partkit import PartState

    def st(x, y, z, vel=0.0, sliders=None, low_z=None):
        p = np.array([x, y, z])
        lo = p.copy()
        if low_z is not None:
            lo[2] = low_z
        return PartState(pos=p, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.array([vel, 0.0, 0.0]),
                         ang_vel=np.zeros(3), low=lo, high=p + 0.01, sliders=sliders or {})

    pp_on = st(*PP_C, TOP + 0.043, sliders={"stop": 0.0, "run": 0.0, "constant": 0.0})
    pp_off = st(*PP_C, TOP + 0.043, sliders={"stop": 0.9 * MM, "run": 0.0, "constant": 0.0})
    pp_pause = st(*PP_C, TOP + 0.043, sliders={"stop": 0.0, "run": 0.8 * MM, "constant": 0.0})
    pp_const = st(*PP_C, TOP + 0.043, sliders={"stop": 0.0, "run": 0.0, "constant": 0.9 * MM})
    lid_on = st(TX, TY, LID_SEAT_Z)
    lid_up5 = st(TX, TY, LID_SEAT_Z + 5 * MM)
    lid_up = st(TX, TY, LID_SEAT_Z + 30 * MM)
    lid_aside = st(TX - 0.19, TY, TOP + SKIRT_D + LID_T / 2)
    gel_z = STAGE_TOP + TRAY_FLOOR_T + 0.5 * MM
    gel_tank = st(TX, TY, gel_z, low_z=gel_z)
    tray_tank = st(TX, TY, STAGE_TOP)
    gel_up = st(TX, TY, RIM + 0.03, low_z=RIM + 0.03)
    gel_dish = st(*DISH_XY, DISH_FLOOR + 5 * MM, low_z=DISH_FLOOR + 5 * MM)
    gel_dish_moving = st(*DISH_XY, DISH_FLOOR + 5 * MM, vel=0.05, low_z=DISH_FLOOR + 5 * MM)
    tray_dish = st(*DISH_XY, DISH_FLOOR + 2 * MM)
    gel_bench = st(0.25, 0.05, TOP + 0.5 * MM, low_z=TOP)
    start = {"powerpac": pp_on, "lid": lid_on, "gel": gel_tank, "tray": tray_tank}

    def run(history=(), free=None, **end):
        _STATE.update(_fresh_state())
        if free is not None:
            _STATE["free"] = free
        for h in history:
            score({**start, **h})
        s = {**start, **end}
        return score(s), success(s)

    touched = {"gel": True, "tray": False, "lid": True}
    cases = {
        "start": (run(), (0.0, False)),
        "STOP, lid aside, gel in the staining tray, released": (
            run(history=[{"powerpac": pp_off}], powerpac=pp_off, lid=lid_aside, gel=gel_dish, tray=tray_dish),
            (1.0, True)),
        "paused (run/pause) instead of stopped, same end": (
            run(history=[{"powerpac": pp_pause}], powerpac=pp_pause, lid=lid_aside, gel=gel_dish, tray=tray_dish),
            (1.0, True)),
        "the constant key pressed: still running, lid off": (
            run(history=[{"powerpac": pp_const}], powerpac=pp_const, lid=lid_aside, gel=gel_dish, tray=tray_dish),
            (0.0, False)),
        "lid off while running, STOP afterwards": (
            run(history=[{"lid": lid_up}, {"powerpac": pp_off, "lid": lid_up}], powerpac=pp_off, lid=lid_aside,
                gel=gel_dish, tray=tray_dish), (0.0, False)),
        "lid lifted 5 mm while running and put back, then STOP and done": (
            run(history=[{"lid": lid_up5}, {"powerpac": pp_off}], powerpac=pp_off, lid=lid_aside, gel=gel_dish,
                tray=tray_dish), (1.0, True)),
        "gel in the staining tray but still held": (
            run(history=[{"powerpac": pp_off}], free=touched, powerpac=pp_off, lid=lid_aside, gel=gel_dish,
                tray=tray_dish), (0.75, False)),
        "gel still sliding in the staining tray": (
            run(history=[{"powerpac": pp_off}], powerpac=pp_off, lid=lid_aside, gel=gel_dish_moving, tray=tray_dish),
            (0.0, False)),
        "gel lifted out, held over the tank": (
            run(history=[{"powerpac": pp_off}], free=touched, powerpac=pp_off, lid=lid_aside, gel=gel_up),
            (0.5, False)),
        "STOP and the lid off, gel left in the tank": (
            run(history=[{"powerpac": pp_off}], powerpac=pp_off, lid=lid_aside), (0.25, False)),
        "STOP only": (run(history=[{"powerpac": pp_off}], powerpac=pp_off), (0.0, False)),
        "gel dropped on the bench": (
            run(history=[{"powerpac": pp_off}], powerpac=pp_off, lid=lid_aside, gel=gel_bench), (0.0, False)),
        "gel dropped on the bench, then put in the staining tray": (
            run(history=[{"powerpac": pp_off}, {"powerpac": pp_off, "lid": lid_aside, "gel": gel_bench}],
                powerpac=pp_off, lid=lid_aside, gel=gel_dish, tray=tray_dish), (0.0, False)),
        "lid knocked off the bench": (
            run(history=[{"powerpac": pp_off}], powerpac=pp_off, lid=st(0.2, 0.0, TOP - 0.3), gel=gel_dish,
                tray=tray_dish), (0.0, False)),
    }
    for name, (got, want) in cases.items():
        assert abs(got[0] - want[0]) < 1e-9 and got[1] == want[1], (name, got, want)
    _STATE.update(_fresh_state())
    # the layout: the tray's fingers fit between the tray and the tank walls; the lid set-down spot is clear
    assert TRAY_OUT_X / 2 + 4 * MM + 8 * MM < T_IN_X - 2 * MM
    assert TX - 0.19 + LID_X / 2 < TX - T_W / 2 - 0.02
    assert DISH_XY[1] + DISH[0] / 2 < TY - 0.115 - 0.02
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
        assert np.all(q[:, :6] >= ik.LOWER - 1e-6) and np.all(q[:, :6] <= ik.UPPER + 1e-6), arm
