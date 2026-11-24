"""Unplug a PV module's MC4 connector at the end of a live string, so the module can be replaced.

At the end of a ground-mount rack, the module's Staubli MC4 plug (PV-KST4), its lead cable-tied to a rack post, hangs
mated with the string's home-run socket (PV-KBT4). The original MC4 "requires a tool to open" (Staubli datasheet
11014112, NEC 2023): its two locking clips release only when the unlocking pins of a Staubli PV-MS spanner are pushed
onto them (Staubli MA270): the spanner is set square across the connector like an open-end spanner, its fork round the
plug's flat front, and the pins on the fork's prongs are pushed along the connector into the plug's two windows, where
they ride up a ramp on each (recessed) clip and press it in against its spring. The latch opens only once both clips are pressed in
0.8 mm. The spanner lies across a Unistrut on the table. Staubli: "PV connections must not be disconnected while
under load" - the string's IMO SI16-PEL64R-2 DC isolator, on a post beside it, is still ON. The module is a Canadian
Solar CS6R-400MS-HL on the rack behind. Sizes and sources: pvgeom.py; scene: assets/site.usda
(work/unplug-pv-connector/make_site.py).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ..."""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import partkit  # noqa: E402,F401
from partkit import part  # noqa: E402
from scenekit import UsdBody, on_reset  # noqa: E402

import pvgeom as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Unplug this solar panel's MC4 connector so the panel can be replaced."

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- the scene: one fixed-base articulation (plug, posts, isolator, module, table strut; the isolator's handle and the socket move)
SITE = UsdBody("site", usd="assets/site.usda", pos=(0.0, 0.0, float(TOP)), size=(0.4, 0.6, 0.3), mass=None,
               articulation=True, material=False, solver_iters=32,
               joint_pos=(("handle_joint", 0.0), ("socket_joint", 0.0), ("clip_p_joint", 0.0), ("clip_n_joint", 0.0)))

# -- the PV-MS spanner: loose, lying across the table strut, fork towards the robot, pins down (frame: x along it, the
# unlocking fork at -x, the cap-nut jaw at +x, pins on the -z face at the fork's prong tips)
TOOL_BLACK = (0.07, 0.07, 0.08)
PANEL = (0.11, 0.11, 0.12)
_L, _W, _T = g.TOOL
_f0 = -_L / 2 + g.FORK[1]  # the fork slot's closed end
_n0 = _L / 2 - g.NUT_JAW[1]  # the cap-nut jaw's closed end
_fp = (_W - g.FORK[0]) / 2  # each fork prong's width
_np = (_W - g.NUT_JAW[0]) / 2
TOOL_BOXES = [
    (((_f0 + _n0) / 2, 0.0, 0.0), (_n0 - _f0, _W, _T), TOOL_BLACK),
    *[(((-_L / 2 + _f0) / 2, t * (g.FORK[0] / 2 + _fp / 2), 0.0), (g.FORK[1], _fp, _T), TOOL_BLACK) for t in (-1, 1)],
    *[(((_L / 2 + _n0) / 2, t * (g.NUT_JAW[0] / 2 + _np / 2), 0.0), (g.NUT_JAW[1], _np, _T), TOOL_BLACK) for t in (-1, 1)],
    # the moulded label panel on its top face (MA270 ill. 1: a recessed panel with the MC4 mark), and its rim
    ((0.0, 0.0, _T / 2 + 0.2 * MM), (0.064, 0.030, 0.4 * MM), PANEL),
    *[((g.PIN_X, t * g.PIN_Y, -_T / 2 - g.PIN[2] / 2), g.PIN, TOOL_BLACK) for t in (-1, 1)],
]
TOOL = part("tool", tuple(float(v) for v in (g.TOOL_POS[0], g.TOOL_POS[1], TOP + g.TOOL_POS[2])), TOOL_BOXES,
            g.TOOL_MASS, friction=0.6, torsional_patch=13 * MM,  # half a pad's length (held in a pinch)
            # the fork's prongs and pins: moulded plastic sliding on the MC4's plastic (est. 0.25 / 0.2); the body, where
            # it is held, keeps the pads' grip
            box_materials=(((1, 2, 6, 7), g.PLASTIC_MU[0], g.PLASTIC_MU[1], "average"),))

OBJECTS = [SITE, TOOL]

# -- the latch, the clips' springs and the isolator's detents, driven every physics step -------------------------------
OFF = math.radians(g.OFF_DEG)
OFF_TOL = math.radians(g.OFF_TOL_DEG)
SNAP = math.radians(3.0)
PRELOAD = math.radians(5.0)
# The handle's switching torque is not published: about 0.4 N m at mid-throw (est.), from an over-centre spring
# that holds it at ON or OFF.
K_HANDLE = 0.4 / (abs(OFF) / 2 + PRELOAD)
D_HANDLE = 0.004
K_LATCH = 2.0e5  # N/m: latched, the socket gives 0.25 mm under a 50 N pull (a real MC4 latch holds far more)
D_LATCH = 200.0
D_DRAG = 40.0  # N s/m: the seals' drag once unlatched (est.)
D_CLIP = 3.0  # N s/m: about critical for the clip's spring and mass
STAY_OPEN = 1.0 * MM  # drawn out this far, the clips' hooks are past the windows' edges and cannot catch again
RETRACT = 0.5 * MM  # drawn out this far (unlatched), the plug's bore holds the clips pressed in as the socket comes out
K_BORE = 2.0e4  # N/m
ON_S, OFF_S = "ON", "OFF"
JOINTS = ["handle_joint", "socket_joint", "clip_p_joint", "clip_n_joint"]

_STATE: dict = {}


def _fresh():
    return {"clips": (0.0, 0.0), "arc": False, "max_w": 0.0, "mech": _STATE.get("mech")}


_STATE.update(_fresh())


def latch_next(latched: bool, press_p: float, press_n: float, w: float) -> bool:
    """The MC4 latch: it opens once both clips are pressed in CLIP_RELEASE (0.8 mm); it closes again when they spring
    back while the socket is still home (drawn out less than 1 mm)."""
    pressed = press_p >= g.CLIP_RELEASE and press_n >= g.CLIP_RELEASE
    if latched and pressed:
        return False
    if not latched and not pressed and w < STAY_OPEN:
        return True
    return latched


class Mechanism:
    """Over-centre detents on the isolator's handle (ON at 0, OFF at -90 deg), the clips' springs, and the MC4 latch on
    the socket's slide. Once the socket is drawn out (0.5 mm, which needs the latch open), the clips are held pressed
    in, as the plug's bore holds them while they slide out through it. It wraps the site's write_data_to_sim, so it acts on the joint positions PhysX reported after
    the last substep."""

    def __init__(self, site):
        self.site, self.dev = site, site.device
        self.ids = site.find_joints(JOINTS, preserve_order=True)[0]
        self.reset()
        write = site.write_data_to_sim

        def write_with_springs():
            self.update()
            write()

        site.write_data_to_sim = write_with_springs

    def q(self):
        q = self.site.data.joint_pos
        q = getattr(q, "torch", q)[0]
        return [float(q[i]) for i in self.ids]

    def _apply(self):
        import torch

        tgt_h = (OFF - PRELOAD) if self.handle == OFF_S else PRELOAD
        k_s, d_s = (K_LATCH, D_LATCH) if self.latched else (0.0, D_DRAG)
        k_c, t_c = (K_BORE, g.CLIP_TRAVEL) if self.out else (g.CLIP_K, 0.0)
        k = torch.tensor([[K_HANDLE, k_s, k_c, k_c]], device=self.dev)
        d = torch.tensor([[D_HANDLE, d_s, D_CLIP, D_CLIP]], device=self.dev)
        self.site.write_joint_stiffness_to_sim_index(stiffness=k, joint_ids=self.ids)
        self.site.write_joint_damping_to_sim_index(damping=d, joint_ids=self.ids)
        self.site.set_joint_position_target_index(target=torch.tensor([[tgt_h, 0.0, -t_c, t_c]], device=self.dev),
                                                  joint_ids=self.ids)

    def update(self):
        qh, qs, qp, qn = self.q()
        press = (-qp, qn)
        _STATE["clips"] = press
        changed = False
        if self.handle == ON_S and qh < OFF / 2 - SNAP:
            self.handle, changed = OFF_S, True
        elif self.handle == OFF_S and qh > OFF / 2 + SNAP:
            self.handle, changed = ON_S, True
        latched = latch_next(self.latched, *press, -qs)
        if latched != self.latched:
            self.latched, changed = latched, True
        out = -qs > RETRACT
        if out != self.out:
            self.out, changed = out, True
        if changed:
            self._apply()

    def reset(self):
        self.handle, self.latched, self.out = ON_S, True, False
        self._apply()


@on_reset
def _reset(environment) -> None:
    site = environment.unwrapped.scene["site"]
    m = _STATE.get("mech")
    if m is None or m.site is not site:
        m = Mechanism(site)
    _STATE.update(_fresh())
    _STATE["mech"] = m
    m.reset()


# -- scoring --------------------------------------------------------------------------------------------------------
def handle_angle(site) -> float:
    return float(site.joints.get("handle_joint", 0.0))


def withdrawn(site) -> float:
    """How far the socket has been drawn out of the plug (m)."""
    return -float(site.joints.get("socket_joint", 0.0))


def isolator_off(site) -> bool:
    return abs(handle_angle(site) - OFF) <= OFF_TOL


def score(objects) -> float:
    """0 for the whole trial once the connector's halves have parted by 3 mm while the isolator was not OFF (within
    15 deg). Otherwise, at the end, with the isolator OFF: 1.0 the socket drawn out 30 mm or more (fully separated:
    its front goes 26 mm into the plug); 0.5 parted 3 mm or more; 0.25 still plugged in. 0 with the isolator not
    OFF."""
    site = objects["site"]
    w = withdrawn(site)
    off = isolator_off(site)
    if w >= g.ARC and not off:
        _STATE["arc"] = True
    _STATE["max_w"] = max(_STATE["max_w"], w)
    if _STATE["arc"] or not off:
        return 0.0
    if w >= g.APART:
        return 1.0
    return 0.5 if w >= g.ARC else 0.25


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------------
PAD_BELOW = 13.2 * MM  # the pads reach this far past the TCP along the approach
MID_OFF = 2.4 * MM  # the TCP sits this far back (against the approach) from the gripped edge's middle


def tool_in_use_centre(x_from_face: float) -> np.ndarray:
    """The spanner's centre when it stands square across the connector, fork down, its fork's closed end on the plug's
    top, its pins along +x; x_from_face: how far the centre is past the plug's front face (+x)."""
    return np.array([g.X0 + x_from_face, g.AXIS_Y, TOP + g.AXIS_Z + g.D / 2 + (g.TOOL[0] / 2 - g.FORK[1])])


# the spanner's centre from the plug's front face: its fork lowered onto the plug's flats with the pins short of the
# windows, then pushed along until the fork stops against the sleeve, where the windows start
CX_START = 2.5 * MM
CX_HOME = g.WINDOW[0] - g.TOOL[2] / 2


def demo(isolator_first: bool = True, use_tool: bool = True, nut_tilt: float = 60.0, nut_back: float = 0.12,
         retreat=(0.18, -0.22, 0.23), grip_high: float = 0.040, nut_back_tilt: float = 35.0, regrasp: bool = True):
    """Right arm: the isolator's handle a quarter turn anticlockwise to OFF. Left arm: the PV-MS from the table strut,
    held by its long edges near the fork; pitched up square across the connector and lowered so its fork sits round
    the plug's flat front, the pins just short of the windows; then pushed along the connector so that the pins ride
    up the clips' ramps and press both clips in, and held there. Right arm, tipped in from the -y side: the socket's
    cap nut drawn 40 mm towards the robot. Then the left arm draws the spanner back out of the windows and lifts it off. ``isolator_first=False`` leaves the isolator ON (the
    naive way); ``use_tool=False`` pulls without the spanner."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit
    tiltkit.install()
    z0 = TOP

    # -- right arm: the isolator (approach +x, fingers on the bar's sides, the TCP on the handle's axis)
    def turn(deg, joint=False):
        t = math.radians(deg)
        return tiltkit.grip(approach=(1, 0, 0), closing=(0, -math.cos(t), -math.sin(t)), joint=joint)

    hx = g.ISO_FACE_X - 2 * MM - 9 * MM - PAD_BELOW - 1.0 * MM  # pad tips 1 mm short of the knob
    hy, hz = g.ISO_Y, z0 + g.HANDLE_Z
    right = []
    if isolator_first:
        right += [
            W(2.5, (0.25, hy, hz), O, "well back from the isolator", turn(0, True)),
            W(1.5, (hx - 0.05, hy, hz), O, "in front of its handle", turn(0)),
            W(1.2, (hx, hy, hz), O, "fingers either side of the bar", turn(0)),
            W(0.5, (hx, hy, hz), O, "settle", turn(0)),
            W(0.8, (hx, hy, hz), C, "grip the bar", turn(0)),
            *[W(0.8, (hx, hy, hz), C, f"turn {d} deg", turn(d)) for d in (-25, -50, -75, -95)],
            W(0.6, (hx, hy, hz), C, "hold at OFF", turn(-95)),
            W(0.8, (hx, hy, hz), O, "let go", turn(-90)),
            W(1.2, (hx - 0.06, hy, hz), O, "back off", turn(-90)),
        ]
    else:
        right += [W(12.0, (0.25, hy, hz), O, "wait in front of the isolator (left ON)", turn(0, True))]
    right.append(W(1.5, (0.25, hy, hz), O, "straight back from the isolator", turn(-90)))
    t_iso = sum(w.duration for w in right)

    # -- left arm: the spanner, held by its long edges 35 mm from its middle towards the fork (a short lever to the
    # pins), the approach tilted 30 deg from the spanner's face so the hand stays high above the connector in use
    GRIP = 0.035
    th = math.radians(30)
    a_pick = np.array([-math.sin(th), 0.0, -math.cos(th)])  # lying flat: down, tilted towards the robot
    a_use = np.array([math.cos(th), 0.0, -math.sin(th)])  # in use: along the pins (+x), tilted down
    pick = tiltkit.grip(approach=tuple(a_pick), closing=(0, -1, 0))
    pick_j = tiltkit.grip(approach=tuple(a_pick), closing=(0, -1, 0), joint=True)
    use = tiltkit.grip(approach=tuple(a_use), closing=(0, -1, 0))
    use_j = tiltkit.grip(approach=tuple(a_use), closing=(0, -1, 0), joint=True)
    tool_open = -(g.TOOL[1] / 2 + 12 * MM)
    gp = np.array([g.TOOL_POS[0] - GRIP, g.TOOL_POS[1], z0 + g.TOOL_POS[2]]) - MID_OFF * a_pick  # the TCP, lying
    AIM_Y = -0.4 * MM  # measured in runs; the plug's chamfered front and the fork centre it the rest of the way

    def tcp_use(x_from_face, dz=0.0, grip=GRIP):
        c = tool_in_use_centre(x_from_face) + np.array([0.0, AIM_Y, dz])
        return tuple(c + np.array([0.0, 0.0, -grip]) - MID_OFF * a_use)

    left = [
        W(2.5, tuple(gp - 0.06 * a_pick), tool_open, "over the spanner, near its fork", pick_j),
        W(1.5, tuple(gp), tool_open, "fingers beside its edges", pick),
        W(0.5, tuple(gp), tool_open, "settle", pick),
        W(0.8, tuple(gp), C, "grip it", pick),
        W(0.4, tuple(gp), C, "hold", pick),
        W(1.5, (gp[0], gp[1], z0 + 0.12), C, "lift", pick),
        W(3.5, tcp_use(CX_START - 0.03, 0.05), C, "pitch it up, fork down, in front of the connector", use_j),
        W(1.5, tcp_use(CX_START, 0.025), C, "fork over the plug's flat front", use),
        W(1.5, tcp_use(CX_START, -3.0 * MM), C, "lower it onto the plug, pins short of the windows", use),
        W(1.5, tcp_use(CX_START, -3.0 * MM), C, "settle", use),
        W(0.5, tcp_use(CX_START, 1.5 * MM), C, "ease off: the windows will hold the pins' height", use),
        # the soft arm gives about 1 mm per 0.6 N here (measured): aim 6 mm past home; the fork stops on the sleeve
        W(4.0, tcp_use(CX_HOME + 6.0 * MM, 1.5 * MM), C, "push it along the flats: the pins ride up the clips' ramps", use),
        W(1.0, tcp_use(CX_HOME + 6.0 * MM, 1.5 * MM), C, "hold the clips pressed", use),
    ]
    hold = tcp_use(CX_HOME + 6.0 * MM, 1.5 * MM)
    if regrasp:  # the pressed clips hold the spanner while the hand takes it again higher up
        # it stands leaning back about 8 deg (measured in runs: its top end 13.5 mm towards +x): aim the grip there
        hold = tuple(np.array(tcp_use(CX_HOME, 0.0, -grip_high)) + np.array([0.0135, 0.0, -0.001]))
        left += [
            W(0.8, tcp_use(CX_HOME + 1.0 * MM, 1.5 * MM), tool_open, "let go: the clips hold it", use),
            W(1.0, tuple(np.array(tcp_use(CX_HOME)) - 0.03 * a_use), tool_open, "back off", use),
            W(1.5, tuple(np.array(hold) - 0.03 * a_use), tool_open, "up to its top end", use),
            W(1.0, hold, tool_open, "fingers beside its edges", use),
            W(0.8, hold, C, "hold it by its top end", use),
        ]
    if not use_tool:
        left = [W(sum(w.duration for w in left), (0.15, 0.30, z0 + 0.20), O, "wait (no tool)", 0.0)]
    t_tool = sum(w.duration for w in left)  # the clips are pressed from here on

    # -- right arm: the socket's cap nut, the hand tipped in from the -y side, fingers closing across it; pull 40 mm
    # towards the robot. It turns over to the tipped grip in one slow joint move straight to a point in line with the
    # cap nut (a Cartesian move from beside the isolator swung the forearm into the isolator's box)
    tn, tb = math.radians(nut_tilt), math.radians(nut_back_tilt)
    # tipped in from the -y side, then tilted back about the closing direction so the hand comes from behind (its
    # elbow stays towards the robot, clear of the isolator)
    a_n = np.array([0.0, math.sin(tn), -math.cos(tn)]) * math.cos(tb) + np.array([1.0, 0.0, 0.0]) * math.sin(tb)
    c_n = (0, -math.cos(tn), -math.sin(tn))
    nut = tiltkit.grip(approach=tuple(a_n), closing=c_n)
    nut_j = tiltkit.grip(approach=tuple(a_n), closing=c_n, joint=True)
    nx = g.SOCKET_REAR_X + (g.SOCKET_CAP[1] - g.SOCKET_CAP[0]) / 2
    nut_open = -18 * MM
    nut_p = np.array([nx, g.AXIS_Y, z0 + g.AXIS_Z])
    pull = np.array([-0.040, 0.0, 0.0])
    right += [
        W(3.0, (retreat[0], retreat[1], z0 + retreat[2]), O, "back and up, clear of the isolator", turn(-90)),
        W(max(0.5, t_tool - 6.5 - t_iso - 3.0), (retreat[0], retreat[1], z0 + retreat[2]), O, "wait", turn(-90)),
        W(5.0, tuple(nut_p - nut_back * a_n), nut_open, "turn over, in line with the cap nut", nut_j),
        W(1.5, tuple(nut_p - 0.03 * a_n), nut_open, "towards the cap nut", nut),
        W(1.2, tuple(nut_p), nut_open, "fingers either side of it", nut),
        W(0.5, tuple(nut_p), nut_open, "settle", nut),
        W(0.8, tuple(nut_p), C, "grip", nut),
        W(0.4, tuple(nut_p), C, "hold", nut),
        W(3.0, tuple(nut_p + pull), C, "draw the socket out", nut),
        W(0.5, tuple(nut_p + pull), C, "hold", nut),
        W(0.8, tuple(nut_p + pull), nut_open, "let go", nut),
        W(1.5, tuple(nut_p + pull - 0.08 * a_n), nut_open, "away", nut),
        W(5.0, (0.20, -0.30, z0 + 0.20), O, "back to its side", 0.0),
    ]
    t_pulled = sum(w.duration for w in right) - 5.3  # the right hand has let go of the drawn-out socket
    if use_tool:
        left += [
            W(max(0.5, t_pulled - t_tool), hold, C, "hold the spanner on while the socket is drawn out", use),
            W(1.5, tuple(np.array(hold) + np.array([CX_START - CX_HOME - 6 * MM, 0, 0])), C,
              "draw it back out of the windows", use),
            W(1.5, tuple(np.array(hold) + np.array([CX_START - CX_HOME - 6 * MM, 0, 0.05])), C, "lift it off", use),
        ]
    t_r, t_l = sum(w.duration for w in right), sum(w.duration for w in left)
    left.append(W(max(0.5, t_r - t_l), left[-1].pos, left[-1].finger, "hold", left[-1].yaw))
    return {"left": left, "right": right}


def self_test():
    """The scoring rules, the latch rule and the parts' sizes against each other."""
    from scenekit import BodyState

    def site(angle_deg=0.0, w=0.0):
        return BodyState(pos=np.zeros(3), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=np.zeros(3), high=np.zeros(3),
                         joints={"handle_joint": math.radians(angle_deg), "socket_joint": -w})

    def run(history=(), **end):
        _STATE.update(_fresh())
        for h in history:
            score({"site": h})
        s = {"site": end.get("site", site())}
        return score(s), success(s)

    cases = {
        "start": (run(), (0.0, False)),
        "isolator OFF only": (run(site=site(-90)), (0.25, False)),
        "OFF, unplugged 40 mm": (run(history=[site(-90), site(-90, 0.01)], site=site(-90, 0.040)), (1.0, True)),
        "OFF at -80 deg (within 15), unplugged 32 mm": (run(site=site(-80, 0.032)), (1.0, True)),
        "handle half way (-60), unplugged": (run(site=site(-60, 0.035)), (0.0, False)),
        "OFF, parted 10 mm": (run(history=[site(-90)], site=site(-90, 0.010)), (0.5, False)),
        "OFF, drawn 26 mm (front just at the face)": (run(history=[site(-90)], site=site(-90, 0.026)), (0.5, False)),
        "unplugged while ON, OFF afterwards": (run(history=[site(0, 0.005)], site=site(-90, 0.035)), (0.0, False)),
        "parted 2 mm while ON (no arc), then done right": (
            run(history=[site(0, 0.002), site(-90)], site=site(-90, 0.035)), (1.0, True)),
        "unplugged with the isolator OFF, then turned back ON": (
            run(history=[site(-90, 0.035)], site=site(0, 0.035)), (0.0, False)),
    }
    for name, (got, want) in cases.items():
        assert abs(got[0] - want[0]) < 1e-9 and got[1] == want[1], (name, got, want)
    _STATE.update(_fresh())

    # the latch: both clips pressed 0.8 mm opens it; one clip, or both short of 0.8 mm, does not; it shuts again only
    # while the socket is home
    R = g.CLIP_RELEASE
    latch = {
        "both pressed": (latch_next(True, R + 1e-4, R + 1e-4, 0.0), False),
        "one pressed": (latch_next(True, 1.2 * MM, 0.0, 0.0), True),
        "both 0.7 mm": (latch_next(True, 0.7 * MM, 0.7 * MM, 0.0), True),
        "released while home: shuts": (latch_next(False, 0.0, 0.0, 0.5 * MM), True),
        "released once drawn out: stays open": (latch_next(False, 0.0, 0.0, 2 * MM), False),
        "still pressed, home: stays open": (latch_next(False, R + 1e-4, R + 1e-4, 0.0), False),
    }
    for name, (got, want) in latch.items():
        assert got == want, (name, got, want)

    # sizes against each other
    h = g.D / 2
    assert 2 * g.FLAT_HALF + 0.2 * MM <= g.FORK[0] <= 2 * g.FLAT_HALF + 0.6 * MM  # the fork fits the plug's flats
    assert g.FORK[0] < g.D - 2 * MM  # and stops against the sleeve, where the windows start
    assert g.CLIP_OUT - g.CLIP_TRAVEL < g.FORK[0] / 2 - 0.3 * MM  # pressed right in, the clips pass inside the fork
    assert g.FLAT_HALF + 0.1 * MM <= g.PIN_IN <= g.FLAT_HALF + 0.3 * MM  # the pins slide along the plug's flats
    assert g.CLIP_PROFILE[1][1] <= g.FLAT_HALF  # each pin meets its clip's ramp, not its front face
    assert g.CLIP_OUT - g.PIN_IN >= g.CLIP_RELEASE + 0.3 * MM  # a pin on the clip's flat presses it past release
    assert g.CLIP_OUT - g.PIN_IN + 0.5 * MM < g.CLIP_TRAVEL  # and the clip can go that far
    assert g.CLIP_OUT <= h - 1.5 * MM  # the clips sit recessed in the windows
    assert g.PIN[0] / 2 <= g.WINDOW_HALF_Z - 1.4 * MM  # the pins fit the windows (+-1.5 mm)
    assert 2 * g.WINDOW_HALF_Z < 6.3 * MM  # a finger pad (6.3 mm wide) cannot get into a window to press a clip
    assert g.CLIP_HALF_Z < g.WINDOW_HALF_Z
    # in use the pins sit on the axis; at the start of the push they are short of the windows, at home on the clips'
    # flats; the fork stays on the flat front until the pins are in the windows
    c0 = tool_in_use_centre(CX_START)
    assert abs(c0[2] + g.PIN_X - (TOP + g.AXIS_Z)) < 1e-9
    tip0 = CX_START + g.TOOL[2] / 2 + g.PIN[2]
    assert tip0 <= g.WINDOW[0] - 2.0 * MM  # lowered with the pins well short of the windows
    assert g.CLIP_PROFILE[2][0] <= CX_HOME + g.TOOL[2] / 2 + g.PIN[2] - 1.5 * MM  # at home the pins are on the flats
    assert CX_HOME + g.TOOL[2] / 2 + g.PIN[2] <= g.WINDOW[1]  # and inside the windows
    assert CX_START - g.TOOL[2] / 2 >= 0.4 * MM  # the fork sits on the flats, clear of the socket's body
    # the cable starts inside the socket over the whole slide, and its straight run reaches past the cap nut at the end
    assert g.SOCKET_REAR_X + 5 * MM < g.CABLE_START_X < g.SOCKET_TIP_X - g.SLIDE - 4 * MM
    assert g.CABLE_STRAIGHT_TO < g.SOCKET_REAR_X - g.SLIDE - 3 * MM
    assert g.SOCKET_IN < g.APART < g.SLIDE  # full credit needs the halves fully apart, within the slide
    # the spanner rests with its middle over the strut, its pins in front of it
    assert g.STRUT_X - g.STRUT / 2 < g.TOOL_POS[0] < g.STRUT_X + g.STRUT / 2
    assert g.TOOL_POS[0] + g.PIN_X + g.PIN[0] / 2 < g.STRUT_X - g.STRUT / 2 - 5 * MM
    return len(cases) + len(latch) + 18


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for kw in ({}, {"isolator_first": False}):
        for arm, waypoints in task.demo(**kw).items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            print(f"{kw} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
            assert np.all(q[:, :6] >= ik.LOWER - 1e-6) and np.all(q[:, :6] <= ik.UPPER + 1e-6), arm
