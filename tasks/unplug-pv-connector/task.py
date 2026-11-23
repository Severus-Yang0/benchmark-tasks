"""Unplug a PV module's MC4 connector at the end of a live string, so the module can be replaced.

At the end of a ground-mount rack, the module's Staubli MC4 plug (PV-KST4), its lead cable-tied to a rack post, hangs
mated with the string's home-run socket (PV-KBT4). The original MC4 "requires a tool to open" (Staubli datasheet
11014112, NEC 2023): its two locking clips release only when the unlocking pins of a Staubli PV-MS spanner are pushed
onto them (Staubli MA270): the spanner is set square across the connector like an open-end spanner, its fork round the
plug's front, and the pins on the fork's prongs go along the connector into the plug's two windows. The spanner lies
across a Unistrut on the table. Staubli: "PV connections must not be disconnected while
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
from scenekit import UsdBody, on_reset, on_step  # noqa: E402

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
               joint_pos=(("handle_joint", 0.0), ("socket_joint", 0.0)))

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
            g.TOOL_MASS, friction=0.6, torsional_patch=6 * MM)

OBJECTS = [SITE, TOOL]

# -- the latch and the isolator's detents, driven every physics step -------------------------------------------------
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
ON_S, OFF_S = "ON", "OFF"

_STATE: dict = {}


def _fresh():
    return {"engaged": False, "arc": False, "max_w": 0.0, "mech": _STATE.get("mech")}


_STATE.update(_fresh())


class Mechanism:
    """Over-centre detents on the isolator's handle (ON at 0, OFF at -90 deg) and the MC4 latch on the socket's slide.
    It wraps the site's write_data_to_sim, so it acts on the joint positions PhysX reported after the last substep."""

    def __init__(self, site):
        import torch

        self.site, self.dev = site, site.device
        names = ["handle_joint", "socket_joint"]
        ids = site.find_joints(names, preserve_order=True)[0]
        self.h, self.s = ids
        site.write_joint_damping_to_sim_index(damping=torch.tensor([[D_HANDLE, D_LATCH]], device=self.dev),
                                              joint_ids=[self.h, self.s])
        self.reset()
        write = site.write_data_to_sim

        def write_with_springs():
            self.update()
            write()

        site.write_data_to_sim = write_with_springs

    def q(self):
        q = self.site.data.joint_pos
        q = getattr(q, "torch", q)[0]
        return float(q[self.h]), float(q[self.s])

    def _apply(self):
        import torch

        tgt_h = (OFF - PRELOAD) if self.handle == OFF_S else PRELOAD
        k_s, d_s = (K_LATCH, D_LATCH) if self.latched else (0.0, D_DRAG)
        self.site.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[K_HANDLE, k_s]], device=self.dev),
                                                     joint_ids=[self.h, self.s])
        self.site.write_joint_damping_to_sim_index(damping=torch.tensor([[D_HANDLE, d_s]], device=self.dev),
                                                   joint_ids=[self.h, self.s])
        self.site.set_joint_position_target_index(target=torch.tensor([[tgt_h, 0.0]], device=self.dev),
                                                  joint_ids=[self.h, self.s])

    def update(self):
        qh, qs = self.q()
        w = -qs
        changed = False
        if self.handle == ON_S and qh < OFF / 2 - SNAP:
            self.handle, changed = OFF_S, True
        elif self.handle == OFF_S and qh > OFF / 2 + SNAP:
            self.handle, changed = ON_S, True
        if self.latched and _STATE["engaged"]:
            self.latched, changed = False, True
        elif not self.latched and not _STATE["engaged"] and w < 1.0 * MM:
            self.latched, changed = True, True
        if changed:
            self._apply()

    def reset(self):
        self.handle, self.latched = ON_S, True
        self._apply()


def tool_engaged(tool) -> bool:
    """The PV-MS's pins pushed onto both clips: the spanner square across the connector, its fork end down (its length
    within 25 deg of vertical), one pin tip each side of the plug (7-12 mm from its axis), 15-26 mm in from the plug's
    front face (in its windows) and within 4 mm of the axis's height."""
    R = tool.rotation()
    if R[2, 0] < math.cos(math.radians(25)):
        return False
    sides = set()
    for t in (tool.to_world(p) for p in g.pin_tips_local()):
        dx, dy, dz = float(t[0]) - g.X0, float(t[1]) - g.AXIS_Y, float(t[2]) - TOP - g.AXIS_Z
        if not (15 * MM <= dx <= 26 * MM) or abs(dz) > 4 * MM:
            return False
        if 7 * MM <= abs(dy) <= 12 * MM:
            sides.add(dy > 0)
    return sides == {True, False}


@on_reset
def _reset(environment) -> None:
    site = environment.unwrapped.scene["site"]
    m = _STATE.get("mech")
    if m is None or m.site is not site:
        m = Mechanism(site)
    _STATE.update(_fresh())
    _STATE["mech"] = m
    m.reset()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states

    st = object_states(environment, ["tool"])
    _STATE["engaged"] = tool_engaged(st["tool"])


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
    15 deg). Otherwise, at the end, with the isolator OFF: 1.0 the halves 20 mm or more apart; 0.5 parted 3 mm or more
    (unlatched); 0.25 still plugged in. 0 with the isolator not OFF."""
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


def demo(isolator_first: bool = True, use_tool: bool = True):
    """Right arm: the isolator's handle a quarter turn anticlockwise to OFF. Left arm: the PV-MS from the table strut,
    pitched up square across the connector and lowered so that its fork goes round the plug's front and its pins into
    the windows; held there. Right arm, tipped in from the -y side: the socket's cap nut drawn 35 mm towards the robot.
    ``isolator_first=False`` leaves the isolator ON (the naive way); ``use_tool=False`` pulls without the spanner."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit
    tiltkit.install()
    down_y = tiltkit.grip(approach=(0, 0, -1), closing=(0, -1, 0))
    down_y_j = tiltkit.grip(approach=(0, 0, -1), closing=(0, -1, 0), joint=True)
    fwd = tiltkit.grip(approach=(1, 0, 0), closing=(0, -1, 0))  # down_y pitched up: the spanner square, fork down
    fwd_j = tiltkit.grip(approach=(1, 0, 0), closing=(0, -1, 0), joint=True)
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
    t_iso = sum(w.duration for w in right)

    # -- left arm: the spanner, gripped by its long edges 40 mm from its middle towards the cap-nut jaw
    GRIP = 0.040
    gx, gy = g.TOOL_POS[0] + GRIP, g.TOOL_POS[1]
    gz = z0 + g.TOOL_POS[2] + 2.4 * MM  # the TCP 2.4 mm above the spanner's middle plane
    tool_open = -(g.TOOL[1] / 2 + 12 * MM)
    # in use: its middle 7 mm in from the plug's front face, the fork's closed end on the plug's top
    cx = g.X0 + 7 * MM
    cz = z0 + g.AXIS_Z + g.D / 2 + (g.TOOL[0] / 2 - g.FORK[1])
    # the TCP (the 2.4 mm offset now along -x); the spanner settles 4 mm towards -y in this pose (measured in a run):
    # aim off by that much
    ux, uy, uz = cx - 2.4 * MM, g.AXIS_Y + 4.0 * MM, cz + GRIP
    left = [
        W(2.5, (gx, gy, gz + 0.06), tool_open, "over the spanner's cap-nut end", down_y_j),
        W(1.5, (gx, gy, gz), tool_open, "fingers beside its edges", down_y),
        W(0.5, (gx, gy, gz), tool_open, "settle", down_y),
        W(0.8, (gx, gy, gz), C, "grip it", down_y),
        W(0.4, (gx, gy, gz), C, "hold", down_y),
        W(1.5, (gx, gy, z0 + 0.17), C, "lift", down_y),
        W(3.0, (ux - 0.04, uy, uz + 0.04), C, "pitch it up, fork down, behind the plug's front", fwd_j),
        W(1.5, (ux, uy, uz + 0.025), C, "fork over the plug's front", fwd),
        W(1.5, (ux, uy, uz), C, "fork round the plug, pins into the windows", fwd),
        W(0.5, (ux, uy, uz - 1.0 * MM), C, "press", fwd),
    ]
    if not use_tool:
        left = [W(sum(w.duration for w in left), (0.15, 0.30, z0 + 0.20), O, "wait (no tool)", 0.0)]
    t_tool = sum(w.duration for w in left)

    # -- right arm: the socket's cap nut, the hand tipped 60 deg in from the -y side (clear of the left hand),
    # fingers closing across it; pull 35 mm towards the robot
    th = math.radians(60)
    a_n = np.array([0.0, math.sin(th), -math.cos(th)])
    nut = tiltkit.grip(approach=tuple(a_n), closing=(0, -math.cos(th), -math.sin(th)))
    nut_j = tiltkit.grip(approach=tuple(a_n), closing=(0, -math.cos(th), -math.sin(th)), joint=True)
    nx = g.SOCKET_REAR_X + (g.SOCKET_CAP[1] - g.SOCKET_CAP[0]) / 2 - 4 * MM  # its rear half, away from the spanner
    nut_open = -18 * MM  # fingers 36 mm apart: clear of the spanner
    nut_p = np.array([nx, g.AXIS_Y, z0 + g.AXIS_Z])
    pull = np.array([-0.035, 0.0, 0.0])
    t_go = max(t_iso, t_tool - 4.0)
    right += [
        W(1.5, (0.25, hy, hz), O, "straight back from the isolator", turn(-90)),
        W(max(0.5, t_go - t_iso - 4.0), (0.25, hy, hz), O, "wait", turn(-90)),
        # the whole arm turns over to the tipped grip here: slowly (a fast joint move makes the soft arm swing)
        W(3.0, (0.23, -0.13, z0 + 0.21), O, "to the side, clear of the spanner", nut_j),
        W(1.5, tuple(nut_p - 0.10 * a_n), nut_open, "in line with the cap nut", nut),
        W(1.5, tuple(nut_p - 0.03 * a_n), nut_open, "towards the cap nut", nut),
        W(1.2, tuple(nut_p), nut_open, "fingers either side of it", nut),
        W(0.5, tuple(nut_p), nut_open, "settle", nut),
        W(0.8, tuple(nut_p), C, "grip", nut),
    ]
    t_r = sum(w.duration for w in right)
    t_ready = max(t_r, t_tool) + 0.3
    right += [
        W(t_ready - t_r, tuple(nut_p), C, "hold", nut),
        W(2.5, tuple(nut_p + pull), C, "draw the socket out", nut),
        W(0.5, tuple(nut_p + pull), C, "hold", nut),
        W(0.8, tuple(nut_p + pull), nut_open, "let go", nut),
        W(1.5, tuple(nut_p + pull - 0.08 * a_n), nut_open, "away", nut),
    ]
    t_r = sum(w.duration for w in right)
    left += [
        W(t_r - sum(w.duration for w in left), (ux, uy, uz - 1.0 * MM), C, "keep the clips pressed", fwd),
        W(1.5, (ux, uy, uz + 0.04), C, "lift the spanner off", fwd),
    ]
    t_l = sum(w.duration for w in left)
    right.append(W(max(0.5, t_l - t_r + 1.0), (0.20, -0.30, z0 + 0.20), O, "back to its side", 0.0))
    left.append(W(1.0, (ux, uy, uz + 0.04), C, "hold", fwd))
    return {"left": left, "right": right}


def self_test():
    """The scoring rules and the tool rule on hand-built states."""
    from partkit import PartState
    from scenekit import BodyState

    def site(angle_deg=0.0, w=0.0):
        return BodyState(pos=np.zeros(3), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=np.zeros(3), high=np.zeros(3),
                         joints={"handle_joint": math.radians(angle_deg), "socket_joint": -w})

    def tool(x, y, z, yaw_deg=0.0, roll_deg=0.0):
        a, r = math.radians(yaw_deg) / 2, math.radians(roll_deg) / 2
        qz = np.array([0, 0, math.sin(a), math.cos(a)])
        qx = np.array([math.sin(r), 0, 0, math.cos(r)])
        x1, y1, z1, w1 = qz
        x2, y2, z2, w2 = qx
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        p = np.array([x, y, z])
        return PartState(pos=p, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)

    def run(history=(), **end):
        _STATE.update(_fresh())
        for h in history:
            score({"site": h})
        s = {"site": end.get("site", site())}
        return score(s), success(s)

    cases = {
        "start": (run(), (0.0, False)),
        "isolator OFF only": (run(site=site(-90)), (0.25, False)),
        "OFF, unplugged 35 mm": (run(history=[site(-90), site(-90, 0.01)], site=site(-90, 0.035)), (1.0, True)),
        "OFF at -80 deg (within 15), unplugged": (run(site=site(-80, 0.03)), (1.0, True)),
        "handle half way (-60), unplugged": (run(site=site(-60, 0.03)), (0.0, False)),
        "OFF, parted 10 mm": (run(history=[site(-90)], site=site(-90, 0.010)), (0.5, False)),
        "unplugged while ON, OFF afterwards": (run(history=[site(0, 0.005)], site=site(-90, 0.035)), (0.0, False)),
        "parted 2 mm while ON (no arc), then done right": (
            run(history=[site(0, 0.002), site(-90)], site=site(-90, 0.03)), (1.0, True)),
        "unplugged with the isolator OFF, then turned back ON": (
            run(history=[site(-90, 0.03)], site=site(0, 0.03)), (0.0, False)),
    }
    for name, (got, want) in cases.items():
        assert abs(got[0] - want[0]) < 1e-9 and got[1] == want[1], (name, got, want)
    _STATE.update(_fresh())
    # the tool rule: square across the connector with its pins in the windows it engages; elsewhere not
    def T(x, y, z, pitch=-90.0, roll=0.0):
        """The spanner pitched about y (default: fork down, pins towards +x) then rolled about world x."""
        p, r = math.radians(pitch) / 2, math.radians(roll) / 2
        qy = np.array([0, math.sin(p), 0, math.cos(p)])
        qx = np.array([math.sin(r), 0, 0, math.cos(r)])
        x1, y1, z1, w1 = qx
        x2, y2, z2, w2 = qy
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        pt = np.array([x, y, z])
        return PartState(pos=pt, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pt, high=pt)

    cz = TOP + g.AXIS_Z + g.D / 2 + (g.TOOL[0] / 2 - g.FORK[1])
    on = T(g.X0 + 7 * MM, g.AXIS_Y, cz)
    assert tool_engaged(on), [on.to_world(p) for p in g.pin_tips_local()]
    assert tool_engaged(T(g.X0 + 3.5 * MM, g.AXIS_Y, cz)) and tool_engaged(T(g.X0 + 13 * MM, g.AXIS_Y, cz))
    assert not tool_engaged(T(g.X0 - 2 * MM, g.AXIS_Y, cz))  # pins short of the windows
    assert not tool_engaged(T(g.X0 + 7 * MM, g.AXIS_Y, cz + 6 * MM))  # not pushed down
    assert not tool_engaged(T(g.X0 + 7 * MM, g.AXIS_Y + 4 * MM, cz))  # both pins on one side
    assert not tool_engaged(T(g.X0 + 7 * MM, g.AXIS_Y, cz, pitch=90.0))  # upside down
    lying = PartState(pos=np.array([g.TOOL_POS[0], g.TOOL_POS[1], TOP + g.TOOL_POS[2]]), quat_xyzw=np.array([0, 0, 0, 1.0]),
                      lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))
    assert not tool_engaged(lying)
    # geometry: the pins clear the plug's narrow front and press the clips (outer 9.4 mm); the fork clears the plug
    assert g.WINDOW_HALF_IN + 1.0 * MM <= g.PIN_IN < g.D / 2
    assert g.FORK[0] / 2 > g.WINDOW_HALF_IN + 2 * MM
    # the spanner rests with its middle over the strut, its pins in front of it and its grip behind it
    assert g.STRUT_X - g.STRUT / 2 < g.TOOL_POS[0] < g.STRUT_X + g.STRUT / 2
    assert g.TOOL_POS[0] + g.PIN_X + g.PIN[0] / 2 < g.STRUT_X - g.STRUT / 2 - 5 * MM
    return len(cases) + 7


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
