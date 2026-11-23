"""Assemble a three-section shock-corded tent pole lying on the table, then let go of it.

The pole is three DAC Featherlite NSL 9 mm aluminium sections, 40 cm each, strung on a 3 mm shock cord that runs
through all three and is tied at both ends (1.20 m assembled). Section 1 (an endpiece) has an insert at its inner end;
section 2 (the centre piece) has a socket at one end and an insert at the other; section 3 (the other endpiece) has a
socket. An insert (8.3 mm, standing 30 mm out of its tube) goes home into the next section's socket (8.5 mm bore)
when the two tube ends meet.

At the start the pole lies folded on the table in front of the robot, as it comes out of its bag: section 2 across
the table, sections 1 and 3 folded back alongside it at their joints (each insert's tip beside its socket's mouth),
fanned away from it by 20 and 15 degrees on the far side, section 3 lying over section 1. The shock cord pulls every
joint together all the time (about 4 N assembled, 4.6 N folded).

Joint states: home: the insert in the socket's bore with the tube ends within 1 mm of each other (1 mm feeler);
started: the insert at least 10 mm into the bore (no more than 20 mm of it showing); otherwise open.

Score (end of the trial): 1.0 if both joints are home and the robot has let go of the pole (no gripper within 3 cm
of it) for 1 s; otherwise 0.4 for each joint home and 0.2 for each joint started.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import dims as d  # noqa: E402
from scenekit import UsdBody, on_reset, on_step  # noqa: E402
from yam_arena import station as _station  # noqa: E402

SCENE = "station"
TOP = _station.TABLE_TOP
E = os.environ.get

# -- the start --------------------------------------------------------------------------------------------------------
XP = 0.30  # the assembled pole's axis, along y, this far in front of the arms' bases
FAN1 = math.radians(20.0)  # section 1 folded back beside section 2, fanned out by this
FAN3 = math.radians(15.0)  # section 3 likewise at the other joint
SIDE = d.R_SOCK + d.R_INS + 0.00005  # at a fold, the insert's tip this far beside the socket's mouth (axis to axis)
CORD_RAMP_S = 0.3  # s, after a reset
Z_AXIS = TOP + d.R_TUBE + 0.0005  # the sections' axis, just above the table


def _rot_from_axis(u):
    """The rotation (matrix) turning a section's z onto u, its y as near up as it goes."""
    import numpy as np

    z = np.asarray(u, float) / np.linalg.norm(u)
    x = np.cross([0.0, 0.0, 1.0], z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def start_poses():
    """Each section's (origin, rotation matrix) at the start (station frame)."""
    import numpy as np

    u2 = np.array([0.0, 1.0, 0.0])
    b1 = np.array([XP, -d.L / 2, Z_AXIS])  # section 2's mouth
    u1 = -np.array([math.sin(FAN1), math.cos(FAN1), 0.0])  # section 1 points back at its tip
    a1 = b1 + np.array([SIDE, 0.0, 0.0]) + d.R_INS * u1  # its insert beside the mouth
    o1 = a1 - d.THROUGH[0] * u1
    a2 = b1 + d.THROUGH[1] * u2  # section 2's insert tip
    o3 = a2 - d.R_INS * u2 + np.array([SIDE, 0.0, d.R_SOCK - d.R_TUBE])  # section 3's mouth beside the insert's end
    plan3 = np.array([math.sin(FAN3), -math.cos(FAN3)])
    # section 3 crosses over section 1: its axis rises from the mouth to clear section 1 where they cross
    m = np.column_stack([plan3, u1[:2]])  # o3 + s plan3 = a1 - t u1: s along section 3, t back along section 1
    s_cross, t_neg = np.linalg.solve(m, (a1 - o3)[:2])
    rise = (Z_AXIS + 2 * d.R_TUBE + 0.0005) - o3[2]
    beta = math.atan2(rise, s_cross) if 0.0 < s_cross < d.THROUGH[2] and 0.0 < t_neg < d.THROUGH[0] else 0.0
    u3 = np.array([plan3[0] * math.cos(beta), plan3[1] * math.cos(beta), math.sin(beta)])
    return [(o1, _rot_from_axis(u1)), (b1, _rot_from_axis(u2)), (o3, _rot_from_axis(u3))]


def _xyzw(rot):
    from yam_arena.ik import matrix_to_quat_xyzw

    return tuple(float(v) for v in matrix_to_quat_xyzw(rot))


_START = start_poses()
SECTIONS = [UsdBody(f"section_{k + 1}", usd=f"assets/section_{k + 1}.usda", pos=tuple(float(v) for v in o),
                    rot_xyzw=_xyzw(r), size=(0.01, 0.01, d.THROUGH[k]), mass=d.MASS[k], friction=d.MU_AL,
                    solver_iters=32, torsional_patch=0.003) for k, (o, r) in enumerate(_START)]
OBJECTS = list(SECTIONS)

HOME_GAP = 0.001  # m: the tube ends within this of each other
STARTED = 0.010  # m: the insert at least this far into the bore
IN_BORE = 0.001  # m: the insert's tip within this of the socket's axis
LET_GO = 0.03  # m: no gripper (TCP) within this of the pole's axis
SUCCESS_HOLD_S = 1.0

_S: dict = {}


def _np(x):
    import numpy as np

    x = getattr(x, "torch", x)
    return np.asarray(x.detach().cpu() if hasattr(x, "detach") else x, dtype=float)


def _quat_to_rot(q):
    import numpy as np

    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- the joints and the shock cord -------------------------------------------------------------------------------------

def joint_geometry(insert_pose, socket_pose):
    """(depth, offset, tip, mouth, socket axis) of a joint: how far the insert's tip is into the socket along its axis
    (m; negative: short of the mouth), and how far the tip is off that axis."""
    import numpy as np

    (pi, ri), (ps, rs) = insert_pose, socket_pose
    tip = pi + ri[:, 2] * (d.L + d.INS_OUT)
    u = rs[:, 2]
    rel = tip - ps
    depth = float(rel @ u)
    return depth, float(np.linalg.norm(rel - depth * u)), tip, ps, u


def joint_state(depth: float, offset: float) -> str:
    """'home', 'started' or 'open' from a joint's depth and offset (m)."""
    if offset > IN_BORE or depth < STARTED:
        return "open"
    return "home" if depth >= d.INS_OUT - HOME_GAP else "started"


def cord_forces(poses, last_path=None, dt=1.0 / 240.0, ramp=1.0):
    """The shock cord's pull on each section: (forces, application points, tension, path length). The cord runs along
    each section's axis and crosses each joint from the insert's tip to the socket's mouth, or, once the tip is in the
    bore, on along the socket's axis; tied at both ends of the pole, it pulls each section where it leaves it, along
    the cord."""
    import numpy as np

    path = sum(d.THROUGH)
    crossings = []
    for j in range(d.N_SECTIONS - 1):
        depth, offset, tip, mouth, u = joint_geometry(poses[j], poses[j + 1])
        if depth > 0.0 and offset < d.R_BORE:
            path -= depth
            crossings.append((j, tip, tip, u))  # pulls the insert on into the bore
        else:  # the cord crosses from the insert to the mouth
            end = tip - poses[j][1][:, 2] * d.R_INS
            gap = end - mouth
            span = float(np.linalg.norm(gap))
            path += span - d.R_INS
            crossings.append((j, end, mouth, -gap / span if span > 1e-9 else u))
    rate = 0.0 if last_path is None else (path - last_path) / dt
    tension = ramp * d.cord_tension(path, rate)
    forces = [[] for _ in range(d.N_SECTIONS)]  # (force, point) on each section
    for j, at_insert, at_socket, e in crossings:
        forces[j].append((tension * e, at_insert))
        forces[j + 1].append((-tension * e, at_socket))
    return forces, tension, path


def _poses(environment):
    sc = environment.unwrapped.scene
    o = _S["origin"]
    out = []
    for spec in SECTIONS:
        body = sc[spec.name]
        out.append((_np(body.data.root_link_pos_w)[0] - o, _quat_to_rot(_np(body.data.root_link_quat_w)[0])))
    return out


def _apply_cord() -> None:
    """Before every physics substep: the cord's pull, as a force and torque about each section's centre of mass."""
    import numpy as np
    import torch

    environment = _S["env"]
    poses = _poses(environment)
    forces, tension, path = cord_forces(poses, _S.get("path"), ramp=min(1.0, _S["t"] / CORD_RAMP_S))
    _S.update(path=path, tension=tension, t=_S["t"] + 1.0 / 240.0)
    for spec, fs in zip(SECTIONS, forces):
        body = environment.unwrapped.scene[spec.name]
        com = _np(body.data.root_com_pos_w)[0] - _S["origin"]
        f = sum((f for f, _ in fs), np.zeros(3))
        tq = sum((np.cross(p - com, f) for f, p in fs), np.zeros(3))
        body.permanent_wrench_composer.set_forces_and_torques_index(
            forces=torch.tensor([[f.tolist()]], dtype=torch.float32, device=body.device),
            torques=torch.tensor([[tq.tolist()]], dtype=torch.float32, device=body.device), body_ids=[0], is_global=True)


# -- scoring ---------------------------------------------------------------------------------------------------------

def grade(states, let_go_s: float) -> float:
    """The score from the joints' states and how long (s) the robot has been off the pole with both joints home."""
    if all(s == "home" for s in states) and let_go_s >= SUCCESS_HOLD_S:
        return 1.0
    return sum(0.4 if s == "home" else 0.2 if s == "started" else 0.0 for s in states)


def _segment_distance(p, a, b) -> float:
    import numpy as np

    ab = b - a
    t = min(1.0, max(0.0, float((p - a) @ ab / (ab @ ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


def _states(environment):
    poses = _poses(environment)
    return poses, [joint_state(*joint_geometry(poses[j], poses[j + 1])[:2]) for j in range(d.N_SECTIONS - 1)]


def _holding(environment, poses) -> bool:
    """Whether a gripper's TCP is within LET_GO of any section's axis (its insert included)."""
    from scenekit import arm_joints
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    for side in ("left", "right"):
        tcp = ik.tcp_pose(arm_joints(environment, side, ARM_JOINTS), _station.BASES[side])[0]
        for k, (p, r) in enumerate(poses):
            if _segment_distance(tcp, p, p + r[:, 2] * d.THROUGH[k]) < LET_GO:
                return True
    return False


def _score_now() -> float:
    from scenekit import env

    environment = env()
    if environment is None or "origin" not in _S:
        return 0.0
    _, states = _states(environment)
    return grade(states, _S.get("let_go_s", 0.0))


def score(objects) -> float:
    return _score_now()


def success(objects) -> bool:
    return _score_now() >= 1.0


# -- the simulation: reset, the cord, the watch -----------------------------------------------------------------------

@on_reset
def _reset(environment) -> None:
    import torch

    sc = environment.unwrapped.scene
    o = _np(sc.env_origins[0])
    for spec in SECTIONS:
        body = sc[spec.name]
        body.write_root_pose_to_sim_index(root_pose=torch.tensor(
            [[o[0] + spec.pos[0], o[1] + spec.pos[1], o[2] + spec.pos[2], *spec.rot_xyzw]], dtype=torch.float32,
            device=body.device))
        body.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=body.device))
    wrapped = _S.get("wrapped", False)
    _S.clear()
    _S.update(t=0.0, step=0, origin=o, env=environment, let_go_s=0.0, t_watch=0.0, wrapped=wrapped)
    if not _S.get("wrapped"):  # the cord's pull, before every physics substep
        first = sc[SECTIONS[0].name]
        write = first.write_data_to_sim

        def write_with_cord():
            _apply_cord()
            write()

        first.write_data_to_sim = write_with_cord
        _S["wrapped"] = True


@on_step
def _watch(environment) -> None:
    _S["step"] += 1
    poses, states = _states(environment)
    held = _holding(environment, poses)
    dt, _S["t_watch"] = _S["t"] - _S["t_watch"], _S["t"]  # since the last control step (physics time)
    _S["let_go_s"] = _S["let_go_s"] + dt if (not held and all(s == "home" for s in states)) else 0.0
    if E("POLE_LOG") and _S["step"] % 5 == 0:
        parts = []
        for j in range(d.N_SECTIONS - 1):
            depth, offset, *_ = joint_geometry(poses[j], poses[j + 1])
            parts.append(f"j{j + 1}={states[j]}(depth={depth * 1000:+.1f}mm off={offset * 1000:.2f}mm)")
        print(f"LOG t={_S['t']:.2f} {' '.join(parts)} cord={_S.get('tension', 0.0):.2f}N held={held} "
              f"let_go={_S['let_go_s']:.2f}s score={_score_now()}", flush=True)


# -- offline checks --------------------------------------------------------------------------------------------------

def self_test() -> int:
    """The scoring rules, the joints' geometry, the cord's pull and the start layout (no GPU)."""
    import numpy as np

    cases = 0

    def check(cond, what):
        nonlocal cases
        assert cond, what
        cases += 1

    check(grade(["home", "home"], 1.0) == 1.0, "both home, let go 1 s: 1.0")
    check(grade(["home", "home"], 5.0) == 1.0, "both home, let go long ago: 1.0")
    check(abs(grade(["home", "home"], 0.5) - 0.8) < 1e-9, "both home, let go only 0.5 s: 0.8")
    check(abs(grade(["home", "home"], 0.0) - 0.8) < 1e-9, "both home, still held: 0.8")
    check(abs(grade(["home", "started"], 2.0) - 0.6) < 1e-9, "one home, one started: 0.6")
    check(abs(grade(["home", "open"], 2.0) - 0.4) < 1e-9, "one home: 0.4")
    check(abs(grade(["started", "started"], 2.0) - 0.4) < 1e-9, "both started: 0.4")
    check(abs(grade(["open", "started"], 2.0) - 0.2) < 1e-9, "one started: 0.2")
    check(grade(["open", "open"], 2.0) == 0.0, "nothing done: 0")
    check(joint_state(d.INS_OUT, 0.0001) == "home", "insert all the way in: home")
    check(joint_state(d.INS_OUT - HOME_GAP, 0.0001) == "home", "ends 1 mm apart: home")
    check(joint_state(d.INS_OUT - 0.002, 0.0001) == "started", "ends 2 mm apart: started")
    check(joint_state(STARTED, 0.0001) == "started", "10 mm in: started")
    check(joint_state(0.009, 0.0001) == "open", "9 mm in: open")
    check(joint_state(-0.001, 0.0) == "open", "short of the mouth: open")
    check(joint_state(d.INS_OUT, 0.003) == "open", "tip beside the socket, not in its bore: open")
    # the joints' geometry: assembled, the tips 30 mm in; at the start, folded and open
    straight = [(np.array([XP, -d.L * 1.5 + d.L * k, Z_AXIS]), _rot_from_axis((0, 1, 0))) for k in range(3)]
    for j in range(2):
        depth, offset, *_ = joint_geometry(straight[j], straight[j + 1])
        check(abs(depth - d.INS_OUT) < 1e-9 and offset < 1e-9, f"assembled joint {j + 1}: home")
    for j in range(2):
        depth, offset, *_ = joint_geometry(_START[j], _START[j + 1])
        check(joint_state(depth, offset) == "open", f"start joint {j + 1}: open")
        check(offset > d.R_BORE + 0.002, f"start joint {j + 1}: folded, the tip beside the mouth ({offset * 1000:.1f} mm)")

    # folded: no two sections overlap (their axes far enough apart all along), section 3 over section 1 where they cross
    def gap(k1, k2, n=200):
        (p1, r1), (p2, r2) = _START[k1], _START[k2]
        ends = [d.THROUGH[k] - (d.R_INS if d.HAS_INSERT[k] else 0.0) for k in (k1, k2)]
        a = [p1 + r1[:, 2] * t for t in np.linspace(0, ends[0], n)]
        b = [p2 + r2[:, 2] * t for t in np.linspace(0, ends[1], n)]
        return min(float(np.linalg.norm(x - y)) for x in a for y in b)

    check(gap(0, 1) > d.R_INS + d.R_SOCK, f"sections 1 and 2 apart ({gap(0, 1) * 1000:.2f} mm)")
    check(gap(1, 2) > d.R_INS + d.R_SOCK, f"sections 2 and 3 apart ({gap(1, 2) * 1000:.2f} mm)")
    check(gap(0, 2) > 2 * d.R_TUBE, f"section 3 clear over section 1 ({gap(0, 2) * 1000:.2f} mm)")
    check(_START[2][1][2, 2] > 0.0, "section 3 rises over section 1")
    # the cord: about 4 N assembled; more when the pole is bent at the joints; never pushes
    f, t_home, p_home = cord_forces(straight)
    check(abs(p_home - d.POLE_LEN) < 1e-9, "assembled: the cord's path is the pole's length")
    check(3.5 < t_home < 4.2, f"assembled tension {t_home:.2f} N")
    check(np.allclose(sum((x for x, _ in f[1]), np.zeros(3)), 0.0, atol=1e-9), "assembled: no net pull on the middle")
    check(f[0][0][0][1] > 0 and f[2][0][0][1] < 0, "assembled: the cord pulls the end sections in towards the middle")
    f0, t0, p0 = cord_forces(_START)
    check(t0 > t_home and p0 > p_home, f"start: the cord stretched further ({t0:.2f} N)")
    total = sum((x for fs in f0 for x, _ in fs), np.zeros(3))
    check(np.allclose(total, 0.0, atol=1e-9), "the cord's pulls cancel over the whole pole")
    check(d.cord_tension(d.CORD_REST - 0.01) == 0.0, "a slack cord pulls nothing")
    check(abs(d.cord_tension(2 * d.CORD_REST) - d.CORD_F100) < 1e-9, "100 % stretch: the datasheet's load")
    # the fit
    check(d.R_BORE - d.R_INS > 0 and d.R_BORE - d.R_INS < 0.0002, "the insert fits its bore with 0.1 mm a side")
    check(d.SOCK > d.INS_OUT, "the socket is deeper than the insert")
    check(d.R_SOCK - d.R_TUBE < 0.0005, "the start height clears the socket's wider end")
    # the start on the table and within the arms' reach
    for k, (o, r) in enumerate(_START):
        for end in (o, o + r[:, 2] * d.THROUGH[k]):
            check(abs(end[1]) < _station.TABLE_SIZE[1] / 2 - 0.03, f"section {k + 1} on the table (y)")
            check(_station.TABLE_CENTER[0] - _station.TABLE_SIZE[0] / 2 + 0.05 < end[0], f"section {k + 1} on the table (x)")
            check(min(math.dist(end[:2], b[:2]) for b in _station.BASES.values()) > 0.20, f"section {k + 1} clear of the bases")
    return cases


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, the joints, the cord and the start layout.
    print(f"self-test: {self_test()} cases pass")
