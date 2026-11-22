"""icekit: a few hundred loose ice cubes for a yam_arena task file (with ``scenekit``), after soilkit.

Isaac Lab tracks each scene asset as one body; a pan of ice is hundreds. An :class:`Ice` spawns as one fixed
(non-physics) asset whose spawner authors every cube straight into USD as its own rigid body with a convex-hull mesh
collider (``profile``, a closed polygon in the cube's y-z plane, extruded ``length`` along its x axis), simulated by
PhysX like any other body. After every control step all cube poses are read through one PhysX rigid-body view
(:func:`cubes`); at every reset after the first they are put back where they started.

The cubes start from ``start_file`` (an ``.npz`` of settled centres and orientations; ``ICE_SAVE=<path>`` writes one
every 20 control steps) or, without one, from a loose jittered lattice filling the box ``pos`` +- ``size``/2, which
settles in the first seconds. Pure Python at import time.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

import scenekit

HERE = Path(__file__).resolve().parent
Color = tuple[float, float, float]


@dataclass(frozen=True)
class Ice(Box):
    """Cubes filling the box ``pos`` +- ``size``/2 (station frame, m)."""

    profile: tuple = ()  # (y, z) points of the cross-section, about the cube's centre (m)
    length: float = 0.038
    density: float = 917.0
    friction: tuple[float, float] = (0.2, 0.1)
    combine: str = "min"
    color: Color = (0.86, 0.93, 0.97)
    seed: int = 0
    layered: bool = False  # lattice: long cubes packed lengthwise, each layer turned a quarter turn on the one below
    start_file: str | None = None
    solver_iters: int = 8

    def _check(self) -> None:
        super()._check()
        if len(self.profile) < 3:
            raise ValueError(f"{self.name}: the profile needs at least three points")


_STATE: dict[str, dict] = {}


def mesh(s: Ice) -> tuple[np.ndarray, list[int], list[int]]:
    """(points, face vertex counts, face vertex indices) of one cube."""
    prof = np.asarray(s.profile, float)
    n = len(prof)
    pts = np.array([(sx * s.length / 2, y, z) for sx in (-1, 1) for y, z in prof])
    counts, idx = [n, n], list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
    for k in range(n):
        j = (k + 1) % n
        counts.append(4)
        idx += [k, j, n + j, n + k]
    return pts, counts, idx


def volume(s: Ice) -> float:
    y, z = np.asarray(s.profile, float).T
    return float(abs(np.dot(y, np.roll(z, -1)) - np.dot(z, np.roll(y, -1))) / 2 * s.length)


def lattice(s: Ice) -> tuple[np.ndarray, np.ndarray]:
    """(centres N x 3, orientations N x 4 xyzw): a loose lattice filling the box, each cube lying flat, turned at
    random about z and flipped at random."""
    rng = np.random.default_rng(s.seed)
    prof = np.asarray(s.profile, float)
    if s.layered:
        return _layered(s, rng, prof)
    span = max(s.length, np.ptp(prof[:, 0]))
    cell = np.array([span + 4e-3, span + 4e-3, np.ptp(prof[:, 1]) + 6e-3])
    lo = np.asarray(s.pos, float) - np.asarray(s.size, float) / 2 + cell / 2
    hi = np.asarray(s.pos, float) + np.asarray(s.size, float) / 2 - cell / 2
    pts = [(x, y, z) for z in np.arange(lo[2], hi[2] + 1e-9, cell[2]) for x in np.arange(lo[0], hi[0] + 1e-9, cell[0])
           for y in np.arange(lo[1], hi[1] + 1e-9, cell[1])]
    p = np.array(pts) + rng.uniform(-1, 1, (len(pts), 3)) * np.array([1.5e-3, 1.5e-3, 0.0])
    yaw = rng.uniform(0, 2 * np.pi, len(p))
    flip = rng.integers(0, 2, len(p))  # flat side down or up: a half turn about x
    q = []
    for a, f in zip(yaw, flip):
        qz = np.array([0.0, 0.0, np.sin(a / 2), np.cos(a / 2)])
        qx = np.array([1.0, 0.0, 0.0, 0.0]) if f else np.array([0.0, 0.0, 0.0, 1.0])
        q.append(_qmul(qz, qx))
    return p, np.array(q)


def _layered(s: Ice, rng, prof) -> tuple[np.ndarray, np.ndarray]:
    """The lattice for long cubes: in each layer the cubes lie side by side, their flat sides (the profile's y) along
    one axis, turned up to 15 degrees either way, flipped at random; each layer a quarter turn on the one below."""
    jitter = np.radians(15)
    a, b = np.ptp(prof[:, 0]), s.length  # the cube's long and short sides seen from above
    long_ = a * np.cos(jitter) + b * np.sin(jitter) + 4e-3
    short = a * np.sin(jitter) + b * np.cos(jitter) + 4e-3
    dz = np.ptp(prof[:, 1]) + 6e-3
    lo = np.asarray(s.pos, float) - np.asarray(s.size, float) / 2
    hi = np.asarray(s.pos, float) + np.asarray(s.size, float) / 2
    p, q = [], []
    for k, z in enumerate(np.arange(lo[2] + dz / 2, hi[2] - dz / 2 + 1e-9, dz)):
        cx, cy = (short, long_) if k % 2 == 0 else (long_, short)  # the flat side along y, then along x
        base = 0.0 if k % 2 == 0 else np.pi / 2
        for x in np.arange(lo[0] + cx / 2, hi[0] - cx / 2 + 1e-9, cx):
            for y in np.arange(lo[1] + cy / 2, hi[1] - cy / 2 + 1e-9, cy):
                yaw = base + rng.uniform(-jitter, jitter) + np.pi * rng.integers(0, 2)
                qz = np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])
                qx = np.array([1.0, 0.0, 0.0, 0.0]) if rng.integers(0, 2) else np.array([0.0, 0.0, 0.0, 1.0])
                p.append((x, y, z))
                q.append(_qmul(qz, qx))
    return np.array(p), np.array(q)


def _qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def start_layout(s: Ice) -> tuple[np.ndarray, np.ndarray]:
    if s.start_file and (HERE / s.start_file).is_file():
        data = np.load(HERE / s.start_file)
        return data["pos"].astype(float), data["quat"].astype(float)
    return lattice(s)


def _author(prim_path: str, s: Ice) -> None:
    """Author the cubes under ``prim_path`` (one env's ice prim), in that env's frame."""
    import isaaclab.sim as sim_utils
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

    from yam_arena.station import linear_rgb

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, prim_path)
    pos, quat = start_layout(s)
    mat_path = f"{prim_path}/physics"
    pm = UsdPhysics.MaterialAPI.Apply(UsdShade.Material.Define(stage, mat_path).GetPrim())
    pm.CreateStaticFrictionAttr(float(s.friction[0]))
    pm.CreateDynamicFrictionAttr(float(s.friction[1]))
    pm.CreateRestitutionAttr(0.0)
    pm.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set(s.combine)
    look = f"{prim_path}/look"
    cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(s.color), roughness=0.15)
    cfg.func(look, cfg)
    pts, counts, idx = mesh(s)
    points = Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts])
    lo, hi = pts.min(0), pts.max(0)
    extent = Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
    mass = float(s.density * volume(s))
    layer = stage.GetEditTarget().GetLayer()
    with Sdf.ChangeBlock():
        for i, (p, q) in enumerate(zip(pos, quat)):
            prim = Sdf.CreatePrimInLayer(layer, f"{prim_path}/c{i:04d}")
            prim.specifier = Sdf.SpecifierDef
            prim.typeName = "Mesh"
            prim.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(
                ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysicsMassAPI",
                 "PhysxRigidBodyAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]))

            def attr(name, tname, value, prim=prim):
                Sdf.AttributeSpec(prim, name, tname).default = value

            attr("points", Sdf.ValueTypeNames.Point3fArray, points)
            attr("faceVertexCounts", Sdf.ValueTypeNames.IntArray, Vt.IntArray(counts))
            attr("faceVertexIndices", Sdf.ValueTypeNames.IntArray, Vt.IntArray(idx))
            attr("extent", Sdf.ValueTypeNames.Float3Array, extent)
            attr("physics:approximation", Sdf.ValueTypeNames.Token, "convexHull")
            attr("xformOp:translate", Sdf.ValueTypeNames.Double3, Gf.Vec3d(*map(float, p)))
            attr("xformOp:orient", Sdf.ValueTypeNames.Quatd, Gf.Quatd(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
            attr("xformOpOrder", Sdf.ValueTypeNames.TokenArray, ["xformOp:translate", "xformOp:orient"])
            attr("physics:mass", Sdf.ValueTypeNames.Float, mass)
            attr("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, 0.5)
            attr("physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float, 0.05)
            attr("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float, 0.5)
            attr("physxRigidBody:maxLinearVelocity", Sdf.ValueTypeNames.Float, 3.0)
            attr("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, int(s.solver_iters))
            attr("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1)
            attr("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 5.0e-5)
            attr("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, 0.002)
            attr("physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.0)
            rel = Sdf.RelationshipSpec(prim, "material:binding", False)
            rel.targetPathList.explicitItems = [Sdf.Path(look)]
            rel = Sdf.RelationshipSpec(prim, "material:binding:physics", False)
            rel.targetPathList.explicitItems = [Sdf.Path(mat_path)]
    _STATE[s.name] = {"root": prim_path, "start": pos.copy(), "qstart": quat.copy(), "now": pos.copy(),
                      "qnow": quat.copy(), "view": None, "steps": 0, "episodes": 0}
    print(f"[ice] {s.name}: {len(pos)} cubes of {mass * 1000:.1f} g under {prim_path}", flush=True)


def _build(s: Ice):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        _author(prim_path, s)
        import omni.usd

        return omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)

    @configclass
    class IceCfg(sim_utils.SpawnerCfg):
        func: object = spawn

    obj = Object(name=s.name, object_type=ObjectType.BASE, spawner_cfg=IceCfg())
    obj.set_initial_pose(Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))
    return obj


def _view(name: str):
    st = _STATE[name]
    if st["view"] is None:
        from isaaclab_physx.physics import PhysxManager

        st["view"] = PhysxManager.get_physics_sim_view().create_rigid_body_view(f"{st['root']}/c*")
        if st["view"].count != len(st["start"]):
            raise RuntimeError(f"ice {name}: the view found {st['view'].count} of {len(st['start'])} cubes")
    return st["view"]


def cubes(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(centres N x 3 in the station frame, orientations N x 4 xyzw) after the last control step."""
    st = _STATE[name]
    return st["now"], st["qnow"]


def _state(environment, s: Ice) -> ObjectState:
    p, _ = cubes(s.name)
    low, high = p.min(0), p.max(0)
    return ObjectState(pos=(low + high) / 2, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=low, high=high)


scenekit.KINDS[Ice] = (_build, _state)


@scenekit.on_step
def _read(environment) -> None:
    try:
        _read_now(environment)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


def _read_now(environment) -> None:
    import warp as wp

    origin = scenekit._np(environment.unwrapped.scene.env_origins[0])
    for name, st in _STATE.items():
        view = _view(name)
        tf = wp.to_torch(view.get_transforms()).cpu().numpy().astype(float)
        st["now"], st["qnow"] = tf[:, :3] - origin, tf[:, 3:7]
        st["steps"] += 1
        save = os.environ.get("ICE_SAVE")
        if save and st["steps"] % 20 == 0:
            v = np.linalg.norm(wp.to_torch(view.get_velocities()).cpu().numpy()[:, :3], axis=1)
            np.savez(save, pos=st["now"], quat=st["qnow"])
            print(f"[ice] step {st['steps']}: saved; top {st['now'][:, 2].max():.4f}, max speed {v.max():.4f} m/s",
                  flush=True)


@scenekit.on_reset
def _put_back(environment) -> None:
    try:
        _put_back_now(environment)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


def _put_back_now(environment) -> None:
    import torch
    import warp as wp

    for name, st in _STATE.items():
        st["episodes"] += 1
        st["steps"] = 0
        if st["episodes"] == 1:  # the first episode: they are where they were authored
            continue
        view = _view(name)
        n = len(st["start"])
        origin = scenekit._np(environment.unwrapped.scene.env_origins[0])
        pose = torch.zeros((n, 7), dtype=torch.float32, device="cuda:0")
        pose[:, :3] = torch.tensor(st["start"] + origin, dtype=torch.float32)
        pose[:, 3:7] = torch.tensor(st["qstart"], dtype=torch.float32)
        idx = wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32, device="cuda:0")
        view.set_transforms(wp.from_torch(pose), indices=idx)
        view.set_velocities(wp.from_torch(torch.zeros((n, 6), dtype=torch.float32, device="cuda:0")), indices=idx)
        st["now"], st["qnow"] = st["start"].copy(), st["qstart"].copy()
