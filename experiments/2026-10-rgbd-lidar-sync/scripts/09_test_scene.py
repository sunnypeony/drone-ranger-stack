#!/usr/bin/env python3
"""Stage 09: Isaac Sim drop test scene (usd-core only).

results/v2/test_drop.usda    -> references ./scene_collision.usdc (v2)
results/v2/test_drop_v1.usda -> references ../scene_collision.usdc (v1)
Each: PhysicsScene (gravity -Z 9.81), three 0.2 m rigid cubes (above the table centre, above the table
edge, above open ground), a DistantLight; metres, Z up. Cube positions come from the table detected in
the corresponding mesh; a downward raycast reports what each cube should land on."""
import os

import numpy as np

from common import exp_path, load_config, parse_args, write_json
from scene_utils import find_table


def raycast_down(scene, xy, z0):
    import open3d as o3d
    ray = o3d.core.Tensor([[xy[0], xy[1], z0, 0, 0, -1]], dtype=o3d.core.Dtype.Float32)
    t = scene.cast_rays(ray)["t_hit"].numpy()[0]
    return None if not np.isfinite(t) else float(z0 - t)


def cube_positions(mesh, traj_xy, T):
    import open3d as o3d
    V = np.asarray(mesh.vertices)
    tab = find_table(V, traj_xy, T)
    if tab is None:
        raise SystemExit("table not found; set test_scene.table_center in the config")
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    c = np.array(tab["center"])
    top = V[:, 2].max() + 1.0
    hc = raycast_down(scene, c, top)
    c_drop, moved = c, False
    lo, hi = T["table_band"]
    if hc is None or not lo <= hc <= hi:
        # hole in the table top at the centre (inconsistent views carve the TSDF): drop the table cube
        # on the nearest grid point whose first hit is inside the table band, and record that it moved
        g = np.arange(-tab["radius"], tab["radius"] + 1e-9, 0.02)
        cand = sorted((float(np.hypot(dx, dy)), dx, dy) for dx in g for dy in g if np.hypot(dx, dy) <= tab["radius"])
        for _, dx, dy in cand:
            h = raycast_down(scene, c + [dx, dy], top)
            if h is not None and lo <= h <= hi:
                c_drop, hc, moved = c + np.array([dx, dy]), h, True
                break
    tab["table_cube_moved_off_centre_m"] = float(np.linalg.norm(c_drop - c)) if moved else 0.0
    # table edge: the direction whose edge point still hits something elevated
    edge, he = None, None
    for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
        p = c + tab["radius"] * np.array([np.cos(ang), np.sin(ang)])
        h = raycast_down(scene, p, top)
        if h is not None and h > T["table_band"][0]:
            edge, he = p, h
            break
    if edge is None:
        edge = c + tab["radius"] * np.array([1.0, 0.0])
        he = raycast_down(scene, edge, top)
    # open ground: ground_offset away from the table, first direction that hits near z=0
    ground, hg = None, None
    for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
        p = c + T["ground_offset"] * np.array([np.cos(ang), np.sin(ang)])
        h = raycast_down(scene, p, top)
        if h is not None and abs(h) < T["table_band"][0] / 2:
            ground, hg = p, h
            break
    if ground is None:
        raise SystemExit("no open ground found around the table")
    d = T["drop_height"]
    cubes = {
        "cube_table": [*c_drop, (hc if hc is not None else tab["top_z"]) + d],
        "cube_table_edge": [*edge, (he if he is not None else tab["top_z"]) + d],
        "cube_ground": [*ground, hg + d],
    }
    below = {"cube_table": hc, "cube_table_edge": he, "cube_ground": hg}
    return tab, cubes, below


def write_scene(path, env_ref, cubes, T):
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    env = UsdGeom.Xform.Define(stage, "/World/Environment")
    env.GetPrim().GetReferences().AddReference(env_ref)
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(T["gravity"])
    colors = {"cube_table": (0.9, 0.1, 0.1), "cube_table_edge": (0.1, 0.4, 0.9), "cube_ground": (0.1, 0.8, 0.2)}
    for name, pos in cubes.items():
        cube = UsdGeom.Cube.Define(stage, f"/World/{name}")
        cube.CreateSizeAttr(T["cube_size"])
        cube.CreateExtentAttr([Gf.Vec3f(-T["cube_size"] / 2), Gf.Vec3f(T["cube_size"] / 2)])
        cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colors[name])])
        prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(T["cube_mass"])
    light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    light.CreateIntensityAttr(T["light_intensity"])
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 30))
    stage.GetRootLayer().Save()
    # check composition: referenced collision mesh must be reachable
    st = Usd.Stage.Open(str(path))
    mesh = st.GetPrimAtPath("/World/Environment/Scene/mesh")
    if not mesh or not mesh.HasAPI(UsdPhysics.CollisionAPI):
        raise SystemExit(f"{path}: reference {env_ref} did not compose a collision mesh")
    return [str(p.GetPath()) for p in st.Traverse()]


def main():
    import open3d as o3d
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    T = cfg["test_scene"]
    res = exp_path(cfg["paths"]["results_dir"])
    v1_out = exp_path(cfg["paths"]["v1_outputs_dir"])
    v2_out = exp_path(cfg["paths"]["outputs_dir"])
    v1_usd = exp_path("results") / cfg["usd"]["filename"]
    jobs = {
        res.name: (v2_out, res / "test_drop.usda", res / cfg["usd"]["filename"]),  # "v2" / "v3"
        "v1": (v1_out, res / "test_drop_v1.usda", v1_usd),
    }
    rep = {}
    for name, (od, path, usd) in jobs.items():
        mesh = o3d.io.read_triangle_mesh(str(od / "mesh" / "mesh_clean.ply"))
        traj = np.load(od / "mesh" / "poses_aligned.npz")["T_world_optical"][:, :2, 3]
        tab, cubes, below = cube_positions(mesh, traj, T)
        ref = os.path.relpath(usd, path.parent)
        prims = write_scene(path, ref, cubes, T)
        rep[name] = {"file": str(path), "reference": ref, "table": tab, "cubes": cubes,
                     "surface_z_below_cube": below, "prims": prims}
        print(f"[{name}] {path} -> {ref}; table {tab}; cubes {cubes}; surface below {below}")
    write_json(exp_path(cfg["paths"]["outputs_dir"]) / "test_scene_report.json", rep)


if __name__ == "__main__":
    main()
