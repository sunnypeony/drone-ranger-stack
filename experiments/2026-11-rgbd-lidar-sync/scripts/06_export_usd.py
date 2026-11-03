#!/usr/bin/env python3
"""Stage 06: mesh_clean.ply -> USD with static triangle-mesh collision (usd-core only, no Isaac Sim).
Also collects the final deliverables into results/."""
import json
import shutil

import numpy as np

from common import die, exp_path, load_config, parse_args, stage_dir, write_json


def write_stage(path, V, F, N, C, cfg, approximation):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
    U = cfg["usd"]
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, U["meters_per_unit"])
    UsdGeom.SetStageUpAxis(stage, getattr(UsdGeom.Tokens, U["up_axis"].lower()))
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, "/World/Scene")
    mesh = UsdGeom.Mesh.Define(stage, "/World/Scene/mesh")
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)  # Open3D triangles are CCW
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1).astype(np.int32)))
    mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(N.astype(np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr([Gf.Vec3f(*V.min(0).tolist()), Gf.Vec3f(*V.max(0).tolist())])
    if C is not None:
        pv = mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec3fArray.FromNumpy(C.astype(np.float32)))
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mc.CreateApproximationAttr(getattr(UsdPhysics.Tokens, approximation))
    # static environment: deliberately no RigidBodyAPI
    stage.GetRootLayer().Save()
    return stage


def main():
    import open3d as o3d
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    U = cfg["usd"]
    out = stage_dir(cfg, "usd")
    mdir = stage_dir(cfg, "mesh")
    m = o3d.io.read_triangle_mesh(str(mdir / "mesh_clean.ply"))
    if len(m.triangles) == 0:
        die("mesh_clean.ply is empty")
    if not m.has_vertex_normals():
        m.compute_vertex_normals()
    V, F = np.asarray(m.vertices), np.asarray(m.triangles)
    N = np.asarray(m.vertex_normals)
    C = np.asarray(m.vertex_colors) if m.has_vertex_colors() else None
    if F.max() >= len(V):
        die("face index out of range")

    files = {}
    main_path = out / U["filename"]
    write_stage(main_path, V, F, N, C, cfg, "none")
    files["main"] = main_path
    if U["write_usda"]:
        p = main_path.with_suffix(".usda")
        write_stage(p, V, F, N, C, cfg, "none")
        files["usda"] = p
    if U["convex_decomp"]:
        p = out / U["convex_decomp_filename"]
        write_stage(p, V, F, N, C, cfg, "convexDecomposition")
        files["convex_decomp"] = p
    rep = {k: str(v) for k, v in files.items()}
    rep.update({"triangles": int(len(F)), "vertices": int(len(V)),
                "sizes_mb": {k: v.stat().st_size / 1e6 for k, v in files.items()}})
    write_json(out / "usd_report.json", rep)
    print(json.dumps(rep, indent=2))
    collect_results(cfg, main_path)


def collect_results(cfg, usd_path):
    res = exp_path(cfg["paths"]["results_dir"])
    res.mkdir(parents=True, exist_ok=True)
    o = exp_path(cfg["paths"]["outputs_dir"])
    required = {
        usd_path: res / usd_path.name,
        o / "mesh" / "mesh_clean.obj": res / "mesh_clean.obj",
        o / "mesh" / "mesh_clean.ply": res / "mesh_clean.ply",
    }
    optional = {  # produced by 01/03 of the same output tree; v2 writes its own report instead
        o / "poses" / "trajectory_topdown.png": res / "trajectory_topdown.png",
        o / "inspect" / "report.json": res / "report.json",
        o / "poses" / "pose_report.json": res / "pose_report.json",
    }
    limit = cfg["mesh"]["max_result_file_mb"] * 1e6
    copies = dict(required)
    for src, dst in optional.items():
        if src.exists():
            copies[src] = dst
        else:
            print(f"[results] optional {src} not present, skipped")
    for src, dst in copies.items():
        if not src.exists():
            die(f"missing deliverable {src}")
        if src.stat().st_size > limit:
            die(f"{src.name} is {src.stat().st_size / 1e6:.1f} MB > {cfg['mesh']['max_result_file_mb']} MB; "
                f"lower mesh.target_triangles and rerun 05/06")
        shutil.copy2(src, dst)
        print(f"[results] {dst} ({dst.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
