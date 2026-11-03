#!/usr/bin/env python3
"""Stage 05: clean / decimate / level the TSDF mesh -> mesh_clean.ply (+colors), mesh_clean.obj, T_align.npy."""
import json

import numpy as np

from common import assert_rigid, die, load_config, load_poses, make_T, parse_args, stage_dir, write_json


def clean(mesh):
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()
    return mesh


def keep_clusters(mesh, ratio):
    tri_cluster, n_tri, areas = mesh.cluster_connected_triangles()
    tri_cluster = np.asarray(tri_cluster)
    areas = np.asarray(areas)
    if len(areas) == 0:
        die("mesh has no triangles")
    total = areas.sum()
    keep = (areas >= ratio * total)
    keep[np.argmax(areas)] = True
    mesh.remove_triangles_by_mask(~keep[tri_cluster])
    mesh.remove_unreferenced_vertices()
    return {"clusters": int(len(areas)), "clusters_kept": int(keep.sum()),
            "area_kept_ratio": float(areas[keep].sum() / total)}


def rot_a_to_b(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1 / (1 + c))


def find_ground(mesh, up, M, o3d):
    pcd = mesh.sample_points_uniformly(M["ground_sample_points"])
    planes = []
    rest = pcd
    for _ in range(M["ground_max_planes"]):
        if len(rest.points) < M["ground_min_inliers"]:
            break
        model, inl = rest.segment_plane(M["ground_distance_threshold"], M["ground_ransac_n"],
                                        M["ground_num_iterations"])
        if len(inl) < M["ground_min_inliers"]:
            break
        n = np.array(model[:3])
        n /= np.linalg.norm(n)
        if np.dot(n, up) < 0:
            n, d = -n, -model[3] / np.linalg.norm(model[:3])
        else:
            d = model[3] / np.linalg.norm(model[:3])
        ang = float(np.degrees(np.arccos(np.clip(np.dot(n, up), -1, 1))))
        planes.append({"normal": n, "d": d, "inliers": len(inl), "angle_deg": ang})
        rest = rest.select_by_index(inl, invert=True)
    cands = [p for p in planes if p["angle_deg"] <= M["ground_max_angle_deg"]]
    return planes, (max(cands, key=lambda p: p["inliers"]) if cands else None)


def main():
    import open3d as o3d
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    M = cfg["mesh"]
    out = stage_dir(cfg, "mesh")
    mesh = o3d.io.read_triangle_mesh(str(stage_dir(cfg, "tsdf") / "mesh_raw.ply"))
    rep = {"raw_triangles": len(mesh.triangles), "raw_vertices": len(mesh.vertices)}
    clean(mesh)
    rep["after_clean_triangles"] = len(mesh.triangles)
    rep.update(keep_clusters(mesh, M["min_cluster_ratio"]))
    rep["after_cluster_triangles"] = len(mesh.triangles)
    clustered = mesh

    z, Ts = load_poses(cfg)
    world_up = str(z["up"])
    hint = M["up_axis_hint"] if M["up_axis_hint"] != "auto" else world_up
    up = {"+z": np.array([0, 0, 1.0]), "-y": np.array([0, -1.0, 0])}[hint]

    T_align = np.eye(4)
    if M["level_ground"]:
        planes, g = find_ground(clustered, up, M, o3d)
        rep["ground_planes"] = [{**p, "normal": p["normal"].tolist()} for p in planes]
        if g is None:
            die(f"level_ground: no plane within {M['ground_max_angle_deg']} deg of up={hint}; planes={rep['ground_planes']}")
        R = rot_a_to_b(g["normal"], np.array([0, 0, 1.0]))
        # plane n.x + d = 0 -> after rotation z = -d
        T_align = make_T(R, [0, 0, g["d"]])
        cam_z = np.einsum("ij,njk->nik", T_align, Ts)[:, 2, 3]
        if np.median(cam_z) < 0:  # trajectory must be above the ground
            die(f"after leveling the camera is below the ground (median z={np.median(cam_z):.2f}); wrong plane?")
        assert_rigid(T_align, cfg, "T_align")
        rep["ground"] = {"normal_before": g["normal"].tolist(), "tilt_corrected_deg": g["angle_deg"],
                         "inliers": g["inliers"], "camera_height_median_m": float(np.median(cam_z))}
    np.save(out / "T_align.npy", T_align)
    Ts_al = np.einsum("ij,njk->nik", T_align, Ts)
    np.savez(out / "poses_aligned.npz", T_world_optical=Ts_al, frame_idx=z["frame_idx"], stamp_ns=z["stamp_ns"])

    target = M["target_triangles"]
    limit = M["max_result_file_mb"] * 1e6
    while True:
        m = o3d.geometry.TriangleMesh(clustered)
        if len(m.triangles) > target:
            m = m.simplify_quadric_decimation(target_number_of_triangles=target)
            clean(m)
        m.transform(T_align)
        m.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(out / "mesh_clean.ply"), m, write_vertex_colors=True)
        o3d.io.write_triangle_mesh(str(out / "mesh_clean.obj"), m, write_vertex_colors=False,
                                   write_triangle_uvs=False)
        sizes = {f: (out / f).stat().st_size for f in ("mesh_clean.ply", "mesh_clean.obj")}
        if max(sizes.values()) <= limit:
            break
        print(f"[size] {sizes} > {M['max_result_file_mb']} MB, halving target_triangles {target} -> {target // 2}")
        target //= 2
    bb = m.get_axis_aligned_bounding_box()
    rep.update({"target_triangles_used": target, "triangles": len(m.triangles), "vertices": len(m.vertices),
                "file_sizes_mb": {k: v / 1e6 for k, v in sizes.items()},
                "bbox_min": np.asarray(bb.min_bound).tolist(), "bbox_max": np.asarray(bb.max_bound).tolist(),
                "T_align": T_align.tolist(), "watertight": bool(m.is_watertight())})
    write_json(out / "mesh_report.json", rep)
    print(json.dumps({k: v for k, v in rep.items() if k not in ("ground_planes",)}, indent=2))
    render_views(m, Ts_al, out)


def render_views(m, Ts, out):
    """Headless sanity images: top view and side view of mesh vertices + camera path."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    V = np.asarray(m.vertices)
    C = np.asarray(m.vertex_colors) if m.has_vertex_colors() else None
    p = Ts[:, :3, 3]
    fig, ax = plt.subplots(1, 3, figsize=(21, 7))
    for a, (i, j, ti) in zip(ax, [(0, 1, "top (x-y)"), (0, 2, "side (x-z)"), (1, 2, "side (y-z)")]):
        a.scatter(V[:, i], V[:, j], s=0.05, c=C)
        a.plot(p[:, i], p[:, j], "b-", lw=0.8)
        a.set_aspect("equal")
        a.set_title(f"mesh_clean {ti} + camera path")
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "mesh_views.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
