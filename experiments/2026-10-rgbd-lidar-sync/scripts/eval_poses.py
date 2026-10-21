#!/usr/bin/env python3
"""Evaluate one or more pose sets with identical, pose-source-independent metrics.

For each pose set (config eval.pose_sets, or --poses name=path ...):
  - leveled camera height std            (ground plane from that set's own accumulated cloud)
  - per-step displacement median / p99
  - fraction of accumulated points inside the +-ground_dist ground band
  - occupied cells of the [slice] height band on a slice_grid XY grid (smear = more cells)
  - /scan -> mesh distance (median / p90) for returns within scan_range_max, against a coarse
    TSDF mesh built from the same poses (time mapping LiDAR<->camera is uncalibrated)

Writes outputs/eval/{compare.json, slices.png}.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from tqdm import tqdm  # noqa: E402

from common import (T_base_laser_from_cfg, die, exp_path, find_bag_dir, get_typestore, interp_T, iter_topic, lidar_time_offset_s, load_config,  # noqa: E402
                    load_frames, load_static_tf, make_T, o3d_intrinsic, open_reader, parse_args,
                    rot_angle_deg, stage_dir, stamp_ns, write_json)

SURFACE, TEXT, TEXT2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE = "#2a78d6"


def extra_args(p):
    p.add_argument("--poses", nargs="*", default=None, metavar="NAME=PATH",
                   help="override eval.pose_sets, e.g. rtabmap=outputs/rtabmap/poses.npz")


def rot_a_to_b(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / (v @ v))


def rgbd(o3d, fdir, row, cfg, E):
    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.io.read_image(str(fdir / row["color"])), o3d.io.read_image(str(fdir / row["depth"])),
        depth_scale=cfg["tsdf"]["depth_scale"], depth_trunc=E["depth_max"], convert_rgb_to_intensity=False)


def accumulated_cloud(o3d, fdir, rows, intr, idx, Ts, cfg, E):
    acc = o3d.geometry.PointCloud()
    for k in tqdm(range(0, len(idx), E["cloud_every"]), desc="  cloud", leave=False):
        p = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd(o3d, fdir, rows[idx[k]], cfg, E), intr)
        acc += p.voxel_down_sample(E["cloud_voxel"]).transform(Ts[k])
    return acc.voxel_down_sample(E["cloud_voxel"])


def find_ground(pcd, E):
    """Largest plane whose normal is within ground_max_tilt_deg of +Z -> (A_level, n, inlier_frac)."""
    rest, n_all = pcd, len(pcd.points)
    for _ in range(E["ground_max_tries"]):
        plane, inl = rest.segment_plane(E["ground_dist"], 3, E["ground_ransac_iters"])
        n = np.array(plane[:3])
        n = n if n[2] > 0 else -n
        if np.degrees(np.arccos(np.clip(n[2], -1, 1))) <= E["ground_max_tilt_deg"]:
            R = rot_a_to_b(n, np.array([0.0, 0.0, 1.0]))
            z0 = np.median((np.asarray(rest.points)[inl] @ R.T)[:, 2])
            return make_T(R, [0, 0, -z0]), n, len(inl) / n_all
        rest = rest.select_by_index(inl, invert=True)
    return None, None, 0.0


def build_mesh(o3d, fdir, rows, intr, idx, Ts, cfg, E):
    vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=E["mesh_voxel"], sdf_trunc=E["mesh_sdf_trunc"],
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    for k in tqdm(range(0, len(idx), E["mesh_frame_stride"]), desc="  tsdf", leave=False):
        vol.integrate(rgbd(o3d, fdir, rows[idx[k]], cfg, E), intr, np.linalg.inv(Ts[k]))
    return vol.extract_triangle_mesh()


def scan_points(cfg, E, intr):
    """Per used scan: (stamp_ns, Nx3 points in laser frame within range [and camera HFOV])."""
    L = cfg["lidar"]
    ts = get_typestore(cfg)
    out = []
    with open_reader(find_bag_dir(cfg)) as reader:
        st = load_static_tf(reader, ts, cfg)
        T_bo = st.lookup(cfg["frames"]["base"], cfg["frames"]["camera_optical"])
        if T_bo is None:
            die("need base->optical in /tf_static")
        T_bl = T_base_laser_from_cfg(cfg, st)  # fixed lidar.T_base_laser if configured, else /tf_static
        T_ol = np.linalg.inv(T_bo) @ T_bl
        half = np.degrees(np.arctan2(intr["width"] / 2, np.array(intr["K"])[0][0])) - E["scan_fov_margin_deg"]
        for i, (_, msg) in enumerate(iter_topic(reader, ts, cfg["topics"]["scan"])):
            if i % E["scan_stride"]:
                continue
            r = np.asarray(msg.ranges, dtype=float)
            a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
            ok = (np.isfinite(r) & (r >= max(L["range_min"], msg.range_min))
                  & (r <= min(E["scan_range_max"], msg.range_max)))
            pl = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())], 1)
            if E["scan_camera_fov_only"]:
                po = pl @ T_ol[:3, :3].T + T_ol[:3, 3]
                pl = pl[(po[:, 2] > 0) & (np.degrees(np.abs(np.arctan2(po[:, 0], po[:, 2]))) <= half)]
            out.append((stamp_ns(msg.header), pl))
    return out, T_ol, T_bl


def load_references(E):
    """[(first, tail, translation, rotation_deg)] from the pose-free head/tail check."""
    import json
    f = exp_path(E["head_tail_reference"])
    if not f.exists():
        print(f"[warn] {f} not found; run check_head_tail.py for the drift metric")
        return []
    r = json.load(open(f))
    first, refs = r["first_frame"], []
    for p in r["per_tail_direct"]:
        if p["n"] and p["spread_max_m"] <= E["ref_max_spread_m"] and p["rot_range_deg"] <= E["ref_max_rot_range_deg"]:
            d = next(x for x in r["pairs_direct"] if x["head"] == first and x["tail"] == p["tail"])
            refs.append((first, p["tail"], np.array(d["translation"]), d["rotation_deg"]))
    return refs


def evaluate(name, path, cfg, E, frames, scans, T_ol, refs):
    import open3d as o3d
    fdir, rows, intr_d = frames
    intr = o3d_intrinsic(intr_d)
    z = np.load(path, allow_pickle=True)
    Ts, idx, st = z["T_world_optical"], z["frame_idx"].astype(int), z["stamp_ns"].astype(np.int64)
    print(f"[{name}] {path}: {len(Ts)} poses")
    pcd = accumulated_cloud(o3d, fdir, rows, intr, idx, Ts, cfg, E)
    A, n, ground_frac = find_ground(pcd, E)
    if A is None:
        die(f"[{name}] no ground plane within {E['ground_max_tilt_deg']} deg of +Z")
    P = np.asarray(pcd.points) @ A[:3, :3].T + A[:3, 3]
    cam = Ts[:, :3, 3] @ A[:3, :3].T + A[:3, 3]
    steps = np.linalg.norm(np.diff(Ts[:, :3, 3], axis=0), axis=1)
    lo, hi = E["slice"]
    sl = P[(P[:, 2] >= lo) & (P[:, 2] <= hi)]
    cells = np.unique(np.floor(sl[:, :2] / E["slice_grid"]).astype(np.int64), axis=0)

    mesh = build_mesh(o3d, fdir, rows, intr, idx, Ts, cfg, E)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    gap = int(cfg["poses"]["tf_interp_max_gap_s"] * 1e9)
    off = int(lidar_time_offset_s(cfg) * 1e9)
    world_pts, n_used = [], 0
    for s, pl in scans:
        T_wo = interp_T(s + off, st, Ts, gap)
        if T_wo is None or not len(pl):
            continue
        T_wl = T_wo @ T_ol
        world_pts.append(pl @ T_wl[:3, :3].T + T_wl[:3, 3])
        n_used += 1
    W = np.concatenate(world_pts).astype(np.float32)
    dist = scene.compute_distance(o3d.core.Tensor(W)).numpy()
    rel = np.linalg.inv(Ts[0]) @ Ts[-1]
    pos = {f: k for k, f in enumerate(idx)}
    ref_err = []
    for a, b, t_ref, r_ref in refs:
        if a in pos and b in pos:
            T = np.linalg.inv(Ts[pos[a]]) @ Ts[pos[b]]
            ref_err.append({"first": a, "tail": b, "ref_distance_m": float(np.linalg.norm(t_ref)),
                            "pose_distance_m": float(np.linalg.norm(T[:3, 3])),
                            "translation_error_m": float(np.linalg.norm(T[:3, 3] - t_ref)),
                            "rotation_pose_deg": rot_angle_deg(T[:3, :3]), "rotation_ref_deg": r_ref})
    res = {
        "path": str(path), "n_poses": len(Ts),
        "path_length_m": float(steps.sum()),
        "head_tail_distance_m": float(np.linalg.norm(rel[:3, 3])),
        "head_tail_rotation_deg": rot_angle_deg(rel[:3, :3]),
        "ground_normal_world": n, "ground_tilt_deg": float(np.degrees(np.arccos(n[2]))),
        "camera_height_m": {"mean": float(cam[:, 2].mean()), "std": float(cam[:, 2].std()),
                            "min": float(cam[:, 2].min()), "max": float(cam[:, 2].max())},
        "step_m": {"median": float(np.median(steps)), "p99": float(np.percentile(steps, 99)),
                   "max": float(steps.max())},
        "ground_band_fraction": float(ground_frac),
        "slice_occupied_cells": int(len(cells)),
        "slice_occupied_area_m2": float(len(cells) * E["slice_grid"] ** 2),
        "n_cloud_points": len(P),
        "mesh_triangles": len(mesh.triangles),
        "head_tail_reference_error": ref_err,
        "scan_to_mesh_m": {"n_scans": n_used, "n_points": len(W), "median": float(np.median(dist)),
                           "p90": float(np.percentile(dist, 90)), "frac_lt_0.1m": float((dist < 0.1).mean())},
    }
    return res, sl, cam


def main():
    args = parse_args(__doc__, extra_args)
    cfg = load_config(args.config, args.set)
    E = cfg["eval"]
    out = stage_dir(cfg, E["out_dir"])
    sets = dict(x.split("=", 1) for x in args.poses) if args.poses else E["pose_sets"]
    sets = {k: exp_path(v) for k, v in sets.items()}
    for k, v in sets.items():
        if not v.exists():
            die(f"pose set {k}: {v} not found")
    frames = load_frames(cfg)
    scans, T_ol, T_bl = scan_points(cfg, E, frames[2])
    print(f"scans: {len(scans)} used (stride {E['scan_stride']}), T_base_laser t = {np.round(T_bl[:3, 3], 4).tolist()}")

    refs = load_references(E)
    print(f"pose-free references: {[(a, b, round(float(np.linalg.norm(t)), 3)) for a, b, t, _ in refs]}")
    results, slices = {}, {}
    for name, path in sets.items():
        results[name], sl, cam = evaluate(name, path, cfg, E, frames, scans, T_ol, refs)
        slices[name] = (sl, cam)
    write_json(out / "compare.json", {"metrics": results, "T_base_laser": T_bl,
                                      "params": {k: E[k] for k in E if k != "pose_sets"}})

    fig, axes = plt.subplots(1, len(slices), figsize=(6 * len(slices), 6), facecolor=SURFACE, squeeze=False)
    for a, (name, (sl, cam)) in zip(axes[0], slices.items()):
        a.set_facecolor(SURFACE)
        a.grid(True, color=GRID, lw=0.6)
        a.set_axisbelow(True)
        for s in a.spines.values():
            s.set_color(GRID)
        a.tick_params(colors=TEXT2, labelsize=8)
        a.scatter(sl[:, 0], sl[:, 1], s=0.3, color=TEXT2, alpha=0.5, linewidths=0)
        a.plot(cam[:, 0], cam[:, 1], color=BLUE, lw=1)
        a.set_aspect("equal")
        a.set_title(f"{name}: {E['slice'][0]}-{E['slice'][1]} m slice, "
                    f"{results[name]['slice_occupied_area_m2']:.1f} m² occupied", fontsize=10, loc="left", color=TEXT)
    fig.tight_layout()
    fig.savefig(out / "slices.png", dpi=110, facecolor=SURFACE)
    plt.close(fig)

    rows = [("camera height std [m]", lambda r: r["camera_height_m"]["std"]),
            ("camera height min / max [m]", lambda r: f"{r['camera_height_m']['min']:.2f} / {r['camera_height_m']['max']:.2f}"),
            ("step median [m]", lambda r: r["step_m"]["median"]),
            ("step p99 [m]", lambda r: r["step_m"]["p99"]),
            ("ground band fraction", lambda r: r["ground_band_fraction"]),
            (f"slice cells ({E['slice_grid']} m)", lambda r: r["slice_occupied_cells"]),
            ("scan->mesh median [m]", lambda r: r["scan_to_mesh_m"]["median"]),
            ("scan->mesh p90 [m]", lambda r: r["scan_to_mesh_m"]["p90"]),
            *[(f"ref err {a}->{b} [m] (ref {np.linalg.norm(t):.2f})",
               (lambda b: lambda r: next(e["translation_error_m"] for e in r["head_tail_reference_error"]
                                         if e["tail"] == b))(b)) for a, b, t, _ in refs],
            ("path length [m]", lambda r: r["path_length_m"]),
            ("head-tail distance [m]", lambda r: r["head_tail_distance_m"])]
    names = list(results)
    print("\n" + f"{'metric':30s}" + "".join(f"{n:>14s}" for n in names))
    for lab, f in rows:
        vals = [f(results[n]) for n in names]
        print(f"{lab:30s}" + "".join(f"{v:>14.3f}" if isinstance(v, float) else f"{v!s:>14s}" for v in vals))
    print(f"-> {out / 'compare.json'}")


if __name__ == "__main__":
    main()
