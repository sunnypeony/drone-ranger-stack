#!/usr/bin/env python3
"""Stage 07 (optional): project /scan into the (leveled) world using the camera trajectory and compare
with a horizontal slice of the mesh at the laser height.

Camera stamps are host now() and the LiDAR<->camera time mapping is not calibrated, so offsets are
expected; this check is for catching gross errors (wrong extrinsic, mirrored axes, wrong scale)."""
import json
import shutil

import numpy as np
from tqdm import tqdm

from common import (T_base_laser_from_cfg, die, lidar_time_offset_s, exp_path, find_bag_dir, get_typestore, interp_T, iter_topic, load_config,
                    load_static_tf, open_reader, parse_args, stage_dir, stamp_ns, write_json)


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import open3d as o3d
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    L, F = cfg["lidar"], cfg["frames"]
    out = stage_dir(cfg, "lidar")
    mdir = stage_dir(cfg, "mesh")
    z = np.load(mdir / "poses_aligned.npz")
    Ts, stamps = z["T_world_optical"], z["stamp_ns"].astype(np.int64)

    ts = get_typestore(cfg)
    with open_reader(find_bag_dir(cfg)) as reader:
        st = load_static_tf(reader, ts, cfg)
        T_bo = st.lookup(F["base"], F["camera_optical"])
        if T_bo is None:
            die("need base->optical in /tf_static")
        T_bl = T_base_laser_from_cfg(cfg, st)
        T_ol = np.linalg.inv(T_bo) @ T_bl  # optical <- laser
        gap = int(cfg["poses"]["tf_interp_max_gap_s"] * 1e9)
        off = int(lidar_time_offset_s(cfg) * 1e9)
        pts, rngs, heights, n_scan, n_used = [], [], [], 0, 0
        for i, (_, msg) in enumerate(tqdm(iter_topic(reader, ts, cfg["topics"]["scan"]), desc="scan")):
            n_scan += 1
            if i % L["scan_stride"]:
                continue
            T_wo = interp_T(stamp_ns(msg.header) + off, stamps, Ts, gap)
            if T_wo is None:
                continue
            r = np.asarray(msg.ranges, dtype=float)
            a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
            ok = np.isfinite(r) & (r >= max(L["range_min"], msg.range_min)) & (r <= min(L["range_max"], msg.range_max))
            P = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum()), np.ones(ok.sum())])
            T_wl = T_wo @ T_ol
            pts.append((T_wl @ P)[:3].T)
            rngs.append(r[ok])
            heights.append(T_wl[2, 3])
            n_used += 1
    if not pts:
        die("no scan could be placed on the trajectory (time ranges do not overlap?)")
    S = np.concatenate(pts)
    R = np.concatenate(rngs)
    h = float(np.median(heights))

    mesh = o3d.io.read_triangle_mesh(str(mdir / "mesh_clean.ply"))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    near = R <= L["compare_range_max"]  # the mesh only exists within the camera's depth range
    d = scene.compute_distance(o3d.core.Tensor(S[near].astype(np.float32))).numpy()
    sample = np.asarray(mesh.sample_points_uniformly(cfg["mesh"]["ground_sample_points"]).points)
    sl = sample[np.abs(sample[:, 2] - h) < L["slice_half_thickness"]]

    rep = {"scans_total": n_scan, "scans_used": n_used, "points": int(len(S)),
           "points_compared": int(near.sum()), "compare_range_m": [L["range_min"], L["compare_range_max"]],
           "laser_height_median_m": h, "laser_height_std_m": float(np.std(heights)),
           "dist_to_mesh_median_m": float(np.median(d)), "dist_to_mesh_p90_m": float(np.percentile(d, 90)),
           "mesh_slice_points": int(len(sl)), "time_offset_s": lidar_time_offset_s(cfg),
           "T_base_laser": T_bl.tolist()}
    write_json(out / "lidar_report.json", rep)
    print(json.dumps(rep, indent=2))

    rng = np.random.default_rng(0)
    Sp = S if len(S) <= L["max_points_plot"] else S[rng.choice(len(S), L["max_points_plot"], replace=False)]
    p = Ts[:, :3, 3]
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.scatter(sl[:, 0], sl[:, 1], s=0.3, c="k", label=f"mesh slice z={h:.2f}±{L['slice_half_thickness']} m")
    Sp_near = S[near][:: max(int(near.sum()) // L["max_points_plot"], 1)]
    ax.scatter(Sp[:, 0], Sp[:, 1], s=0.2, c="orange", alpha=0.2, label="LiDAR scans (all ranges)")
    ax.scatter(Sp_near[:, 0], Sp_near[:, 1], s=0.3, c="r", alpha=0.4,
               label=f"LiDAR <= {L['compare_range_max']} m (used for distance)")
    ax.plot(p[:, 0], p[:, 1], "b-", lw=1, label="camera path")
    m = L["plot_extent_margin"]
    lo = np.minimum(sl.min(0)[:2] if len(sl) else p.min(0)[:2], p.min(0)[:2]) - m
    hi = np.maximum(sl.max(0)[:2] if len(sl) else p.max(0)[:2], p.max(0)[:2]) + m
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", markerscale=10)
    ax.set_title(f"LiDAR vs mesh: dist median {rep['dist_to_mesh_median_m']:.3f} m, p90 {rep['dist_to_mesh_p90_m']:.3f} m")
    fig.tight_layout()
    fig.savefig(out / "lidar_vs_mesh.png", dpi=120)
    plt.close(fig)
    res = exp_path(cfg["paths"]["results_dir"])
    res.mkdir(parents=True, exist_ok=True)
    shutil.copy2(out / "lidar_vs_mesh.png", res / "lidar_vs_mesh.png")


if __name__ == "__main__":
    main()
