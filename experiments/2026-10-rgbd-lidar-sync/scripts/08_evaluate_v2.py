#!/usr/bin/env python3
"""Stage 08 (v2): ablation evaluation v1 vs v2 variants.

For each variant a TSDF mesh is built (04/05 into outputs/v2/ablation/<name>/; the final variant uses
outputs/v2/ which run_v2.sh builds before this stage). Metrics:
  heldout_*   held-out scans (index % 2 != constraint_parity, never used in 03b) -> mesh distance (<= 4 m)
  head_tail   first/last camera distance
  slice_cells occupied 5 cm cells of the 0.4-1.2 m height band (same depth frames for every variant;
              fewer cells = sharper, less ghosting)
  tilt        ground-normal residual
Writes results/v2/report.json and comparison figures."""
import importlib
import json
import shutil
import subprocess
import sys

import cv2
import numpy as np

import lidar2d as L2
from common import (EXP_DIR, T_base_laser_from_cfg, cutoff_count, exp_path, find_bag_dir, get_typestore, interp_T,
                    load_config, load_frames, load_static_tf, open_reader, parse_args, stage_dir, write_json)
from scene_utils import find_table

m3 = importlib.import_module("03_estimate_poses")
m3b = importlib.import_module("03b_lidar_refine")


def closure_reference(cfg, scans, body, bins, T_bl, T_ob, ground_z, dt, first_ns, last_ns):
    """Start->end relative pose measured by LiDAR from HELD-OUT scans only (wide CSM + ICP).
    Submaps use the v1 trajectory only for local (+-0.4 s) aggregation."""
    V, Lc, E = cfg["v2"], cfg["lidar"], cfg["eval"]
    v1 = np.load(exp_path(cfg["paths"]["v1_outputs_dir"]) / "poses" / "poses.npz")
    traj = m3b.Traj(v1["stamp_ns"], v1["T_world_optical"], T_ob)
    held = [s for i, s in enumerate(scans) if i % 2 != V["constraint_parity"]]
    local = [m3b.leveled_scan(s, V, Lc, traj, T_bl, ground_z, body, bins, dt)[0] for s in held]
    st = np.array([s["stamp"] + dt for s in held])
    ia, ib = int(np.argmin(np.abs(st - first_ns))), int(np.argmin(np.abs(st - last_ns)))
    A = m3b.submap(ia, held, local, traj, V, dt)
    B = m3b.submap(ib, held, local, traj, V, dt)
    W = dict(V)
    W.update(csm_xy_range=E["closure_csm_xy_range"], csm_yaw_range_deg=E["closure_csm_yaw_range_deg"])
    c = L2.correlative_match(B, A, np.zeros(3), W, np.random.default_rng(0))
    r = L2.icp2d(B, A, c["T"], V) if c else None
    if r is None:
        return None
    return {"Z": r["T"].tolist(), "fitness": r["fitness"], "rmse": r["rmse"], "csm_ratio": c["ratio"],
            "t_start_ns": int(st[ia]), "t_end_ns": int(st[ib]),
            "distance_m": float(np.linalg.norm(r["T"][:2]))}


def closure_error(ref, Ts, stamps, T_ob):
    traj = m3b.Traj(stamps, Ts, T_ob)
    est = L2.between(traj.at(ref["t_start_ns"]), traj.at(ref["t_end_ns"]))
    d = L2.between(np.array(ref["Z"]), est)
    return float(np.linalg.norm(d[:2])), float(np.degrees(abs(d[2])))


def run_stage(script, config, sets):
    cmd = [sys.executable, str(EXP_DIR / "scripts" / script), "--config", config]
    for s in sets:
        cmd += ["--set", s]
    print("[run]", " ".join(cmd[1:]))
    subprocess.run(cmd, check=True)


def variant_dirs(cfg):
    """With a cutoff, v1 is re-meshed from its own poses truncated to the cutoff (ablation/v1), so
    every variant is evaluated on the same frames; without one, the v1 outputs are used as they are."""
    root = exp_path(cfg["paths"]["outputs_dir"])
    V = cfg["v2"]
    d = {"v1": root / "ablation" / "v1" if cfg.get("cutoff_s") is not None else exp_path(cfg["paths"]["v1_outputs_dir"])}
    for n in V["variants"]:
        d[n] = root if n == V["final_variant"] else root / "ablation" / n
    return d


def heldout_points(cfg, scans, body, bins, T_bl, Ts, stamps, dt):
    """Held-out scans -> 3D world points (aligned frame), their ranges and beam angles (laser frame)."""
    V, Lc = cfg["v2"], cfg["lidar"]
    T_ob = None
    pts, rngs, angs = [], [], []
    for i, s in enumerate(scans):
        if i % 2 == V["constraint_parity"]:
            continue
        T_wo = interp_T(s["stamp"] + dt, stamps, Ts, int(cfg["poses"]["tf_interp_max_gap_s"] * 1e9))
        if T_wo is None:
            continue
        p, r = L2.scan_points(s, Lc, body, bins)
        if len(p) == 0:
            continue
        if T_ob is None:
            T_ob = np.linalg.inv(cfg["_T_bo"])
        W = (T_wo @ T_ob @ T_bl @ np.c_[p, np.ones(len(p))].T)[:3].T
        h = W[:, 2]  # aligned world: ground at z = 0
        m = (h >= V["min_height_above_ground"]) & (h <= V["max_height_above_ground"])
        pts.append(W[m])
        rngs.append(r[m])
        angs.append(np.degrees(np.arctan2(p[m, 1], p[m, 0])))
    return np.concatenate(pts), np.concatenate(rngs), np.concatenate(angs)


def crispness(cfg, scans, body, bins, T_bl, T_ob, Ts, stamps, dt, ground_z):
    """Held-out LiDAR map sharpness: median distance from held-out points to held-out points of scans
    more than crisp_min_gap_s apart (within crisp_radius). Uses the variant poses (unaligned world)."""
    from scipy.spatial import cKDTree
    V, Lc, E = cfg["v2"], cfg["lidar"], cfg["eval"]
    traj = m3b.Traj(stamps, Ts, T_ob)
    held = [s for i, s in enumerate(scans) if i % 2 != V["constraint_parity"]][:: E["crisp_scan_stride"]]
    P, T = [], []
    for s in held:
        t = s["stamp"] + dt
        if not traj.inside(t):
            continue
        q, _ = m3b.leveled_scan(s, V, Lc, traj, T_bl, ground_z, body, bins, dt)
        P.append(L2.transform_pts(traj.at(t), q))
        T.append(np.full(len(q), t / 1e9))
    P, T = np.concatenate(P), np.concatenate(T)
    tree = cKDTree(P)
    d = []
    for k in range(0, len(P), E["crisp_point_stride"]):
        idx = np.array(tree.query_ball_point(P[k], E["crisp_radius"]), int)
        far = idx[np.abs(T[idx] - T[k]) > E["crisp_min_gap_s"]]
        if len(far):
            d.append(np.min(np.linalg.norm(P[far] - P[k], axis=1)))
    return float(np.median(d)), len(d)


def table_triangles(d, center, E):
    """Raw TSDF triangles in the table region (aligned frame): low = table carved by inconsistent views."""
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(d / "tsdf" / "mesh_raw.ply"))
    m.transform(np.load(d / "mesh" / "T_align.npy"))
    V, F = np.asarray(m.vertices), np.asarray(m.triangles)
    c = V[F].mean(1)
    lo, hi = E["table_check_band"]
    return int(((np.linalg.norm(c[:, :2] - center, axis=1) < E["table_check_radius"]) & (c[:, 2] > lo) & (c[:, 2] < hi)).sum())


def slice_points(cfg, fdir, rows, intr, frames, Ts):
    E = cfg["eval"]
    K = np.array(intr["K"])
    out, tags = [], []
    for f in frames:
        d = cv2.imread(str(fdir / rows[f]["depth"]), cv2.IMREAD_UNCHANGED).astype(float) / cfg["tsdf"]["depth_scale"]
        v, u = np.mgrid[0:d.shape[0]:4, 0:d.shape[1]:4]
        z = d[::4, ::4]
        m = (z > 0) & (z <= E["depth_max"])
        X = np.stack([(u[m] - K[0, 2]) * z[m] / K[0, 0], (v[m] - K[1, 2]) * z[m] / K[1, 1], z[m], np.ones(m.sum())])
        W = (Ts[f] @ X)[:3].T
        lo, hi = E["slice_band"]
        s = (W[:, 2] >= lo) & (W[:, 2] <= hi)
        out.append(W[s])
        tags.append(np.full(s.sum(), f / len(rows)))
    return np.concatenate(out), np.concatenate(tags)


def main():
    import open3d as o3d
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    V, E = cfg["v2"], cfg["eval"]
    res_dir = exp_path(cfg["paths"]["results_dir"])
    res_dir.mkdir(parents=True, exist_ok=True)
    out = stage_dir(cfg, "eval")
    dirs = variant_dirs(cfg)

    fdir, rows, intr = load_frames(cfg)
    n_cut = cutoff_count(cfg, rows)
    rows = rows[:n_cut]
    if cfg.get("cutoff_s") is not None:
        print(f"cutoff {cfg['cutoff_s']} s: evaluating frames 0..{n_cut - 1}; v1 re-meshed from its truncated poses")
        v1p = np.load(exp_path(cfg["paths"]["v1_outputs_dir"]) / "poses" / "poses.npz", allow_pickle=True)
        vd = dirs["v1"] / "poses"
        vd.mkdir(parents=True, exist_ok=True)
        keep = v1p["frame_idx"] < n_cut
        np.savez(vd / "poses.npz", **{k: (v1p[k][keep] if k in ("T_world_optical", "frame_idx", "stamp_ns") else v1p[k])
                                      for k in v1p.files})

    # ---- meshes for non-final variants (and the truncated v1)
    for n, d in dirs.items():
        if n == V["final_variant"] or (n == "v1" and cfg.get("cutoff_s") is None):
            continue
        if not (d / "mesh" / "mesh_clean.ply").exists():
            sets = [f"paths.outputs_dir={d.relative_to(EXP_DIR)}", f"paths.results_dir={(d / 'results').relative_to(EXP_DIR)}"]
            run_stage("04_integrate_tsdf.py", args.config, sets)
            run_stage("05_postprocess_mesh.py", args.config, sets)

    pdir = stage_dir(cfg, "poses")
    v2rep = json.load(open(pdir / "v2_pose_report.json"))
    calib = json.load(open(pdir / "lidar_calib.json"))
    dt = int(calib["time_offset_s"] * 1e9)
    blur = np.load(pdir / "blur.npz")["score"] < v2rep["blur"]["threshold"]
    gr = np.load(exp_path(cfg["paths"]["v1_outputs_dir"]) / "poses" / "ckpt_ground.npz")
    frames = [f for f in range(0, len(rows), E["frame_every"]) if not blur[f]]

    ts = get_typestore(cfg)
    with open_reader(find_bag_dir(cfg)) as reader:
        st = load_static_tf(reader, ts, cfg)
        cfg["_T_bo"] = st.lookup(cfg["frames"]["base"], cfg["frames"]["camera_optical"])
        scans = L2.load_scans(reader, ts, cfg["topics"]["scan"])
    T_bl = T_base_laser_from_cfg(cfg)
    T_ob = np.linalg.inv(cfg["_T_bo"])
    body, bins, _, _ = L2.detect_body_sector(scans, V)
    stamps = np.array([int(r["stamp_ns"]) for r in rows], np.int64)
    ref = closure_reference(cfg, scans, body, bins, T_bl, T_ob, v2rep["ground_z_world"], dt, stamps[0], stamps[-1])
    print(f"held-out LiDAR closure reference: {ref}")

    K = np.array(intr["K"])
    fov_half = float(np.degrees(np.arctan(K[0, 2] / K[0, 0])))  # camera horizontal half-FOV (laser x = camera axis)
    v1m = o3d.io.read_triangle_mesh(str(dirs["v1"] / "mesh" / "mesh_clean.ply"))
    tv1 = find_table(np.asarray(v1m.vertices),
                     np.load(dirs["v1"] / "mesh" / "poses_aligned.npz")["T_world_optical"][:, :2, 3], cfg["test_scene"])
    table_center = np.array(tv1["center"]) if tv1 else np.array(cfg["test_scene"]["table_center"])
    table, data = {}, {}
    for n, d in dirs.items():
        z = np.load(d / "mesh" / "poses_aligned.npz")
        Ts = z["T_world_optical"]
        Tu = np.load(d / "poses" / "poses.npz")["T_world_optical"]
        ce = closure_error(ref, Tu, stamps, T_ob) if ref else (None, None)
        mesh = o3d.io.read_triangle_mesh(str(d / "mesh" / "mesh_clean.ply"))
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        P, R, Ang = heldout_points(cfg, scans, body, bins, T_bl, Ts, z["stamp_ns"].astype(np.int64), dt)
        near = R <= E["compare_range_max"]
        dist = scene.compute_distance(o3d.core.Tensor(P[near].astype(np.float32))).numpy()
        infov = near & (np.abs(Ang) <= fov_half)
        dist_fov = scene.compute_distance(o3d.core.Tensor(P[infov].astype(np.float32))).numpy()
        S, tags = slice_points(cfg, fdir, rows, intr, frames, Ts)
        cells = len(np.unique(np.floor(S[:, :2] / E["slice_grid"]).astype(np.int64), axis=0))
        tilt = m3.tilt_residual_deg(Ts, gr["normals"][:len(Ts)], gr["valid"][:len(Ts)], np.array([0, 0, 1.0]))
        mrep = json.load(open(d / "mesh" / "mesh_report.json"))
        crisp, ncr = crispness(cfg, scans, body, bins, T_bl, T_ob, Tu, stamps, dt, v2rep["ground_z_world"])
        table[n] = {"heldout_fov_dist_median_m": float(np.median(dist_fov)),
                    "heldout_fov_dist_p90_m": float(np.percentile(dist_fov, 90)),
                    "heldout_dist_median_m": float(np.median(dist)), "heldout_dist_p90_m": float(np.percentile(dist, 90)),
                    "heldout_points": int(near.sum()), "heldout_fov_points": int(infov.sum()),
                    "lidar_crispness_median_m": crisp, "lidar_crispness_samples": ncr,
                    "table_region_raw_triangles": table_triangles(d, table_center, E),
                    "head_tail_m": float(np.linalg.norm(Ts[-1][:3, 3] - Ts[0][:3, 3])),
                    "closure_err_xy_m": ce[0], "closure_err_deg": ce[1],
                    "path_length_m": m3.path_length(Ts),
                    "slice_cells": int(cells), "slice_points": int(len(S)),
                    "tilt_median_deg": tilt["median"], "tilt_p90_deg": tilt["p90"],
                    "mesh_triangles": mrep["triangles"]}
        data[n] = {"Ts": Ts, "S": S, "tags": tags, "P": P[near], "mesh": mesh}
        print(n, json.dumps(table[n]))

    final = V["final_variant"]
    plots(out, data, final, dirs, E)
    tbl = {n: data[n] for n in ("v1", final)}
    table_detect = {n: find_table(np.asarray(tbl[n]["mesh"].vertices), tbl[n]["Ts"][:, :2, 3], cfg["test_scene"])
                    for n in tbl}

    # ---- optional finer TSDF for the final variant
    extra = {}
    if E.get("extra_voxel"):
        vx = E["extra_voxel"]
        ed = exp_path(cfg["paths"]["outputs_dir"]) / f"vox{int(round(vx * 1000)):03d}"
        (ed / "poses").mkdir(parents=True, exist_ok=True)
        shutil.copy2(pdir / "poses.npz", ed / "poses" / "poses.npz")
        if not (ed / "mesh" / "mesh_clean.ply").exists():
            sets = [f"paths.outputs_dir={ed.relative_to(EXP_DIR)}", f"tsdf.voxel_length={vx}",
                    f"tsdf.sdf_trunc={4 * vx}"]
            run_stage("04_integrate_tsdf.py", args.config, sets)
            run_stage("05_postprocess_mesh.py", args.config, sets)
        m2 = o3d.io.read_triangle_mesh(str(ed / "mesh" / "mesh_clean.ply"))
        extra = {"voxel": vx, "dir": str(ed), "mesh_report": json.load(open(ed / "mesh" / "mesh_report.json"))}
        table_plot(out / "table_region_voxel.png", {"2 cm (final)": data[final]["mesh"], f"{vx * 100:.1f} cm": m2,
                                                     "v1 2 cm": data["v1"]["mesh"]}, table_detect[final])

    rep = {"ablation": table, "closure_reference_heldout": ref, "fov_half_deg": fov_half,
           "cutoff_s": cfg.get("cutoff_s"), "n_frames": n_cut,
           "table_check_center": table_center.tolist(), "time_offset_s": calib["time_offset_s"],
           "v2_pose_report": v2rep,
           "table_detection": table_detect, "extra_voxel": extra,
           "notes": "held-out scans (odd index) were never used as constraints; metrics on all variants use "
                    "the same held-out scans, the same depth frames and the same time offset"}
    write_json(out / "report.json", rep)
    shutil.copy2(out / "report.json", res_dir / "report.json")
    for f in ("slice_v1_vs_v2.png", "trajectory_comparison.png", "lidar_overlay_v1_vs_v2.png", "table_region_voxel.png",
              "slice_ablation.png"):
        if (out / f).exists():
            shutil.copy2(out / f, res_dir / f)
    for f in ("lidar_calib.png",):
        if (pdir / f).exists():
            shutil.copy2(pdir / f, res_dir / f)
    print_table(table)


def print_table(t):
    cols = ["heldout_fov_dist_median_m", "heldout_fov_dist_p90_m", "heldout_dist_median_m", "lidar_crispness_median_m",
            "closure_err_xy_m", "closure_err_deg", "head_tail_m", "slice_cells", "table_region_raw_triangles",
            "tilt_median_deg"]
    print(f"{'variant':14s} " + " ".join(f"{c[:18]:>18s}" for c in cols))
    for n, r in t.items():
        print(f"{n:14s} " + " ".join(f"{r[c]:18.3f}" if isinstance(r[c], float) else f"{str(r[c]):>18s}" for c in cols))


def plots(out, data, final, dirs, E):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = list(data)
    lim_lo = np.min([d["S"][:, :2].min(0) for d in data.values()], axis=0)
    lim_hi = np.max([d["S"][:, :2].max(0) for d in data.values()], axis=0)
    # slice: v1 vs final
    for fname, sel in (("slice_v1_vs_v2.png", ["v1", final]), ("slice_ablation.png", names)):
        fig, ax = plt.subplots(1, len(sel), figsize=(8 * len(sel), 8), squeeze=False)
        for a, n in zip(ax[0], sel):
            d = data[n]
            a.scatter(d["S"][:, 0], d["S"][:, 1], s=0.15, c=d["tags"], cmap="jet")
            a.set_title(f"{n}: {E['slice_band'][0]}-{E['slice_band'][1]} m slice, by time")
            a.set_xlim(lim_lo[0], lim_hi[0])
            a.set_ylim(lim_lo[1], lim_hi[1])
            a.set_aspect("equal")
            a.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / fname, dpi=90)
        plt.close(fig)
    # trajectories
    fig, ax = plt.subplots(figsize=(9, 9))
    styles = ["k-", "c--", "g-.", "m:", "b-", "r--", "y-.", "k:"]
    for n, sty in zip(names, styles * (len(names) // len(styles) + 1)):
        p = data[n]["Ts"][:, :3, 3]
        ax.plot(p[:, 0], p[:, 1], sty, lw=1.2, label=f"{n} (head-tail {np.linalg.norm(p[-1] - p[0]):.2f} m)")
        ax.plot(p[0, 0], p[0, 1], "go")
        ax.plot(p[-1, 0], p[-1, 1], "rs")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title("trajectories (aligned world, x-y); green = start, red = end")
    fig.tight_layout()
    fig.savefig(out / "trajectory_comparison.png", dpi=110)
    plt.close(fig)
    # lidar overlay
    fig, ax = plt.subplots(1, 2, figsize=(18, 9))
    for a, n in zip(ax, ["v1", final]):
        d = data[n]
        V = np.asarray(d["mesh"].vertices)
        h = np.median(d["P"][:, 2])
        sl = V[np.abs(V[:, 2] - h) < 0.1]
        a.scatter(sl[:, 0], sl[:, 1], s=0.3, c="k", label=f"mesh vertices z={h:.2f}±0.1 m")
        a.scatter(d["P"][:, 0], d["P"][:, 1], s=0.2, c="r", alpha=0.3, label="held-out LiDAR (<= 4 m)")
        p = d["Ts"][:, :3, 3]
        a.plot(p[:, 0], p[:, 1], "b-", lw=0.8, label="camera path")
        a.set_aspect("equal")
        a.set_xlim(lim_lo[0], lim_hi[0])
        a.set_ylim(lim_lo[1], lim_hi[1])
        a.grid(alpha=0.3)
        a.legend(loc="upper right", markerscale=10)
        a.set_title(n)
    fig.tight_layout()
    fig.savefig(out / "lidar_overlay_v1_vs_v2.png", dpi=100)
    plt.close(fig)


def table_plot(path, meshes, tdet):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if tdet is None:
        return
    c, r = np.array(tdet["center"]), tdet["radius"] + 0.5
    fig, ax = plt.subplots(len(meshes), 2, figsize=(14, 6 * len(meshes)), squeeze=False)
    for row, (n, m) in enumerate(meshes.items()):
        V = np.asarray(m.vertices)
        C = np.asarray(m.vertex_colors) if m.has_vertex_colors() else None
        s = (np.abs(V[:, 0] - c[0]) < r) & (np.abs(V[:, 1] - c[1]) < r) & (V[:, 2] > 0.05) & (V[:, 2] < 1.5)
        cc = C[s] if C is not None else "k"
        ax[row, 0].scatter(V[s, 0], V[s, 1], s=1, c=cc)
        ax[row, 0].set_title(f"{n}: table region top view (z 0.05-1.5 m)")
        ax[row, 1].scatter(V[s, 0], V[s, 2], s=1, c=cc)
        ax[row, 1].set_title(f"{n}: side view x-z (legs would be vertical strokes below ~0.7 m)")
        for a in ax[row]:
            a.set_aspect("equal")
            a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


if __name__ == "__main__":
    main()
