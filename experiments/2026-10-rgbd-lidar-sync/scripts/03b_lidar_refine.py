#!/usr/bin/env python3
"""Stage 03b (v2): decoupled pose refinement with 2D LiDAR constraints.

roll / pitch / z come from the v1 ground-plane-constrained visual odometry and are kept fixed;
x / y / yaw are re-estimated in an SE(2) pose graph (scipy least_squares + Cauchy loss) with
  visual odometry edges, (audited) visual loop edges, LiDAR adjacent-keyframe ICP edges and
  LiDAR loop edges (correlative scan matching + ICP).
Ablation variants (configs/v2.yaml v2.variants) are written to outputs/v2/ablation/<name>/poses/,
the final variant also to outputs/v2/poses/.
Only scans with index % 2 == constraint_parity are used here; the others are held out for evaluation.

v3 options (all default to the v2 behaviour): `cutoff_s` (ignore frames / scans after it),
`v2.use_v1_visual_loops`, `v2.drop_forced_head_tail`, and `v2.pf_loop_edges` + variant flag
`pf_loops` (pose-free camera loop edges from pf_loop_edges.py; only the "constraint" group is
used, the "eval" group is only scored).
"""
import importlib
import json
import shutil

import cv2
import numpy as np
from tqdm import tqdm

import lidar2d as L2
from common import (T_base_laser_from_cfg, assert_rigid, cutoff_count, die, exp_path, find_bag_dir,
                    get_typestore, interp_T, load_config, load_frames, load_static_tf, open_reader, parse_args,
                    rot_angle_deg, stage_dir, write_json)

m3 = importlib.import_module("03_estimate_poses")


# ----------------------------------------------------------------------------- blur
def blur_scores(fdir, rows, out):
    cache = out / "blur.npz"
    if cache.exists():
        z = np.load(cache)
        if len(z["score"]) == len(rows):
            return z["score"]
    s = np.zeros(len(rows))
    for i, r in enumerate(tqdm(rows, desc="blur (Laplacian variance)")):
        g = cv2.imread(str(fdir / r["color"]), cv2.IMREAD_GRAYSCALE)
        s[i] = cv2.Laplacian(g, cv2.CV_64F).var()
    np.savez(cache, score=s)
    return s


def runs_of(mask):
    """Contiguous True runs -> [[start, end], ...] (inclusive)."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append([start, i - 1])
            start = None
    if start is not None:
        out.append([start, len(mask) - 1])
    return out


# ----------------------------------------------------------------------------- visual chains
def v1_artifacts(cfg):
    d = exp_path(cfg["paths"]["v1_outputs_dir"]) / "poses"
    odo = np.load(d / "ckpt_odometry.npz")
    gr = np.load(d / "ckpt_ground.npz")
    loops = json.load(open(d / "ckpt_loops.json"))
    final = np.load(d / "poses.npz")
    if not np.array_equal(gr["kf"], np.arange(len(gr["kf"]))) or not np.array_equal(odo["kf"], gr["kf"]):
        die("v1 checkpoints must use keyframe_stride=1")
    return d, odo, gr, loops, final


def sharp_odometry(cfg, fc, kf, odo_v1, out):
    """Odometry between consecutive sharp keyframes; pairs already computed in v1 are reused."""
    import open3d as o3d
    P = cfg["poses"]
    path = out / "ckpt_odometry_sharp.npz"
    if not path.exists():
        lut = {(int(odo_v1["kf"][k]), int(odo_v1["kf"][k + 1])): k for k in range(len(odo_v1["status"]))}
        n = len(kf) - 1
        trans, infos = np.zeros((n, 4, 4)), np.zeros((n, 6, 6))
        status, fit = np.full(n, -1, int), np.full(n, np.nan)
        for k in range(n):
            j = lut.get((int(kf[k]), int(kf[k + 1])))
            if j is not None:
                trans[k], infos[k], status[k] = odo_v1["trans"][j], odo_v1["infos"][j], odo_v1["status"][j]
                fit[k] = odo_v1["fit"][j] if "fit" in odo_v1.files else np.nan
        print(f"sharp odometry: reused {int((status >= 0).sum())}/{n} pairs from v1")
        np.savez(path, trans=trans, infos=infos, status=status, kf=kf, fit=fit)
    return m3.run_odometry(fc, kf, P, path, o3d)


def ground_chain(cfg, trans, kf, gr, T_bo, up):
    P = cfg["poses"]
    odo = np.einsum("ij,njk->nik", T_bo, m3.chain(trans))
    Ts, ok = m3.apply_ground_constraint(odo, gr["normals"][kf], gr["heights"][kf], gr["valid"][kf], up, P)
    if not ok:
        die("ground constraint could not be applied")
    return Ts


class Traj:
    """Node trajectory: SE(2) of the leveled base + fixed roll/pitch + z, with time interpolation."""

    def __init__(self, stamps, T_wo, T_ob):
        self.t = np.asarray(stamps, np.int64)
        Twb = np.einsum("nij,jk->nik", T_wo, T_ob)
        parts = [L2.split_6dof(T) for T in Twb]
        self.se2 = np.array([p[0] for p in parts])
        self.rp = np.array([p[1] for p in parts])
        self.z = np.array([p[2] for p in parts])

    def at(self, t):
        th = np.unwrap(self.se2[:, 2])
        tt = self.t.astype(float)
        return np.array([np.interp(t, tt, self.se2[:, 0]), np.interp(t, tt, self.se2[:, 1]),
                         L2.wrap(np.interp(t, tt, th))])

    def nearest(self, t):
        i = int(np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1))
        return i if abs(self.t[i] - t) < abs(self.t[i - 1] - t) else i - 1

    def inside(self, t):
        return self.t[0] <= t <= self.t[-1]


# ----------------------------------------------------------------------------- scans -> leveled 2D
def leveled_scan(scan, V, Lc, traj, T_bl, ground_z, body, bins, dt_ns, yaw_off=0.0):
    pts, rng = L2.scan_points(scan, Lc, body, bins, yaw_off)
    if len(pts) == 0:
        return np.zeros((0, 2)), np.zeros(0)
    t = scan["stamp"] + dt_ns
    k = traj.nearest(t)
    pb = (T_bl[:3, :3] @ pts.T).T + T_bl[:3, 3]
    q = (traj.rp[k] @ pb.T).T
    h = traj.z[k] + q[:, 2] - ground_z
    m = (h >= V["min_height_above_ground"]) & (h <= V["max_height_above_ground"])
    return q[m, :2], rng[m]


def calib_objectives(scans_c, V, Lc, traj, T_bl, ground_z, body, bins, dt_s, yaw_off, mesh_scene, T_align,
                     mesh_range_max):
    """(1) local consistency: median NN distance between scans calib_pair_gap apart, projected with the
    trajectory; (2) median distance of the scans to the v1 mesh (3D)."""
    from scipy.spatial import cKDTree
    import open3d as o3d
    dt = int(dt_s * 1e9)
    sel = range(0, len(scans_c) - V["calib_pair_gap"], V["calib_scan_stride"])
    cons, pts3 = [], []
    for i in sel:
        a, b = scans_c[i], scans_c[i + V["calib_pair_gap"]]
        if not (traj.inside(a["stamp"] + dt) and traj.inside(b["stamp"] + dt)):
            continue
        pa, _ = leveled_scan(a, V, Lc, traj, T_bl, ground_z, body, bins, dt, yaw_off)
        pb, _ = leveled_scan(b, V, Lc, traj, T_bl, ground_z, body, bins, dt, yaw_off)
        if len(pa) < V["icp_min_points"] or len(pb) < V["icp_min_points"]:
            continue
        wa = L2.transform_pts(traj.at(a["stamp"] + dt), pa)
        wb = L2.transform_pts(traj.at(b["stamp"] + dt), pb)
        d, _ = cKDTree(wb).query(wa)
        cons.append(np.median(np.minimum(d, V["calib_nn_cap"])))
        k = traj.nearest(a["stamp"] + dt)
        se2 = traj.at(a["stamp"] + dt)
        Twb = L2.merge_6dof(se2, traj.rp[k], traj.z[k])
        pl, rl = L2.scan_points(a, Lc, body, bins, yaw_off)
        pl = pl[rl <= mesh_range_max]  # the mesh only exists within the RGB-D range
        P3 = (T_align @ Twb @ T_bl @ np.c_[pl, np.ones(len(pl))].T)[:3].T
        pts3.append(P3)
    P3 = np.concatenate(pts3)
    dm = mesh_scene.compute_distance(o3d.core.Tensor(P3.astype(np.float32))).numpy()
    return float(np.mean(cons)), float(np.median(dm))


def peak_info(xs, ys, V):
    ys = np.asarray(ys, float)
    w = V["calib_smooth"]
    ysm = np.convolve(np.pad(ys, w // 2, mode="edge"), np.ones(w) / w, mode="valid") if w > 1 else ys
    k = int(np.argmin(ysm))
    nmin = int(sum(1 for i in range(1, len(ysm) - 1) if ysm[i] < ysm[i - 1] and ysm[i] <= ysm[i + 1]))
    contrast = float((min(ysm[0], ysm[-1]) - ysm[k]) / max(ysm[k], 1e-12))
    clear = 0 < k < len(ysm) - 1 and nmin == 1 and contrast >= V["calib_peak_min_contrast"]
    return {"best": float(xs[k]), "contrast": contrast, "local_minima": nmin, "clear_single_peak": bool(clear)}


# ----------------------------------------------------------------------------- graph pieces
def submap(idx, scans_c, local, traj, V, dt):
    """Aggregate constraint scans idx-n..idx+n into scan idx's leveled frame (visual odometry)."""
    c = traj.at(scans_c[idx]["stamp"] + dt)
    pts = []
    for k in range(max(0, idx - V["submap_scans"]), min(len(scans_c), idx + V["submap_scans"] + 1)):
        if len(local[k]) == 0:
            continue
        rel = L2.between(c, traj.at(scans_c[k]["stamp"] + dt))
        pts.append(L2.transform_pts(rel, local[k]))
    return L2.voxel_down2d(np.concatenate(pts), V["submap_voxel"]) if pts else np.zeros((0, 2))


def select_keyframes(scans_c, traj, V, dt):
    kfs, last = [], None
    for i, s in enumerate(scans_c):
        t = s["stamp"] + dt
        if not traj.inside(t):
            continue
        p = traj.at(t)
        if last is None or np.linalg.norm(p[:2] - last[:2]) >= V["kf_dist"] or \
                abs(L2.wrap(p[2] - last[2])) >= np.radians(V["kf_angle_deg"]):
            kfs.append(i)
            last = p
    return kfs


def pose_diff(a, b):
    d = L2.between(a, b)
    return float(np.linalg.norm(d[:2])), float(np.degrees(abs(d[2])))


def visual_loop_list(cfg, loops_cache, n_cut, n_v1):
    """Accepted v1 visual loops with both ends before the cutoff. With drop_forced_head_tail, the
    pairs v1 forced between its first and last loop_force_head_tail keyframes are dropped too."""
    P, V = cfg["poses"], cfg["v2"]
    h = P["loop_force_head_tail"]
    out, n_forced = [], 0
    for key, r in loops_cache.items():
        if r and r["fitness"] >= P["loop_fitness_min"] and r["rmse"] <= P["loop_rmse_max"]:
            b, a = (int(x) for x in key.split("_"))
            if V.get("drop_forced_head_tail") and min(a, b) < h and max(a, b) >= n_v1 - h:
                n_forced += 1
                continue
            if max(a, b) >= n_cut:
                continue
            out.append((a, b, np.array(r["T"])))  # T = T_b_a (camera a -> camera b)
    return out, n_forced


def load_pf_edges(cfg):
    """Accepted pose-free loop edges [(first, frame, T_first_frame, group)]."""
    path = cfg["v2"].get("pf_loop_edges")
    if not path:
        return []
    f = exp_path(path)
    if not f.exists():
        die(f"{f} missing; run scripts/pf_loop_edges.py --config <this config> first")
    r = json.load(open(f))
    return [(e["first"], e["frame"], np.array(e["T_first_frame"]), e["group"]) for e in r["edges"] if e["accepted"]]


def pf_residuals(T_full, pf):
    """6DoF residual of every pose-free edge on the final poses (works for interpolated frames too)."""
    out = []
    for a, b, T_ref, g in pf:
        if b >= len(T_full):
            continue
        T = np.linalg.inv(T_full[a]) @ T_full[b]
        out.append({"first": a, "frame": b, "group": g, "ref_distance_m": float(np.linalg.norm(T_ref[:3, 3])),
                    "translation_error_m": float(np.linalg.norm(T[:3, 3] - T_ref[:3, 3])),
                    "rotation_error_deg": rot_angle_deg(T_ref[:3, :3].T @ T[:3, :3])})
    return out


def vis_loop_edge(a, b, T_b_a, traj, node_of, T_bo, T_ob):
    """Camera loop (a -> b) -> SE(2) measurement between(node b, node a) in leveled base frames."""
    ia, ib = node_of[a], node_of[b]
    M = T_bo @ T_b_a @ T_ob
    A = np.eye(4)
    A[:3, :3] = traj.rp[ia]
    B = np.eye(4)
    B[:3, :3] = traj.rp[ib]
    return ib, ia, L2.se2_from_T4(B @ M @ np.linalg.inv(A))


def main():
    import open3d as o3d
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    V, Lc, F = cfg["v2"], cfg["lidar"], cfg["frames"]
    out = stage_dir(cfg, "poses")
    fdir, rows, intr = load_frames(cfg)
    n_v1 = len(rows)
    n_cut = cutoff_count(cfg, rows)
    rows = rows[:n_cut]
    stamps = np.array([int(r["stamp_ns"]) for r in rows], np.int64)
    v1dir, odo_v1, gr, loops_cache, final_v1 = v1_artifacts(cfg)
    if n_cut < n_v1:
        print(f"cutoff {cfg['cutoff_s']} s: using frames 0..{n_cut - 1} of {n_v1}")
    rng = np.random.default_rng(0)

    ts = get_typestore(cfg)
    with open_reader(find_bag_dir(cfg)) as reader:
        st = load_static_tf(reader, ts, cfg)
        T_bo = st.lookup(F["base"], F["camera_optical"])
        scans = L2.load_scans(reader, ts, cfg["topics"]["scan"])
    if T_bo is None:
        die("v2 needs T_base_optical from /tf_static")
    T_ob = np.linalg.inv(T_bo)
    T_bl = T_base_laser_from_cfg(cfg)
    up = np.array([0, 0, 1.0])
    report = {"T_base_laser": T_bl.tolist(), "n_scans": len(scans), "cutoff_s": cfg.get("cutoff_s"),
              "n_frames": n_cut, "n_frames_extracted": n_v1}

    # ---- 1. blurry frames
    score = blur_scores(fdir, rows, out)
    thr = V["blur_threshold"] if V["blur_threshold"] is not None else float(np.percentile(score, V["blur_percentile"]))
    blurry = score < thr
    blurry[0] = False  # frame 0 defines the world
    sharp = np.where(~blurry)[0]
    runs = runs_of(blurry)
    report["blur"] = {"threshold": thr, "percentile": V["blur_percentile"], "n_blurry": int(blurry.sum()),
                      "n_frames": len(rows), "runs": runs, "longest_run_frames": max((b - a + 1 for a, b in runs), default=0),
                      "blurry_in_1850_2475": int(blurry[1850:2476].sum())}
    print(f"blur: threshold {thr:.1f}, {int(blurry.sum())} blurry frames in {len(runs)} runs")

    # ---- 2. visual chains (ground constrained)
    fc = m3.FrameCache(fdir, rows, intr, cfg)
    kf_all = np.arange(len(rows))
    T_all = ground_chain(cfg, odo_v1["trans"][:n_cut - 1], kf_all, gr, T_bo, up)
    tr_s, inf_s, stat_s = sharp_odometry(cfg, fc, sharp, odo_v1, out)
    T_sharp = ground_chain(cfg, tr_s, sharp, gr, T_bo, up)
    chains = {
        "all": {"kf": kf_all, "traj": Traj(stamps[kf_all], T_all, T_ob), "status": odo_v1["status"][:n_cut - 1],
                "T": T_all},
        "sharp": {"kf": sharp, "traj": Traj(stamps[sharp], T_sharp, T_ob), "status": stat_s, "T": T_sharp},
    }
    report["sharp_odometry"] = {"pairs": int(len(stat_s)), "failures": int((stat_s == 1).sum()),
                                "icp_fallbacks": int((stat_s == 2).sum())}
    hv = gr["valid"][:n_cut]
    ground_z = float(np.median(T_all[hv, 2, 3] - gr["heights"][:n_cut][hv]))
    report["ground_z_world"] = ground_z

    # ---- 3. scans: body sector, split
    body, bins, frac, sector = L2.detect_body_sector(scans, V)
    report["body_sector_deg"] = sector
    report["body_sector_max_persistence"] = float(frac.max())
    print(f"operator/body sector(s): {sector} (max near-return persistence {frac.max():.2f})")
    # parity on the index in the full scan list (eval_poses.py uses the same indexing), then the cutoff
    scans_c = [s for i, s in enumerate(scans) if i % 2 == V["constraint_parity"] and s["stamp"] <= stamps[-1]]
    report["scans_constraint"] = len(scans_c)
    report["scans_heldout"] = len(scans) - len(scans_c)

    # ---- 4. calibration search (diagnostic) on the v1 final trajectory + v1 mesh
    v1_mesh = o3d.io.read_triangle_mesh(str(exp_path(cfg["paths"]["v1_outputs_dir"]) / "mesh" / "mesh_clean.ply"))
    T_align_v1 = np.load(exp_path(cfg["paths"]["v1_outputs_dir"]) / "mesh" / "T_align.npy")
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(v1_mesh))
    traj_v1 = Traj(final_v1["stamp_ns"], final_v1["T_world_optical"], T_ob)
    calib_path = out / "lidar_calib.json"
    if calib_path.exists():
        calib = json.load(open(calib_path))
        print(f"[resume] calibration from {calib_path}")
    else:
        a0, a1, da = V["calib_time_offsets"]
        dts = np.round(np.arange(a0, a1 + 1e-9, da), 4)
        c_dt = [calib_objectives(scans_c, V, Lc, traj_v1, T_bl, ground_z, body, bins, d, 0.0, scene, T_align_v1,
                                 cfg['eval']['compare_range_max'])
                for d in tqdm(dts, desc="calib time offset")]
        pk_dt = {k: peak_info(dts, [c[i] for c in c_dt], V) for i, k in enumerate(("consistency", "mesh"))}
        adopt = V["calib_adopt_time_offset"] and pk_dt["consistency"]["clear_single_peak"]
        dt_adopt = pk_dt["consistency"]["best"] if adopt else 0.0
        y0, y1, dy = V["calib_yaw_offsets"]
        yaws = np.round(np.arange(y0, y1 + 1e-9, dy), 3)
        c_y = [calib_objectives(scans_c, V, Lc, traj_v1, T_bl, ground_z, body, bins, dt_adopt, y, scene, T_align_v1,
                                 cfg['eval']['compare_range_max'])
               for y in tqdm(yaws, desc="calib yaw offset")]
        pk_y = {k: peak_info(yaws, [c[i] for c in c_y], V) for i, k in enumerate(("consistency", "mesh"))}
        calib = {"time_offsets": dts.tolist(), "time_consistency": [c[0] for c in c_dt], "time_mesh": [c[1] for c in c_dt],
                 "time_peak": pk_dt, "time_offset_adopted": bool(adopt), "time_offset_s": float(dt_adopt),
                 "yaw_offsets": yaws.tolist(), "yaw_consistency": [c[0] for c in c_y], "yaw_mesh": [c[1] for c in c_y],
                 "yaw_peak": pk_y,
                 "yaw_warning": any(p["clear_single_peak"] and abs(p["best"]) > V["calib_yaw_warn_deg"] for p in pk_y.values()),
                 "yaw_applied": False}
        write_json(calib_path, calib)
        plot_calib(out / "lidar_calib.png", calib)
    report["calibration"] = {k: v for k, v in calib.items() if not isinstance(v, list)}
    dt = int(calib["time_offset_s"] * 1e9)
    print(f"time offset: {calib['time_offset_s']} s (adopted={calib['time_offset_adopted']}); "
          f"yaw peaks {calib['yaw_peak']}")

    # ---- 5. LiDAR keyframes, submaps, adjacent ICP (on the all-frames chain)
    traj0 = chains["all"]["traj"]
    local = [leveled_scan(s, V, Lc, traj0, T_bl, ground_z, body, bins, dt)[0] for s in scans_c]
    kfs = select_keyframes(scans_c, traj0, V, dt)
    subs = {i: submap(i, scans_c, local, traj0, V, dt) for i in kfs}
    caches = {i: L2.normals2d(subs[i], V["icp_normal_k"]) for i in kfs if len(subs[i]) >= V["icp_min_points"]}
    adj = []
    rej = {"icp_failed": 0, "fitness": 0, "rmse": 0, "correction": 0}
    for a, b in zip(kfs[:-1], kfs[1:]):
        if a not in caches or len(subs[b]) < V["icp_min_points"]:
            rej["icp_failed"] += 1
            continue
        z0 = L2.between(traj0.at(scans_c[a]["stamp"] + dt), traj0.at(scans_c[b]["stamp"] + dt))
        r = L2.icp2d(subs[b], subs[a], z0, V, caches[a])
        if r is None:
            rej["icp_failed"] += 1
            continue
        dxy, dyaw = pose_diff(z0, r["T"])
        if r["fitness"] < V["adj_fitness_min"]:
            rej["fitness"] += 1
        elif r["rmse"] > V["adj_rmse_max"]:
            rej["rmse"] += 1
        elif dxy > V["adj_max_correction_xy"] or dyaw > V["adj_max_correction_deg"]:
            rej["correction"] += 1
        else:
            adj.append((a, b, r["T"], r["fitness"], r["rmse"], dxy, dyaw))
    # LiDAR vs visual-odometry rotation over the accepted adjacent edges, per time segment
    seg_s = V["rotation_diag_segment_s"]
    t0s = scans_c[0]["stamp"]
    segs = {}
    for a, b, Z, *_ in adj:
        z0 = L2.between(traj0.at(scans_c[a]["stamp"] + dt), traj0.at(scans_c[b]["stamp"] + dt))
        k = int((scans_c[a]["stamp"] - t0s) / 1e9 // seg_s)
        segs.setdefault(k, [0.0, 0.0])
        segs[k][0] += float(np.degrees(Z[2]))
        segs[k][1] += float(np.degrees(z0[2]))
    abs_l = sum(abs(v[0]) for v in segs.values())
    abs_v = sum(abs(v[1]) for v in segs.values())
    report["rotation_lidar_vs_visual"] = {
        "segment_s": seg_s, "per_segment_deg_lidar_visual": {int(k * seg_s): [round(v[0], 1), round(v[1], 1)]
                                                             for k, v in sorted(segs.items())},
        "abs_ratio": abs_l / max(abs_v, 1e-9)}
    print(f"rotation LiDAR/visual (sum |segment yaw|): {abs_l / max(abs_v, 1e-9):.3f}")
    report["camera_lidar_lateral_scale"] = camera_lidar_scale(scans, V, Lc, body, bins, fdir, rows, intr, stamps,
                                                              T_bo, T_bl, dt)
    print(f"camera<->LiDAR lateral scale diagnostic: {report['camera_lidar_lateral_scale']}")
    report["lidar_keyframes"] = len(kfs)
    report["lidar_adjacent"] = {"accepted": len(adj), "rejected": rej,
                                "correction_vs_visual_median_m": float(np.median([x[5] for x in adj])) if adj else None,
                                "correction_vs_visual_median_deg": float(np.median([x[6] for x in adj])) if adj else None}
    print(f"LiDAR keyframes {len(kfs)}, adjacent edges accepted {len(adj)}, rejected {rej}")

    # ---- 6. variants
    if V.get("use_v1_visual_loops", True):
        vis_loops, n_forced = visual_loop_list(cfg, loops_cache, n_cut, n_v1)
    else:
        vis_loops, n_forced = [], 0
    report["visual_loops_v1"] = len(vis_loops)
    report["visual_loops_forced_dropped"] = n_forced
    pf = load_pf_edges(cfg)
    report["pf_loop_edges"] = {"constraint": sum(g == "constraint" for *_, g in pf), "eval": sum(g == "eval" for *_, g in pf)}
    loop_cache = {n: {} for n in V["variants"]}  # per variant: scan_node offsets differ between node sets
    results = {}
    for name, flags in V["variants"].items():
        use_sharp, use_audit = flags["blur_removal"], flags["audit_visual_loops"]
        use_adj, use_loops = flags["lidar_adjacent"], flags["lidar_loops"]
        use_pf = flags.get("pf_loops", False)
        ch = chains["sharp" if use_sharp else "all"]
        nodes, traj, status = ch["kf"], ch["traj"], ch["status"]
        node_of = {int(f): k for k, f in enumerate(nodes)}
        vr = {}

        def scan_node(i):
            t = scans_c[i]["stamp"] + dt
            n = traj.nearest(t)
            delta = L2.between(traj.se2[n], traj.at(t))
            return n, delta

        # per-step visual odometry sigmas (failed / extrapolated steps are weak)
        sc = np.where(np.asarray(status) == 1, V["sigma_vis_odo_fail_scale"], 1.0)
        step_sigma = np.stack([V["sigma_vis_odo_xy"] * sc, np.radians(V["sigma_vis_odo_deg"]) * sc], 1)

        # visual loops (audited in "full")
        vl_edges, audit = [], {"kept": 0, "rejected": 0, "inconclusive_downweighted": 0, "dropped_blurry_endpoint": 0}
        for a, b, T in vis_loops:
            if a not in node_of or b not in node_of:
                audit["dropped_blurry_endpoint"] += 1
                continue
            i, j, z = vis_loop_edge(a, b, T, traj, node_of, T_bo, T_ob)
            sxy, syaw = V["sigma_vis_loop_xy"], np.radians(V["sigma_vis_loop_deg"])
            if use_audit:
                verdict = audit_visual_loop(a, b, i, j, z, stamps, scans_c, local, traj, V, dt, rng)
                if verdict == "rejected":
                    audit["rejected"] += 1
                    continue
                if verdict == "inconclusive":
                    audit["inconclusive_downweighted"] += 1
                    sxy *= V["audit_inconclusive_sigma_scale"]
                    syaw *= V["audit_inconclusive_sigma_scale"]
                else:
                    audit["kept"] += 1
            else:
                audit["kept"] += 1
            vl_edges.append((i, j, z, sxy, syaw, a, b))

        def lidar_adj_edges():
            e = []
            for a, b, Z, *_ in adj:
                na, da = scan_node(a)
                nb, db = scan_node(b)
                if na == nb:
                    continue
                e.append((na, nb, L2.compose(L2.compose(da, Z), L2.inverse(db)),
                          V["sigma_lidar_adj_xy"], np.radians(V["sigma_lidar_adj_deg"])))
            return e

        base_edges = [x[:5] for x in vl_edges]
        labels = ["vis_loop"] * len(vl_edges)
        pf_used, pf_dropped = 0, []
        if use_pf:
            for a, b, T_ref, g in pf:
                if g != "constraint":
                    continue
                if a not in node_of or b not in node_of:
                    pf_dropped.append(b)  # blurry endpoint: not a node of the sharp chain
                    continue
                i, j, z = vis_loop_edge(b, a, T_ref, traj, node_of, T_bo, T_ob)  # camera b -> camera a
                base_edges.append((i, j, z, V["sigma_pf_loop_xy"], np.radians(V["sigma_pf_loop_deg"])))
                labels.append("pf_loop")
                pf_used += 1
        if use_adj:
            la = lidar_adj_edges()
            base_edges += la
            labels += ["lidar_adj"] * len(la)
        edges, elabels = base_edges, labels
        X, ginfo = L2.solve_keyframe_graph(traj.se2, step_sigma, edges, elabels, V)
        lidar_loops = []
        loop_stats = {"candidates": 0, "accepted": 0, "rej_match": 0, "rej_ambiguous": 0, "rej_visual_inconsistent": 0}
        if use_loops:
            for rnd in range(V["loop_rounds"]):
                new = find_lidar_loops(kfs, scans_c, subs, caches, traj, X, scan_node, V, dt, rng, loop_cache[name],
                                       vl_edges, stamps, loop_stats, {(x[2], x[3]) for x in lidar_loops})
                lidar_loops += new
                # x = (scan b, scan a, node b, node a, between(node b, node a), fitness, rmse, csm ratio)
                edges = base_edges + [(x[2], x[3], x[4], V["sigma_lidar_loop_xy"], np.radians(V["sigma_lidar_loop_deg"]))
                                      for x in lidar_loops]
                elabels = labels + ["lidar_loop"] * len(lidar_loops)
                X, ginfo = L2.solve_keyframe_graph(traj.se2, step_sigma, edges, elabels, V)
                print(f"[{name}] loop round {rnd + 1}: +{len(new)} LiDAR loops (total {len(lidar_loops)})")
                if not new:
                    break
        # final 6DoF for every frame
        Tn = np.array([L2.merge_6dof(X[k], traj.rp[k], traj.z[k]) @ T_bo for k in range(len(nodes))])
        if use_sharp:
            T_full = np.zeros((len(rows), 4, 4))
            T_full[nodes] = Tn
            big = int(1e18)
            for f in np.where(blurry)[0]:
                T = interp_T(int(stamps[f]), stamps[nodes], Tn, big)
                T_full[f] = T if T is not None else Tn[int(np.argmin(np.abs(stamps[nodes] - stamps[f])))]
            mask = ~blurry
        else:
            T_full, mask = Tn, np.ones(len(rows), bool)
        for k, T in enumerate(T_full):
            assert_rigid(T, cfg, f"{name} pose {k}")
        res = pf_residuals(T_full, pf)
        vr["pf_loop_edges_used"] = pf_used
        vr["pf_loop_edges_dropped_blurry"] = pf_dropped
        vr["pf_residuals"] = res
        for g in ("constraint", "eval"):
            e = [x["translation_error_m"] for x in res if x["group"] == g]
            vr[f"pf_residual_{g}_median_m"] = float(np.median(e)) if e else None
        r2551 = [x for x in res if x["frame"] == 2551]
        vr["pf_residual_2551_m"] = r2551[0]["translation_error_m"] if r2551 else None
        vr.update({"nodes": len(nodes), "visual_odometry_edges": len(nodes) - 1,
                   "visual_loop_edges": len(vl_edges), "visual_loop_audit": audit if use_audit else None,
                   "flags": flags,
                   "lidar_adjacent_edges": len(lidar_adj_edges()) if use_adj else 0,
                   "lidar_loop_edges": len(lidar_loops), "lidar_loop_stats": loop_stats if use_loops else None,
                   "head_tail_m": float(np.linalg.norm(T_full[-1][:3, 3] - T_full[0][:3, 3]))})
        results[name] = vr
        gl = np.array(ginfo["labels"])
        vr["graph"] = {"cost": ginfo["cost"], "nfev": ginfo["nfev"], "status": ginfo["status"],
                       "graph_nodes": ginfo["n_nodes"],
                       "whitened_residual_median_by_type": {
                           t: float(np.median(ginfo["edge_whitened_norm"][gl == t])) for t in sorted(set(gl))}}
        write_variant(cfg, name, T_full, mask, stamps, rows, T_bo, vr)
        np.savez(exp_path(cfg["paths"]["outputs_dir"]) / "ablation" / name / "poses" / "graph.npz",
                 init=traj.se2, X=X, I=np.array([e[0] for e in edges]), J=np.array([e[1] for e in edges]),
                 Z=np.array([e[2] for e in edges]), S=np.array([[e[3], e[4]] for e in edges]),
                 labels=np.array(elabels), step_sigma=step_sigma)
    report["variants"] = results
    final = V["final_variant"]
    src = exp_path(cfg["paths"]["outputs_dir"]) / "ablation" / final / "poses"
    for f in ("poses.npz", "trajectory.csv", "pose_report.json", "trajectory_topdown.png"):
        shutil.copy2(src / f, out / f)
    write_json(out / "v2_pose_report.json", report)
    print(json.dumps({k: v for k, v in report.items() if k not in ("blur",)}, indent=1, default=str)[:4000])
    print(f"blur runs (frame idx): {report['blur']['runs'][:40]}")


def camera_lidar_scale(scans, V, Lc, body, bins, fdir, rows, intr, stamps, T_bo, T_bl, dt):
    """Trajectory-free check of the camera intrinsics against the LiDAR: for synchronized scan / depth
    pairs, scale the camera's lateral coordinates by s (fx' = fx / s) and measure how well the depth
    slice at laser height matches the scan. Diagnostic only; nothing is changed."""
    from scipy.spatial import cKDTree
    K = np.array(intr["K"])
    data = []
    for i in range(0, len(scans), V["scale_diag_scan_stride"]):
        t = scans[i]["stamp"] + dt
        f = int(np.argmin(np.abs(stamps - t)))
        if abs(stamps[f] - t) > V["scale_diag_max_dt_s"] * 1e9:
            continue
        d = cv2.imread(str(fdir / rows[f]["depth"]), cv2.IMREAD_UNCHANGED).astype(float) / 1000.0
        v, u = np.mgrid[0:d.shape[0]:2, 0:d.shape[1]:2]
        z = d[::2, ::2]
        mk = (z > Lc["range_min"]) & (z < V["scale_diag_max_range"])
        uu, vv, zz = u[mk], v[mk], z[mk]
        up = -(vv - K[1, 2]) * zz / K[1, 1]
        band = np.abs(up - (T_bl[2, 3] - T_bo[2, 3])) < V["scale_diag_band"]
        if band.sum() < 100:
            continue
        p, rr = L2.scan_points(scans[i], Lc, body, bins)
        pb = (T_bl @ np.c_[p, np.ones(len(p))].T)[:3].T[:, :2][rr < V["scale_diag_max_range"]]
        data.append((uu[band], zz[band], pb))
    scales = np.round(np.arange(*V["scale_diag_grid"]), 3)
    cost = []
    for sc in scales:
        meds = []
        half = np.degrees(np.arctan((K[0, 2]) * sc / K[0, 0]))
        for uu, zz, pb in data:
            xo = (uu - K[0, 2]) * zz / K[0, 0] * sc
            cam = np.stack([zz + T_bo[0, 3], -xo], 1)
            ang = np.degrees(np.arctan2(pb[:, 1], pb[:, 0] - T_bo[0, 3]))
            q = pb[np.abs(ang) < 0.95 * half]
            if len(q) >= 10:
                meds.append(np.median(np.minimum(cKDTree(cam).query(q)[0], 0.5)))
        cost.append(float(np.mean(meds)))
    pk = peak_info(scales, cost, V)
    return {"pairs": len(data), "scales": scales.tolist(), "cost": cost, "peak": pk,
            "implied_fx": float(K[0, 0] / pk["best"]), "fx_camera_info": float(K[0, 0])}


def audit_visual_loop(a, b, i, j, z, stamps, scans_c, local, traj, V, dt, rng):
    """LiDAR check of a visual loop (frames a -> b). Returns kept / rejected / inconclusive."""
    st = np.array([s["stamp"] + dt for s in scans_c])
    sa, sb = int(np.argmin(np.abs(st - stamps[a]))), int(np.argmin(np.abs(st - stamps[b])))
    if abs(st[sa] - stamps[a]) > V["audit_scan_max_dt_s"] * 1e9 or abs(st[sb] - stamps[b]) > V["audit_scan_max_dt_s"] * 1e9:
        return "inconclusive"
    A, B = submap(sa, scans_c, local, traj, V, dt), submap(sb, scans_c, local, traj, V, dt)
    if len(A) < V["icp_min_points"] or len(B) < V["icp_min_points"]:
        return "inconclusive"
    # predicted pose of scan a in scan b from the visual loop (z = between(node b, node a))
    da = L2.between(traj.se2[j], traj.at(st[sa]))
    db = L2.between(traj.se2[i], traj.at(st[sb]))
    pred = L2.compose(L2.compose(L2.inverse(db), z), da)
    cache = L2.normals2d(B, V["icp_normal_k"])
    r = L2.icp2d(A, B, pred, V, cache)
    ok = r is not None and r["fitness"] >= V["loop_fitness_min"] and r["rmse"] <= V["loop_rmse_max"]
    if not ok:
        c = L2.correlative_match(A, B, pred, V, rng)
        if c is None or c["ratio"] < V["loop_csm_ratio_min"]:
            return "inconclusive"
        r = L2.icp2d(A, B, c["T"], V, cache)
        if r is None or r["fitness"] < V["loop_fitness_min"] or r["rmse"] > V["loop_rmse_max"]:
            return "inconclusive"
    dxy, dyaw = pose_diff(pred, r["T"])
    return "kept" if dxy <= V["audit_agree_xy"] and dyaw <= V["audit_agree_deg"] else "rejected"


def find_lidar_loops(kfs, scans_c, subs, caches, traj, X, scan_node, V, dt, rng, cache, vl_edges, stamps, stats, done):
    """Candidates from the current estimate X; CSM (+-30 deg, +-0.5 m) then ICP; checks vs visual loops."""
    gap = V["loop_min_gap_s"] * 1e9
    info = []
    for i in kfs:
        n, d = scan_node(i)
        info.append((i, n, d, L2.compose(X[n], d), scans_c[i]["stamp"] + dt))
    out = []
    for ia, (a, na, da, pa, ta) in enumerate(info):
        cands = []
        for ib, (b, nb, db, pb, tb) in enumerate(info[:ia]):
            if ta - tb > gap and np.linalg.norm(pa[:2] - pb[:2]) < V["loop_max_dist"]:
                cands.append((np.linalg.norm(pa[:2] - pb[:2]), ib))
        for _, ib in sorted(cands)[: V["loop_max_candidates_per_kf"]]:
            b, nb, db, pb, tb = info[ib]
            if (nb, na) in done or b not in caches or len(subs[a]) < V["icp_min_points"]:
                continue
            key = (b, a)
            stats["candidates"] += 1
            z0 = L2.between(pb, pa)  # pose of scan a in scan b
            if key not in cache:
                c = L2.correlative_match(subs[a], subs[b], z0, V, rng)
                r = L2.icp2d(subs[a], subs[b], c["T"], V, caches[b]) if c else None
                cache[key] = (c, r)
            c, r = cache[key]
            if r is None or r["fitness"] < V["loop_fitness_min"] or r["rmse"] > V["loop_rmse_max"]:
                stats["rej_match"] += 1
                continue
            if c["ratio"] < V["loop_csm_ratio_min"]:
                stats["rej_ambiguous"] += 1
                continue
            if not visual_consistent(r["T"], ta, tb, vl_edges, stamps, traj, V):
                stats["rej_visual_inconsistent"] += 1
                continue
            stats["accepted"] += 1
            zn = L2.compose(L2.compose(db, r["T"]), L2.inverse(da))  # between(node b, node a)
            out.append((b, a, nb, na, zn, r["fitness"], r["rmse"], c["ratio"]))
            done.add((nb, na))
    return out


def visual_consistent(Z, ta, tb, vl_edges, stamps, traj, V):
    """If a kept visual loop connects times near (tb, ta), its prediction must agree with Z."""
    w = V["loop_visual_window_s"] * 1e9
    for i, j, z, _, _, fa, fb in vl_edges:  # z = between(node of fb, node of fa)
        if abs(stamps[fb] - tb) <= w and abs(stamps[fa] - ta) <= w:
            pred = L2.compose(L2.compose(L2.between(traj.at(tb), traj.at(stamps[fb])), z),
                              L2.between(traj.at(stamps[fa]), traj.at(ta)))
            dxy, dyaw = pose_diff(pred, Z)
            if dxy > V["loop_visual_agree_xy"] or dyaw > V["loop_visual_agree_deg"]:
                return False
    return True


def write_variant(cfg, name, Ts, mask, stamps, rows, T_bo, vr):
    d = exp_path(cfg["paths"]["outputs_dir"]) / "ablation" / name / "poses"
    d.mkdir(parents=True, exist_ok=True)
    idx = np.arange(len(rows))
    np.savez(d / "poses.npz", T_world_optical=Ts, frame_idx=idx, stamp_ns=stamps, world="base_link_t0", up="+z",
             source=f"v2_{name}", T_base_optical=T_bo, has_T_base_optical=True, tsdf_mask=mask)
    m3.save_trajectory(d, "trajectory", rows, idx, Ts, "+z")
    m3.plot_topdown(d / "trajectory_topdown.png", [(name, Ts, "b-")], "+z",
                    f"v2 {name}: head-tail {vr['head_tail_m']:.2f} m")
    write_json(d / "pose_report.json", {**vr, "path_length_m": m3.path_length(Ts),
                                        "n_tsdf_frames": int(mask.sum())})


def plot_calib(path, c):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    for col, (xs, a, b, lab, pk) in enumerate([
            (c["time_offsets"], c["time_consistency"], c["time_mesh"], "time offset [s]", c["time_peak"]),
            (c["yaw_offsets"], c["yaw_consistency"], c["yaw_mesh"], "yaw offset [deg]", c["yaw_peak"])]):
        ax[0, col].plot(xs, a, "o-")
        ax[0, col].set_title(f"scan-to-scan consistency (best {pk['consistency']['best']}, "
                             f"clear={pk['consistency']['clear_single_peak']})")
        ax[1, col].plot(xs, b, "o-", color="tab:orange")
        ax[1, col].set_title(f"scan-to-v1-mesh median (best {pk['mesh']['best']}, clear={pk['mesh']['clear_single_peak']})")
        for r in range(2):
            ax[r, col].set_xlabel(lab)
            ax[r, col].set_ylabel("m")
            ax[r, col].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
