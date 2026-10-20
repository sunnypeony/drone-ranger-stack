#!/usr/bin/env python3
"""Stage 03 (checkpoint 2): per-frame T_world_optical from bag TF or Open3D RGB-D odometry + loop closure.

Conventions
  T_a_b maps points in frame b to frame a:  a_P = T_a_b @ b_P.
  Open3D compute_rgbd_odometry(source=i, target=j) returns T_j_i (camera i -> camera j).
  PoseGraph node i stores T_cam0_cami; edge (i, j) stores T_j_i.
"""
import csv
import json
import time

import numpy as np
from tqdm import tqdm

from common import (assert_rigid, coverage, die, find_bag_dir, get_typestore, interp_T, load_config,
                    load_dynamic_tf, load_frames, load_static_tf, make_T, o3d_intrinsic, open_reader,
                    parse_args, rot_angle_deg, rot_to_quat, stage_dir, write_json)


# ----------------------------------------------------------------------------- IO helpers
class FrameCache:
    def __init__(self, fdir, rows, intr, cfg):
        import open3d as o3d
        self.o3d, self.fdir, self.rows, self.cfg = o3d, fdir, rows, cfg
        self.intr = o3d_intrinsic(intr)
        self._c = {}

    def rgbd(self, i, depth_trunc, intensity=True):
        key = (i, depth_trunc, intensity)
        if key not in self._c:
            o3d = self.o3d
            r = self.rows[i]
            color = o3d.io.read_image(str(self.fdir / r["color"]))
            depth = o3d.io.read_image(str(self.fdir / r["depth"]))
            if np.asarray(color).shape[:2] != np.asarray(depth).shape[:2]:
                die(f"frame {i}: color/depth size mismatch")
            self._c[key] = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, depth_scale=self.cfg["tsdf"]["depth_scale"], depth_trunc=depth_trunc,
                convert_rgb_to_intensity=intensity)
            if len(self._c) > 64:
                self._c.pop(next(iter(self._c)))
        return self._c[key]

    def pcd(self, i, depth_trunc):
        return self.o3d.geometry.PointCloud.create_from_rgbd_image(self.rgbd(i, depth_trunc, False), self.intr)

    def valid_ratio(self, i, depth_trunc):
        return float((np.asarray(self.rgbd(i, depth_trunc).depth) > 0).mean())


# ----------------------------------------------------------------------------- bag_tf
def poses_from_bag(cfg, rows, T_base_optical, times, Ts):
    gap = int(cfg["poses"]["tf_interp_max_gap_s"] * 1e9)
    out_idx, out_T = [], []
    for i, r in enumerate(rows):
        T_ob = interp_T(int(r["stamp_ns"]), times, Ts, gap)
        if T_ob is None:
            continue
        out_idx.append(i)
        out_T.append(T_ob @ T_base_optical)
    return np.array(out_idx, int), np.array(out_T)


# ----------------------------------------------------------------------------- open3d odometry
def odo_option(P, o3d):
    return o3d.pipelines.odometry.OdometryOption(
        iteration_number_per_pyramid_level=o3d.utility.IntVector(P["odometry_iterations"]),
        depth_diff_max=P["odometry_depth_diff_max"], depth_min=P["odometry_min_depth"],
        depth_max=P["odometry_depth_max"])


def rgbd_odometry(fc, i, j, init, P, o3d):
    """Returns (success, T_j_i, info)."""
    src = fc.rgbd(i, P["odometry_depth_max"])
    tgt = fc.rgbd(j, P["odometry_depth_max"])
    ok, T, info = o3d.pipelines.odometry.compute_rgbd_odometry(
        src, tgt, fc.intr, init, o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), odo_option(P, o3d))
    return bool(ok), np.asarray(T), np.asarray(info)


def step_ok(T, info, P):
    return (np.linalg.norm(T[:3, 3]) <= P["odometry_max_translation"]
            and rot_angle_deg(T[:3, :3]) <= P["odometry_max_rotation_deg"]
            and info[5, 5] >= P["odometry_info_min"])


def run_odometry(fc, kf, P, ckpt_path, o3d):
    """Sequential odometry between consecutive keyframes with checkpointing."""
    n = len(kf) - 1
    trans = np.zeros((n, 4, 4))
    infos = np.zeros((n, 6, 6))
    status = np.full(n, -1, int)  # -1 todo, 0 ok, 1 failed->extrapolated
    if ckpt_path.exists():
        z = np.load(ckpt_path)
        if len(z["status"]) == n and np.array_equal(z["kf"], kf):
            trans, infos, status = z["trans"].copy(), z["infos"].copy(), z["status"].copy()
            print(f"[resume] odometry checkpoint: {int((status >= 0).sum())}/{n} pairs done")
        else:
            print("[resume] odometry checkpoint does not match keyframes, recomputing")
    t0, done0 = time.time(), int((status >= 0).sum())
    pbar = tqdm(range(n), desc="odometry", initial=done0)
    for k in range(n):
        if status[k] >= 0:
            continue
        i, j = kf[k], kf[k + 1]
        v_prev = trans[k - 1] if k > 0 and status[k - 1] >= 0 else np.eye(4)
        good = False
        vr = min(fc.valid_ratio(i, P["odometry_depth_max"]), fc.valid_ratio(j, P["odometry_depth_max"]))
        if vr >= P["odometry_min_valid_ratio"]:
            for init in (np.eye(4), v_prev):
                ok, T, info = rgbd_odometry(fc, i, j, init, P, o3d)
                if ok and step_ok(T, info, P):
                    good = True
                    break
        if good:
            trans[k], infos[k], status[k] = T, info, 0
        else:  # constant-velocity extrapolation, weak edge
            trans[k], infos[k], status[k] = v_prev, np.eye(6) * P["odometry_fail_info_scale"], 1
        pbar.update(1)
        if (k + 1) % P["checkpoint_every"] == 0:
            np.savez(ckpt_path, trans=trans, infos=infos, status=status, kf=kf)
            el = time.time() - t0
            dn = int((status >= 0).sum()) - done0
            pbar.set_postfix(fail=int((status == 1).sum()), eta_s=int(el / max(dn, 1) * (n - k - 1)))
    pbar.close()
    np.savez(ckpt_path, trans=trans, infos=infos, status=status, kf=kf)
    return trans, infos, status


def chain(trans):
    """Accumulate T_cam0_cami from T_{i+1}_i."""
    poses = [np.eye(4)]
    for T in trans:
        poses.append(poses[-1] @ np.linalg.inv(T))
    return np.array(poses)


# ----------------------------------------------------------------------------- loop closure
def colored_icp(fc, i, j, init, P, o3d):
    reg = o3d.pipelines.registration
    src_full, tgt_full = fc.pcd(i, P["odometry_depth_max"]), fc.pcd(j, P["odometry_depth_max"])
    T = init.copy()
    for it, s in zip(P["icp_max_iter"], P["icp_radius_scales"]):
        r = P["icp_voxel"] * s
        src, tgt = src_full.voxel_down_sample(r), tgt_full.voxel_down_sample(r)
        if len(src.points) < P["icp_min_points"] or len(tgt.points) < P["icp_min_points"]:
            return None
        prm = o3d.geometry.KDTreeSearchParamHybrid(radius=r * P["icp_normal_radius_scale"], max_nn=P["icp_normal_max_nn"])
        src.estimate_normals(prm)
        tgt.estimate_normals(prm)
        try:
            res = reg.registration_colored_icp(
                src, tgt, r, T, reg.TransformationEstimationForColoredICP(),
                reg.ICPConvergenceCriteria(relative_fitness=P["icp_rel_eps"], relative_rmse=P["icp_rel_eps"],
                                           max_iteration=it))
        except RuntimeError:
            return None
        T = np.asarray(res.transformation)
    src, tgt = src_full.voxel_down_sample(P["icp_voxel"]), tgt_full.voxel_down_sample(P["icp_voxel"])
    ev = reg.evaluate_registration(src, tgt, P["icp_voxel"] * P["icp_eval_dist_scale"], T)
    info = reg.get_information_matrix_from_point_clouds(src, tgt, P["icp_voxel"] * P["icp_eval_dist_scale"], T)
    return {"T": T, "fitness": float(ev.fitness), "rmse": float(ev.inlier_rmse), "info": np.asarray(info)}


def try_loop(fc, i, j, est_T_j_i, P, o3d):
    best = None
    for init in (est_T_j_i, np.eye(4)):
        ok, T_odo, info_odo = rgbd_odometry(fc, i, j, init, P, o3d)
        icp_init = T_odo if (ok and info_odo[5, 5] >= P["loop_odometry_info_min"]) else init
        r = colored_icp(fc, i, j, icp_init, P, o3d)
        if r is None:
            continue
        r["odo_ok"] = ok
        if best is None or r["fitness"] > best["fitness"]:
            best = r
    return best


def loop_candidates(kf, stamps, poses, P):
    n = len(kf)
    gap = int(P["loop_min_gap_s"] * 1e9)
    cands = set()
    pos = poses[:, :3, 3]
    for a in range(0, n, P["loop_query_stride"]):
        scored = []
        for b in range(0, n, P["loop_candidate_stride"]):
            if abs(int(stamps[a]) - int(stamps[b])) <= gap or b >= a:
                continue
            d = np.linalg.norm(pos[a] - pos[b])
            ang = rot_angle_deg(poses[a][:3, :3].T @ poses[b][:3, :3])
            if d < P["loop_max_dist"] and ang < P["loop_max_angle_deg"]:
                scored.append((d, b))
        for _, b in sorted(scored)[: P["loop_max_candidates_per_frame"]]:
            cands.add((b, a))
    forced = set()
    h = P["loop_force_head_tail"]
    for b in range(0, min(h, n), P["loop_force_stride"]):
        for a in range(max(n - h, 0), n, P["loop_force_stride"]):
            if abs(int(stamps[a]) - int(stamps[b])) > gap:
                forced.add((b, a))
    return sorted(cands), sorted(forced - cands)


def run_open3d(cfg, fdir, rows, intr, out):
    import open3d as o3d
    P = cfg["poses"]
    fc = FrameCache(fdir, rows, intr, cfg)
    kf = np.arange(0, len(rows), P["keyframe_stride"])
    stamps = np.array([int(rows[i]["stamp_ns"]) for i in kf], np.int64)
    trans, infos, status = run_odometry(fc, kf, P, out / "ckpt_odometry.npz", o3d)
    odo_poses = chain(trans)

    # loop closure with checkpoint cache
    cache_path = out / "ckpt_loops.json"
    cache = json.load(open(cache_path)) if cache_path.exists() else {}
    cands, forced = loop_candidates(kf, stamps, odo_poses, P)
    print(f"loop candidates: gated={len(cands)} forced head/tail={len(forced)} (cached {len(cache)})")
    accepted = []
    t0 = time.time()
    todo = [(c, False) for c in cands] + [(c, True) for c in forced]
    for n_done, ((b, a), is_forced) in enumerate(tqdm(todo, desc="loops")):
        key = f"{kf[b]}_{kf[a]}"
        if key not in cache:
            est = np.linalg.inv(odo_poses[b]) @ odo_poses[a]  # T_b_a  (source a -> target b)
            r = try_loop(fc, kf[a], kf[b], est, P, o3d)
            cache[key] = None if r is None else {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in r.items()}
            if (n_done + 1) % P["checkpoint_every"] == 0:
                json.dump(cache, open(cache_path, "w"))
        r = cache[key]
        if r and r["fitness"] >= P["loop_fitness_min"] and r["rmse"] <= P["loop_rmse_max"]:
            accepted.append((a, b, np.array(r["T"]), np.array(r["info"]), r["fitness"], r["rmse"], is_forced))
    json.dump(cache, open(cache_path, "w"))
    print(f"loops accepted {len(accepted)}/{len(todo)} in {time.time() - t0:.0f}s")

    reg = o3d.pipelines.registration
    pg = reg.PoseGraph()
    for T in odo_poses:
        pg.nodes.append(reg.PoseGraphNode(T))
    for k in range(len(trans)):
        pg.edges.append(reg.PoseGraphEdge(k, k + 1, trans[k], infos[k], uncertain=False))
    for a, b, T, info, *_ in accepted:
        pg.edges.append(reg.PoseGraphEdge(a, b, T, info, uncertain=True))
    if accepted:
        opt = reg.GlobalOptimizationOption(
            max_correspondence_distance=P["pg_max_correspondence_distance"],
            edge_prune_threshold=P["pg_edge_prune_threshold"],
            preference_loop_closure=P["pg_preference_loop_closure"], reference_node=P["pg_reference_node"])
        reg.global_optimization(pg, reg.GlobalOptimizationLevenbergMarquardt(),
                                reg.GlobalOptimizationConvergenceCriteria(), opt)
    o3d.io.write_pose_graph(str(out / "pose_graph.json"), pg)
    opt_poses = np.array([np.asarray(nd.pose) for nd in pg.nodes])
    # normalise so node 0 is identity (reference node is fixed anyway)
    opt_poses = np.einsum("ij,njk->nik", np.linalg.inv(opt_poses[0]), opt_poses)
    n_loop_kept = sum(1 for e in pg.edges if e.uncertain)
    info = {
        "odometry_pairs": int(len(trans)),
        "odometry_failures": int((status == 1).sum()),
        "odometry_failure_indices": [int(kf[k]) for k in np.where(status == 1)[0]],
        "loop_candidates_gated": len(cands), "loop_candidates_forced": len(forced),
        "loop_edges_accepted": len(accepted), "loop_edges_forced_accepted": sum(1 for x in accepted if x[6]),
        "loop_edges_after_pruning": n_loop_kept,
        "loop_fitness": [round(x[4], 3) for x in accepted], "loop_rmse": [round(x[5], 4) for x in accepted],
        "pre_optimization_head_tail_dist": float(np.linalg.norm(odo_poses[-1][:3, 3] - odo_poses[0][:3, 3])),
    }
    return kf, opt_poses, odo_poses, info


# ----------------------------------------------------------------------------- outputs
def up_axis_of(world):
    return "-y" if world == "first_optical" else "+z"


def path_length(Ts):
    p = Ts[:, :3, 3]
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def topdown_axes(up):
    return (0, 1, "x", "y") if up == "+z" else (0, 2, "x", "z")


def save_trajectory(out, name, rows, idx, Ts, up, extra=None):
    with open(out / f"{name}.csv", "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i, T in zip(idx, Ts):
            q = rot_to_quat(T[:3, :3])
            t = T[:3, 3]
            f.write(f"{int(rows[i]['stamp_ns']) / 1e9:.9f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                    f"{q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}\n")


def plot_topdown(path, trajs, up, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    a, b, la, lb = topdown_axes(up)
    fig, ax = plt.subplots(figsize=(7, 7))
    for label, Ts, style in trajs:
        p = Ts[:, :3, 3]
        ax.plot(p[:, a], p[:, b], style, lw=1.2, label=label)
        ax.plot(p[0, a], p[0, b], "go", ms=9)
        ax.plot(p[-1, a], p[-1, b], "rs", ms=9)
        # camera heading arrows (optical +z) every ~N poses
        step = max(len(Ts) // 25, 1)
        for T in Ts[::step]:
            d = T[:3, 2]
            ax.arrow(T[a, 3], T[b, 3], d[a] * 0.15, d[b] * 0.15, head_width=0.03, color="gray", alpha=0.5)
    ax.plot([], [], "go", label="start")
    ax.plot([], [], "rs", label="end")
    ax.set_xlabel(f"{la} [m]")
    ax.set_ylabel(f"{lb} [m]")
    if up == "-y":
        ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def accumulated_preview(cfg, fdir, rows, intr, idx, Ts, out, up):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import open3d as o3d
    P = cfg["poses"]
    fc = FrameCache(fdir, rows, intr, cfg)
    acc = o3d.geometry.PointCloud()
    tags = []
    sel = list(range(0, len(idx), P["preview_every"]))
    for s in tqdm(sel, desc="preview"):
        pc = fc.pcd(idx[s], P["preview_depth_max"]).voxel_down_sample(P["preview_voxel"])
        pc.transform(Ts[s])
        acc += pc
        tags.append(np.full(len(pc.points), s / max(len(idx) - 1, 1)))
    o3d.io.write_point_cloud(str(out / "accumulated_preview.ply"), acc)
    # images for headless inspection: true color + colored by time (ghosting shows as offset colors)
    Pn = np.asarray(acc.points)
    C = np.asarray(acc.colors)
    tt = np.concatenate(tags) if tags else np.zeros(0)
    a, b, la, lb = topdown_axes(up)
    upi = 2 if up == "+z" else 1
    sgn = 1 if up == "+z" else -1
    fig, ax = plt.subplots(1, 3, figsize=(21, 7))
    ax[0].scatter(Pn[:, a], Pn[:, b], s=0.1, c=C)
    ax[0].set_title("accumulated preview, top view (rgb)")
    ax[1].scatter(Pn[:, a], Pn[:, b], s=0.1, c=tt, cmap="jet")
    ax[1].set_title("top view, colored by time (blue=start, red=end)")
    ax[2].scatter(Pn[:, a], sgn * Pn[:, upi], s=0.1, c=tt, cmap="jet")
    ax[2].set_title("side view (horizontal vs up)")
    for k in range(2):
        ax[k].set_xlabel(la), ax[k].set_ylabel(lb), ax[k].set_aspect("equal")
        if up == "-y":
            ax[k].invert_yaxis()
    ax[2].set_xlabel(la), ax[2].set_ylabel("up"), ax[2].set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out / "accumulated_preview.png", dpi=110)
    plt.close(fig)
    return len(acc.points)


def ate_rigid(A, B):
    """RMSE between two trajectories after best rigid alignment (Kabsch)."""
    pa, pb = A[:, :3, 3], B[:, :3, 3]
    ma, mb = pa.mean(0), pb.mean(0)
    U, _, Vt = np.linalg.svd((pb - mb).T @ (pa - ma))
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ S @ Vt
    res = (pa - ma) @ R.T + mb - pb
    return float(np.sqrt((res ** 2).sum(1).mean()))


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config)
    P, F = cfg["poses"], cfg["frames"]
    out = stage_dir(cfg, "poses")
    fdir, rows, intr = load_frames(cfg)
    stamps = np.array([int(r["stamp_ns"]) for r in rows], np.int64)

    bag = find_bag_dir(cfg)
    ts = get_typestore(cfg)
    with open_reader(bag) as reader:
        st = load_static_tf(reader, ts, cfg)
        times, Ts_ob, odom_src = load_dynamic_tf(reader, ts, cfg, F["odom"], F["base"])
    optical = intr.get("frame_id") or F["camera_optical"]
    T_base_optical = st.lookup(F["base"], optical)
    if T_base_optical is None and optical != F["camera_optical"]:
        T_base_optical = st.lookup(F["base"], F["camera_optical"])
    if T_base_optical is not None:
        assert_rigid(T_base_optical, cfg, "T_base_optical")
    cov = coverage(stamps, times, int(P["tf_interp_max_gap_s"] * 1e9))
    bag_ok = odom_src is not None and T_base_optical is not None and cov > cfg["inspect"]["odom_coverage_threshold"]
    print(f"odom->base source={odom_src} samples={len(times)} coverage={cov:.3f}; T_base_optical={'yes' if T_base_optical is not None else 'no'}")

    src = P["pose_source"]
    if src == "auto":
        src = "bag_tf" if bag_ok else "open3d"
    if src == "bag_tf" and not bag_ok:
        die(f"pose_source=bag_tf but odom->base coverage={cov:.2f} / T_base_optical missing")
    print(f"pose_source -> {src}")

    report = {"pose_source": src, "odom_coverage": cov, "odom_source": odom_src,
              "n_frames_extracted": len(rows), "T_base_optical": None if T_base_optical is None else T_base_optical.tolist()}
    results = {}
    if src == "bag_tf" or (P["compare_both"] and bag_ok):
        idx, Ts = poses_from_bag(cfg, rows, T_base_optical, times, Ts_ob)
        results["bag_tf"] = (idx, Ts, "odom", {"dropped_out_of_coverage": len(rows) - len(idx)})
    if src == "open3d" or P["compare_both"]:
        kf, opt, odo, info = run_open3d(cfg, fdir, rows, intr, out)
        if T_base_optical is not None:
            world = "base_link_t0"
            Ts = np.einsum("ij,njk->nik", T_base_optical, opt)
            Ts_odo = np.einsum("ij,njk->nik", T_base_optical, odo)
        else:
            world, Ts, Ts_odo = "first_optical", opt, odo
        results["open3d"] = (kf, Ts, world, info)
        results["open3d_odometry_only"] = (kf, Ts_odo, world, {})

    idx, Ts, world, extra = results[src]
    for k, T in enumerate(Ts):
        assert_rigid(T, cfg, f"T_world_optical[{k}]")
    up = up_axis_of(world)
    np.savez(out / "poses.npz", T_world_optical=Ts, frame_idx=idx, stamp_ns=stamps[idx], world=world, up=up,
             source=src, T_base_optical=np.eye(4) if T_base_optical is None else T_base_optical,
             has_T_base_optical=T_base_optical is not None)
    save_trajectory(out, "trajectory", rows, idx, Ts, up)

    p = Ts[:, :3, 3]
    report.update({
        "world_frame": world, "up_axis": up, "n_posed_frames": int(len(idx)),
        "duration_s": float((stamps[idx[-1]] - stamps[idx[0]]) / 1e9),
        "head_tail_distance_m": float(np.linalg.norm(p[-1] - p[0])),
        "head_tail_rotation_deg": rot_angle_deg(Ts[0][:3, :3].T @ Ts[-1][:3, :3]),
        "path_length_m": path_length(Ts),
        "bbox_min": p.min(0).tolist(), "bbox_max": p.max(0).tolist(),
    })
    report.update(extra)
    trajs = [(src, Ts, "b-")]
    if "open3d_odometry_only" in results and src == "open3d":
        trajs.append(("odometry only (before PGO)", results["open3d_odometry_only"][1], "c--"))
    if len(results) > 1 and "bag_tf" in results and "open3d" in results:
        ib, Tb, *_ = results["bag_tf"]
        io_, To, *_ = results["open3d"]
        common_ = np.intersect1d(ib, io_)
        if len(common_) > 3:
            A = Tb[np.searchsorted(ib, common_)]
            B = To[np.searchsorted(io_, common_)]
            report["comparison"] = {
                "frames": int(len(common_)), "ate_rmse_rigid_m": ate_rigid(A, B),
                "bag_tf_head_tail_m": float(np.linalg.norm(Tb[-1][:3, 3] - Tb[0][:3, 3])),
                "open3d_head_tail_m": float(np.linalg.norm(To[-1][:3, 3] - To[0][:3, 3])),
                "bag_tf_path_m": path_length(Tb), "open3d_path_m": path_length(To)}
            for name, (ii, TT, w, _) in results.items():
                if name != src:
                    save_trajectory(out, f"trajectory_{name}", rows, ii, TT, up_axis_of(w))
    plot_topdown(out / "trajectory_topdown.png", trajs, up,
                 f"{src}: head-tail {report['head_tail_distance_m']:.2f} m, path {report['path_length_m']:.1f} m")
    report["accumulated_preview_points"] = accumulated_preview(cfg, fdir, rows, intr, idx, Ts, out, up)
    write_json(out / "pose_report.json", report)
    print(json.dumps({k: v for k, v in report.items() if k not in ("loop_fitness", "loop_rmse",
                                                                    "odometry_failure_indices")}, indent=2))
    if report["head_tail_distance_m"] > P["head_tail_warn_m"]:
        print(f"[WARN] head-tail distance {report['head_tail_distance_m']:.2f} m > {P['head_tail_warn_m']} m")


if __name__ == "__main__":
    main()
