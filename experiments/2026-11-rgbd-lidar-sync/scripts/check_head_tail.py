#!/usr/bin/env python3
"""Pose-free start/end adjudication: did the walk actually return to its start?

Samples n frames from the head and tail segments and registers every head x tail pair with
FPFH + RANSAC global registration (identity / no initial guess) followed by point-to-plane
ICP. No odometry or pose-graph result is used anywhere.

To make the 25 results comparable they are all chained to the same quantity, the pose of the
last frame in the first frame's camera:
    T_first_last = T_first_hi . T_hi_tj . T_tj_last
where the head-internal (hi -> first) and tail-internal (last -> tj) links are registered
the same pose-free way. If the loop really closed, the 25 chains agree on a small offset.

Writes outputs/head_tail/{head_tail.json, head_tail_rgb.png, head_tail_offsets.png}.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from tqdm import tqdm  # noqa: E402

from common import (die, load_config, load_frames, o3d_intrinsic, parse_args, rot_angle_deg,  # noqa: E402
                    stage_dir, write_json)

SURFACE, TEXT, TEXT2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"


class Registrar:
    def __init__(self, fdir, rows, intr, cfg):
        import open3d as o3d
        self.o3d, self.fdir, self.rows, self.cfg = o3d, fdir, rows, cfg
        self.C = cfg["head_tail_check"]
        self.intr = o3d_intrinsic(intr)
        self._feat = {}
        self.kept_frac = {}
        o3d.utility.random.seed(self.C["ransac_seed"])

    def features(self, i):
        if i in self._feat:
            return self._feat[i]
        o3d, C, r = self.o3d, self.C, self.rows[i]
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.io.read_image(str(self.fdir / r["color"])), o3d.io.read_image(str(self.fdir / r["depth"])),
            depth_scale=self.cfg["tsdf"]["depth_scale"], depth_trunc=C["depth_max"],
            convert_rgb_to_intensity=False)
        full = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intr)
        n_all = len(full.points)
        if C["remove_ground"]:
            _, inl = full.segment_plane(C["ground_dist"], 3, C["ground_ransac_iters"])
            if len(inl) >= C["ground_min_frac"] * n_all:
                full = full.select_by_index(inl, invert=True)
        self.kept_frac[i] = len(full.points) / max(n_all, 1)
        pyr = []
        for v in C["icp_voxels"]:
            p = full.voxel_down_sample(v)
            p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=v * C["normal_radius_scale"], max_nn=30))
            pyr.append(p)
        v = C["voxel"]
        down = full.voxel_down_sample(v)
        down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=v * C["normal_radius_scale"], max_nn=30))
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            down, o3d.geometry.KDTreeSearchParamHybrid(radius=v * C["fpfh_radius_scale"], max_nn=C["fpfh_max_nn"]))
        self._feat[i] = (down, fpfh, pyr)
        return self._feat[i]

    def register(self, src, tgt):
        """T_tgt_src (src camera -> tgt camera), pose-free."""
        if src == tgt:
            return {"T": np.eye(4), "ransac_fitness": 1.0, "fitness": 1.0, "rmse": 0.0}
        reg, C = self.o3d.pipelines.registration, self.C
        ds, fs, ps = self.features(src)
        dt, ft, pt = self.features(tgt)
        dist = C["voxel"] * C["ransac_dist_scale"]
        ran = reg.registration_ransac_based_on_feature_matching(
            ds, dt, fs, ft, True, dist, reg.TransformationEstimationPointToPoint(False), 3,
            [reg.CorrespondenceCheckerBasedOnEdgeLength(C["ransac_edge_length"]),
             reg.CorrespondenceCheckerBasedOnDistance(dist)],
            reg.RANSACConvergenceCriteria(C["ransac_max_iter"], C["ransac_confidence"]))
        T = ran.transformation
        for v, it, a, b in zip(C["icp_voxels"], C["icp_max_iter"], ps, pt):
            res = reg.registration_icp(a, b, v * C["icp_corr_scale"], T,
                                       reg.TransformationEstimationPointToPlane(),
                                       reg.ICPConvergenceCriteria(max_iteration=it))
            T = res.transformation
        return {"T": T, "ransac_fitness": float(ran.fitness), "fitness": float(res.fitness),
                "rmse": float(res.inlier_rmse)}


def summary(T):
    return {"translation": T[:3, 3], "distance_m": float(np.linalg.norm(T[:3, 3])),
            "rotation_deg": rot_angle_deg(T[:3, :3])}


def plot_rgb(path, fdir, rows, head, tail):
    import cv2
    n = len(head)
    fig, axes = plt.subplots(n, 2, figsize=(9, 2.9 * n), facecolor=SURFACE)
    for k, (h, t) in enumerate(zip(head, tail)):
        for a, i, lab in ((axes[k, 0], h, "head"), (axes[k, 1], t, "tail")):
            img = cv2.imread(str(fdir / rows[i]["color"]))[..., ::-1]
            a.imshow(img)
            a.set_xticks([])
            a.set_yticks([])
            t_s = (int(rows[i]["stamp_ns"]) - int(rows[0]["stamp_ns"])) / 1e9
            a.set_title(f"{lab}: frame {i}  (t = {t_s:.1f} s)", fontsize=9, loc="left", color=TEXT)
    fig.tight_layout()
    fig.savefig(path, dpi=100, facecolor=SURFACE)
    plt.close(fig)


def plot_offsets(path, chains, refs, fit_min):
    """Top view in the first frame's optical coordinates: x right, z forward."""
    fig, ax = plt.subplots(figsize=(6.5, 6), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=TEXT2, labelsize=8)
    good = [c for c in chains if c["valid"]]
    bad = [c for c in chains if not c["valid"]]
    for group, col, lab in ((good, BLUE, f"chain valid (all links fitness >= {fit_min})"),
                            (bad, TEXT2, "chain has a weak link")):
        if group:
            xs = np.array([c["T_first_last"]["translation"] for c in group])
            ax.scatter(xs[:, 0], xs[:, 2], s=60 if col == BLUE else 40, color=col, alpha=0.9 if col == BLUE else 0.5,
                       edgecolor=SURFACE, lw=1.5, label=lab, zorder=3)
    ax.scatter([0], [0], marker="+", s=150, color=TEXT, zorder=4)
    ax.annotate("first frame", (0, 0), textcoords="offset points", xytext=(8, -14), fontsize=8, color=TEXT)
    th = np.linspace(0, 2 * np.pi, 200)
    for (name, r), col in zip(refs.items(), (ORANGE, AQUA)):
        ax.plot(r * np.cos(th), r * np.sin(th), color=col, lw=1.2, ls="--", label=f"{name}: {r} m")
    ax.set_aspect("equal")
    ax.set_xlabel("x (right) [m]", color=TEXT2)
    ax.set_ylabel("z (forward) [m]", color=TEXT2)
    ax.set_title("last-frame position in first-frame camera (25 pose-free chains)", fontsize=10, loc="left", color=TEXT)
    ax.legend(fontsize=8, frameon=False, labelcolor=TEXT2, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)


def plot_sweep(path, sweep, fit_min, max_rot):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True, facecolor=SURFACE)
    t = np.array([x["t_s"] for x in sweep])
    d = np.array([x["distance_m"] for x in sweep])
    f = np.array([x["fitness"] for x in sweep])
    rot = np.array([x["rotation_deg"] for x in sweep])
    far = rot > max_rot
    ok = (f >= fit_min) & ~far
    for a in (a1, a2):
        a.set_facecolor(SURFACE)
        a.grid(True, color=GRID, lw=0.6)
        a.set_axisbelow(True)
        for s in a.spines.values():
            s.set_color(GRID)
        a.tick_params(colors=TEXT2, labelsize=8)
    a1.scatter(t[ok], d[ok], s=30, color=BLUE, label=f"fitness >= {fit_min}", zorder=3)
    a1.scatter(t[~ok & ~far], d[~ok & ~far], s=20, color=TEXT2, alpha=0.4, label="fitness too low", zorder=2)
    a1.scatter(t[far], d[far], s=24, marker="x", color=ORANGE, label=f"rotation > {max_rot:.0f} deg (no shared view)",
               zorder=2)
    a1.set_ylim(0, d.max() * 1.05)
    a1.set_ylabel("distance to first frame [m]", color=TEXT2)
    a1.set_title("pose-free registration of late frames against frame 0", fontsize=10, loc="left", color=TEXT)
    a1.legend(fontsize=8, frameon=False, labelcolor=TEXT2)
    a2.plot(t, f, color=BLUE, lw=2)
    a2.axhline(fit_min, color=TEXT2, lw=1, ls="--")
    a2.set_ylabel("ICP fitness", color=TEXT2)
    a2.set_xlabel("time since start [s]", color=TEXT2)
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    C = cfg["head_tail_check"]
    out = stage_dir(cfg, C["out_dir"])
    fdir, rows, intr = load_frames(cfg)
    n = len(rows)
    h0, h1 = C["head_frames"]
    if h1 > n or n - C["tail_frames"] < h1:
        die(f"head {C['head_frames']} / tail {C['tail_frames']} do not fit in {n} frames")
    head = np.linspace(h0, h1 - 1, C["n_samples"]).round().astype(int).tolist()
    tail = np.linspace(n - C["tail_frames"], n - 1, C["n_samples"]).round().astype(int).tolist()
    first, last = head[0], tail[-1]
    R = Registrar(fdir, rows, intr, cfg)

    links = {}
    jobs = [(h, first) for h in head] + [(last, t) for t in tail] + [(t, h) for h in head for t in tail]
    for src, tgt in tqdm(jobs, desc="pose-free registration"):
        links[(src, tgt)] = R.register(src, tgt)

    fit_min = C["fitness_min"]
    pairs, chains = [], []
    for h in head:
        for t in tail:
            a, b, c = links[(h, first)], links[(t, h)], links[(last, t)]
            X = a["T"] @ b["T"] @ c["T"]       # T_first_hi . T_hi_tj . T_tj_last
            pairs.append({"head": h, "tail": t, **summary(b["T"]),
                          **{k: b[k] for k in ("ransac_fitness", "fitness", "rmse")}})
            chains.append({"head": h, "tail": t, "T_first_last": summary(X), "T": X,
                           "link_fitness": [a["fitness"], b["fitness"], c["fitness"]],
                           "valid": min(a["fitness"], b["fitness"], c["fitness"]) >= fit_min})
    valid = [c for c in chains if c["valid"]]
    result = {"first_frame": first, "last_frame": last, "head_frames": head, "tail_frames": tail,
              "dt_first_last_s": (int(rows[last]["stamp_ns"]) - int(rows[first]["stamp_ns"])) / 1e9,
              "n_pairs": len(chains), "n_valid_chains": len(valid),
              "head_internal": [{"frame": h, **summary(links[(h, first)]["T"]), "fitness": links[(h, first)]["fitness"]}
                                for h in head],
              "tail_internal": [{"frame": t, **summary(links[(last, t)]["T"]), "fitness": links[(last, t)]["fitness"]}
                                for t in tail],
              "pairs_direct": pairs}
    if valid:
        tr = np.array([c["T_first_last"]["translation"] for c in valid])
        d = np.array([c["T_first_last"]["distance_m"] for c in valid])
        rot = np.array([c["T_first_last"]["rotation_deg"] for c in valid])
        med = np.median(tr, axis=0)
        agree = [c for c in valid
                 if np.linalg.norm(c["T_first_last"]["translation"] - med) <= C["agree_trans_m"]
                 and abs(c["T_first_last"]["rotation_deg"] - np.median(rot)) <= C["agree_rot_deg"]]
        result["chained_first_last"] = {
            "median_translation": med, "median_distance_m": float(np.linalg.norm(med)),
            "distance_m": {"min": d.min(), "p25": np.percentile(d, 25), "median": np.median(d),
                           "p75": np.percentile(d, 75), "max": d.max()},
            "rotation_deg": {"min": rot.min(), "median": np.median(rot), "max": rot.max()},
            "spread_from_median_m": {"median": float(np.median(np.linalg.norm(tr - med, axis=1))),
                                     "max": float(np.linalg.norm(tr - med, axis=1).max())},
            "n_agreeing": len(agree)}
    sweep = []
    for t in tqdm(range(n - C["sweep_frames"], n, C["sweep_stride"]), desc="sweep vs first"):
        r = R.register(t, first)
        sweep.append({"frame": t, "t_s": (int(rows[t]["stamp_ns"]) - int(rows[first]["stamp_ns"])) / 1e9,
                      **summary(r["T"]), "fitness": r["fitness"], "rmse": r["rmse"]})
    result["sweep_vs_first"] = sweep
    trusted = [x for x in sweep if x["fitness"] >= fit_min and x["rotation_deg"] <= C["max_plausible_rot_deg"]]
    if trusted:
        b = min(trusted, key=lambda x: x["distance_m"])
        result["closest_return"] = {k: b[k] for k in ("frame", "t_s", "distance_m", "rotation_deg", "fitness")}
    plot_sweep(out / "head_tail_sweep.png", sweep, fit_min, C["max_plausible_rot_deg"])
    per_tail = []
    for t in tail:
        ps = [p for p in pairs if p["tail"] == t and p["fitness"] >= fit_min]
        if not ps:
            per_tail.append({"tail": t, "n": 0})
            continue
        tr = np.array([p["translation"] for p in ps])
        med = np.median(tr, axis=0)
        per_tail.append({"tail": t, "n": len(ps), "t_s": (int(rows[t]["stamp_ns"]) - int(rows[first]["stamp_ns"])) / 1e9,
                         "median_distance_m": float(np.median([p["distance_m"] for p in ps])),
                         "median_rotation_deg": float(np.median([p["rotation_deg"] for p in ps])),
                         "spread_max_m": float(np.linalg.norm(tr - med, axis=1).max()),
                         "rot_range_deg": float(np.ptp([p["rotation_deg"] for p in ps])),
                         "median_fitness": float(np.median([p["fitness"] for p in ps]))})
    result["per_tail_direct"] = per_tail
    result["head_motion_max_m"] = max(x["distance_m"] for x in result["head_internal"])
    result["points_kept_after_ground_removal"] = {int(k): v for k, v in R.kept_frac.items()}
    result["chains"] = [{k: v for k, v in c.items() if k != "T"} for c in chains]
    result["reference_head_tail_m"] = C["reference_head_tail_m"]
    write_json(out / "head_tail.json", result)
    plot_rgb(out / "head_tail_rgb.png", fdir, rows, head, tail)
    plot_offsets(out / "head_tail_offsets.png", chains, C["reference_head_tail_m"], fit_min)

    print(f"\nfirst frame {first}, last frame {last} ({result['dt_first_last_s']:.1f} s apart)")
    print("head links (h -> first):", [(x["frame"], round(x["distance_m"], 3), round(x["fitness"], 2))
                                       for x in result["head_internal"]])
    print("tail links (last -> t): ", [(x["frame"], round(x["distance_m"], 3), round(x["fitness"], 2))
                                       for x in result["tail_internal"]])
    print(f"{'head':>5s} {'tail':>5s} {'direct d':>9s} {'rot':>6s} {'fit':>5s} {'rmse':>6s} | "
          f"{'chained d':>9s} {'rot':>6s} valid")
    for p, c in zip(pairs, chains):
        print(f"{p['head']:5d} {p['tail']:5d} {p['distance_m']:9.3f} {p['rotation_deg']:6.1f} {p['fitness']:5.2f} "
              f"{p['rmse']:6.3f} | {c['T_first_last']['distance_m']:9.3f} {c['T_first_last']['rotation_deg']:6.1f} "
              f"{c['valid']}")
    if valid:
        s = result["chained_first_last"]
        print(f"\nvalid chains {len(valid)}/{len(chains)}, agreeing {s['n_agreeing']}; "
              f"first->last distance median {s['distance_m']['median']:.3f} m "
              f"(IQR {s['distance_m']['p25']:.3f}-{s['distance_m']['p75']:.3f}), "
              f"rotation median {s['rotation_deg']['median']:.1f} deg, "
              f"spread from median {s['spread_from_median_m']['median']:.3f} m")
    else:
        print("\nno valid chain: head/tail registration not trusted")
    if "closest_return" in result:
        print(f"\nclosest trusted return to frame {first}: {result['closest_return']}")
    print("\nper tail frame, direct vs the 5 (static) head frames:")
    for p in per_tail:
        if p["n"]:
            print(f"  tail {p['tail']:5d} (t={p['t_s']:.1f}s): d {p['median_distance_m']:.3f} m, rot "
                  f"{p['median_rotation_deg']:.1f} deg, spread {p['spread_max_m']:.3f} m / "
                  f"{p['rot_range_deg']:.1f} deg, fit {p['median_fitness']:.2f} (n={p['n']})")
        else:
            print(f"  tail {p['tail']:5d}: no trusted registration")
    print(f"references: {C['reference_head_tail_m']}  -> {out}")


if __name__ == "__main__":
    main()
