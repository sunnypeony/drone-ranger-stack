#!/usr/bin/env python3
"""Pose-free check of the 2D LiDAR mounting against the camera depth (needs 02's frames).

Every scan_every-th extracted frame is paired with the nearest /scan (<= max_dt_s apart). The returns inside
the camera's field of view are put through a candidate base<-laser rotation (pitch, roll; translation and
yaw from /tf_static or lidar.T_base_laser), projected into the depth image, and compared with the depth
there. The pitch / roll with the most returns within `tol` of the depth wins. No trajectory is involved,
so this separates "the sensors disagree" from "the poses are wrong".

Reports depth / LiDAR ratios per range for the current extrinsic and the fitted one, plus the agreement
for scaled focal lengths (checks fx). Writes <outputs>/lidar/tilt_fit.json; put the fitted angles into
lidar.T_base_laser = [x, y, z, yaw, pitch, roll] of the dataset config to use them in 07 / eval_poses.
"""
import json

import cv2
import numpy as np
from tqdm import tqdm

from common import (T_base_laser_from_cfg, die, find_bag_dir, get_typestore, iter_topic, load_config, load_frames,
                    load_static_tf, make_T, open_reader, parse_args, rpy_to_R, stage_dir, stamp_ns, write_json)


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    C = cfg["lidar_tilt_fit"]
    fdir, rows, intr = load_frames(cfg)
    K = np.array(intr["K"])
    st_frames = np.array([int(r["stamp_ns"]) for r in rows], np.int64)

    ts = get_typestore(cfg)
    scans = []
    with open_reader(find_bag_dir(cfg)) as reader:
        stf = load_static_tf(reader, ts, cfg)
        T_bo = stf.lookup(cfg["frames"]["base"], cfg["frames"]["camera_optical"])
        if T_bo is None:
            die("need base->optical in /tf_static")
        T_bl0 = T_base_laser_from_cfg(cfg, stf)
        for _, m in tqdm(iter_topic(reader, ts, cfg["topics"]["scan"]), desc="scan"):
            scans.append((stamp_ns(m.header), m.angle_min, m.angle_increment, np.asarray(m.ranges, float)))
    if not scans:
        die(f"no messages on {cfg['topics']['scan']}")
    ss = np.array([s[0] for s in scans], np.int64)
    hfov = np.arctan2(intr["width"] / 2, K[0, 0])

    data = []
    for k in range(0, len(rows), C["frame_every"]):
        j = int(np.argmin(np.abs(ss - st_frames[k])))
        if abs(ss[j] - st_frames[k]) > C["max_dt_s"] * 1e9:
            continue
        _, a0, da, rg = scans[j]
        th = (a0 + da * np.arange(len(rg)) + np.pi) % (2 * np.pi) - np.pi
        ok = (np.abs(th) < hfov) & np.isfinite(rg) & (rg >= C["range_min"]) & (rg <= C["range_max"])
        depth = cv2.imread(str(fdir / rows[k]["depth"]), cv2.IMREAD_UNCHANGED).astype(float) * C["depth_scale"]
        P = np.stack([rg[ok] * np.cos(th[ok]), rg[ok] * np.sin(th[ok]), np.zeros(ok.sum())])
        data.append((depth, P))
    if not data:
        die("no frame / scan pairs within max_dt_s")
    print(f"{len(data)} frame/scan pairs")

    T_ob = np.linalg.inv(T_bo)

    def pairs(R_bl, f_scale=1.0):
        """(lidar z in camera, depth at the projected pixel) for every return that lands on valid depth."""
        T_ol = T_ob @ make_T(R_bl, T_bl0[:3, 3])
        out = []
        for depth, P in data:
            Q = T_ol[:3, :3] @ P + T_ol[:3, 3:4]
            Z = Q[2]
            front = Z > 0.3
            u = np.round(K[0, 2] + K[0, 0] * f_scale * Q[0, front] / Z[front]).astype(int)
            v = np.round(K[1, 2] + K[1, 1] * f_scale * Q[1, front] / Z[front]).astype(int)
            m = (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
            d = depth[v[m], u[m]]
            g = d > 0
            out.append(np.stack([Z[front][m][g], d[g]], 1))
        return np.concatenate(out)

    def agree(zd):
        return float(np.mean(np.abs(zd[:, 1] / zd[:, 0] - 1) < C["tol"])) if len(zd) else 0.0

    def ratios(zd):
        res = {}
        for lo, hi in C["range_bins"]:
            s = (zd[:, 0] >= lo) & (zd[:, 0] < hi)
            if s.sum() >= 30:
                res[f"{lo}-{hi}"] = {"n": int(s.sum()), "median_depth_over_lidar": float(np.median(zd[s, 1] / zd[s, 0]))}
        return res

    # yaw of the configured / tf_static extrinsic, kept fixed; search pitch and roll on top of it
    R0 = T_bl0[:3, :3]
    yaw0 = float(np.degrees(np.arctan2(R0[1, 0], R0[0, 0])))
    cur = pairs(R0)
    grid = [(p, r) for p in np.arange(-C["search_deg"], C["search_deg"] + 1e-9, C["coarse_step_deg"])
            for r in np.arange(-C["search_deg"] / 2, C["search_deg"] / 2 + 1e-9, C["coarse_step_deg"])]
    score = {pr: agree(pairs(rpy_to_R(pr[1], pr[0], yaw0))) for pr in tqdm(grid, desc="coarse")}
    p0, r0 = max(score, key=score.get)
    s = C["coarse_step_deg"]
    fine = [(p, r) for p in np.arange(p0 - s, p0 + s + 1e-9, C["fine_step_deg"])
            for r in np.arange(r0 - s, r0 + s + 1e-9, C["fine_step_deg"])]
    score = {pr: agree(pairs(rpy_to_R(pr[1], pr[0], yaw0))) for pr in tqdm(fine, desc="fine")}
    pb, rb = max(score, key=score.get)
    R_best = rpy_to_R(rb, pb, yaw0)
    best = pairs(R_best)

    rep = {"frame_scan_pairs": len(data), "tol": C["tol"], "yaw_deg_fixed": yaw0,
           "current": {"agree": agree(cur), "ratios": ratios(cur)},
           "fitted": {"pitch_deg": round(float(pb), 2), "roll_deg": round(float(rb), 2), "agree": agree(best),
                      "ratios": ratios(best)},
           "focal_scale_agree_at_fit": {str(f): agree(pairs(R_best, f)) for f in C["focal_scales"]},
           "T_base_laser_config": [*np.round(T_bl0[:3, 3], 4).tolist(), round(yaw0, 2), round(float(pb), 2),
                                   round(float(rb), 2)]}
    write_json(stage_dir(cfg, "lidar") / "tilt_fit.json", rep)
    print(json.dumps(rep, indent=2))


if __name__ == "__main__":
    main()
