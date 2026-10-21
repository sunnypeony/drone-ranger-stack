#!/usr/bin/env python3
"""Stage 02: pair RGB+depth by header.stamp, filter/undistort, write color/, depth/ (uint16 mm), frames.csv."""
import csv
import json

import cv2
import numpy as np
from tqdm import tqdm

from common import (camera_info_to_dict, check_disk, check_K_vs_size, decode_color, decode_depth_m, die,
                    dir_size_bytes, find_bag_dir, get_typestore, human, iter_topic, iter_topic_raw,
                    load_config, open_reader, parse_args, stage_dir, stamp_ns, write_json)


def match_pairs(rgb_st, dep_st, tol_ns):
    """Return list of (rgb_stamp, depth_stamp). Exact first; nearest within tol as fallback."""
    dep_sorted = np.array(sorted(set(dep_st)), np.int64)
    dset = set(dep_st)
    pairs, used = [], set()
    for r in sorted(set(rgb_st)):
        if r in dset:
            pairs.append((r, r))
            used.add(r)
    if tol_ns > 0 and len(dep_sorted):
        exact_r = {p[0] for p in pairs}
        for r in sorted(set(rgb_st) - exact_r):
            j = int(np.searchsorted(dep_sorted, r))
            cands = [dep_sorted[k] for k in (j - 1, j) if 0 <= k < len(dep_sorted)]
            best = min(cands, key=lambda d: abs(d - r))
            if abs(best - r) <= tol_ns and best not in used:
                pairs.append((r, int(best)))
                used.add(int(best))
    pairs.sort()
    return pairs


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config)
    ex, T = cfg["extract"], cfg["topics"]
    out = stage_dir(cfg, "frames")
    (out / "color").mkdir(exist_ok=True)
    (out / "depth").mkdir(exist_ok=True)
    check_disk(cfg, out)
    bag = find_bag_dir(cfg)
    ts = get_typestore(cfg)

    with open_reader(bag) as reader:
        # header stamps (deserializing is required to read header)
        rgb_st = [stamp_ns(m.header) for _, m in iter_topic(reader, ts, T["rgb"])]
        dep_st = [stamp_ns(m.header) for _, m in iter_topic(reader, ts, T["depth"])]
        ci = None
        for _, m in iter_topic(reader, ts, T["camera_info"]):
            ci = camera_info_to_dict(m)
            break
        if ci is None:
            die(f"no CameraInfo on {T['camera_info']}")
        pairs = match_pairs(rgb_st, dep_st, int(ex["match_tolerance_ns"]))
        n_rgb_dup = len(rgb_st) - len(set(rgb_st))
        kept = pairs[:: ex["frame_stride"]]
        keep_rgb = {r: i for i, (r, _) in enumerate(kept)}
        keep_dep = {d: i for i, (_, d) in enumerate(kept)}
        print(f"rgb={len(rgb_st)} depth={len(dep_st)} pairs={len(pairs)} kept(stride {ex['frame_stride']})={len(kept)}")

        K = np.array(ci["K"])
        D = np.array(ci["D"])
        W, H = ci["width"], ci["height"]
        check_K_vs_size(K, W, H, "CameraInfo")
        d_zero = bool(np.all(np.abs(D) < ex["zero_d_eps"]))
        und = ex["undistort"]
        do_undistort = (not d_zero) if und == "auto" else bool(und)
        if do_undistort and ci["distortion_model"] not in ("plumb_bob", "rational_polynomial"):
            die(f"undistort requested but distortion_model={ci['distortion_model']} is unsupported")
        newK = K.copy()
        if do_undistort:
            newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (W, H), ex["undistort_alpha"], (W, H))
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, newK, (W, H), cv2.CV_32FC1)
        print(f"undistort={do_undistort} (mode={und}, D_all_zero={d_zero})")

        # write color frames
        for _, msg in tqdm(iter_topic(reader, ts, T["rgb"]), total=len(rgb_st), desc="color"):
            s = stamp_ns(msg.header)
            if s not in keep_rgb:
                continue
            i = keep_rgb.pop(s)  # pop: ignore duplicates with identical stamp
            rgb = decode_color(msg)
            if rgb.shape[:2] != (H, W):
                die(f"color frame {s} is {rgb.shape[1]}x{rgb.shape[0]}, CameraInfo says {W}x{H}")
            if do_undistort:
                rgb = cv2.remap(rgb, m1, m2, cv2.INTER_LINEAR)
            cv2.imwrite(str(out / "color" / f"{i:06d}.png"), rgb[:, :, ::-1])

        dmin, dmax = ex["depth_min"], ex["depth_max"]
        valid_ratio = []
        for _, msg in tqdm(iter_topic(reader, ts, T["depth"]), total=len(dep_st), desc="depth"):
            s = stamp_ns(msg.header)
            if s not in keep_dep:
                continue
            i = keep_dep.pop(s)
            d = decode_depth_m(msg, cfg)
            if d.shape != (H, W):
                die(f"depth frame {s} is {d.shape[1]}x{d.shape[0]}, color/CameraInfo is {W}x{H}")
            d[(d < dmin) | (d > dmax)] = 0.0
            mm = np.round(d * 1000.0).astype(np.uint16)
            if ex["denoise"] == "median":
                mm = cv2.medianBlur(mm, ex["median_ksize"])
            elif ex["denoise"] == "bilateral":
                f = mm.astype(np.float32)
                f2 = cv2.bilateralFilter(f, ex["bilateral_d"], ex["bilateral_sigma_color_mm"], ex["bilateral_sigma_space"])
                mm = np.where(mm > 0, np.round(f2), 0).astype(np.uint16)
            if do_undistort:
                mm = cv2.remap(mm, m1, m2, cv2.INTER_NEAREST)  # NN: never blend depth across edges
            valid_ratio.append(float((mm > 0).mean()))
            cv2.imwrite(str(out / "depth" / f"{i:06d}.png"), mm)

    if keep_rgb or keep_dep:
        die(f"missing written frames: rgb {len(keep_rgb)} depth {len(keep_dep)}")

    with open(out / "frames.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "stamp_ns", "depth_stamp_ns", "color", "depth"])
        for i, (r, d) in enumerate(kept):
            w.writerow([i, r, d, f"color/{i:06d}.png", f"depth/{i:06d}.png"])
    intr = {"width": W, "height": H, "K": newK.tolist(), "K_original": K.tolist(), "D_original": D.tolist(),
            "distortion_model": ci["distortion_model"], "undistorted": do_undistort,
            "frame_id": ci["frame_id"], "depth_scale": 1000.0}
    write_json(out / "intrinsics.json", intr)
    summary = {
        "rgb_msgs": len(rgb_st), "depth_msgs": len(dep_st), "rgb_duplicate_stamps": n_rgb_dup,
        "pairs": len(pairs), "dropped_rgb_unpaired": len(set(rgb_st)) - len(pairs),
        "dropped_depth_unpaired": len(set(dep_st)) - len(pairs),
        "match_tolerance_ns": ex["match_tolerance_ns"], "frame_stride": ex["frame_stride"], "kept": len(kept),
        "undistorted": do_undistort, "depth_valid_ratio_mean": float(np.mean(valid_ratio)) if valid_ratio else 0,
        "duration_s": (kept[-1][0] - kept[0][0]) / 1e9 if kept else 0,
        "disk": human(dir_size_bytes(out)),
    }
    write_json(out / "extract_report.json", summary)
    print(json.dumps(summary, indent=2))
    print(f"du -sh {out}: {summary['disk']}")


if __name__ == "__main__":
    main()
