#!/usr/bin/env python3
"""Stage 01 (checkpoint 1): inspect the bag -> outputs/inspect/report.json + sample frame dumps."""
import numpy as np

from common import (Timer, camera_info_to_dict, check_K_vs_size, coverage, decode_color, decode_depth_m,
                    dir_size_bytes, find_bag_dir, get_typestore, human, iter_topic, load_config,
                    load_dynamic_tf, load_static_tf, open_reader, parse_args, stage_dir, stamp_ns,
                    write_json)


def header_stamps(reader, ts, topic):
    """Deserialize only to get header stamps (cheap enough for 3 min of data)."""
    out = []
    for _, msg in iter_topic(reader, ts, topic):
        out.append(stamp_ns(msg.header))
    return np.asarray(out, np.int64)


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    out = stage_dir(cfg, "inspect")
    bag = find_bag_dir(cfg)
    ts = get_typestore(cfg)
    T, F, I = cfg["topics"], cfg["frames"], cfg["inspect"]
    report = {"bag_dir": str(bag), "bag_size_bytes": dir_size_bytes(bag), "bag_size": human(dir_size_bytes(bag))}
    errors = []

    with open_reader(bag) as reader:
        report["storage"] = sorted(p.name for p in bag.iterdir() if p.suffix in (".db3", ".mcap"))
        report["bag_duration_s"] = reader.duration / 1e9
        topics = {}
        for c in reader.connections:
            n = c.msgcount
            info = topics.setdefault(c.topic, {"type": c.msgtype, "count": 0})
            info["count"] += n
        # first/last bag time per topic
        for topic, info in topics.items():
            conns = [c for c in reader.connections if c.topic == topic]
            tmin = tmax = None
            for _, t, _ in reader.messages(connections=conns):
                tmin = t if tmin is None else min(tmin, t)
                tmax = t if tmax is None else max(tmax, t)
            info["first_ns"], info["last_ns"] = tmin, tmax
            dur = (tmax - tmin) / 1e9 if tmin is not None else 0
            info["duration_s"] = dur
            info["rate_hz"] = (info["count"] - 1) / dur if dur > 0 else None
        report["topics"] = topics
        for key in ("rgb", "depth", "camera_info"):
            if T[key] not in topics:
                errors.append(f"required topic {T[key]} ({key}) missing; available: {sorted(topics)}")
        if errors:
            report["errors"] = errors
            write_json(out / "report.json", report)
            for e in errors:
                print("[ERROR]", e)
            raise SystemExit(1)

        # ---- image encodings / sizes
        with Timer("header stamps"):
            rgb_st = header_stamps(reader, ts, T["rgb"])
            dep_st = header_stamps(reader, ts, T["depth"])
            ci_st = header_stamps(reader, ts, T["camera_info"])

        first = {}
        for key in ("rgb", "depth"):
            for _, msg in iter_topic(reader, ts, T[key]):
                first[key] = {"encoding": msg.encoding, "width": msg.width, "height": msg.height,
                              "step": msg.step, "is_bigendian": int(msg.is_bigendian),
                              "frame_id": msg.header.frame_id}
                break
        report["images"] = first
        if first["depth"]["encoding"] not in I["allowed_depth_encodings"]:
            errors.append(f"depth encoding {first['depth']['encoding']} not in {I['allowed_depth_encodings']}")
        if (first["rgb"]["width"], first["rgb"]["height"]) != (first["depth"]["width"], first["depth"]["height"]):
            errors.append(f"rgb size {first['rgb']['width']}x{first['rgb']['height']} != depth size "
                          f"{first['depth']['width']}x{first['depth']['height']} (depth not aligned to RGB?)")

        # ---- camera info
        ci = None
        for _, msg in iter_topic(reader, ts, T["camera_info"]):
            ci = camera_info_to_dict(msg)
            break
        D = np.array(ci["D"])
        ci["D_all_zero"] = bool(np.all(np.abs(D) < cfg["extract"]["zero_d_eps"]))
        report["camera_info"] = ci
        if (ci["width"], ci["height"]) != (first["rgb"]["width"], first["rgb"]["height"]):
            errors.append(f"CameraInfo size {ci['width']}x{ci['height']} != RGB size")
        check_K_vs_size(np.array(ci["K"]), first["rgb"]["width"], first["rgb"]["height"], "CameraInfo")

        # ---- stamp matching
        rs, ds, cs = set(rgb_st.tolist()), set(dep_st.tolist()), set(ci_st.tolist())
        both = rs & ds
        report["stamp_match"] = {
            "rgb_count": len(rgb_st), "depth_count": len(dep_st), "camera_info_count": len(ci_st),
            "rgb_unique": len(rs), "depth_unique": len(ds),
            "rgb_depth_exact": len(both),
            "rgb_depth_exact_ratio_of_rgb": len(both) / max(len(rs), 1),
            "rgb_depth_exact_ratio_of_depth": len(both) / max(len(ds), 1),
            "rgb_caminfo_exact_ratio_of_rgb": len(rs & cs) / max(len(rs), 1),
            "triple_exact": len(both & cs),
        }
        if len(both) == 0 and len(rgb_st) and len(dep_st):
            j = np.clip(np.searchsorted(dep_st, rgb_st), 1, len(dep_st) - 1)
            nn = np.minimum(np.abs(dep_st[j] - rgb_st), np.abs(dep_st[j - 1] - rgb_st))
            report["stamp_match"]["rgb_depth_nn_offset_ms"] = {
                "median": float(np.median(nn) / 1e6), "p90": float(np.percentile(nn, 90) / 1e6)}
        hs = report["stamp_match"]
        hs["header_minus_bag_time_ms_rgb_median"] = None  # filled below

        # ---- TF
        st = load_static_tf(reader, ts, cfg)
        T_bo = st.lookup(F["base"], F["camera_optical"])
        T_bl = st.lookup(F["base"], F["laser"])
        report["tf_static"] = {
            "frames": st.frames(),
            "edges": {c: p for c, (p, _) in st.edges.items()},
            "base_to_optical_exists": T_bo is not None,
            "base_to_laser_exists": T_bl is not None,
            "T_base_optical": None if T_bo is None else T_bo.tolist(),
            "T_base_laser": None if T_bl is None else T_bl.tolist(),
            "policy": st.policy,
            "conflicts": st.conflicts,
        }
        if F["camera_optical"] not in st.frames() and ci["frame_id"] != F["camera_optical"]:
            print(f"[warn] configured optical frame {F['camera_optical']} not in tf_static and != CameraInfo frame "
                  f"{ci['frame_id']}")
        dyn_frames = {}
        for _, msg in iter_topic(reader, ts, T["tf"]):
            for tr in msg.transforms:
                k = f"{tr.header.frame_id}->{tr.child_frame_id}"
                dyn_frames[k] = dyn_frames.get(k, 0) + 1
        report["tf_dynamic_pairs"] = dyn_frames
        times, Ts, src = load_dynamic_tf(reader, ts, cfg, F["odom"], F["base"])
        cov = coverage(np.array(sorted(both)), times, int(I["max_tf_gap_s"] * 1e9))
        report["odometry"] = {
            "odom_topic_present": T.get("odom") in topics,
            "odom_to_base_source": src,
            "samples": int(len(times)),
            "coverage_of_rgbd_pairs": cov,
            "available": bool(src is not None and cov >= I["odom_coverage_threshold"]),
            "span_s": float((times[-1] - times[0]) / 1e9) if len(times) > 1 else 0.0,
        }

        # ---- depth statistics + header vs bag time
        stride = I["depth_stats_every"]
        valid, inrange, med = [], [], []
        hb = []
        dmin, dmax = cfg["extract"]["depth_min"], cfg["extract"]["depth_max"]
        for i, (t, msg) in enumerate(iter_topic(reader, ts, T["depth"])):
            if i % stride:
                continue
            d = decode_depth_m(msg, cfg)
            v = d > 0
            valid.append(v.mean())
            inrange.append(((d >= dmin) & (d <= dmax)).mean())
            if v.any():
                med.append(float(np.median(d[v])))
            hb.append((t - stamp_ns(msg.header)) / 1e6)
        report["depth_stats"] = {
            "sampled": len(valid),
            "valid_ratio_mean": float(np.mean(valid)), "valid_ratio_min": float(np.min(valid)),
            "valid_ratio_max": float(np.max(valid)),
            "in_range_ratio_mean": float(np.mean(inrange)),
            "median_depth_m_mean": float(np.mean(med)) if med else None,
            "range_used": [dmin, dmax],
        }
        hs["bag_minus_header_ms_depth_median"] = float(np.median(hb))
        hs.pop("header_minus_bag_time_ms_rgb_median")

        # ---- sample frame
        pairs = sorted(both)
        if pairs:
            k = min(I["sample_frame_index"], len(pairs) - 1)
            target = pairs[k]
            rgb = dep = None
            for _, msg in iter_topic(reader, ts, T["rgb"]):
                if stamp_ns(msg.header) == target:
                    rgb = decode_color(msg)
                    break
            for _, msg in iter_topic(reader, ts, T["depth"]):
                if stamp_ns(msg.header) == target:
                    dep = decode_depth_m(msg, cfg)
                    break
            dump_sample(out, rgb, dep, np.array(ci["K"]), cfg)
            report["sample_frame"] = {"pair_index": k, "stamp_ns": target}

    report["errors"] = errors
    write_json(out / "report.json", report)
    print_summary(report)
    if errors:
        for e in errors:
            print("[ERROR]", e)
        raise SystemExit(1)


def dump_sample(out, rgb, dep, K, cfg):
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import open3d as o3d

    cv2.imwrite(str(out / "rgb.png"), rgb[:, :, ::-1])
    dmax = cfg["extract"]["depth_max"]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4))
    ax[0].imshow(rgb)
    ax[0].set_title("rgb")
    im = ax[1].imshow(np.where(dep > 0, dep, np.nan), cmap="turbo", vmin=0, vmax=dmax)
    ax[1].set_title(f"depth [m] valid={np.mean(dep > 0):.2f}")
    plt.colorbar(im, ax=ax[1])
    for a in ax:
        a.axis("off")
    fig.tight_layout()
    fig.savefig(out / "depth_color.png", dpi=110)
    plt.close(fig)

    s = cfg["inspect"]["sample_pointcloud_stride"]
    h, w = dep.shape
    v, u = np.mgrid[0:h:s, 0:w:s]
    z = dep[::s, ::s]
    m = (z >= cfg["extract"]["depth_min"]) & (z <= dmax)
    x = (u[m] - K[0, 2]) * z[m] / K[0, 0]
    y = (v[m] - K[1, 2]) * z[m] / K[1, 1]
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.stack([x, y, z[m]], 1)))
    pc.colors = o3d.utility.Vector3dVector(rgb[::s, ::s][m] / 255.0)
    o3d.io.write_point_cloud(str(out / "frame0.ply"), pc)

    # top-down + side projection of the single frame (sanity: metric scale, table shape)
    P = np.asarray(pc.points)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].scatter(P[:, 0], P[:, 2], s=0.2, c=np.asarray(pc.colors))
    ax[0].set_xlabel("x [m]"), ax[0].set_ylabel("z (forward) [m]"), ax[0].set_aspect("equal")
    ax[0].set_title("frame0 top view (optical x-z)")
    ax[1].scatter(P[:, 2], -P[:, 1], s=0.2, c=np.asarray(pc.colors))
    ax[1].set_xlabel("z (forward) [m]"), ax[1].set_ylabel("-y (up) [m]"), ax[1].set_aspect("equal")
    ax[1].set_title("frame0 side view")
    fig.tight_layout()
    fig.savefig(out / "frame0_projections.png", dpi=110)
    plt.close(fig)


def print_summary(r):
    print("=" * 78)
    print(f"bag: {r['bag_dir']}  size={r['bag_size']}  duration={r['bag_duration_s']:.1f}s  storage={r['storage']}")
    print(f"{'topic':32s} {'type':38s} {'count':>6s} {'Hz':>7s}")
    for t, i in sorted(r["topics"].items()):
        hz = f"{i['rate_hz']:.2f}" if i["rate_hz"] else "-"
        print(f"{t:32s} {i['type']:38s} {i['count']:6d} {hz:>7s}")
    for k, v in r["images"].items():
        print(f"{k}: {v['encoding']} {v['width']}x{v['height']} frame_id={v['frame_id']}")
    ci = r["camera_info"]
    print(f"CameraInfo frame={ci['frame_id']} model={ci['distortion_model']} D_all_zero={ci['D_all_zero']}")
    print(f"  K={np.array(ci['K']).round(3).tolist()}")
    print(f"  D={np.round(ci['D'], 5).tolist()}")
    print(f"stamp match: {r['stamp_match']}")
    tfs = r["tf_static"]
    print(f"tf_static: base->optical={tfs['base_to_optical_exists']} base->laser={tfs['base_to_laser_exists']}")
    print(f"  edges(child->parent): {tfs['edges']}")
    print(f"tf dynamic pairs: {r['tf_dynamic_pairs']}")
    print(f"odometry: {r['odometry']}")
    print(f"depth stats: {r['depth_stats']}")
    print("=" * 78)


if __name__ == "__main__":
    main()
