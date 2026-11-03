#!/usr/bin/env python3
"""Stage 04: integrate posed RGB-D frames into a ScalableTSDFVolume -> mesh_raw.ply, tsdf_pointcloud.ply.

Resume: the legacy ScalableTSDFVolume cannot be serialised, so a finished run is recorded in
done.json together with a hash of its inputs; a rerun with identical inputs is skipped. Partial
progress snapshots (progress.json) are informational only - an interrupted run re-integrates.
"""
import hashlib
import json
import time

import numpy as np
from tqdm import tqdm

from common import (assert_rigid, die, load_config, load_frames, load_poses, o3d_intrinsic, parse_args,
                    stage_dir, write_json)


def input_hash(cfg, poses_path):
    h = hashlib.sha1()
    h.update(json.dumps(cfg["tsdf"], sort_keys=True).encode())
    h.update(json.dumps(cfg["extract"]["depth_max"]).encode())
    h.update(open(poses_path, "rb").read())
    return h.hexdigest()


def main():
    import open3d as o3d
    args = parse_args(__doc__, lambda p: p.add_argument("--force", action="store_true", help="ignore done.json"))
    cfg = load_config(args.config, args.set)
    C = cfg["tsdf"]
    out = stage_dir(cfg, "tsdf")
    fdir, rows, intr = load_frames(cfg)
    z, Ts = load_poses(cfg)
    idx = z["frame_idx"]
    digest = input_hash(cfg, stage_dir(cfg, "poses") / "poses.npz")
    done = out / "done.json"
    if not args.force and done.exists() and json.load(open(done)).get("hash") == digest \
            and (out / "mesh_raw.ply").exists():
        print(f"[skip] TSDF already integrated with identical inputs ({done})")
        print(json.load(open(done)))
        return

    if not np.isclose(C["sdf_trunc"], 4 * C["voxel_length"]):
        print(f"[warn] sdf_trunc={C['sdf_trunc']} is not 4 x voxel_length={C['voxel_length']}")
    depth_trunc = C["depth_trunc"] if C["depth_trunc"] is not None else cfg["extract"]["depth_max"]
    intrinsic = o3d_intrinsic(intr)
    vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=C["voxel_length"], sdf_trunc=C["sdf_trunc"],
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    mask = z["tsdf_mask"] if "tsdf_mask" in z.files else np.ones(len(idx), bool)  # v2: blurry frames excluded
    sel = [k for k in range(0, len(idx), C["frame_stride"]) if mask[k]]
    print(f"integrating {len(sel)} of {len(idx)} posed frames ({int((~mask).sum())} excluded by tsdf_mask)")
    t0 = time.time()
    for n, k in enumerate(tqdm(sel, desc="integrate")):
        T = Ts[k]
        assert_rigid(T, cfg, f"pose {k}")
        r = rows[idx[k]]
        color = o3d.io.read_image(str(fdir / r["color"]))
        depth = o3d.io.read_image(str(fdir / r["depth"]))
        if np.asarray(color).shape[:2] != np.asarray(depth).shape[:2] or \
                np.asarray(depth).shape != (intr["height"], intr["width"]):
            die(f"frame {idx[k]}: color/depth/intrinsics size mismatch")
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=C["depth_scale"], depth_trunc=depth_trunc, convert_rgb_to_intensity=False)
        vol.integrate(rgbd, intrinsic, np.linalg.inv(T))  # extrinsic = world->camera
        if (n + 1) % C["checkpoint_every"] == 0:
            el = time.time() - t0
            write_json(out / "progress.json", {"integrated": n + 1, "total": len(sel), "elapsed_s": el,
                                               "eta_s": el / (n + 1) * (len(sel) - n - 1)})

    print("extracting mesh ...")
    mesh = vol.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    pcd = vol.extract_point_cloud()
    o3d.io.write_triangle_mesh(str(out / "mesh_raw.ply"), mesh)
    o3d.io.write_point_cloud(str(out / "tsdf_pointcloud.ply"), pcd)
    bb = mesh.get_axis_aligned_bounding_box()
    rep = {"hash": digest, "frames_integrated": len(sel), "triangles": len(mesh.triangles),
           "vertices": len(mesh.vertices), "pointcloud_points": len(pcd.points),
           "bbox_min": np.asarray(bb.min_bound).tolist(), "bbox_max": np.asarray(bb.max_bound).tolist(),
           "voxel_length": C["voxel_length"], "sdf_trunc": C["sdf_trunc"], "depth_trunc": depth_trunc,
           "elapsed_s": time.time() - t0}
    write_json(done, rep)
    print(json.dumps({k: v for k, v in rep.items() if k != "hash"}, indent=2))
    if len(mesh.triangles) == 0:
        die("TSDF produced an empty mesh (check poses / depth units / depth_trunc)")


if __name__ == "__main__":
    main()
