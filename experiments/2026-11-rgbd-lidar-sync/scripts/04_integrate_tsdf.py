#!/usr/bin/env python3
"""Stage 04: integrate posed RGB-D frames into a ScalableTSDFVolume -> mesh_raw.ply, tsdf_pointcloud.ply.

Optional far band (`tsdf.far`, null = off): stereo depth noise grows ~z^2 (OAK-D: ~5 cm at 5 m, ~20 cm
at 9.5 m), so one voxel / sdf_trunc cannot fit both a table at 2 m and a house wall at 9 m. The far
band integrates depth in [far.depth_min, far.depth_trunc] into a second, coarser volume in the same
pass; far-mesh triangles within far.near_overlap_m of the near mesh are dropped (the near mesh wins
where both exist) and the rest is appended, so 05 still reads a single mesh_raw.ply.

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
    args = parse_args(__doc__, lambda p: p.add_argument("--force", action="store_true", help="ignore done.json"))
    cfg = load_config(args.config, args.set)
    import open3d as o3d
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
    F = C.get("far")
    if F:
        if F["depth_min"] >= depth_trunc:
            print(f"[warn] tsdf.far.depth_min={F['depth_min']} >= near depth_trunc={depth_trunc}: "
                  f"no overlap band, the seam between the two meshes may show")
        far_vol = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=F["voxel_length"], sdf_trunc=F["sdf_trunc"],
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        far_min_raw = F["depth_min"] * C["depth_scale"]
        print(f"near band <= {depth_trunc} m (voxel {C['voxel_length']}, sdf_trunc {C['sdf_trunc']}); "
              f"far band {F['depth_min']}-{F['depth_trunc']} m (voxel {F['voxel_length']}, sdf_trunc {F['sdf_trunc']})")

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
        if F:
            d = np.asarray(depth).copy()
            d[d < far_min_raw] = 0
            rgbd_far = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, o3d.geometry.Image(d), depth_scale=C["depth_scale"], depth_trunc=F["depth_trunc"],
                convert_rgb_to_intensity=False)
            far_vol.integrate(rgbd_far, intrinsic, np.linalg.inv(T))
        if (n + 1) % C["checkpoint_every"] == 0:
            el = time.time() - t0
            write_json(out / "progress.json", {"integrated": n + 1, "total": len(sel), "elapsed_s": el,
                                               "eta_s": el / (n + 1) * (len(sel) - n - 1)})

    print("extracting mesh ...")
    mesh = vol.extract_triangle_mesh()
    pcd = vol.extract_point_cloud()
    far_rep = None
    if F:
        far_mesh = far_vol.extract_triangle_mesh()
        n_far = len(far_mesh.triangles)
        if len(mesh.triangles) and n_far:
            scene = o3d.t.geometry.RaycastingScene()
            scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
            dist = scene.compute_distance(
                o3d.core.Tensor(np.asarray(far_mesh.vertices, np.float32))).numpy()
            drop = (dist[np.asarray(far_mesh.triangles)] < F["near_overlap_m"]).any(axis=1)
            far_mesh.remove_triangles_by_mask(drop)
            far_mesh.remove_unreferenced_vertices()
        far_rep = {"far_triangles_extracted": n_far, "far_triangles_kept": len(far_mesh.triangles),
                   "near_triangles": len(mesh.triangles)}
        print(f"far band: {n_far} triangles, {len(far_mesh.triangles)} kept after removing overlap with near mesh")
        mesh += far_mesh
        pcd += far_vol.extract_point_cloud()
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(out / "mesh_raw.ply"), mesh)
    o3d.io.write_point_cloud(str(out / "tsdf_pointcloud.ply"), pcd)
    bb = mesh.get_axis_aligned_bounding_box()
    rep = {"hash": digest, "frames_integrated": len(sel), "triangles": len(mesh.triangles),
           "vertices": len(mesh.vertices), "pointcloud_points": len(pcd.points),
           "bbox_min": np.asarray(bb.min_bound).tolist(), "bbox_max": np.asarray(bb.max_bound).tolist(),
           "voxel_length": C["voxel_length"], "sdf_trunc": C["sdf_trunc"], "depth_trunc": depth_trunc,
           "far": F or None, **(far_rep or {}), "elapsed_s": time.time() - t0}
    write_json(done, rep)
    print(json.dumps({k: v for k, v in rep.items() if k != "hash"}, indent=2))
    if len(mesh.triangles) == 0:
        die("TSDF produced an empty mesh (check poses / depth units / depth_trunc)")


if __name__ == "__main__":
    main()
