#!/usr/bin/env python3
"""Ray-cast mesh_clean.ply from real camera poses and put each render under the RGB frame of that pose
-> <outputs>/mesh/render_vs_rgb.jpg (+ copied to results/). White = no mesh along the ray.

Needs no display (Open3D RaycastingScene, colours interpolated from vertex colours). Missing near
objects here (that the RGB shows) usually mean 05's cluster filter dropped them."""
import csv
import json
import shutil

import cv2
import numpy as np

from common import exp_path, load_config, parse_args, stage_dir


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    import open3d as o3d
    C = cfg["render_views"]
    fdir, mdir = stage_dir(cfg, "frames"), stage_dir(cfg, "mesh")
    z = np.load(mdir / "poses_aligned.npz")
    Ts, idx = z["T_world_optical"], z["frame_idx"]
    rows = list(csv.DictReader(open(fdir / "frames.csv")))
    intr = json.load(open(fdir / "intrinsics.json"))
    K, W, H = np.array(intr["K"]), intr["width"], intr["height"]

    mesh = o3d.io.read_triangle_mesh(str(mdir / "mesh_clean.ply"))
    F, Cv = np.asarray(mesh.triangles), np.asarray(mesh.vertex_colors)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays_cam = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones((H, W))], -1).reshape(-1, 3)

    picks = C["frames"] or np.linspace(0, len(idx) - 1, C["n_auto"]).astype(int).tolist()
    tiles = []
    for p in picks:
        j = min(int(np.searchsorted(idx, p)), len(idx) - 1)  # nearest posed frame at or after p
        T = Ts[j]
        dw = rays_cam @ T[:3, :3].T
        r = scene.cast_rays(o3d.core.Tensor(np.c_[np.broadcast_to(T[:3, 3], dw.shape), dw].astype(np.float32)))
        pid, uv = r["primitive_ids"].numpy(), r["primitive_uvs"].numpy()
        hit = pid != scene.INVALID_ID
        img = np.full((H * W, 3), 255.0)
        w = np.c_[1 - uv[hit].sum(1), uv[hit]]
        img[hit] = (Cv[F[pid[hit]]] * w[:, :, None]).sum(1) * 255
        ren = img.reshape(H, W, 3)[:, :, ::-1].astype(np.uint8)
        real = cv2.imread(str(fdir / rows[idx[j]]["color"]))
        t = np.vstack([real, ren])
        cv2.putText(t, f"frame {idx[j]}", (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        tiles.append(t)
    cols = C["columns"]
    while len(tiles) % cols:
        tiles.append(np.full_like(tiles[0], 255))
    grid = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
    grid = cv2.resize(grid, None, fx=C["scale"], fy=C["scale"])
    out = mdir / "render_vs_rgb.jpg"
    cv2.imwrite(str(out), grid, [cv2.IMWRITE_JPEG_QUALITY, 88])
    res = exp_path(cfg["paths"]["results_dir"])
    res.mkdir(parents=True, exist_ok=True)
    shutil.copy2(out, res / out.name)
    print(f"wrote {out} ({len(picks)} views)")


if __name__ == "__main__":
    main()
