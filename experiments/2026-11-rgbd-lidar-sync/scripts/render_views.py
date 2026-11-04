#!/usr/bin/env python3
"""Ray-cast mesh_clean.ply from real camera poses and put each render under the RGB frame of that pose
-> <outputs>/mesh/render_vs_rgb.jpg, and from outside the scene -> <outputs>/mesh/birdseye.jpg
(both copied to results/). White = no mesh along the ray.

render_vs_rgb only checks that what each frame saw was integrated: a surface seen from one pose renders
back correctly from that pose even when other views of it landed elsewhere (pose drift, depth noise), so
it looks good by construction. birdseye shows the whole scene from viewpoints no camera had, which is
what a USD viewer shows; check global shape (straight fence, flat wall, closed yard) there.

Needs no display (Open3D RaycastingScene, colours interpolated from vertex colours). Missing near
objects here (that the RGB shows) usually mean 05's cluster filter dropped them."""
import csv
import json
import shutil

import cv2
import numpy as np

from common import exp_path, load_config, parse_args, stage_dir


def cast(scene, F, Cv, eye, dirs, shade=None):
    """Colour per ray (white = miss); shade = face normals for Lambert-like shading, None = flat."""
    import open3d as o3d
    r = scene.cast_rays(o3d.core.Tensor(np.c_[np.broadcast_to(eye, dirs.shape), dirs].astype(np.float32)))
    pid, uv = r["primitive_ids"].numpy(), r["primitive_uvs"].numpy()
    hit = pid != scene.INVALID_ID
    img = np.full((len(dirs), 3), 255.0)
    w = np.c_[1 - uv[hit].sum(1), uv[hit]]
    col = (Cv[F[pid[hit]]] * w[:, :, None]).sum(1) * 255
    if shade is not None:
        dn = dirs[hit] / np.linalg.norm(dirs[hit], axis=1, keepdims=True)
        col *= (0.5 + 0.5 * np.abs((shade[pid[hit]] * dn).sum(1)))[:, None]
    img[hit] = col
    return img


def birdseye(scene, F, Cv, Nf, centre, B):
    """Views from outside: (azimuth, elevation) pairs on a sphere of radius B['distance_m'] around centre."""
    W, H, f = B["width"], B["height"], B["focal_px"]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rc = np.stack([(u - W / 2) / f, (v - H / 2) / f, np.ones((H, W))], -1).reshape(-1, 3)
    tiles = []
    for az, el in B["views"]:
        a, e = np.radians(az), np.radians(el)
        eye = centre + B["distance_m"] * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        zc = (centre - eye) / np.linalg.norm(centre - eye)
        xc = np.cross(zc, [0, 0, 1.0] if el < 89 else [1.0, 0, 0])
        xc /= np.linalg.norm(xc)
        img = cast(scene, F, Cv, eye, rc @ np.stack([xc, np.cross(zc, xc), zc]), Nf)
        t = img.reshape(H, W, 3)[:, :, ::-1].astype(np.uint8).copy()
        cv2.putText(t, f"az {az} el {el}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        tiles.append(t)
    return tiles


def grid_image(tiles, cols, scale):
    while len(tiles) % cols:
        tiles.append(np.full_like(tiles[0], 255))
    grid = np.vstack([np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)])
    return cv2.resize(grid, None, fx=scale, fy=scale)


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
        ren = cast(scene, F, Cv, T[:3, 3], rays_cam @ T[:3, :3].T).reshape(H, W, 3)[:, :, ::-1].astype(np.uint8)
        real = cv2.imread(str(fdir / rows[idx[j]]["color"]))
        t = np.vstack([real, ren])
        cv2.putText(t, f"frame {idx[j]}", (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        tiles.append(t)
    outs = [mdir / "render_vs_rgb.jpg"]
    cv2.imwrite(str(outs[0]), grid_image(tiles, C["columns"], C["scale"]), [cv2.IMWRITE_JPEG_QUALITY, 88])
    B = C["birdseye"]
    if B:
        mesh.compute_triangle_normals()
        centre = np.r_[np.median(Ts[:, :2, 3], 0), B["target_height_m"]]  # middle of the camera path
        tiles = birdseye(scene, F, Cv, np.asarray(mesh.triangle_normals), centre, B)
        outs.append(mdir / "birdseye.jpg")
        cv2.imwrite(str(outs[1]), grid_image(tiles, B["columns"], B["scale"]), [cv2.IMWRITE_JPEG_QUALITY, 88])
    res = exp_path(cfg["paths"]["results_dir"])
    res.mkdir(parents=True, exist_ok=True)
    for out in outs:
        shutil.copy2(out, res / out.name)
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
