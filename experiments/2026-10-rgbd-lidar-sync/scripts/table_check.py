#!/usr/bin/env python3
"""Is the table actually in the mesh? Ray-cast coverage of the table top for several meshes.

For each <name>=<outputs dir> (holding mesh/mesh_clean.ply + mesh/poses_aligned.npz from 05), the
table is detected as in 09_test_scene.py; then a grid of vertical rays over a disk of
table_check.coverage_radius_frac x table radius reports the fraction whose first hit lies inside
test_scene.table_band (table top / pots). A mesh whose table was "hollowed out" by inconsistent
viewpoints shows low coverage even if the table is still detected from its rim.

Writes <outputs_dir>/table_check/{table_check.json, table_check.png}.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from common import exp_path, load_config, parse_args, stage_dir, write_json  # noqa: E402
from scene_utils import find_table  # noqa: E402


def extra(p):
    p.add_argument("meshes", nargs="+", metavar="NAME=OUTPUTS_DIR")


def check(name, od, T, C):
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(str(od / "mesh" / "mesh_clean.ply"))
    traj = np.load(od / "mesh" / "poses_aligned.npz")["T_world_optical"][:, :2, 3]
    V = np.asarray(mesh.vertices)
    tab = find_table(V, traj, T)
    if tab is None:
        return {"table": None, "top_coverage": 0.0, "triangles": len(mesh.triangles)}, None, None
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    c = np.array(tab["center"])
    r = tab["radius"] * C["coverage_radius_frac"]
    g = np.arange(-r, r + 1e-9, C["grid"])
    gx, gy = np.meshgrid(g, g)
    inside = gx ** 2 + gy ** 2 <= r ** 2
    pts = np.stack([gx[inside] + c[0], gy[inside] + c[1]], 1)
    z0 = V[:, 2].max() + 1.0
    rays = np.c_[pts, np.full(len(pts), z0), np.zeros((len(pts), 2)), -np.ones(len(pts))].astype(np.float32)
    t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
    h = np.where(np.isfinite(t), z0 - t, np.nan)
    lo, hi = T["table_band"]
    cover = float(np.mean((h >= lo) & (h <= hi)))
    # height map of a wider window for the figure
    w = C["map_half_width"]
    m = np.arange(-w, w + 1e-9, C["grid"])
    mx, my = np.meshgrid(m + c[0], m + c[1])
    rays = np.c_[mx.ravel(), my.ravel(), np.full(mx.size, z0), np.zeros((mx.size, 2)),
                 -np.ones(mx.size)].astype(np.float32)
    tm = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
    hm = np.where(np.isfinite(tm), z0 - tm, np.nan).reshape(mx.shape)
    return {"table": tab, "coverage_radius_m": r, "n_rays": int(len(pts)), "top_coverage": cover,
            "hit_height_median_m": float(np.nanmedian(h)), "triangles": len(mesh.triangles)}, hm, (c, w)


def main():
    args = parse_args(__doc__, extra)
    cfg = load_config(args.config, args.set)
    T, C = cfg["test_scene"], cfg["table_check"]
    out = stage_dir(cfg, "table_check")
    res, maps = {}, {}
    for item in args.meshes:
        name, d = item.split("=", 1)
        res[name], hm, cw = check(name, exp_path(d), T, C)
        if hm is None:
            print(f"{name:10s} no table found (no elevated blob in {T['table_band']} m near the trajectory)")
            continue
        maps[name] = (hm, cw)
        print(f"{name:10s} table centre {np.round(res[name]['table']['center'], 2).tolist()} "
              f"r={res[name]['table']['radius']:.2f} m, top coverage {res[name]['top_coverage']:.2f} "
              f"(median hit {res[name]['hit_height_median_m']:.2f} m, {res[name]['n_rays']} rays)")
    write_json(out / "table_check.json", res)
    fig, axes = plt.subplots(1, len(maps), figsize=(3.6 * len(maps), 3.8), facecolor="#fcfcfb", squeeze=False)
    for a, (name, (hm, (c, w))) in zip(axes[0], maps.items()):
        im = a.imshow(hm, origin="lower", extent=[c[0] - w, c[0] + w, c[1] - w, c[1] + w], vmin=0,
                      vmax=T["table_band"][1] + 0.2, cmap="viridis")
        a.set_title(f"{name}: top coverage {res[name]['top_coverage']:.2f}", fontsize=9, loc="left")
        a.set_xticks([])
        a.set_yticks([])
        a.add_patch(plt.Circle(c, res[name]["coverage_radius_m"], fill=False, color="#eb6834", lw=1))
    fig.colorbar(im, ax=axes[0].tolist(), shrink=0.8, label="first-hit height [m]")
    fig.savefig(out / "table_check.png", dpi=120, facecolor="#fcfcfb")
    plt.close(fig)


if __name__ == "__main__":
    main()
