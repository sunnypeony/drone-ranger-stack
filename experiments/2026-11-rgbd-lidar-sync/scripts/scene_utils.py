"""Helpers shared by the v2 evaluation and the Isaac test scene."""
import numpy as np
from scipy import ndimage


def find_table(V, traj_xy, ts):
    """Largest elevated blob (vertices with z in ts['table_band']) near the trajectory centroid.
    Returns centre (x, y), radius (95th pct), top z, or None."""
    if ts.get("table_center") is not None:
        c = np.asarray(ts["table_center"], float)
        sel = np.linalg.norm(V[:, :2] - c, axis=1) < ts["table_search_radius"]
    else:
        c = traj_xy.mean(0)
        sel = np.linalg.norm(V[:, :2] - c, axis=1) < ts["table_search_radius"]
    lo, hi = ts["table_band"]
    P = V[sel & (V[:, 2] >= lo) & (V[:, 2] <= hi)]
    if len(P) < 20:
        return None
    res = 0.05
    mn = P[:, :2].min(0)
    ij = np.floor((P[:, :2] - mn) / res).astype(int)
    grid = np.zeros(ij.max(0) + 1, bool)
    grid[ij[:, 0], ij[:, 1]] = True
    lab, n = ndimage.label(ndimage.binary_closing(grid, iterations=2))
    if n == 0:
        return None
    ids = lab[ij[:, 0], ij[:, 1]]
    best = np.bincount(ids[ids > 0]).argmax()
    Q = P[ids == best]
    centre = Q[:, :2].mean(0)
    r = float(np.percentile(np.linalg.norm(Q[:, :2] - centre, axis=1), 95))
    return {"center": centre.tolist(), "radius": r, "top_z": float(np.percentile(Q[:, 2], 95)),
            "points": int(len(Q))}
