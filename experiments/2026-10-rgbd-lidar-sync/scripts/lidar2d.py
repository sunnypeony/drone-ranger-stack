"""2D LiDAR tools for v2: scan I/O, operator-body sector detection, point-to-line ICP, correlative
scan matching and an SE(2) pose graph (scipy least_squares, robust loss). numpy/scipy only.

SE(2) poses are arrays (x, y, theta). `compose(a, b)` = a ∘ b, `between(a, b)` = a⁻¹ ∘ b
(pose of b expressed in a). A graph edge (i, j, z) means between(p_i, p_j) ≈ z.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage, sparse
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from common import iter_topic, stamp_ns


# ----------------------------------------------------------------------------- SE(2)
def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def rot2(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s], [s, c]])


def compose(a, b):
    return np.array([*(a[:2] + rot2(a[2]) @ b[:2]), wrap(a[2] + b[2])])


def inverse(a):
    return np.array([*(-rot2(a[2]).T @ a[:2]), wrap(-a[2])])


def between(a, b):
    return compose(inverse(a), b)


def transform_pts(p, pts):
    return pts @ rot2(p[2]).T + p[:2]


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def split_6dof(T):
    """T_world_base -> (se2 (x, y, yaw), R_rp (roll/pitch part, R = Rz(yaw) @ R_rp), z)."""
    yaw = yaw_of(T[:3, :3])
    return np.array([T[0, 3], T[1, 3], yaw]), Rz(yaw).T @ T[:3, :3], float(T[2, 3])


def merge_6dof(se2, R_rp, z):
    T = np.eye(4)
    T[:3, :3] = Rz(se2[2]) @ R_rp
    T[:3, 3] = [se2[0], se2[1], z]
    return T


def se2_from_T4(T):
    return np.array([T[0, 3], T[1, 3], yaw_of(T[:3, :3])])


# ----------------------------------------------------------------------------- scans
def load_scans(reader, ts, topic):
    out = []
    for _, m in iter_topic(reader, ts, topic):
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        out.append({"stamp": stamp_ns(m.header), "r": r, "a": wrap(a), "range_min": m.range_min,
                    "range_max": m.range_max})
    return out


def detect_body_sector(scans, L):
    """Angle bins that see a near return (< body_max_range) in most scans = the operator / mount."""
    bins = np.arange(-180, 180 + 1e-9, L["body_bin_deg"])
    nb = len(bins) - 1
    hits = np.zeros(nb)
    for s in scans:
        near = np.isfinite(s["r"]) & (s["r"] > L["body_min_range"]) & (s["r"] < L["body_max_range"])
        b = np.clip(np.digitize(np.degrees(s["a"][near]), bins) - 1, 0, nb - 1)
        hits[np.unique(b)] += 1
    frac = hits / max(len(scans), 1)
    body = frac >= L["body_min_persistence"]
    m = L["body_margin_bins"]
    grown = body.copy()
    for k in range(1, m + 1):  # circular dilation
        grown |= np.roll(body, k) | np.roll(body, -k)
    sectors = []
    if grown.any() and not grown.all():
        start = int(np.argmax(~grown))  # begin scanning at a free bin so sectors are contiguous
        cur = None
        for k in range(nb):
            i = (start + k) % nb
            if grown[i] and cur is None:
                cur = [i, i]
            elif grown[i]:
                cur[1] = i
            elif cur is not None:
                sectors.append(cur)
                cur = None
        if cur is not None:
            sectors.append(cur)
    sector_deg = [[float(bins[a]), float(bins[b + 1])] for a, b in sectors]
    return grown, bins, frac, sector_deg


def scan_points(scan, L, body_mask, bins, yaw_offset_deg=0.0):
    """Valid returns -> Nx3 points in the laser frame (z=0) with range + body-sector filtering."""
    r, a = scan["r"], scan["a"]
    ok = np.isfinite(r) & (r >= L["range_min"]) & (r <= L["range_max"])
    if body_mask is not None and body_mask.any():
        b = np.clip(np.digitize(np.degrees(a), bins) - 1, 0, len(body_mask) - 1)
        ok &= ~body_mask[b]
    aa = a[ok] + np.radians(yaw_offset_deg)
    return np.stack([r[ok] * np.cos(aa), r[ok] * np.sin(aa), np.zeros(ok.sum())], 1), r[ok]


def voxel_down2d(pts, res):
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / res).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


# ----------------------------------------------------------------------------- ICP
def normals2d(pts, k):
    tree = cKDTree(pts)
    k = min(k, len(pts))
    _, nn = tree.query(pts, k=k)
    nb = pts[nn]
    c = nb - nb.mean(1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", c, c)
    w, v = np.linalg.eigh(cov)
    n = v[:, :, 0]  # smallest eigenvalue = line normal
    planarity = 1 - w[:, 0] / np.maximum(w[:, 1], 1e-12)
    return tree, n, planarity


def icp2d(src, tgt, init, P, tgt_cache=None):
    """Point-to-line ICP. Returns pose of the source frame in the target frame, fitness, rmse."""
    if len(src) < P["icp_min_points"] or len(tgt) < P["icp_min_points"]:
        return None
    tree, nrm, plan = tgt_cache if tgt_cache is not None else normals2d(tgt, P["icp_normal_k"])
    T = np.array(init, float)
    for dmax in P["icp_max_dist"]:
        for _ in range(P["icp_iters"]):
            q = transform_pts(T, src)
            d, j = tree.query(q, distance_upper_bound=dmax)
            m = np.isfinite(d) & (plan[np.minimum(j, len(tgt) - 1)] >= P["icp_min_planarity"])
            if m.sum() < P["icp_min_points"]:
                return None
            qm, n, t = q[m], nrm[j[m]], tgt[j[m]]
            e = np.einsum("ij,ij->i", n, qm - t)
            J = np.stack([n[:, 0], n[:, 1], -n[:, 0] * qm[:, 1] + n[:, 1] * qm[:, 0]], 1)
            ae = np.abs(e)
            w = np.where(ae <= P["icp_huber"], 1.0, P["icp_huber"] / np.maximum(ae, 1e-12))
            H = (J * w[:, None]).T @ J + np.eye(3) * 1e-9
            dx = -np.linalg.solve(H, (J * w[:, None]).T @ e)
            T = compose(np.array([dx[0], dx[1], dx[2]]), T)
            if np.linalg.norm(dx[:2]) < 1e-5 and abs(dx[2]) < 1e-6:
                break
    q = transform_pts(T, src)
    d, j = tree.query(q, distance_upper_bound=P["icp_max_dist"][-1])
    m = np.isfinite(d)
    if m.sum() < P["icp_min_points"]:
        return None
    e = np.einsum("ij,ij->i", nrm[j[m]], q[m] - tgt[j[m]])
    return {"T": T, "fitness": float(m.mean()), "rmse": float(np.sqrt(np.mean(e ** 2))), "n": int(m.sum())}


# ----------------------------------------------------------------------------- correlative matching
class LikelihoodField:
    def __init__(self, tgt, P):
        res = P["csm_resolution"]
        lo = tgt.min(0) - P["csm_margin"]
        hi = tgt.max(0) + P["csm_margin"]
        shape = np.ceil((hi - lo) / res).astype(int) + 1
        occ = np.ones(shape, bool)
        ij = np.floor((tgt - lo) / res).astype(int)
        occ[ij[:, 0], ij[:, 1]] = False
        dist = ndimage.distance_transform_edt(occ) * res
        self.field = np.exp(-0.5 * (dist / P["csm_sigma"]) ** 2)
        self.lo, self.res, self.shape = lo, res, shape

    def score(self, pts):
        """pts: (..., N, 2) -> mean likelihood over points (out of bounds = 0)."""
        ij = np.floor((pts - self.lo) / self.res).astype(int)
        inside = (ij[..., 0] >= 0) & (ij[..., 1] >= 0) & (ij[..., 0] < self.shape[0]) & (ij[..., 1] < self.shape[1])
        i0 = np.clip(ij[..., 0], 0, self.shape[0] - 1)
        i1 = np.clip(ij[..., 1], 0, self.shape[1] - 1)
        return np.where(inside, self.field[i0, i1], 0.0).mean(-1)


def correlative_match(src, tgt, init, P, rng):
    """Exhaustive search over yaw x translation around `init` (pose of src in tgt frame).
    Returns best pose, its score and the ratio best / best-outside-the-peak (ambiguity)."""
    if len(src) < P["icp_min_points"] or len(tgt) < P["icp_min_points"]:
        return None
    lf = LikelihoodField(tgt, P)
    if len(src) > P["csm_max_points"]:
        src = src[rng.choice(len(src), P["csm_max_points"], replace=False)]
    yaws = init[2] + np.radians(np.arange(-P["csm_yaw_range_deg"], P["csm_yaw_range_deg"] + 1e-9, P["csm_yaw_step_deg"]))
    steps = np.arange(-P["csm_xy_range"], P["csm_xy_range"] + 1e-9, P["csm_xy_step"])
    dx, dy = np.meshgrid(steps, steps, indexing="ij")
    off = np.stack([dx.ravel(), dy.ravel()], 1) + init[:2]
    scores = np.zeros((len(yaws), len(off)))
    for k, th in enumerate(yaws):
        rp = src @ rot2(th).T
        scores[k] = lf.score(rp[None, :, :] + off[:, None, :])
    k, t = np.unravel_index(np.argmax(scores), scores.shape)
    best = np.array([off[t, 0], off[t, 1], wrap(yaws[k])])
    # ambiguity: best score among hypotheses far from the peak
    far_t = np.linalg.norm(off - off[t], axis=1) > P["csm_peak_exclusion_xy"]
    far_k = np.abs(wrap(yaws - yaws[k])) > np.radians(P["csm_peak_exclusion_deg"])
    far = far_k[:, None] | far_t[None, :]
    second = scores[far].max() if far.any() else 0.0
    return {"T": best, "score": float(scores[k, t]), "ratio": float(scores[k, t] / max(second, 1e-9))}


# ----------------------------------------------------------------------------- SE(2) graph
def optimize_graph(init, edges, P, fixed=0):
    """init: Nx3. edges: list of (i, j, z(3), sigma_xy, sigma_yaw). Robust (loss=P['pg_loss'])."""
    n = len(init)
    E = len(edges)
    I = np.array([e[0] for e in edges], int)
    J = np.array([e[1] for e in edges], int)
    Z = np.array([e[2] for e in edges], float).reshape(E, 3)
    W = np.array([[1 / e[3], 1 / e[3], 1 / e[4]] for e in edges], float).reshape(E, 3)
    anchor = np.asarray(init[fixed], float).copy()
    wa = P["pg_anchor_weight"]

    def residual(x):
        X = x.reshape(n, 3)
        ti, tj = X[I, :2], X[J, :2]
        c, s = np.cos(X[I, 2]), np.sin(X[I, 2])
        d = tj - ti
        px = c * d[:, 0] + s * d[:, 1]
        py = -s * d[:, 0] + c * d[:, 1]
        r = np.stack([px - Z[:, 0], py - Z[:, 1], wrap(X[J, 2] - X[I, 2] - Z[:, 2])], 1) * W
        ra = (X[fixed] - anchor) * wa
        ra[2] = wrap(X[fixed, 2] - anchor[2]) * wa
        return np.concatenate([r.ravel(), ra])

    def jac(x):
        X = x.reshape(n, 3)
        c, s = np.cos(X[I, 2]), np.sin(X[I, 2])
        d = X[J, :2] - X[I, :2]
        dpx_dth = -s * d[:, 0] + c * d[:, 1]
        dpy_dth = -c * d[:, 0] - s * d[:, 1]
        rows, cols, vals = [], [], []
        base = np.arange(E) * 3

        def add(r, cidx, v):
            rows.append(r)
            cols.append(cidx)
            vals.append(v)
        # px row
        add(base, 3 * I, -c * W[:, 0]); add(base, 3 * I + 1, -s * W[:, 0]); add(base, 3 * I + 2, dpx_dth * W[:, 0])
        add(base, 3 * J, c * W[:, 0]); add(base, 3 * J + 1, s * W[:, 0])
        # py row
        add(base + 1, 3 * I, s * W[:, 1]); add(base + 1, 3 * I + 1, -c * W[:, 1]); add(base + 1, 3 * I + 2, dpy_dth * W[:, 1])
        add(base + 1, 3 * J, -s * W[:, 1]); add(base + 1, 3 * J + 1, c * W[:, 1])
        # theta row
        add(base + 2, 3 * I + 2, -W[:, 2]); add(base + 2, 3 * J + 2, W[:, 2])
        ra = 3 * E + np.arange(3)
        add(ra, 3 * fixed + np.arange(3), np.full(3, wa))
        A = sparse.coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                              shape=(3 * E + 3, 3 * n))
        return A.toarray() if P["pg_tr_solver"] == "exact" else A.tocsr()

    res = least_squares(residual, np.asarray(init, float).ravel(), jac=jac, method="trf", loss=P["pg_loss"],
                        f_scale=P["pg_f_scale"], x_scale="jac", max_nfev=P["pg_max_nfev"], tr_solver=P["pg_tr_solver"],
                        verbose=0)
    X = res.x.reshape(n, 3)
    X[:, 2] = wrap(X[:, 2])
    r = residual(res.x)[: 3 * E].reshape(E, 3)
    return X, {"cost": float(res.cost), "nfev": int(res.nfev), "status": int(res.status),
               "edge_whitened_norm": np.linalg.norm(r, axis=1)}


def solve_keyframe_graph(init, step_sigma, edges, labels, P):
    """Pose graph on every `pg_node_stride`-th chain pose.

    init: Nx3 chain poses (visual odometry); step_sigma: (N-1)x2 per-step (sigma_xy, sigma_yaw);
    edges: chain-level (i, j, z, sigma_xy, sigma_yaw) non-odometry measurements.
    Odometry between graph nodes is composed exactly from the chain (sigma grows with sqrt(steps));
    other measurements are re-anchored to the nearest node with the chain offset. The per-node
    correction X_n ∘ init_n⁻¹ is interpolated back to every chain pose."""
    n = len(init)
    nodes = np.unique(np.r_[np.arange(0, n, P["pg_node_stride"]), n - 1])
    near = np.clip(np.searchsorted(nodes, np.arange(n)), 0, len(nodes) - 1)
    lo = np.clip(near - 1, 0, len(nodes) - 1)
    near = np.where(np.abs(nodes[lo] - np.arange(n)) < np.abs(nodes[near] - np.arange(n)), lo, near)
    delta = [between(init[nodes[near[k]]], init[k]) for k in range(n)]
    red, red_lab = [], []
    for a, b in zip(range(len(nodes) - 1), range(1, len(nodes))):
        s = np.sqrt((step_sigma[nodes[a]:nodes[b]] ** 2).sum(0))
        red.append((a, b, between(init[nodes[a]], init[nodes[b]]), s[0], s[1]))
        red_lab.append("vis_odo")
    for (i, j, z, sxy, syaw), lab in zip(edges, labels):
        ni, nj = near[i], near[j]
        if ni == nj:
            continue
        red.append((ni, nj, compose(compose(delta[i], z), inverse(delta[j])), sxy, syaw))
        red_lab.append(lab)
    X, info = optimize_graph(init[nodes], red, P)
    corr = np.array([compose(X[k], inverse(init[nodes[k]])) for k in range(len(nodes))])
    th = np.unwrap(corr[:, 2])
    idx = np.arange(n)
    C = np.stack([np.interp(idx, nodes, corr[:, 0]), np.interp(idx, nodes, corr[:, 1]),
                  np.interp(idx, nodes, th)], 1)
    full = np.array([compose(C[k], init[k]) for k in range(n)])
    info["labels"] = red_lab
    info["n_nodes"] = int(len(nodes))
    return full, info
