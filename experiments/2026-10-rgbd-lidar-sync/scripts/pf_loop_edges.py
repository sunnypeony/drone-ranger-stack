#!/usr/bin/env python3
"""Pose-free loop edges: late frames registered against the static head segment.

For every candidate tail frame t (pf_loops.window_s, clipped to cutoff_s) and each of the
head_tail_check.n_samples static head frames h, FPFH + RANSAC + ICP (ground removed, identity
init; see check_head_tail.py) gives T_h_t. Chained through the head-internal link,
T_first_h . T_h_t is an independent estimate of the same edge first -> t. An edge is accepted
only if all estimates pass fitness / overlap gates and agree with their medoid within
max_spread_m / max_spread_deg. Accepted edges are split alternately into a constraint group
(used by 03b) and an evaluation group (never used as a constraint); fixed_eval_frames always
go to evaluation.

Writes <outputs_dir>/pf_loops/{loop_edges.json, loop_edges.png}.
"""
import importlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from tqdm import tqdm  # noqa: E402

from common import (cutoff_count, load_config, load_frames, parse_args, rot_angle_deg, stage_dir,  # noqa: E402
                    write_json)

ht = importlib.import_module("check_head_tail")


def medoid(Ts):
    t = np.array([T[:3, 3] for T in Ts])
    k = int(np.argmin([np.linalg.norm(t - x, axis=1).sum() for x in t]))
    return k, Ts[k]


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    C, H = cfg["pf_loops"], cfg["head_tail_check"]
    out = stage_dir(cfg, "pf_loops")
    fdir, rows, intr = load_frames(cfg)
    n_cut = cutoff_count(cfg, rows)
    t_rel = (np.array([int(r["stamp_ns"]) for r in rows], np.int64) - int(rows[0]["stamp_ns"])) / 1e9
    h0, h1 = H["head_frames"]
    head = np.linspace(h0, h1 - 1, H["n_samples"]).round().astype(int).tolist()
    first = head[0]
    w0, w1 = C["window_s"]
    cand = set(int(i) for i in np.nonzero((t_rel >= w0) & (t_rel <= w1))[0][:: C["stride"]])
    cand |= {int(i) for i in C["include_frames"]}
    cand = sorted(i for i in cand if i < n_cut)
    print(f"{len(cand)} candidate tail frames in {w0}-{min(w1, t_rel[n_cut - 1]):.1f} s "
          f"(cutoff {cfg.get('cutoff_s')} s -> {n_cut} frames), heads {head}")

    R = ht.Registrar(fdir, rows, intr, cfg)
    head_link = {h: R.register(h, first) for h in head}
    edges = []
    for t in tqdm(cand, desc="pose-free loop edges"):
        est, fits, rots = [], [], []
        for h in head:
            r = R.register(t, h)
            est.append(head_link[h]["T"] @ r["T"])     # T_first_t via head h
            fits.append(r["fitness"])
            rots.append(rot_angle_deg(r["T"][:3, :3]))
        k, Tm = medoid(est)
        dev_t = [float(np.linalg.norm(T[:3, 3] - Tm[:3, 3])) for T in est]
        dev_r = [rot_angle_deg(Tm[:3, :3].T @ T[:3, :3]) for T in est]
        e = {"first": first, "frame": t, "t_s": float(t_rel[t]), "T_first_frame": Tm,
             "distance_m": float(np.linalg.norm(Tm[:3, 3])), "rotation_deg": rot_angle_deg(Tm[:3, :3]),
             "spread_m": max(dev_t), "spread_deg": max(dev_r), "fitness_min": min(fits),
             "fitness_median": float(np.median(fits)), "max_rel_rot_deg": max(rots)}
        why = []
        if e["fitness_min"] < H["fitness_min"]:
            why.append("fitness")
        if e["max_rel_rot_deg"] > C["max_rot_deg"]:
            why.append("no_overlap")
        if e["spread_m"] > C["max_spread_m"]:
            why.append("spread_m")
        if e["spread_deg"] > C["max_spread_deg"]:
            why.append("spread_deg")
        e["accepted"], e["rejected_by"] = not why, why
        edges.append(e)

    acc = [e for e in edges if e["accepted"]]
    fixed = set(C["fixed_eval_frames"])
    group = C["first_group"]
    for e in acc:
        if e["frame"] in fixed:
            e["group"] = "eval"
            continue
        e["group"] = group
        group = "eval" if group == "constraint" else "constraint"
    missing_fixed = sorted(fixed - {e["frame"] for e in acc})
    rep = {"first_frame": first, "head_frames": head, "cutoff_s": cfg.get("cutoff_s"), "n_frames_cut": n_cut,
           "params": C, "n_candidates": len(edges), "n_accepted": len(acc),
           "n_constraint": sum(e["group"] == "constraint" for e in acc),
           "n_eval": sum(e["group"] == "eval" for e in acc),
           "fixed_eval_frames_not_accepted": missing_fixed,
           "rejections": {k: sum(k in e["rejected_by"] for e in edges)
                          for k in ("fitness", "no_overlap", "spread_m", "spread_deg")},
           "edges": edges}
    write_json(out / "loop_edges.json", rep)
    plot(out / "loop_edges.png", edges, C)
    print(f"accepted {len(acc)}/{len(edges)}: constraint {rep['n_constraint']}, eval {rep['n_eval']}; "
          f"rejections {rep['rejections']}")
    for e in acc:
        print(f"  [{e['group']:10s}] frame {e['frame']} t={e['t_s']:.1f}s d={e['distance_m']:.3f} m "
              f"rot={e['rotation_deg']:.1f} spread {e['spread_m'] * 100:.1f} cm / {e['spread_deg']:.2f} deg "
              f"fit {e['fitness_min']:.2f}")
    if missing_fixed:
        print(f"[warn] fixed eval frames not accepted: {missing_fixed}")


def plot(path, edges, C):
    SURFACE, TEXT2, GRID = ht.SURFACE, ht.TEXT2, ht.GRID
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, facecolor=SURFACE)
    t = np.array([e["t_s"] for e in edges])
    for a in (a1, a2):
        a.set_facecolor(SURFACE)
        a.grid(True, color=GRID, lw=0.6)
        a.set_axisbelow(True)
        for s in a.spines.values():
            s.set_color(GRID)
        a.tick_params(colors=TEXT2, labelsize=8)
    styles = {"constraint": (ht.BLUE, "o", "accepted, constraint"), "eval": (ht.ORANGE, "D", "accepted, held out"),
              None: (TEXT2, "x", "rejected")}
    for g, (col, mk, lab) in styles.items():
        sel = [i for i, e in enumerate(edges) if (e.get("group") if e["accepted"] else None) == g]
        if not sel:
            continue
        a1.scatter(t[sel], [min(edges[i]["distance_m"], 3.0) for i in sel], color=col, marker=mk, s=30, label=lab,
                   zorder=3 if g else 2, alpha=1 if g else 0.5)
        a2.scatter(t[sel], [edges[i]["spread_m"] * 100 for i in sel], color=col, marker=mk, s=30, zorder=3 if g else 2,
                   alpha=1 if g else 0.5)
    a2.axhline(C["max_spread_m"] * 100, color=TEXT2, lw=1, ls="--")
    a1.set_ylabel("distance to frame 0 [m] (clipped 3)", color=TEXT2)
    a2.set_ylabel("spread of 5 estimates [cm]", color=TEXT2)
    a2.set_yscale("log")
    a2.set_xlabel("time since start [s]", color=TEXT2)
    a1.set_title("pose-free loop edges (5 static head frames per tail frame)", fontsize=10, loc="left", color=ht.TEXT)
    a1.legend(fontsize=8, frameon=False, labelcolor=TEXT2)
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)


if __name__ == "__main__":
    main()
