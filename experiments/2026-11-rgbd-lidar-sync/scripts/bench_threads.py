#!/usr/bin/env python3
"""Time each OpenMP-heavy pipeline step at several thread counts (and process x thread combos).

Every measurement runs in a fresh process with OMP_NUM_THREADS=N set before Open3D starts ("env"),
or unset ("default" = Open3D's own choice). Some Open3D code ignores later runtime changes, so a
mechanism check per step also runs: runtime limit 1 with the env unset ("rt1"), and env 1 + runtime
limit 8 ("env1+rt8"); the runtime switch works for a step only if rt1 ~ env 1 and env1+rt8 ~ env 8.
Process x thread combos (pools) set OMP_NUM_THREADS in each child's environment.
Workloads (config bench.*): odometry = consecutive pairs (RGB-D odometry + geometric check, as in
03); loop_registration = cached v1 loop pairs through 03's try_loop, plus pose-free FPFH + RANSAC +
ICP pairs (check_head_tail); tsdf = ScalableTSDF integration of consecutive frames (+ mesh
extraction); mesh = 05's clean-up / clustering / ground / decimation on outputs/tsdf/mesh_raw.ply.
Image files are read once beforehand so the timings are compute, not disk.
Each measurement is repeated bench.repeats times, round-robin over all settings (so slow drift on a
shared host does not land on one setting); the table reports the median and the range. Before each
run the cores busy with other work are sampled from /proc/stat (the load average is no use here: it
mostly reflects the previous run's spinning OpenMP threads).
Refuses to start when other work already keeps more than bench.max_busy_cores cores busy.

Writes <outputs_dir>/bench/threads.json and prints a markdown table.
"""
import importlib
import json
import multiprocessing as mp
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from common import (die, exp_path, load_config, load_frames, omp_step, parse_args, stage_dir,
                    write_json)

STEPS = ("odometry", "loop_registration", "tsdf", "mesh")


def extra(p):
    p.add_argument("--worker", choices=STEPS + ("odometry_pool", "loop_pool"), help=argparse_hide())
    p.add_argument("--threads", type=int, default=None)
    p.add_argument("--procs", type=int, default=1)
    p.add_argument("--steps", nargs="*", default=list(STEPS))
    p.add_argument("--allow-load", action="store_true", help="run even if the machine is busy")


def argparse_hide():
    import argparse
    return argparse.SUPPRESS


def warm_files(fdir, rows, idx):
    for i in idx:
        for k in ("color", "depth"):
            with open(fdir / rows[i][k], "rb") as f:
                f.read()


# ----------------------------------------------------------------------------- workloads
def run_odometry(cfg, B, frames=None):
    import open3d as o3d
    m3 = importlib.import_module("03_estimate_poses")
    fdir, rows, intr = load_frames(cfg)
    P = cfg["poses"]
    pairs = frames if frames is not None else list(range(B["odometry_start"], B["odometry_start"] + B["odometry_pairs"]))
    warm_files(fdir, rows, pairs + [pairs[-1] + 1])
    fc = m3.FrameCache(fdir, rows, intr, cfg)
    t0 = time.perf_counter()
    with omp_step(cfg, "odometry"):
        for i in pairs:
            ok, T, info = m3.rgbd_odometry(fc, i, i + 1, np.eye(4), P, o3d)
            m3.verify_step(fc, i, i + 1, T, P, o3d)
    return time.perf_counter() - t0, len(pairs)


def loop_pairs(cfg, B):
    cache = json.load(open(exp_path(B["loop_cache"])))
    keys = [k for k in sorted(cache) if cache[k]][:: B["loop_pick_stride"]][: B["loop_pairs"]]
    out = []
    for k in keys:
        b, a = (int(x) for x in k.split("_"))
        out.append((a, b, np.array(cache[k]["T"])))
    return out


def run_loops(cfg, B, pairs=None):
    import open3d as o3d
    m3 = importlib.import_module("03_estimate_poses")
    fdir, rows, intr = load_frames(cfg)
    pairs = pairs if pairs is not None else loop_pairs(cfg, B)
    warm_files(fdir, rows, sorted({x for a, b, _ in pairs for x in (a, b)}))
    fc = m3.FrameCache(fdir, rows, intr, cfg)
    t0 = time.perf_counter()
    for a, b, T in pairs:
        m3.try_loop(fc, a, b, T, cfg["poses"], o3d)  # applies runtime.omp_threads.loop_registration
    return time.perf_counter() - t0, len(pairs)


def run_fpfh(cfg, B):
    ht = importlib.import_module("check_head_tail")
    fdir, rows, intr = load_frames(cfg)
    pairs = [(t, B["fpfh_head"]) for t in B["fpfh_tails"]]
    warm_files(fdir, rows, sorted({x for p in pairs for x in p}))
    R = ht.Registrar(fdir, rows, intr, cfg)
    t0 = time.perf_counter()
    for s, t in pairs:
        R.register(s, t)  # applies runtime.omp_threads.loop_registration (features computed inside)
    return time.perf_counter() - t0, len(pairs)


def run_tsdf(cfg, B):
    import open3d as o3d
    from common import load_poses, o3d_intrinsic
    fdir, rows, intr = load_frames(cfg)
    C = cfg["tsdf"]
    z, Ts = load_poses(cfg)
    sel = list(range(B["tsdf_start"], B["tsdf_start"] + B["tsdf_frames"]))
    warm_files(fdir, rows, [int(z["frame_idx"][k]) for k in sel])
    K = o3d_intrinsic(intr)
    depth_trunc = C["depth_trunc"] if C["depth_trunc"] is not None else cfg["extract"]["depth_max"]
    with omp_step(cfg, "tsdf"):
        vol = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=C["voxel_length"], sdf_trunc=C["sdf_trunc"],
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        t0 = time.perf_counter()
        for k in sel:
            r = rows[int(z["frame_idx"][k])]
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.io.read_image(str(fdir / r["color"])), o3d.io.read_image(str(fdir / r["depth"])),
                depth_scale=C["depth_scale"], depth_trunc=depth_trunc, convert_rgb_to_intensity=False)
            vol.integrate(rgbd, K, np.linalg.inv(Ts[k]))
        t_int = time.perf_counter() - t0
        t0 = time.perf_counter()
        vol.extract_triangle_mesh()
        t_ext = time.perf_counter() - t0
    return t_int, len(sel), t_ext


def run_mesh(cfg, B):
    import open3d as o3d
    m5 = importlib.import_module("05_postprocess_mesh")
    M = cfg["mesh"]
    mesh = o3d.io.read_triangle_mesh(str(exp_path(B["mesh_raw"])))
    n_raw = len(mesh.triangles)
    with omp_step(cfg, "mesh"):
        t0 = time.perf_counter()
        m5.clean(mesh)
        m5.keep_clusters(mesh, M["min_cluster_ratio"])
        m = mesh.simplify_quadric_decimation(target_number_of_triangles=M["target_triangles"])
        m.compute_vertex_normals()
        dt = time.perf_counter() - t0
    return dt, n_raw


# ----------------------------------------------------------------------------- process pools
def _pool_init(cfg_path, sets):
    global _CFG
    _CFG = load_config(cfg_path, sets)  # OMP_NUM_THREADS comes from the environment (set by run_pool)


def _pool_odo(chunk):
    return run_odometry(_CFG, _CFG["bench"], frames=chunk)[0]


def _pool_loop(chunk):
    return run_loops(_CFG, _CFG["bench"], pairs=chunk)[0]


def run_pool(args, cfg, B, kind):
    if kind == "odometry_pool":
        items = list(range(B["odometry_start"], B["odometry_start"] + B["odometry_pairs"]))
        fn = _pool_odo
    else:
        items = loop_pairs(cfg, B)
        fn = _pool_loop
    chunks = [items[i:: args.procs] for i in range(args.procs)]
    os.environ["OMP_NUM_THREADS"] = str(args.threads)  # inherited by the spawned children
    with mp.get_context("spawn").Pool(args.procs, initializer=_pool_init,
                                      initargs=(args.config, args.set)) as pool:
        pool.map(_noop, range(args.procs))  # start-up / imports not timed
        t0 = time.perf_counter()
        pool.map(fn, [c for c in chunks if c])
        return time.perf_counter() - t0, len(items)


def _noop(_):
    import open3d  # noqa: F401
    return 0


# ----------------------------------------------------------------------------- driver
def worker(args, cfg):
    B = cfg["bench"]
    if args.worker in ("odometry_pool", "loop_pool"):
        dt, n = run_pool(args, cfg, B, args.worker)
        return {"seconds": dt, "items": n}
    if args.worker == "odometry":
        dt, n = run_odometry(cfg, B)
        return {"seconds": dt, "items": n}
    if args.worker == "loop_registration":
        dt, n = run_loops(cfg, B)
        df, nf = run_fpfh(cfg, B)
        return {"seconds": dt, "items": n, "fpfh_seconds": df, "fpfh_items": nf}
    if args.worker == "tsdf":
        dt, n, de = run_tsdf(cfg, B)
        return {"seconds": dt, "items": n, "extract_seconds": de}
    dt, n = run_mesh(cfg, B)
    return {"seconds": dt, "items": n}


def busy_cores(interval=1.0):
    """Cores busy over `interval` s (all processes; nothing of ours runs between measurements)."""
    def snap():
        v = [int(x) for x in open("/proc/stat").readline().split()[1:]]
        return sum(v), v[3] + v[4]
    t0, i0 = snap()
    time.sleep(interval)
    t1, i1 = snap()
    return (1 - (i1 - i0) / max(t1 - t0, 1)) * (os.cpu_count() or 1)


def spawn(args, step, env_threads, rt_threads=None, procs=1):
    """One fresh worker process. env_threads: OMP_NUM_THREADS (None = unset); rt_threads: runtime limit
    via runtime.omp_threads.<step> (None = none). Pools: env_threads is per child, rt_threads unused."""
    pool = step.endswith("_pool")
    cmd = [sys.executable, str(Path(__file__)), "--config", args.config, "--worker", step,
           "--threads", str(env_threads or 0), "--procs", str(procs)]
    for s in args.set:
        cmd += ["--set", s]
    for st in STEPS:  # only the runtime limit under test may be active
        cmd += ["--set", f"runtime.omp_threads.{st}=" + (str(rt_threads) if (st == step and rt_threads) else "null")]
    env = {k: v for k, v in os.environ.items() if k != "OMP_NUM_THREADS"}
    if env_threads is not None and not pool:
        env["OMP_NUM_THREADS"] = str(env_threads)
    busy = busy_cores()
    out = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if out.returncode:
        die(f"worker {step} env={env_threads} rt={rt_threads} failed:\n{out.stderr[-3000:]}")
    res = json.loads(out.stdout.strip().splitlines()[-1])
    mode = ("pool" if pool else "default" if env_threads is None and rt_threads is None
            else f"rt{rt_threads}" if env_threads is None else f"env{env_threads}+rt{rt_threads}" if rt_threads
            else "env")
    res.update({"step": step, "threads": env_threads if env_threads is not None else rt_threads, "procs": procs,
                "mode": mode, "busy_cores_before": busy})
    return res


def main():
    args = parse_args(__doc__, extra)
    cfg = load_config(args.config, args.set)
    if args.worker:
        print(json.dumps(worker(args, cfg)))
        return
    B = cfg["bench"]
    if os.environ.get("OMP_NUM_THREADS"):
        print("[note] OMP_NUM_THREADS is set in this shell; workers run without it")
    busy = busy_cores(2.0)
    if busy > B["max_busy_cores"] and not args.allow_load:
        die(f"machine is busy ({busy:.1f} cores in use > bench.max_busy_cores {B['max_busy_cores']}); "
            "close other jobs or --allow-load")
    out = stage_dir(cfg, "bench")
    lo, hi = B["mechanism_check_threads"]
    plan = []
    for step in args.steps:
        plan += [(step, n, None, 1) for n in B["threads"]] + [(step, None, None, 1)]
        plan += [(step, None, lo, 1), (step, lo, hi, 1)]  # mechanism check (env lo / env hi are in the sweep)
    plan += [(kind, t, None, p) for kind in B["pool_steps"] for p, t in B["pool_combos"]]
    rows = []
    for r_i in range(B["repeats"]):
        for step, n_env, n_rt, p in plan:
            r = spawn(args, step, n_env, n_rt, procs=p)
            r["repeat"] = r_i
            rows.append(r)
            extra_s = "".join(f", {k} {r[k]:.2f}s" for k in ("fpfh_seconds", "extract_seconds") if k in r)
            print(f"[{r_i + 1}/{B['repeats']}] {step:17s} {p:2d}p {r['mode']:10s} threads {r['threads']!s:>4}: "
                  f"{r['seconds']:.2f}s for {r['items']}{extra_s} (other load {r['busy_cores_before']:.1f} cores)",
                  flush=True)
    summary = summarise(rows)
    best = {}
    for step in args.steps:
        cand = {k: v for k, v in summary.items() if k[0] == step and k[3] in ("env", "default")}
        best[step] = min(cand.items(), key=lambda kv: kv[1]["total_median"])[0][1]  # None = default
    write_json(out / "threads.json", {"host_cpus": os.cpu_count(), "params": B, "results": rows, "fastest": best,
                                      "summary": [{"step": k[0], "threads": k[1], "procs": k[2], "mode": k[3], **v}
                                                  for k, v in summary.items()]})
    print_table(summary)
    print(f"\nfastest (median): {best}  -> {out / 'threads.json'}")


def summarise(rows):
    """(step, threads, procs, mechanism) -> median / min / max of seconds (+ sub-timings)."""
    groups = {}
    for r in rows:
        groups.setdefault((r["step"], r["threads"], r["procs"], r["mode"]), []).append(r)
    out = {}
    for k, rs in groups.items():
        tot = [r["seconds"] + r.get("fpfh_seconds", 0) + r.get("extract_seconds", 0) for r in rs]
        d = {"n": len(rs), "items": rs[0]["items"], "total_median": float(np.median(tot)),
             "total_min": float(min(tot)), "total_max": float(max(tot)),
             "busy_cores_max": float(max(r["busy_cores_before"] for r in rs))}
        for sub in ("seconds", "fpfh_seconds", "extract_seconds"):
            if sub in rs[0]:
                d[f"{sub}_median"] = float(np.median([r[sub] for r in rs]))
        out[k] = d
    return out


def print_table(summary):
    print("\n| step | procs x threads | mode | median [s] | min-max [s] | other load max [cores] |")
    print("|---|---|---|---|---|---|")
    for (step, t, p, mech), v in summary.items():
        print(f"| {step} | {p} x {t} | {mech} | {v['total_median']:.2f} | {v['total_min']:.2f}-{v['total_max']:.2f} | "
              f"{v['busy_cores_max']:.1f} |")


if __name__ == "__main__":
    main()
