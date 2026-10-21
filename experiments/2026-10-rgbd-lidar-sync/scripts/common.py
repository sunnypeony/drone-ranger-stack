"""Shared helpers: config, paths, bag access, image decoding, TF math."""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import yaml

EXP_DIR = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = EXP_DIR / "configs" / "default.yaml"


# ----------------------------------------------------------------------------- config / paths
def parse_args(description: str, extra=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="path to YAML config")
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="override a config value, e.g. --set tsdf.voxel_length=0.015 (YAML-parsed value)")
    if extra:
        extra(p)
    return p.parse_args()


def _deep_merge(base: dict, over: dict) -> dict:
    """Recursive merge; a dict carrying `_replace: true` replaces the inherited value instead."""
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and v.get("_replace"):
            out[k] = {kk: vv for kk, vv in v.items() if kk != "_replace"}
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: str | Path, overrides: list[str] | None = None) -> dict:
    """Load YAML; a top-level `base: other.yaml` key inherits (deep-merges) another config file."""
    path = Path(path)
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if cfg.get("base"):
        cfg = _deep_merge(load_config(path.parent / cfg.pop("base")), cfg)
    for item in overrides or []:
        key, _, val = item.partition("=")
        node = cfg
        *parents, leaf = key.split(".")
        for k in parents:
            node = node.setdefault(k, {})
        node[leaf] = yaml.safe_load(val)
    # Must happen before the first `import open3d` (all scripts import it lazily, after this).
    # The right OpenMP thread count depends on the stage: Open3D's default pool (one thread per
    # core) makes RGB-D odometry ~9x slower on a 32-core host (2.2 s vs 0.24 s per pair), while
    # ScalableTSDF integration is ~140x slower with a single thread (2.4 s vs 1/60 s per frame).
    # runtime.omp_threads_by_script[<script stem>] wins over runtime.omp_num_threads; null = library
    # default; "auto" = min(CPU count, runtime.omp_auto_max) so the same config is multi-threaded on
    # a small cloud VM without oversubscribing a large host; an explicit OMP_NUM_THREADS in the
    # environment wins over all of these.
    rt = cfg.get("runtime", {})
    n = (rt.get("omp_threads_by_script") or {}).get(Path(sys.argv[0]).stem, rt.get("omp_num_threads"))
    if n == "auto":
        n = max(1, min(os.cpu_count() or 1, int(rt.get("omp_auto_max", 4))))
    if n:
        os.environ.setdefault("OMP_NUM_THREADS", str(n))
    return cfg


def exp_path(p: str | Path) -> Path:
    p = Path(p)
    return p if p.is_absolute() else EXP_DIR / p


def stage_dir(cfg: dict, stage: str) -> Path:
    d = exp_path(cfg["paths"]["outputs_dir"]) / stage
    d.mkdir(parents=True, exist_ok=True)
    return d


def find_bag_dir(cfg: dict) -> Path:
    if cfg["paths"].get("bag_dir"):
        d = exp_path(cfg["paths"]["bag_dir"])
        if not (d / "metadata.yaml").exists():
            die(f"configured bag_dir {d} has no metadata.yaml")
        return d
    raw = exp_path(cfg["paths"]["raw_dir"])
    hits = sorted(m.parent for m in raw.rglob("metadata.yaml"))
    if not hits:
        die(f"no ROS2 bag (metadata.yaml) found under {raw}; run 00_download.py first")
    if len(hits) > 1:
        print(f"[warn] multiple bags found, using {hits[0]}: {[str(h) for h in hits]}")
    return hits[0]


def free_disk_gb(path: Path) -> float:
    return shutil.disk_usage(path).free / 1e9


def check_disk(cfg: dict, where: Path | None = None) -> None:
    where = where or EXP_DIR
    free = free_disk_gb(where)
    if free < cfg["download"]["min_free_disk_gb"]:
        die(f"free disk {free:.1f} GB < min_free_disk_gb={cfg['download']['min_free_disk_gb']} GB")


def dir_size_bytes(path: Path) -> int:
    return sum(f.stat().st_size for f in Path(path).rglob("*") if f.is_file())


def human(nbytes: float) -> str:
    for unit in ["B", "KB", "MB", "GB", "TB"]:
        if nbytes < 1024:
            return f"{nbytes:.1f}{unit}"
        nbytes /= 1024
    return f"{nbytes:.1f}PB"


def die(msg: str, code: int = 1):
    print(f"[ERROR] {msg}", file=sys.stderr)
    sys.exit(code)


def write_json(path: Path, obj) -> None:
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, default=_json_default)


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(type(o))


class Timer:
    def __init__(self, label: str):
        self.label = label

    def __enter__(self):
        self.t0 = time.time()
        return self

    def __exit__(self, *a):
        print(f"[time] {self.label}: {time.time() - self.t0:.1f}s")


# ----------------------------------------------------------------------------- bag access
def get_typestore(cfg: dict):
    from rosbags.typesys import Stores, get_typestore as _gt
    return _gt(getattr(Stores, cfg.get("typestore", "ROS2_HUMBLE")))


STORAGE_BY_SUFFIX = {".db3": "sqlite3", ".mcap": "mcap"}


def _fixed_bag_dir(bag_dir: Path) -> Path:
    """Some trimming tools write `storage_identifier: ""`, which rosbags rejects.
    Instead of editing the raw data, write a patched metadata.yaml into outputs/ next to
    symlinks of the storage files and read the bag from there."""
    meta = yaml.safe_load(open(bag_dir / "metadata.yaml"))
    info = meta["rosbag2_bagfile_information"]
    if info.get("storage_identifier"):
        return bag_dir
    files = info.get("relative_file_paths") or []
    kinds = {STORAGE_BY_SUFFIX.get(Path(f).suffix) for f in files}
    if len(kinds) != 1 or None in kinds:
        die(f"metadata.yaml has empty storage_identifier and storage cannot be inferred from {files}")
    info["storage_identifier"] = kinds.pop()
    fixed = EXP_DIR / "outputs" / "bag_metadata_fix" / bag_dir.name
    fixed.mkdir(parents=True, exist_ok=True)
    for f in files:
        link = fixed / f
        if not link.exists():
            link.symlink_to((bag_dir / f).resolve())
    with open(fixed / "metadata.yaml", "w") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)
    print(f"[info] bag metadata had empty storage_identifier -> using patched copy {fixed} "
          f"(storage={info['storage_identifier']})")
    return fixed


def open_reader(bag_dir: Path):
    from rosbags.rosbag2 import Reader
    return Reader(_fixed_bag_dir(bag_dir))


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def iter_topic(reader, typestore, topic: str):
    """Yield (bag_time_ns, msg) for one topic."""
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        return
    for conn, t, raw in reader.messages(connections=conns):
        yield t, typestore.deserialize_cdr(raw, conn.msgtype)


def iter_topic_raw(reader, topic: str):
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        return
    for conn, t, raw in reader.messages(connections=conns):
        yield conn, t, raw


# ----------------------------------------------------------------------------- images
def decode_color(msg) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 RGB."""
    import cv2
    enc = msg.encoding.lower()
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data) if not isinstance(msg.data, np.ndarray) else msg.data, np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        return img.copy() if enc == "rgb8" else img[:, :, ::-1].copy()
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)[:, :, :3]
        return img.copy() if enc == "rgba8" else img[:, :, ::-1].copy()
    if enc in ("mono8",):
        img = buf.reshape(h, msg.step)[:, :w]
        return np.repeat(img[:, :, None], 3, axis=2)
    if enc in ("yuv422", "uyvy", "yuv422_yuy2", "yuyv"):
        img = buf.reshape(h, msg.step)[:, : w * 2].reshape(h, w, 2)
        code = cv2.COLOR_YUV2RGB_UYVY if enc in ("yuv422", "uyvy") else cv2.COLOR_YUV2RGB_YUYV
        return cv2.cvtColor(img, code)
    if enc in ("nv12",):
        img = buf[: h * w * 3 // 2].reshape(h * 3 // 2, w)
        return cv2.cvtColor(img, cv2.COLOR_YUV2RGB_NV12)
    raise ValueError(f"unsupported color encoding {msg.encoding}")


def decode_depth_m(msg, cfg: dict) -> np.ndarray:
    """sensor_msgs/Image depth -> HxW float32 metres (0 = invalid)."""
    enc = msg.encoding
    h, w = msg.height, msg.width
    buf = bytes(msg.data) if not isinstance(msg.data, np.ndarray) else msg.data.tobytes()
    be = bool(msg.is_bigendian)
    ex = cfg["extract"]
    if enc in ("16UC1", "mono16"):
        arr = np.frombuffer(buf, dtype=">u2" if be else "<u2").reshape(h, msg.step // 2)[:, :w]
        d = arr.astype(np.float32) * float(ex["depth_16uc1_scale"])
    elif enc == "32FC1":
        arr = np.frombuffer(buf, dtype=">f4" if be else "<f4").reshape(h, msg.step // 4)[:, :w]
        d = arr.astype(np.float32) * float(ex["depth_32fc1_scale"])
    else:
        raise ValueError(f"unsupported depth encoding {enc} (need 16UC1 or 32FC1)")
    d = np.where(np.isfinite(d), d, 0.0).astype(np.float32)
    return d


def camera_info_to_dict(msg) -> dict:
    K = np.array(msg.k, dtype=float).reshape(3, 3)
    D = np.array(msg.d, dtype=float)
    return {
        "width": int(msg.width),
        "height": int(msg.height),
        "K": K.tolist(),
        "D": D.tolist(),
        "distortion_model": msg.distortion_model,
        "frame_id": msg.header.frame_id,
    }


def check_K_vs_size(K: np.ndarray, w: int, h: int, label: str = "") -> None:
    cx, cy = K[0, 2], K[1, 2]
    if not (0 < cx < w and 0 < cy < h):
        die(f"{label} principal point ({cx:.1f},{cy:.1f}) outside image {w}x{h}: K does not match image size")
    if abs(cx - w / 2) > 0.25 * w or abs(cy - h / 2) > 0.25 * h:
        die(f"{label} principal point ({cx:.1f},{cy:.1f}) far from image centre of {w}x{h}: K likely for another resolution")
    if K[0, 0] <= 0 or K[1, 1] <= 0:
        die(f"{label} invalid focal length in K")


# ----------------------------------------------------------------------------- SE(3)
def quat_to_rot(x, y, z, w) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x y z w


def make_T(R=None, t=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def tf_msg_to_T(tr) -> np.ndarray:
    q, p = tr.rotation, tr.translation
    return make_T(quat_to_rot(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def assert_rigid(T: np.ndarray, cfg: dict, label: str = "") -> None:
    R = T[:3, :3]
    det = np.linalg.det(R)
    orth = np.abs(R.T @ R - np.eye(3)).max()
    tol_d, tol_o = cfg["poses"]["rigid_det_tol"], cfg["poses"]["rigid_orth_tol"]
    if abs(det - 1) > tol_d or orth > tol_o or not np.allclose(T[3], [0, 0, 0, 1]) or not np.all(np.isfinite(T)):
        die(f"{label} is not a valid rigid transform (det={det:.6f}, orth_err={orth:.2e})")


def rot_angle_deg(R: np.ndarray) -> float:
    c = (np.trace(R) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def interp_T(t_query: int, times: np.ndarray, Ts: np.ndarray, max_gap_ns: int):
    """Interpolate SE(3) at t_query from sorted times. Translation linear, rotation slerp.
    Returns None if outside coverage or the bracketing gap is too large."""
    from scipy.spatial.transform import Rotation, Slerp
    if len(times) == 0 or t_query < times[0] or t_query > times[-1]:
        return None
    i = int(np.searchsorted(times, t_query))
    if i < len(times) and times[i] == t_query:
        return Ts[i].copy()
    i0, i1 = i - 1, i
    if times[i1] - times[i0] > max_gap_ns:
        return None
    a = (t_query - times[i0]) / float(times[i1] - times[i0])
    sl = Slerp([0.0, 1.0], Rotation.from_matrix(np.stack([Ts[i0][:3, :3], Ts[i1][:3, :3]])))
    R = sl([a]).as_matrix()[0]
    t = (1 - a) * Ts[i0][:3, 3] + a * Ts[i1][:3, 3]
    return make_T(R, t)


class StaticTF:
    """Static TF tree (parent->child) with chain lookup.
    policy="first": the first message for a child frame wins, later differing ones are logged as conflicts."""

    def __init__(self, policy: str = "first"):
        if policy not in ("first", "last"):
            die(f"unknown tf_static_policy {policy}")
        self.policy = policy
        self.edges: dict[str, tuple[str, np.ndarray]] = {}  # child -> (parent, T_parent_child)
        self.conflicts: list[dict] = []

    def add(self, parent: str, child: str, T: np.ndarray):
        child, parent = child.lstrip("/"), parent.lstrip("/")
        if child in self.edges:
            p0, T0 = self.edges[child]
            if p0 != parent or not np.allclose(T0, T, atol=1e-9):
                self.conflicts.append({"child": child, "kept_parent": p0, "kept_T": T0.tolist(),
                                       "other_parent": parent, "other_T": T.tolist(), "policy": self.policy})
            if self.policy == "first":
                return
        self.edges[child] = (parent, T)

    def _chain_to_root(self, frame):
        chain, f = [frame], frame
        while f in self.edges:
            f = self.edges[f][0]
            chain.append(f)
            if len(chain) > 64:
                break
        return chain

    def T_root_frame(self, frame):
        T = np.eye(4)
        f = frame
        while f in self.edges:
            p, Tp = self.edges[f]
            T = Tp @ T
            f = p
        return f, T

    def lookup(self, target: str, source: str):
        """T_target_source, or None if not connected."""
        rt, Tt = self.T_root_frame(target)
        rs, Ts = self.T_root_frame(source)
        if rt != rs:
            return None
        return np.linalg.inv(Tt) @ Ts

    def frames(self):
        s = set(self.edges)
        s.update(p for p, _ in self.edges.values())
        return sorted(s)


def T_base_laser_from_cfg(cfg: dict, static_tf: "StaticTF | None" = None) -> np.ndarray:
    """Fixed, measured laser extrinsic from config (`lidar.T_base_laser`: [x, y, z, yaw_deg]) if set,
    otherwise base->laser from /tf_static."""
    fixed = cfg.get("lidar", {}).get("T_base_laser")
    if fixed is not None:
        x, y, z, yaw = fixed
        c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        return make_T(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]), [x, y, z])
    if static_tf is None:
        die("no fixed lidar.T_base_laser in config and no /tf_static given")
    T = static_tf.lookup(cfg["frames"]["base"], cfg["frames"]["laser"])
    if T is None:
        die("base->laser missing in /tf_static")
    return T


def lidar_time_offset_s(cfg: dict) -> float:
    """`lidar.time_offset_s`: a number, or "auto" = value adopted by 03b (outputs/<tree>/poses/lidar_calib.json)."""
    v = cfg["lidar"].get("time_offset_s", 0.0)
    if v != "auto":
        return float(v)
    p = stage_dir(cfg, "poses") / "lidar_calib.json"
    return float(json.load(open(p))["time_offset_s"]) if p.exists() else 0.0


def load_static_tf(reader, typestore, cfg) -> StaticTF:
    st = StaticTF(cfg["frames"].get("tf_static_policy", "first"))
    for _, msg in iter_topic(reader, typestore, cfg["topics"]["tf_static"]):
        for tr in msg.transforms:
            st.add(tr.header.frame_id, tr.child_frame_id, tf_msg_to_T(tr.transform))
    for c in st.conflicts:
        print(f"[warn] /tf_static conflict for {c['child']}: keeping {c['policy']} message "
              f"(t={np.round(np.array(c['kept_T'])[:3, 3], 4).tolist()}), ignoring "
              f"t={np.round(np.array(c['other_T'])[:3, 3], 4).tolist()}")
    return st


def load_dynamic_tf(reader, typestore, cfg, parent: str, child: str):
    """Collect parent->child transforms from /tf (and /odom if tf lacks them). Returns (times_ns, Ts, source)."""
    times, Ts = [], []
    for _, msg in iter_topic(reader, typestore, cfg["topics"]["tf"]):
        for tr in msg.transforms:
            if tr.header.frame_id.lstrip("/") == parent and tr.child_frame_id.lstrip("/") == child:
                times.append(stamp_ns(tr.header))
                Ts.append(tf_msg_to_T(tr.transform))
    source = "tf"
    if not times and cfg["topics"].get("odom"):
        for _, msg in iter_topic(reader, typestore, cfg["topics"]["odom"]):
            if msg.header.frame_id.lstrip("/") == parent and msg.child_frame_id.lstrip("/") == child:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                if not np.all(np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w])) or (q.x == q.y == q.z == q.w == 0):
                    continue  # RTAB-Map publishes null pose when lost
                times.append(stamp_ns(msg.header))
                Ts.append(make_T(quat_to_rot(q.x, q.y, q.z, q.w), [p.x, p.y, p.z]))
        source = "odom_topic"
    if not times:
        return np.zeros(0, np.int64), np.zeros((0, 4, 4)), None
    order = np.argsort(times)
    times = np.asarray(times, np.int64)[order]
    Ts = np.asarray(Ts)[order]
    keep = np.concatenate([[True], np.diff(times) > 0])
    return times[keep], Ts[keep], source


def coverage(query_ns: np.ndarray, times: np.ndarray, max_gap_ns: int) -> float:
    if len(times) == 0 or len(query_ns) == 0:
        return 0.0
    i = np.searchsorted(times, query_ns)
    ok = (query_ns >= times[0]) & (query_ns <= times[-1])
    i = np.clip(i, 1, len(times) - 1)
    gap = times[i] - times[i - 1]
    exact = np.isin(query_ns, times)
    ok &= (gap <= max_gap_ns) | exact
    return float(ok.mean())


# ----------------------------------------------------------------------------- frames dataset
def frames_dir(cfg: dict) -> Path:
    """Extracted frames can be shared between output trees (v2 reuses the v1 frames)."""
    fd = cfg["paths"].get("frames_dir")
    return exp_path(fd) if fd else stage_dir(cfg, "frames")


def load_frames(cfg: dict):
    import csv
    d = frames_dir(cfg)
    rows = list(csv.DictReader(open(d / "frames.csv")))
    intr = json.load(open(d / "intrinsics.json"))
    return d, rows, intr


def cutoff_count(cfg: dict, rows) -> int:
    """Number of leading frames with stamp - stamp[0] <= cutoff_s (all frames if cutoff_s is null).
    The bag was cut by hand from a longer recording, so its last frame carries no meaning."""
    c = cfg.get("cutoff_s")
    if c is None:
        return len(rows)
    t = (np.array([int(r["stamp_ns"]) for r in rows], np.int64) - int(rows[0]["stamp_ns"])) / 1e9
    return int(np.searchsorted(t, c, side="right"))


def o3d_intrinsic(intr: dict):
    import open3d as o3d
    K = np.array(intr["K"])
    return o3d.camera.PinholeCameraIntrinsic(intr["width"], intr["height"], K[0, 0], K[1, 1], K[0, 2], K[1, 2])


def load_poses(cfg: dict, aligned: bool = False):
    d = stage_dir(cfg, "poses")
    z = np.load(d / "poses.npz", allow_pickle=True)
    Ts = z["T_world_optical"]
    if aligned:
        ta = stage_dir(cfg, "mesh") / "T_align.npy"
        if ta.exists():
            A = np.load(ta)
            Ts = np.einsum("ij,njk->nik", A, Ts)
    return z, Ts
