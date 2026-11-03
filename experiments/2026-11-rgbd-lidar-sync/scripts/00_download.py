#!/usr/bin/env python3
"""Stage 00: download the rosbag from Hugging Face into data/raw/ (and extract archives)."""
import os
import tarfile
import zipfile
from pathlib import Path

from common import (check_disk, die, dir_size_bytes, exp_path, free_disk_gb, human, load_config,
                    parse_args)


def _report_http_error(e):
    resp = getattr(e, "response", None)
    if resp is not None:
        host = getattr(getattr(resp, "request", None), "url", "?")
        print(f"[ERROR] HTTP {resp.status_code} for {host}")
        for h in ("x-deny-reason", "X-Deny-Reason"):
            if h in resp.headers:
                print(f"[ERROR] proxy x-deny-reason: {resp.headers[h]}")


def _safe_members(tf: tarfile.TarFile, dest: Path):
    dest = dest.resolve()
    for m in tf.getmembers():
        target = (dest / m.name).resolve()
        if not str(target).startswith(str(dest)) or m.issym() or m.islnk():
            die(f"refusing unsafe archive member {m.name}")
        yield m


def extract(archive: Path, dest: Path, cfg) -> None:
    marker = dest / f".extracted_{archive.name}"
    if marker.exists():
        print(f"[skip] {archive.name} already extracted")
        return
    print(f"[extract] {archive} -> {dest}")
    check_disk(cfg, dest)
    if archive.suffix == ".zip":
        with zipfile.ZipFile(archive) as z:
            for n in z.namelist():
                if not str((dest / n).resolve()).startswith(str(dest.resolve())):
                    die(f"refusing unsafe zip member {n}")
            z.extractall(dest)
    else:
        with tarfile.open(archive) as tf:
            tf.extractall(dest, members=_safe_members(tf, dest))
    marker.touch()
    if cfg["download"]["delete_archive_after_extract"]:
        archive.unlink()


def main():
    args = parse_args(__doc__)
    cfg = load_config(args.config, args.set)
    dl = cfg["download"]
    raw = exp_path(cfg["paths"]["raw_dir"])
    raw.mkdir(parents=True, exist_ok=True)
    check_disk(cfg, raw)

    from huggingface_hub import snapshot_download
    from huggingface_hub.errors import HfHubHTTPError, RepositoryNotFoundError

    token = os.environ.get(dl["token_env"]) or None
    path = None
    for repo_type in [dl["repo_type"]] + [t for t in ("dataset", "model") if t != dl["repo_type"]]:
        try:
            print(f"[download] {dl['repo_id']} (repo_type={repo_type}) -> {raw}")
            path = snapshot_download(repo_id=dl["repo_id"], repo_type=repo_type, revision=dl["revision"],
                                     local_dir=str(raw), token=token)
            break
        except RepositoryNotFoundError:
            print(f"[warn] not found as repo_type={repo_type}")
        except HfHubHTTPError as e:
            _report_http_error(e)
            die("download failed (see host / x-deny-reason above)")
    if path is None:
        die(f"repository {dl['repo_id']} not found (private? set ${dl['token_env']})")

    if dl["extract_archives"]:
        for a in sorted(raw.iterdir()):
            if a.is_file() and (a.name.endswith((".tar.gz", ".tgz", ".tar", ".zip"))):
                extract(a, raw, cfg)

    bags = sorted(m.parent for m in raw.rglob("metadata.yaml"))
    print(f"[done] raw dir {raw}: {human(dir_size_bytes(raw))}, free disk {free_disk_gb(raw):.1f} GB")
    for b in bags:
        files = [f.name for f in b.iterdir() if f.suffix in (".db3", ".mcap")]
        print(f"  bag: {b}  ({human(dir_size_bytes(b))}) storage files: {files}")
    if not bags:
        die("no bag directory (metadata.yaml) found after download")


if __name__ == "__main__":
    main()
