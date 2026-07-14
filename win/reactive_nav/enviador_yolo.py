#!/usr/bin/env python3
"""Copy YOLO latest-signal state from the laptop to the TurtleBot.

Run this on the laptop/local repo next to ``win/yolo/recibidor.py``. The YOLO
receiver writes ``output/signals/latest_signal.json`` locally; this helper keeps
the TurtleBot copy fresh so ``reactive_navigator.py`` can react to signs.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = REPO_ROOT / "output" / "signals" / "latest_signal.json"
DEFAULT_ROBOT = os.environ.get("ROBOT_SSH_TARGET", "robot@robot-hostname")
DEFAULT_REMOTE_PATH = "/home/ubuntu/output/signals/latest_signal.json"
DEFAULT_QR_SOURCE = REPO_ROOT / "output" / "signals" / "latest_qr_event.json"
DEFAULT_QR_REMOTE_PATH = "/home/ubuntu/output/signals/latest_qr_event.json"


def run_command(command: list[str], *, quiet: bool = False, input_bytes: bytes | None = None) -> bool:
    result = subprocess.run(
        command,
        input=input_bytes,
        stdout=subprocess.DEVNULL if quiet else None,
        stderr=subprocess.DEVNULL if quiet else None,
        check=False,
    )
    return result.returncode == 0


def ensure_remote_dir(robot: str, remote_path: str) -> bool:
    remote_dir = str(Path(remote_path).parent)
    return run_command(["ssh", robot, "mkdir", "-p", remote_dir])


def copy_state_atomic(source: Path, robot: str, remote_path: str, *, quiet: bool) -> bool:
    """Copy to a remote temporary file, then atomically install it."""

    remote_tmp = f"{remote_path}.tmp.{os.getpid()}.{time.monotonic_ns()}"
    try:
        payload = source.read_bytes()
    except OSError:
        return False

    install_command = (
        f"cat > {shlex.quote(remote_tmp)} && "
        f"mv {shlex.quote(remote_tmp)} {shlex.quote(remote_path)}"
    )
    if run_command(["ssh", robot, install_command], quiet=quiet, input_bytes=payload):
        return True
    run_command(["ssh", robot, "rm", "-f", remote_tmp], quiet=True)
    return False


def append_sync_log(path: Path | None, payload: dict) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=True) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default=str(DEFAULT_SOURCE),
        help="Local latest_signal.json written by win/yolo/recibidor.py.",
    )
    parser.add_argument(
        "--robot",
        default=DEFAULT_ROBOT,
        help="SSH target for the TurtleBot.",
    )
    parser.add_argument(
        "--remote-path",
        default=DEFAULT_REMOTE_PATH,
        help="Destination path read by reactive_navigator.py.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Polling interval in seconds.",
    )
    parser.add_argument(
        "--copy-every-interval",
        action="store_true",
        help="Copy every interval instead of only when the source mtime changes.",
    )
    parser.add_argument(
        "--qr-source",
        default=None,
        help=f"Optional validated QR state file (for example {DEFAULT_QR_SOURCE}).",
    )
    parser.add_argument(
        "--qr-remote-path",
        default=DEFAULT_QR_REMOTE_PATH,
        help="Remote validated QR event destination.",
    )
    parser.add_argument("--log-path", default=None, help="Optional structured sync JSONL log.")
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress successful copy messages.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = Path(args.source).expanduser().resolve()
    qr_source = Path(args.qr_source).expanduser().resolve() if args.qr_source else None
    log_path = Path(args.log_path).expanduser().resolve() if args.log_path else None
    interval = max(0.05, float(args.interval))

    print(f"[YOLO-SYNC] source={source}")
    print(f"[YOLO-SYNC] destination={args.robot}:{args.remote_path}")
    print(f"[YOLO-SYNC] interval={interval:.2f}s")

    if not ensure_remote_dir(args.robot, args.remote_path):
        print("[YOLO-SYNC] ERROR: could not create remote signal directory", file=sys.stderr)
        return 2
    if qr_source is not None and not ensure_remote_dir(args.robot, args.qr_remote_path):
        print("[QR-SYNC] ERROR: could not create remote QR directory", file=sys.stderr)
        return 2
    if qr_source is not None:
        print(f"[QR-SYNC] source={qr_source}")
        print(f"[QR-SYNC] destination={args.robot}:{args.qr_remote_path}")

    sources = [("yolo", source, args.remote_path)]
    if qr_source is not None:
        sources.append(("qr", qr_source, args.qr_remote_path))
    last_mtime_ns = {kind: None for kind, _path, _remote in sources}
    last_missing_log = 0.0
    while True:
        for kind, local_path, remote_path in sources:
            try:
                stat = local_path.stat()
            except FileNotFoundError:
                now = time.monotonic()
                if now - last_missing_log >= 2.0:
                    print(f"[{kind.upper()}-SYNC] waiting for {local_path}")
                    last_missing_log = now
                continue

            changed = stat.st_mtime_ns != last_mtime_ns[kind]
            if changed or args.copy_every_interval:
                copy_started_at = time.time()
                success = copy_state_atomic(local_path, args.robot, remote_path, quiet=args.quiet)
                copied_at = time.time()
                record = {
                    "timestamp": copied_at,
                    "kind": kind,
                    "source": str(local_path),
                    "remote_path": remote_path,
                    "source_age_s": max(0.0, copied_at - stat.st_mtime),
                    "transfer_duration_s": max(0.0, copied_at - copy_started_at),
                    "success": success,
                }
                if kind == "qr":
                    try:
                        record["event_id"] = json.loads(local_path.read_text(encoding="utf-8")).get("event_id")
                    except (OSError, json.JSONDecodeError):
                        record["event_id"] = None
                append_sync_log(log_path, record)
                if success:
                    last_mtime_ns[kind] = stat.st_mtime_ns
                    if not args.quiet:
                        print(f"[{kind.upper()}-SYNC] copied {local_path.name} age={record['source_age_s']:.2f}s")
                else:
                    print(f"[{kind.upper()}-SYNC] WARN: atomic copy failed; retrying", file=sys.stderr)

        time.sleep(interval)


if __name__ == "__main__":
    raise SystemExit(main())
