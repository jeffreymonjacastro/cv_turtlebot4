#!/usr/bin/env python3
"""Copy YOLO latest-signal state from the laptop to the TurtleBot.

Run this on the laptop/local repo next to ``win/yolo/recibidor.py``. The YOLO
receiver writes ``output/signals/latest_signal.json`` locally; this helper keeps
the TurtleBot copy fresh so ``reactive_navigator.py`` can react to signs.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = REPO_ROOT / "output" / "signals" / "latest_signal.json"
DEFAULT_ROBOT = "ubuntu@10.60.199.200"
DEFAULT_REMOTE_PATH = "/home/ubuntu/output/signals/latest_signal.json"


def run_command(command: list[str], *, quiet: bool = False) -> bool:
    result = subprocess.run(
        command,
        stdout=subprocess.DEVNULL if quiet else None,
        stderr=subprocess.DEVNULL if quiet else None,
        check=False,
    )
    return result.returncode == 0


def ensure_remote_dir(robot: str, remote_path: str) -> bool:
    remote_dir = str(Path(remote_path).parent)
    return run_command(["ssh", robot, "mkdir", "-p", remote_dir])


def copy_signal(source: Path, robot: str, remote_path: str, *, quiet: bool) -> bool:
    return run_command(["scp", "-q", str(source), f"{robot}:{remote_path}"], quiet=quiet)


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
        "--quiet",
        action="store_true",
        help="Suppress successful copy messages.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = Path(args.source).expanduser().resolve()
    interval = max(0.05, float(args.interval))

    print(f"[YOLO-SYNC] source={source}")
    print(f"[YOLO-SYNC] destination={args.robot}:{args.remote_path}")
    print(f"[YOLO-SYNC] interval={interval:.2f}s")

    if not ensure_remote_dir(args.robot, args.remote_path):
        print("[YOLO-SYNC] ERROR: could not create remote signal directory", file=sys.stderr)
        return 2

    last_mtime_ns = None
    last_missing_log = 0.0
    while True:
        try:
            stat = source.stat()
        except FileNotFoundError:
            now = time.monotonic()
            if now - last_missing_log >= 2.0:
                print(f"[YOLO-SYNC] waiting for {source}")
                last_missing_log = now
            time.sleep(interval)
            continue

        changed = stat.st_mtime_ns != last_mtime_ns
        if changed or args.copy_every_interval:
            if copy_signal(source, args.robot, args.remote_path, quiet=args.quiet):
                last_mtime_ns = stat.st_mtime_ns
                if not args.quiet:
                    age = time.time() - stat.st_mtime
                    print(f"[YOLO-SYNC] copied latest_signal.json age={age:.2f}s")
            else:
                print("[YOLO-SYNC] WARN: scp failed; retrying", file=sys.stderr)

        time.sleep(interval)


if __name__ == "__main__":
    raise SystemExit(main())
