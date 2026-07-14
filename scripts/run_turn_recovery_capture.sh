#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/ubuntu/reactive_nav_test}"
NAV_SCRIPT="${NAV_SCRIPT:-$WORKSPACE_ROOT/reactive_nav/reactive_navigator.py}"
PROFILE_FILE="${PROFILE_FILE:-$WORKSPACE_ROOT/reactive_nav/configs/wall_follow_tuned.yaml}"
if [[ -d "$WORKSPACE_ROOT" ]]; then
  DEFAULT_OUTPUT_ROOT="/home/ubuntu/output/robot_runs"
else
  DEFAULT_OUTPUT_ROOT="$REPO_ROOT/output/robot_runs"
fi
OUTPUT_ROOT="${OUTPUT_ROOT:-$DEFAULT_OUTPUT_ROOT}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-2}"
TELEMETRY_PORT="${TELEMETRY_PORT:-6612}"
SIGNAL_STATE_PATH="${SIGNAL_STATE_PATH:-/home/ubuntu/output/signals/latest_signal.json}"
QR_LOG_PATH="${QR_LOG_PATH:-/home/ubuntu/output/qr_log.jsonl}"
ROBOT_BASE_NAME="${ROBOT_BASE_NAME:-wall_follow_tuned}"
RECORD_BAG="${RECORD_BAG:-1}"
DURATION_SEC="${DURATION_SEC:-}"
FORCE_SIGNAL="${FORCE_SIGNAL:-auto}"

usage() {
  cat <<'EOF'
Usage: run_turn_recovery_capture.sh <scenario> [options]

Scenarios:
  angle_offset_dryrun
  left_turn
  right_turn
  front_blocked_recovery

Options:
  --run-name NAME        Override the generated run name
  --output-root PATH     Override the robot output root
  --workspace-root PATH  Override the robot workspace root
  --profile-file PATH    Override the YAML profile file
  --nav-script PATH      Override the reactive_navigator.py path
  --no-bag               Skip rosbag recording
  --bag                  Force rosbag recording
  --duration-sec N       Stop capture after N seconds
  --force-signal DIR     Write a synthetic left/right/stop signal before launch
  --no-force-signal      Do not write a synthetic signal for turn scenarios
  --help                 Show this help
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

if [[ "$1" == "--help" || "$1" == "-h" ]]; then
  usage
  exit 0
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # The helper is meant to be copy/paste friendly on the robot.
  # Sourcing ROS here avoids relying on the operator's shell state.
  # ROS setup scripts may read unset AMENT_* variables, so relax nounset only
  # around the source call and restore strictness immediately after.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

if [[ -d "$WORKSPACE_ROOT" ]]; then
  cd "$WORKSPACE_ROOT"
elif [[ -d "$REPO_ROOT" ]]; then
  cd "$REPO_ROOT"
fi

if [[ ! -f "$NAV_SCRIPT" && -f "$REPO_ROOT/ubuntu/reactive_nav/reactive_navigator.py" ]]; then
  NAV_SCRIPT="$REPO_ROOT/ubuntu/reactive_nav/reactive_navigator.py"
fi

if [[ ! -f "$PROFILE_FILE" && -f "$REPO_ROOT/ubuntu/reactive_nav/configs/wall_follow_tuned.yaml" ]]; then
  PROFILE_FILE="$REPO_ROOT/ubuntu/reactive_nav/configs/wall_follow_tuned.yaml"
fi

SCENARIO="$1"
shift

RUN_NAME=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-name)
      RUN_NAME="${2:?missing value for --run-name}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:?missing value for --output-root}"
      shift 2
      ;;
    --workspace-root)
      WORKSPACE_ROOT="${2:?missing value for --workspace-root}"
      NAV_SCRIPT="$WORKSPACE_ROOT/reactive_nav/reactive_navigator.py"
      PROFILE_FILE="$WORKSPACE_ROOT/reactive_nav/configs/wall_follow_tuned.yaml"
      shift 2
      ;;
    --profile-file)
      PROFILE_FILE="${2:?missing value for --profile-file}"
      shift 2
      ;;
    --nav-script)
      NAV_SCRIPT="${2:?missing value for --nav-script}"
      shift 2
      ;;
    --no-bag)
      RECORD_BAG=0
      shift
      ;;
    --bag)
      RECORD_BAG=1
      shift
      ;;
    --duration-sec)
      DURATION_SEC="${2:?missing value for --duration-sec}"
      shift 2
      ;;
    --force-signal)
      FORCE_SIGNAL="${2:?missing value for --force-signal}"
      shift 2
      ;;
    --no-force-signal)
      FORCE_SIGNAL="none"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

case "$SCENARIO" in
  angle_offset_dryrun)
    DRY_RUN=true
    ENABLE_MOTION=false
    SCENARIO_LABEL="angle_offset_dryrun"
    ;;
  left_turn)
    DRY_RUN=false
    ENABLE_MOTION=true
    SCENARIO_LABEL="left_turn_capture"
    if [[ "$FORCE_SIGNAL" == "auto" ]]; then
      FORCE_SIGNAL="left"
    fi
    ;;
  right_turn)
    DRY_RUN=false
    ENABLE_MOTION=true
    SCENARIO_LABEL="right_turn_capture"
    if [[ "$FORCE_SIGNAL" == "auto" ]]; then
      FORCE_SIGNAL="right"
    fi
    ;;
  front_blocked_recovery)
    DRY_RUN=false
    ENABLE_MOTION=true
    SCENARIO_LABEL="front_blocked_recovery"
    if [[ "$FORCE_SIGNAL" == "auto" ]]; then
      FORCE_SIGNAL="none"
    fi
    ;;
  *)
    echo "Unknown scenario: $SCENARIO" >&2
    usage
    exit 2
    ;;
esac

if [[ ! -f "$NAV_SCRIPT" ]]; then
  echo "Missing navigator script: $NAV_SCRIPT" >&2
  exit 1
fi

if [[ ! -f "$PROFILE_FILE" ]]; then
  echo "Missing profile file: $PROFILE_FILE" >&2
  exit 1
fi

PROFILE_LABEL="$(basename "$PROFILE_FILE" .yaml)"
if [[ "$ROBOT_BASE_NAME" == "wall_follow_tuned" && "$PROFILE_LABEL" != "wall_follow_tuned" ]]; then
  ROBOT_BASE_NAME="$PROFILE_LABEL"
fi

if [[ -z "$RUN_NAME" ]]; then
  RUN_NAME="$(date +%Y%m%d_%H%M%S)_${ROBOT_BASE_NAME}_${SCENARIO_LABEL}"
fi

RUN_DIR="$OUTPUT_ROOT/$RUN_NAME"

mkdir -p "$RUN_DIR/collision_frames"

cat > "$RUN_DIR/operator_note.md" <<EOF
# Operator note

Profile: $PROFILE_LABEL
Scenario tested: $SCENARIO
Start pose / environment:
Expected behavior:
Observed behavior:
Approximate failure time:
Did it enter recovery?
Did it spin/circle?
Did it scrape/hit corner?
Was e-stop/manual intervention needed?
Any visible LiDAR/camera issue:
EOF

cp "$PROFILE_FILE" "$RUN_DIR/profile.yaml"

echo "Starting capture:"
echo "  scenario: $SCENARIO"
echo "  run dir:  $RUN_DIR"
echo "  profile:  $PROFILE_FILE"
echo "  bag:      $RECORD_BAG"
echo "  signal:   $FORCE_SIGNAL"
if [[ -n "$DURATION_SEC" ]]; then
  echo "  duration: ${DURATION_SEC}s"
fi

export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export RUN_NAME

if [[ "$FORCE_SIGNAL" != "auto" && "$FORCE_SIGNAL" != "none" ]]; then
  mkdir -p "$(dirname "$SIGNAL_STATE_PATH")"
  python3 - "$SIGNAL_STATE_PATH" "$FORCE_SIGNAL" "$RUN_NAME" <<'PY'
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
direction = sys.argv[2].lower()
run_name = sys.argv[3]
if direction not in {"left", "right", "stop"}:
    raise SystemExit(f"unsupported forced signal: {direction}")
payload = {
    "direction": direction,
    "confidence": 0.99,
    "timestamp": time.time(),
    "bbox_area_ratio": 0.20,
    "bbox_center_x_ratio": 0.50,
    "actionable": True,
    "source_frame_time": f"synthetic:{run_name}",
}
path.write_text(json.dumps(payload), encoding="utf-8")
PY
fi

bag_pid=""
cleanup() {
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$RECORD_BAG" == "1" ]]; then
  ros2 bag record \
    /scan \
    /cmd_vel \
    /hazard_detection \
    -o "$RUN_DIR/bag" &
  bag_pid="$!"
fi

nav_cmd=(
  python3 -B "$NAV_SCRIPT" --ros-args
  --params-file "$PROFILE_FILE"
  -p dry_run:="$DRY_RUN"
  -p enable_motion:="$ENABLE_MOTION"
  -p telemetry_port:="$TELEMETRY_PORT"
  -p signal_state_path:="$SIGNAL_STATE_PATH"
  -p qr_log_path:="$QR_LOG_PATH"
  -p persistent_log_path:="$RUN_DIR/reactive_nav_debug.jsonl"
  -p collision_log_path:="$RUN_DIR/collision_events.jsonl"
  -p collision_image_dir:="$RUN_DIR/collision_frames"
)

set +e
if [[ -n "$DURATION_SEC" ]]; then
  timeout --signal=INT "$DURATION_SEC" "${nav_cmd[@]}"
else
  "${nav_cmd[@]}"
fi
nav_status=$?
set -e

cleanup

if [[ "$nav_status" -eq 124 ]]; then
  echo "Capture duration elapsed; stopped navigator with SIGINT."
elif [[ "$nav_status" -ne 0 ]]; then
  echo "Navigator exited with status: $nav_status" >&2
  exit "$nav_status"
fi

echo "Run directory: $RUN_DIR"
if [[ "$RECORD_BAG" == "1" ]]; then
  echo "Bag saved under: $RUN_DIR/bag"
fi
