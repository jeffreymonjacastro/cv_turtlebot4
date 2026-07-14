RUN_ID="$(date +%Y%m%d_%H%M%S)_wall_follow_less_conservative_autonomous"
mkdir -p "output/perception_runs/$RUN_ID"

uv run python win/yolo/recibidor.py \
  --enable-qr \
  --qr-max-hz 5.0 \
  --qr-confirm-count 1 \
  --qr-duplicate-cooldown-s 10.0 \
  --qr-event-path output/signals/latest_qr_event.json \
  --perception-log "output/perception_runs/$RUN_ID/laptop_perception.jsonl"

