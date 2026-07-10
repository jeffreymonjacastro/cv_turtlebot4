RUN_ID="$(date +%Y%m%d_%H%M%S)_wall_follow_less_conservative_autonomous"
mkdir -p "output/perception_runs/$RUN_ID"

uv run python win/yolo/recibidor.py \
  --enable-qr \
  --qr-event-path output/signals/latest_qr_event.json \
  --perception-log "output/perception_runs/$RUN_ID/laptop_perception.jsonl"

