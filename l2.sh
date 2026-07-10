RUN_ID="$(ls -td output/perception_runs/*_wall_follow_less_conservative_autonomous | head -n 1 | xargs basename)"

uv run python win/reactive_nav/enviador_yolo.py \
  --robot turtlebot4 \
  --source output/signals/latest_signal.json \
  --remote-path /home/ubuntu/output/signals/latest_signal.json \
  --qr-source output/signals/latest_qr_event.json \
  --qr-remote-path /home/ubuntu/output/signals/latest_qr_event.json \
  --interval 0.2 \
  --log-path "output/perception_runs/$RUN_ID/sync.jsonl"

