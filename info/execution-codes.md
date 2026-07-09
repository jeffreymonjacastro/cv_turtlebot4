## Cosas de ejecucion

### Ejecutar navigation de reactive_nav

```sh
cd /home/ubuntu/reactive
python3 -B reactive_navigator.py --ros-args \
  --params-file configs/wall_follow_safe.yaml \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl
```

