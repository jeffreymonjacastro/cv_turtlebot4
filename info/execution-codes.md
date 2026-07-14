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

python3 win/yolo/recibidor.py 10.60.199.200

python3 ubuntu/reactive_nav/enviador_yolo.py

python3 -B reactive_navigator.py --ros-args --params-file configs/wall_follow_safe.yaml -p dry_run:=false -p enable_motion:=true -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json

python3 -B debug_image_udp_sender.py --ros-args -p port:=6610 -p image_topic:=/oakd/rgb/preview/image_raw -p jpeg_quality:=80 -p send_hz:=5