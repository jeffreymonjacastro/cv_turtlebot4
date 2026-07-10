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

python3 win/reactive_nav/enviador_yolo.py

python3 -B reactive_navigator.py --ros-args --params-file configs/wall_follow_safe.yaml -p dry_run:=false -p enable_motion:=true -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json

python3 -B debug_image_udp_sender.py --ros-args -p port:=6610 -p image_topic:=/oakd/rgb/preview/image_raw -p jpeg_quality:=80 -p send_hz:=5

### Test YOLO

#### Turtle

cd ~/reactive
python3 -B debug_image_udp_sender.py --ros-args \
 -p port:=6610 \
 -p image_topic:=/oakd/rgb/preview/image_raw \
 -p jpeg_quality:=80 \
 -p send_hz:=5.0

python3 -B signal_udp_receiver.py \
 --port 6611 \
 --output /home/ubuntu/output/signals/latest_signal.json

#### Win

python win/yolo/recibidor.py 10.60.199.200

python win/reactive_nav/enviador_yolo.py --robot-ip 10.60.199.200 --port 6611

#### ROS2

watch -n 0.2 cat /home/ubuntu/output/signals/latest_signal.json

ros2 service call /oakd/start_camera std_srvs/srv/Trigger "{}"

ros2 topic hz /oakd/rgb/preview/image_raw
