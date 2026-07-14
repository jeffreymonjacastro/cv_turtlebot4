import base64
import time

import cv2
import numpy as np

from win.yolo.frame_stream import decode_img_parts


def test_decode_img_parts_preserves_timestamps_and_frame_id():
    image = np.zeros((12, 16, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    packet = [
        "IMG",
        "2",
        "turtlebot4_rensso_mora",
        "123",
        "456000000",
        base64.b64encode(encoded.tobytes()).decode("ascii"),
    ]

    received_at = time.time()
    frame = decode_img_parts(packet, received_at=received_at)

    assert frame.image.shape[:2] == (12, 16)
    assert frame.domain_id == 2
    assert frame.robot_name == "turtlebot4_rensso_mora"
    assert frame.source_frame_time == "123.456000000"
    assert frame.frame_id == "turtlebot4_rensso_mora:123.456000000"
    assert frame.received_at == received_at
