import json

from ubuntu.reactive_nav.signal_udp_receiver import parse_signal_packet, write_signal_state
from win.reactive_nav.enviador_yolo import build_signal_packet


def test_signal_udp_packet_round_trip(tmp_path):
    source = tmp_path / "latest_signal.json"
    output = tmp_path / "robot_signal.json"
    source.write_text(
        json.dumps(
            {
                "direction": "left",
                "confidence": 0.95,
                "timestamp": 123.0,
                "bbox_area_ratio": 0.20,
                "bbox_center_x_ratio": 0.50,
                "actionable": True,
            }
        ),
        encoding="utf-8",
    )

    packet, sent = build_signal_packet(source)
    received = parse_signal_packet(packet)
    write_signal_state(output, received)

    assert sent["direction"] == "left"
    assert json.loads(output.read_text(encoding="utf-8"))["direction"] == "left"
