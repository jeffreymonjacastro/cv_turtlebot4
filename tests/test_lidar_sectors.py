import math
import random

import pytest

from scripts.replay_nav_scenarios import all_invalid_scan, corridor_scan, noisy_corridor_scan
from ubuntu.reactive_nav.lidar_sectors import extract_sectors


def test_extracts_expected_sector_distances_from_corridor_scan():
    sectors = extract_sectors(
        corridor_scan(
            front=1.70,
            front_center=1.55,
            front_left=0.95,
            front_right=1.05,
            left=0.62,
            right=0.68,
            rear=0.90,
        )
    )

    assert sectors.valid_count == sectors.total_count
    assert sectors.distance("front_center") == pytest.approx(1.55)
    assert sectors.distance("left") == pytest.approx(0.62)
    assert sectors.distance("right") == pytest.approx(0.68)
    assert sectors.rear_distance == pytest.approx(0.90)


def test_filters_nan_inf_and_out_of_range_values_robustly():
    scan = corridor_scan(front=1.0, left=0.7, right=0.8)
    scan.ranges[180] = math.nan
    scan.ranges[181] = math.inf
    scan.ranges[182] = scan.range_min / 2.0
    scan.ranges[183] = scan.range_max + 50.0

    sectors = extract_sectors(scan)

    assert sectors.valid_count < sectors.total_count
    assert sectors.distance("front") is not None
    assert sectors.distance("front") <= scan.range_max


def test_all_invalid_scan_has_no_valid_points_and_empty_sectors():
    sectors = extract_sectors(all_invalid_scan(0.0, random.Random(0)))

    assert sectors.valid_count == 0
    assert sectors.distance("front") is None
    assert sectors.distance("left") is None
    assert sectors.distance("right") is None


def test_noisy_scan_keeps_front_sector_usable():
    sectors = extract_sectors(noisy_corridor_scan(0.0, random.Random(2)))

    assert sectors.valid_count > 0
    assert sectors.distance("front") is not None
    assert 1.5 <= sectors.distance("front") <= 2.1
