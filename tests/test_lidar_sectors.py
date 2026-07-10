import math
import random
from types import SimpleNamespace

import pytest

from scripts.replay_nav_scenarios import all_invalid_scan, corridor_scan, noisy_corridor_scan
from ubuntu.reactive_nav.lidar_sectors import extract_sectors


def _single_obstacle_scan(angle_deg: float, distance_m: float = 0.35):
    """Return a scan with a compact known obstacle and otherwise open space."""
    ranges = [4.0] * 361
    # The sector distance uses a robust 10th percentile, so represent a small
    # physical object across several rays instead of relying on a lone outlier.
    for obstacle_angle in range(int(round(angle_deg)) - 5, int(round(angle_deg)) + 6):
        normalized = ((obstacle_angle + 180) % 360) - 180
        ranges[int(normalized) + 180] = distance_m
    return SimpleNamespace(
        ranges=ranges,
        range_min=0.05,
        range_max=4.0,
        angle_min=-math.pi,
        angle_increment=math.pi / 180.0,
    )


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


def test_angle_offset_zero_keeps_front_obstacle_in_front_sector():
    sectors = extract_sectors(_single_obstacle_scan(0.0), angle_offset_deg=0.0)

    assert sectors.distance("front_center") == pytest.approx(0.35)
    assert sectors.distance("left") == pytest.approx(4.0)
    assert sectors.distance("right") == pytest.approx(4.0)


def test_positive_angle_offset_rotates_front_obstacle_to_left_sector():
    sectors = extract_sectors(_single_obstacle_scan(0.0), angle_offset_deg=90.0)

    assert sectors.distance("front_center") == pytest.approx(4.0)
    assert sectors.distance("left") == pytest.approx(0.35)


def test_negative_angle_offset_rotates_front_obstacle_to_right_sector():
    sectors = extract_sectors(_single_obstacle_scan(0.0), angle_offset_deg=-90.0)

    assert sectors.distance("front_center") == pytest.approx(4.0)
    assert sectors.distance("right") == pytest.approx(0.35)


def test_front_obstacle_maps_to_front_after_positive_offset_compensation():
    sectors = extract_sectors(_single_obstacle_scan(-90.0), angle_offset_deg=90.0)

    assert sectors.distance("front_center") == pytest.approx(0.35)


def test_left_obstacle_maps_to_left_after_positive_offset_compensation():
    sectors = extract_sectors(_single_obstacle_scan(0.0), angle_offset_deg=90.0)

    assert sectors.distance("left") == pytest.approx(0.35)


def test_right_obstacle_maps_to_right_after_positive_offset_compensation():
    sectors = extract_sectors(_single_obstacle_scan(180.0), angle_offset_deg=90.0)

    assert sectors.distance("right") == pytest.approx(0.35)
