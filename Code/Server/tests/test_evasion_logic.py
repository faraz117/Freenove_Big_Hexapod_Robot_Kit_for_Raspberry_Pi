# -*- coding: utf-8 -*-
"""
choose_evasion_direction and record_point are the decision-making core of the
obstacle-avoidance patrol - pure logic over sensor readings, testable without any
hardware at all by feeding in synthetic scans.
"""
import pytest


def test_choose_evasion_direction_picks_the_clearer_side(control_instance):
    scan = {
        (30, 90): 200,   # left, clear
        (90, 90): 15,    # center, blocked
        (150, 90): 20,   # right, still close
    }
    assert control_instance.choose_evasion_direction(scan) == 'left'


def test_choose_evasion_direction_prefers_right_when_thats_clearer(control_instance):
    scan = {
        (30, 90): 20,
        (90, 90): 15,
        (150, 90): 200,
    }
    assert control_instance.choose_evasion_direction(scan) == 'right'


def test_choose_evasion_direction_backs_away_when_both_sides_blocked(control_instance):
    scan = {
        (30, 90): 10,
        (90, 90): 10,
        (150, 90): 12,
    }
    assert control_instance.choose_evasion_direction(scan) == 'back'


@pytest.mark.parametrize("distance,camera_hit,expected_status", [
    (300, False, '0'),   # nothing on either sensor
    (10, False, 'U'),    # only the ultrasonic sensor sees it
    (300, True, 'C'),    # only the camera heuristic sees it
    (10, True, 'B'),     # both agree
])
def test_record_point_combines_both_sensors_into_one_status(control_instance, monkeypatch,
                                                              distance, camera_hit, expected_status):
    monkeypatch.setattr(control_instance, "check_camera_obstacle", lambda pan, tilt: camera_hit)
    status = control_instance.record_point(90, 90, distance)
    assert status == expected_status
    assert control_instance.detection_grid[(90, 90)] == expected_status
