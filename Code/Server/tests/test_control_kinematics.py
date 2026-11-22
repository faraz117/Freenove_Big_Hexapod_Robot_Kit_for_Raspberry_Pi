# -*- coding: utf-8 -*-
"""Pure-math smoke tests for the leg inverse-kinematics helpers. These don't touch
hardware at all, so a regression here is a pure logic bug, not a wiring issue."""
import pytest


def test_restrict_value_clamps_both_directions(control_instance):
    assert control_instance.restrict_value(5, 0, 10) == 5
    assert control_instance.restrict_value(-5, 0, 10) == 0
    assert control_instance.restrict_value(15, 0, 10) == 10


def test_map_value_linear_mapping(control_instance):
    assert control_instance.map_value(5, 0, 10, 0, 100) == 50
    assert control_instance.map_value(0, 0, 10, 0, 100) == 0
    assert control_instance.map_value(10, 0, 10, 0, 100) == 100


@pytest.mark.parametrize("x,y,z", [
    (0, 140, 0),      # the default/neutral leg position used throughout control.py
    (10, 130, -10),
    (-15, 120, 15),
])
def test_coordinate_to_angle_round_trips_through_angle_to_coordinate(control_instance, x, y, z):
    # angle_to_coordinate is the forward kinematics, coordinate_to_angle is the inverse -
    # feeding one into the other should land back close to the original point. This is
    # exactly the kind of math that's easy to subtly break while editing control.py.
    a, b, c = control_instance.coordinate_to_angle(x, y, z)
    x2, y2, z2 = control_instance.angle_to_coordinate(a, b, c)
    assert abs(x2 - x) <= 2
    assert abs(y2 - y) <= 2
    assert abs(z2 - z) <= 2
