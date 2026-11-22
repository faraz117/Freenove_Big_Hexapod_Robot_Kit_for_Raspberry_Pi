# -*- coding: utf-8 -*-
"""voltage_to_percent doesn't touch self at all, so it can be called unbound without
constructing the whole GUI/Server stack - keeps this smoke test cheap and hardware-free."""
import pytest


def test_voltage_to_percent_clamps_and_scales():
    pytest.importorskip("PyQt5")
    from main import MyWindow

    assert MyWindow.voltage_to_percent(None, 8.4) == 100
    assert MyWindow.voltage_to_percent(None, 6.0) == 0
    assert MyWindow.voltage_to_percent(None, 7.2) == 50
    # Must clamp, not return out-of-range percentages for a battery past full/empty.
    assert MyWindow.voltage_to_percent(None, 9.0) == 100
    assert MyWindow.voltage_to_percent(None, 3.0) == 0
