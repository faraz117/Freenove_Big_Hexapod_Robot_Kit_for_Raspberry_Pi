# -*- coding: utf-8 -*-
"""
Cheapest, highest-coverage smoke test: every server module should import without
raising. Catches syntax errors, typos, and broken imports introduced on a feature
branch before they ever reach the robot. Deliberately excludes myCode.py (runs real
servo commands at module level) and test.py (a manual hardware-test harness, not a
library module) - neither is meant to be imported as a dependency.
"""
import importlib

import pytest

NON_GUI_MODULES = [
    "adc",
    "buzzer",
    "camera",
    "command",
    "control",
    "imu",
    "kalman",
    "led",
    "parameter",
    "pca9685",
    "pid",
    "rpi_ledpixel",
    "server",
    "servo",
    "spi_ledpixel",
    "tcp_server",
    "Thread",
    "ultrasonic",
]


@pytest.mark.parametrize("module_name", NON_GUI_MODULES)
def test_module_imports_cleanly(module_name):
    importlib.import_module(module_name)


@pytest.mark.parametrize("module_name", ["main", "ui_server"])
def test_gui_module_imports_cleanly(module_name):
    pytest.importorskip("PyQt5")
    importlib.import_module(module_name)
