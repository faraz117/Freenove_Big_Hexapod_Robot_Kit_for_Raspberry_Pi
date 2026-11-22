# -*- coding: utf-8 -*-
"""
Makes the Server code importable and testable without real Raspberry Pi hardware
(I2C, SPI, camera, GPIO). CI runners have none of that, so every hardware-only
module is replaced with a MagicMock before any application code imports it.

gpiozero is the exception: it's pip-installable and ships MockFactory specifically
for this purpose, so GPIO-backed classes (OutputDevice, DistanceSensor) are tested
through gpiozero's real code path instead of being opaquely mocked away.
"""
import os
import sys
from unittest.mock import MagicMock

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
SERVER_DIR = os.path.dirname(TESTS_DIR)
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")  # headless PyQt5, if it's available

_HARDWARE_ONLY_MODULES = ["smbus", "spidev", "mpu6050", "picamera2", "picamera2.encoders",
                          "picamera2.outputs", "libcamera", "rpi_ws281x"]
for _name in _HARDWARE_ONLY_MODULES:
    sys.modules[_name] = MagicMock()

# A couple of these are referenced as class-level constants/types at import time
# (imu.py: mpu6050.ACCEL_RANGE_2G; camera.py: Transform(...)), so the mock needs to
# tolerate attribute access and calls the same way the real module would.
sys.modules["mpu6050"].mpu6050.ACCEL_RANGE_2G = 0
sys.modules["mpu6050"].mpu6050.GYRO_RANGE_250DEG = 0
# IMU.calculate_average_sensor_data() sums 100 readings of these on every IMU() construction;
# left as auto-generated MagicMocks, that's 100 iterations of slow Mock-on-Mock arithmetic
# (many seconds) for every single test that builds a Control(). Real floats make it instant.
sys.modules["mpu6050"].mpu6050.return_value.get_accel_data.return_value = {'x': 0.0, 'y': 0.0, 'z': 0.0}
sys.modules["mpu6050"].mpu6050.return_value.get_gyro_data.return_value = {'x': 0.0, 'y': 0.0, 'z': 0.0}

try:
    from gpiozero import Device
    from gpiozero.pins.mock import MockFactory
    Device.pin_factory = MockFactory()
except ImportError:
    pass  # gpiozero itself isn't installed; tests that need it will skip individually

import pytest


@pytest.fixture(autouse=True)
def _reset_mock_gpio_pins():
    # Each Control() reserves GPIO4 (servo_power_disable) via MockFactory and nothing
    # in the app code ever releases it - harmless on real hardware (one process, one
    # reservation for its whole lifetime), but across many tests each constructing
    # their own Control(), the second one onward fails with GPIOPinInUse unless the
    # mock factory's reservations are cleared between tests.
    yield
    try:
        from gpiozero import Device
        Device.pin_factory.reset()
    except ImportError:
        pass


@pytest.fixture
def control_instance():
    """A real Control() instance with hardware mocked out - safe to construct because
    Control.__init__ only creates its background threads, it never starts them."""
    from control import Control
    return Control()
