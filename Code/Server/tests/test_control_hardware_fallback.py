# -*- coding: utf-8 -*-
"""
Covers the I2C-not-responding degradation path added after the robot's servo board
lost power and crash-looped: Control() must come up in a degraded-but-alive state
instead of raising, must recover via reinitialize_hardware(), and must demote itself
again if a real servo call fails mid-operation later.
"""
import control as control_module
from servo import NullServo


def test_control_degrades_gracefully_when_imu_and_servo_fail(monkeypatch):
    def boom():
        raise OSError("[Errno 5] Input/output error")

    monkeypatch.setattr(control_module, "IMU", boom)
    monkeypatch.setattr(control_module, "Servo", boom)

    c = control_module.Control()

    assert c.hardware_ok is False
    assert c.imu is None
    assert isinstance(c.servo, NullServo)
    # calibrate()/set_leg_angles() run unconditionally in __init__ - they must not raise
    # just because the servo is a stub.
    c.set_leg_angles()


def test_reinitialize_hardware_recovers_once_i2c_comes_back(monkeypatch):
    def boom():
        raise OSError("[Errno 5] Input/output error")

    monkeypatch.setattr(control_module, "IMU", boom)
    monkeypatch.setattr(control_module, "Servo", boom)
    c = control_module.Control()
    assert c.hardware_ok is False

    # Undo the patch - "I2C recovered" - and retry.
    monkeypatch.undo()
    assert c.reinitialize_hardware() is True
    assert c.hardware_ok is True
    assert not isinstance(c.servo, NullServo)


def test_reinitialize_hardware_is_a_noop_when_already_healthy(control_instance):
    assert control_instance.hardware_ok is True
    assert control_instance.reinitialize_hardware() is True


def test_servo_failure_mid_operation_demotes_to_null_servo(control_instance):
    assert control_instance.hardware_ok is True
    control_instance._handle_servo_failure(OSError("[Errno 5] Input/output error"))
    assert control_instance.hardware_ok is False
    assert isinstance(control_instance.servo, NullServo)
