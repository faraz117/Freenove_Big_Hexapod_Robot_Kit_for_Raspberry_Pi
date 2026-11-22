# -*- coding: utf-8 -*-
"""NullServo and SupervisedServo are what let the whole system (GUI, autonomous patrol)
keep running when the PCA9685 boards aren't responding on I2C - these are the safety net,
so they get their own direct tests rather than only being exercised indirectly."""
from servo import NullServo, SupervisedServo


def test_null_servo_is_a_silent_noop():
    stub = NullServo()
    stub.set_servo_angle(0, 90)  # must not raise
    stub.relax()                 # must not raise


def test_supervised_servo_passes_through_when_healthy():
    calls = []

    class FakeServo:
        def set_servo_angle(self, channel, angle):
            calls.append((channel, angle))

        def relax(self):
            calls.append("relax")

    failures = []
    wrapped = SupervisedServo(FakeServo(), on_failure=failures.append)
    wrapped.set_servo_angle(3, 45)
    wrapped.relax()

    assert calls == [(3, 45), "relax"]
    assert failures == []


def test_supervised_servo_reports_failure_instead_of_raising():
    class BrokenServo:
        def set_servo_angle(self, channel, angle):
            raise OSError("[Errno 5] Input/output error")

        def relax(self):
            raise OSError("[Errno 5] Input/output error")

    failures = []
    wrapped = SupervisedServo(BrokenServo(), on_failure=failures.append)

    wrapped.set_servo_angle(0, 90)  # must not raise - this is the whole point
    wrapped.relax()

    assert len(failures) == 2
    assert all(isinstance(e, OSError) for e in failures)
