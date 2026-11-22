# -*- coding: utf-8 -*-
"""Sanity check that the wire-protocol command constants haven't been accidentally
renamed/removed - the client and server only agree on behavior through these strings."""
from command import COMMAND as cmd

EXPECTED_COMMANDS = [
    "CMD_MOVE", "CMD_LED_MOD", "CMD_LED", "CMD_SONIC", "CMD_BUZZER", "CMD_HEAD",
    "CMD_BALANCE", "CMD_ATTITUDE", "CMD_POSITION", "CMD_RELAX", "CMD_POWER",
    "CMD_CALIBRATION", "CMD_CAMERA", "CMD_SERVOPOWER",
]


def test_all_expected_commands_exist_and_are_their_own_name():
    for name in EXPECTED_COMMANDS:
        assert getattr(cmd, name) == name
