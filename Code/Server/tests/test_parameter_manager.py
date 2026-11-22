# -*- coding: utf-8 -*-
"""ParameterManager is pure file I/O (PCB/Pi version config) - test it against a
throwaway file, never the real params.json this robot actually uses."""
import os

from parameter import ParameterManager


def _manager_for(tmp_path):
    manager = ParameterManager()
    manager.file_path = os.path.join(str(tmp_path), "params.json")
    return manager


def test_create_and_read_back_param_file(tmp_path):
    manager = _manager_for(tmp_path)
    manager.create_param_file()

    assert manager.file_exists()
    assert manager.validate_params()
    assert manager.get_pcb_version() == 2


def test_set_param_round_trips(tmp_path):
    manager = _manager_for(tmp_path)
    manager.create_param_file()

    manager.set_param("Pcb_Version", 1)
    assert manager.get_param("Pcb_Version") == 1


def test_validate_params_rejects_missing_file(tmp_path):
    manager = _manager_for(tmp_path)
    assert manager.validate_params() is False


def test_validate_params_rejects_bad_json(tmp_path):
    manager = _manager_for(tmp_path)
    with open(manager.file_path, "w") as f:
        f.write("not valid json{")
    assert manager.validate_params() is False
