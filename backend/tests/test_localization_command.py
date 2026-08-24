import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "localization_command.py"
SPEC = importlib.util.spec_from_file_location("localization_command_under_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
resolve_map_name = MODULE.resolve_map_name


def test_explicit_map_wins():
    assert resolve_map_name(" test_104 ", "robot4", {}, {}, []) == "test_104"


def test_pose_only_uses_live_current_map_before_stale_sources():
    assert resolve_map_name(
        "",
        "robot4",
        {"current_map": "test_101"},
        {"current_map": "test_102"},
        [{"id": "robot4", "current_map": "test_103"}],
    ) == "test_101"


def test_pose_only_falls_back_to_persisted_then_spec():
    specs = [{"id": "robot4", "current_map": "test_103"}]
    assert resolve_map_name("", "robot4", {}, {"current_map": "test_102"}, specs) == "test_102"
    assert resolve_map_name("", "robot4", {}, {}, specs) == "test_103"
