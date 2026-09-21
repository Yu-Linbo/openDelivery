import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "backend"
sys.path.insert(0, str(BACKEND))

import robot_settings  # noqa: E402


class RobotSettingsStoreTest(unittest.TestCase):
    def test_settings_are_persisted_under_the_selected_robot_only(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "robot_settings.json"
            store = robot_settings.RobotSettingsStore(path)
            store.save("robot2", {
                "max_linear_speed": 0.25,
                "max_angular_speed": 0.7,
                "inflation_radius": 0.65,
            })
            store.save("robot3", {
                "max_linear_speed": 0.15,
                "max_angular_speed": 0.5,
                "inflation_radius": 0.45,
            })

            self.assertEqual(store.get("robot2")["settings"]["max_linear_speed"], 0.25)
            self.assertEqual(store.get("robot3")["settings"]["max_linear_speed"], 0.15)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(set(payload["robots"]), {"robot2", "robot3"})

    def test_rejects_unsafe_or_non_finite_values(self):
        with self.assertRaisesRegex(ValueError, "between"):
            robot_settings.normalize_settings({"inflation_radius": 0.1})
        with self.assertRaisesRegex(ValueError, "finite"):
            robot_settings.normalize_settings({"max_linear_speed": float("nan")})
        with self.assertRaisesRegex(ValueError, "invalid robot_id"):
            robot_settings.validate_robot_id("../robot2")

    def test_browser_legacy_names_are_normalized(self):
        value = robot_settings.normalize_settings({
            "maxSpeed": 0.3,
            "angularSpeed": 0.75,
            "safetyDistance": 0.7,
        })
        self.assertEqual(value, {
            "max_linear_speed": 0.3,
            "max_angular_speed": 0.75,
            "inflation_radius": 0.7,
        })


class RobotSettingsRuntimeTest(unittest.TestCase):
    def test_runtime_apply_targets_only_the_selected_robot_namespace(self):
        completed = subprocess.CompletedProcess([], 0, "Set parameter successful", "")
        runner = mock.Mock(return_value=completed)
        result = robot_settings.apply_runtime("robot2", robot_settings.DEFAULT_SETTINGS, runner)

        self.assertEqual(result["state"], "applied")
        self.assertEqual(runner.call_count, 6)
        commands = "\n".join(call.args[0] for call in runner.call_args_list)
        self.assertIn("/robot2/navigation/controller_server", commands)
        self.assertIn("/robot2/navigation/local_costmap/local_costmap", commands)
        self.assertIn("/robot2/navigation/global_costmap/global_costmap", commands)
        self.assertNotIn("/robot3/", commands)
        self.assertIn("FollowPath.max_vel_x", commands)
        self.assertIn("inflation_layer.inflation_radius", commands)

    def test_failed_runtime_apply_remains_pending_for_restart(self):
        completed = subprocess.CompletedProcess([], 1, "", "node not found")
        result = robot_settings.apply_runtime(
            "robot2", robot_settings.DEFAULT_SETTINGS, lambda _command: completed
        )
        self.assertEqual(result["state"], "pending_restart")
        self.assertEqual(len(result["errors"]), 6)


class RobotSettingsStartupContractTest(unittest.TestCase):
    def test_sim_bringup_passes_store_and_launch_applies_all_nav2_values(self):
        bringup = (
            ROOT / "src" / "system" / "system" / "scripts" / "sim_bringup.sh"
        ).read_text(encoding="utf-8")
        launch = (
            ROOT / "params" / "launch" / "nav_bringup" / "stack.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("robot_settings_file:=", bringup)
        for name in (
            "max_linear_speed",
            "max_angular_speed",
            "inflation_radius",
            "max_vel_x",
            "max_speed_xy",
            "max_vel_theta",
            "max_rotational_vel",
        ):
            self.assertIn(name, launch)

    def test_web_page_uses_server_api_and_has_no_fake_browser_settings(self):
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        script = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
        server = (ROOT / "backend" / "server.py").read_text(encoding="utf-8")
        self.assertIn('id="settings-robot-id"', html)
        self.assertIn('name="maxLinearSpeed"', html)
        self.assertIn('name="maxAngularSpeed"', html)
        self.assertIn('name="inflationRadius"', html)
        self.assertIn("/api/robot/settings", script)
        self.assertNotIn('localStorage.setItem(SETTINGS_KEY', script)
        self.assertIn('path == "/api/robot/settings"', server)


if __name__ == "__main__":
    unittest.main()
