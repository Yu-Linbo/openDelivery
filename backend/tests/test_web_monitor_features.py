import os
import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "backend"
sys.path.insert(0, str(BACKEND))
os.environ.setdefault("ROBOT_POSE_MODE", "none")

import robot_motion_api as motion  # noqa: E402


class WebMonitorFeatureTest(unittest.TestCase):
    def tearDown(self):
        motion._TELEOP_PROCESSES.clear()
        motion._TELEOP_SEQUENCE.clear()
        motion._TELEOP_STATE.clear()
        motion._TELEOP_LEASE_DEADLINE.clear()

    def test_teleop_requires_confirmation_and_session_sequence(self):
        with self.assertRaises(ValueError):
            motion.set_teleop_velocity(
                "robot2", 0.2, 0, active=True, confirmed=False,
                session_id="browser", sequence=1,
            )
        with self.assertRaises(ValueError):
            motion.set_teleop_velocity(
                "robot2", 0.2, 0, active=True, confirmed=True,
                session_id="", sequence=0,
            )

    @mock.patch.object(motion, "_start_ros_process")
    def test_stale_start_cannot_override_newer_stop(self, start_process):
        fake = mock.Mock()
        fake.poll.return_value = None
        start_process.return_value = fake
        with mock.patch.object(motion, "_stop_teleop_process"), mock.patch.object(
            motion, "_ros_run", return_value={"ok": True}
        ):
            stopped = motion.set_teleop_velocity(
                "robot2", 0, 0, active=False, confirmed=True,
                session_id="browser", sequence=2,
            )
            stale = motion.set_teleop_velocity(
                "robot2", 0.2, 0, active=True, confirmed=True,
                session_id="browser", sequence=1,
            )
        self.assertFalse(stopped["active"])
        self.assertTrue(stale["stale"])
        start_process.assert_not_called()

    @mock.patch.object(motion, "_ensure_teleop_watchdog")
    @mock.patch.object(motion, "_start_ros_process")
    def test_active_teleop_sets_a_short_lease(self, start_process, ensure_watchdog):
        fake = mock.Mock()
        fake.poll.return_value = None
        start_process.return_value = fake
        result = motion.set_teleop_velocity(
            "robot2", 0.2, 0, active=True, confirmed=True,
            session_id="browser", sequence=1,
        )
        self.assertTrue(result["active"])
        self.assertLessEqual(result["lease_sec"], 1.0)
        self.assertIn("robot2", motion._TELEOP_LEASE_DEADLINE)
        ensure_watchdog.assert_called_once()

    def test_expired_teleop_stops_publisher_and_publishes_zero(self):
        motion._TELEOP_PROCESSES["robot2"] = mock.Mock()
        motion._TELEOP_STATE["robot2"] = ("browser", 0.2, 0.0)
        motion._TELEOP_LEASE_DEADLINE["robot2"] = 10.0
        with mock.patch.object(motion, "_stop_teleop_process") as stop, mock.patch.object(
            motion, "_ros_run", return_value={"ok": True}
        ) as ros_run:
            expired = motion._expire_teleop_leases(now=10.1)
        self.assertEqual(expired, ["robot2"])
        stop.assert_called_once_with("robot2")
        self.assertNotIn("robot2", motion._TELEOP_LEASE_DEADLINE)
        self.assertIn("linear: {x: 0.0", ros_run.call_args.args[0])

    def test_monitor_dom_and_script_contain_requested_features(self):
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        js = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
        css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
        server_py = (ROOT / "backend" / "server.py").read_text(encoding="utf-8")
        self.assertIn('<select id="reloc-robot-id">', html)
        self.assertIn('id="btn-reloc-record"', html)
        self.assertIn('<option value="relocalization">重定位点</option>', html)
        self.assertIn('<option value="elevator_inside">电梯内点</option>', html)
        self.assertIn('<option value="elevator_waiting">电梯等待点</option>', html)
        self.assertIn('id="semantic-map-toggle"', html)
        self.assertIn('id="custom-points-toggle"', html)
        self.assertIn("显示点位", html)
        self.assertNotIn("显示自定义点位", html)
        self.assertIn('id="btn-map-editor-open"', html)
        self.assertIn('id="standalone-map-editor"', html)
        self.assertIn('data-map-editor="canvas"', html)
        self.assertIn('data-map-editor="map-name"', html)
        self.assertIn('data-map-editor="load-map"', html)
        self.assertNotIn('data-map-editor="point-tool"', html)
        self.assertIn('data-map-editor="pick-point"', html)
        self.assertIn('data-map-editor="semantic-trigger"', html)
        self.assertIn('data-map-editor="semantic-menu"', html)
        self.assertIn('data-map-editor="show-meter-grid"', html)
        self.assertIn('<input type="checkbox" id="grid-toggle" checked title="0.5 米网格" />\n                  显示网格', html)
        self.assertIn('data-map-editor="show-meter-grid" checked title="0.5 米网格" />显示网格', html)
        self.assertIn('data-map-editor="show-grid" checked />显示底图', html)
        self.assertNotIn('<option value="points">点位信息</option>', html)
        self.assertNotIn('data-map-editor="point-x"', html)
        self.assertNotIn('data-map-editor="point-yaw"', html)
        self.assertIn('data-map-editor="layer-undo"', html)
        self.assertIn('data-map-editor="layer-discard"', html)
        self.assertIn('data-map-editor="layer-save"', html)
        self.assertIn('data-map-editor="point-undo"', html)
        self.assertIn('data-map-editor="point-discard"', html)
        self.assertIn('data-map-editor="point-save"', html)
        self.assertNotIn('data-map-editor="undo"', html)
        self.assertNotIn('data-map-editor="discard"', html)
        self.assertIn("点位设置", html)
        self.assertNotIn('data-map-editor="semantic-swatch"', html)
        self.assertIn('aria-modal="true"', html)
        self.assertIn('class="monitor-teleop-rail"', html)
        self.assertGreater(html.index('id="standalone-map-editor"'), html.index('class="monitor-teleop-rail"'))
        self.assertIn('data-teleop="forward"', html)
        self.assertIn("syncOnlineRobotSelect", js)
        self.assertIn("/api/robot/relocalization/record", js)
        self.assertIn('type === "relocalization" ? "重定位点"', js)
        self.assertIn('point.type === "custom" || point.type === "relocalization" ? 5 : 7', js)
        self.assertIn("/api/robot/motion/teleop", js)
        self.assertIn("floor_id: floorId", js)
        self.assertIn("payload.task", js)
        self.assertIn("task.work_queue", js)
        self.assertNotIn("当前后端尚未接入持久任务队列", js)
        self.assertIn('"type": "navigation_task"', server_py)
        self.assertIn("rcq.enqueue_command(command)", server_py)
        self.assertIn("visibilitychange", js)
        self.assertIn("teleopHeartbeatTimer", js)
        self.assertIn("teleopHeldRobotId", js)
        self.assertIn("mapEditorDirtyLayers", js)
        self.assertIn("setMapEditorDialogOpen", js)
        self.assertIn("undoMapEditorChange", js)
        self.assertIn("discardMapEditorChanges", js)
        self.assertIn("pushMapEditorUndoSnapshot", js)
        self.assertIn("ev.ctrlKey || ev.metaKey", js)
        self.assertIn("option.style.backgroundColor", js)
        self.assertIn("rgba(15, 23, 42, 0.88)", js)
        self.assertIn("mapEditorResumeAfterPick", js)
        self.assertIn('markMapEditorDirty("points")', js)
        self.assertIn("editingPoints", js)
        self.assertIn('tool === "add"', js)
        self.assertIn('["add", "新增点位"]', js)
        self.assertIn("minmax(210px, 240px)", css)
        self.assertIn(".monitor-teleop-rail", css)
        self.assertIn(".map-editor-float", css)
        self.assertIn(".map-editor-backdrop", css)
        self.assertIn("backdrop-filter: blur(2.2px)", css)
        self.assertIn("height: calc(100vh - 36px)", css)
        self.assertIn("right: 278px", css)
        self.assertIn("position: fixed", css)
        self.assertIn("window.StandaloneMapEditor.open(activeFloor)", js)
        self.assertIn("window.OPEN_DELIVERY_API_BASE_URL = API_BASE_URL", js)
        self.assertIn('row.model?`${row.id}(${row.model})`:row.id', js)
        self.assertIn('contentEl.classList.toggle("content--monitor", next === "monitor")', js)
        self.assertIn("max-width: 1400px", css)
        self.assertIn(".content.content--monitor", css)

    def test_standalone_editor_tool_modes_are_exclusive(self):
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        editor = (ROOT / "web" / "map_editor.js").read_text(encoding="utf-8")
        self.assertIn('<option value="raster">栅格地图</option>', html)
        self.assertIn('<option value="0">障碍 · 黑色</option>', html)
        self.assertNotIn('data-map-editor="semantic-swatch"', html)
        self.assertIn('s.activePanel=panel', editor)
        self.assertIn('cancelPointPick(false)', editor)
        self.assertIn('ui["show-grid"].checked=true', editor)
        self.assertIn('ui["show-semantic"].checked=true', editor)
        self.assertIn('ui["show-points"].checked=true', editor)
        self.assertIn('if(ui["show-grid"].checked)ctx.drawImage(s.raster', editor)
        self.assertIn('selectedPointId', editor)
        self.assertIn('data-select-point', editor)
        self.assertIn('point.type==="elevator"||point.type==="elevator_inside"?"#a78bfa"', editor)
        self.assertIn('ctx.fillStyle="#0f172a"', editor)
        self.assertIn('startPointPick("add")', editor)
        self.assertIn('replace(/[^a-z0-9_-]+/g,"_")', editor)
        self.assertIn('replace(/^_+|_+$/g,"")', editor)
        self.assertIn('||"point"', editor)
        self.assertIn('s.points.some((point)=>point.id===id)', editor)
        self.assertIn('return id.slice(0,64)', editor)
        self.assertIn('data-repick-point', editor)
        self.assertIn('s.pointPickStep===0', editor)
        self.assertIn('s.moveSnapshotTaken', editor)
        self.assertIn('const step=.5/Number(s.meta.resolution)', editor)
        self.assertIn('function saveLayers()', editor)
        self.assertIn('function savePoints()', editor)
        self.assertIn('function undoScope(scope)', editor)
        self.assertIn('undoScope(s.activePanel==="points"?"points":"layers")', editor)


if __name__ == "__main__":
    unittest.main()
