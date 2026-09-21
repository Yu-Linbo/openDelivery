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
import ros_tf_bridge  # noqa: E402


class WebMonitorFeatureTest(unittest.TestCase):
    def tearDown(self):
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

    @mock.patch.object(motion, "_send_teleop_command", return_value={"ok": True})
    def test_stale_start_cannot_override_newer_stop(self, send_teleop):
        with mock.patch.object(motion, "_send_teleop_command", send_teleop):
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
        send_teleop.assert_called_once_with("robot2", 0.0, 0.0, active=False)

    @mock.patch.object(motion, "_ensure_teleop_watchdog")
    @mock.patch.object(motion, "_send_teleop_command", return_value={"ok": True})
    def test_active_teleop_sets_a_short_lease(
        self, send_teleop, ensure_watchdog
    ):
        result = motion.set_teleop_velocity(
            "robot2", 0.2, 0, active=True, confirmed=True,
            session_id="browser", sequence=1,
        )
        self.assertTrue(result["active"])
        self.assertLessEqual(result["lease_sec"], 1.0)
        self.assertIn("robot2", motion._TELEOP_LEASE_DEADLINE)
        send_teleop.assert_called_once_with("robot2", 0.2, 0.0, active=True)
        ensure_watchdog.assert_called_once()

    def test_expired_teleop_stops_publisher_and_publishes_zero(self):
        motion._TELEOP_STATE["robot2"] = ("browser", 0.2, 0.0)
        motion._TELEOP_LEASE_DEADLINE["robot2"] = 10.0
        with mock.patch.object(
            motion, "_send_teleop_command", return_value={"ok": True}
        ) as send_teleop:
            expired = motion._expire_teleop_leases(now=10.1)
        self.assertEqual(expired, ["robot2"])
        self.assertNotIn("robot2", motion._TELEOP_LEASE_DEADLINE)
        send_teleop.assert_called_once_with("robot2", 0.0, 0.0, active=False)

    def test_teleop_uses_bridge_queue(self):
        with mock.patch(
            "ros_command_queue.enqueue_command_and_wait", return_value={"ok": True}
        ) as enqueue:
            motion._send_teleop_command("robot2", 0.2, 0.0, active=True)
        command = enqueue.call_args.args[0]
        self.assertEqual(command["type"], "teleop")
        self.assertEqual(command["robot_id"], "robot2")
        self.assertTrue(command["active"])

    def test_bridge_routes_teleop_to_joy_input_and_zeroes_on_stop(self):
        bridge = mock.Mock()
        bridge._teleop_state = {}
        bridge._teleop_desired_status = {}
        bridge._ensure_teleop_interfaces = mock.Mock()
        bridge._publish_teleop_twist = mock.Mock()
        with mock.patch.object(ros_tf_bridge.time, "monotonic", return_value=10.0):
            ros_tf_bridge.OpenDeliveryTfBridgeNode._handle_teleop_command(
                bridge,
                {
                    "robot_id": "robot2",
                    "linear": 0.2,
                    "angular": 0.0,
                    "active": True,
                    "lease_sec": 0.8,
                },
            )
        self.assertEqual(bridge._teleop_desired_status["robot2"], "JOY")
        self.assertEqual(bridge._teleop_state["robot2"]["deadline"], 10.8)
        bridge._publish_teleop_twist.assert_called_once_with("robot2", 0.2, 0.0)

        ros_tf_bridge.OpenDeliveryTfBridgeNode._handle_teleop_command(
            bridge, {"robot_id": "robot2", "active": False}
        )
        self.assertNotIn("robot2", bridge._teleop_state)
        self.assertEqual(bridge._teleop_desired_status["robot2"], "AUTO")
        bridge._publish_teleop_twist.assert_called_with("robot2", 0.0, 0.0)

        with self.assertRaises(ValueError):
            ros_tf_bridge.OpenDeliveryTfBridgeNode._handle_teleop_command(
                bridge,
                {"robot_id": "robot2", "linear": 1.3, "active": True},
            )

    def test_bridge_teleop_lease_expiry_stops_and_restores_auto(self):
        bridge = mock.Mock()
        bridge._teleop_state = {
            "robot2": {"linear": 0.2, "angular": 0.0, "deadline": 10.0}
        }
        bridge._teleop_desired_status = {"robot2": "JOY"}
        bridge._publish_teleop_twist = mock.Mock()
        bridge._request_teleop_control_status = mock.Mock()
        with mock.patch.object(ros_tf_bridge.time, "monotonic", return_value=10.1):
            ros_tf_bridge.OpenDeliveryTfBridgeNode._tick_teleop(bridge)
        self.assertNotIn("robot2", bridge._teleop_state)
        self.assertEqual(bridge._teleop_desired_status["robot2"], "AUTO")
        bridge._publish_teleop_twist.assert_called_once_with("robot2", 0.0, 0.0)
        bridge._request_teleop_control_status.assert_called_once_with("robot2", "AUTO")

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
        self.assertIn('id="openclaw-chat-trigger"', html)
        self.assertIn('id="openclaw-chat-panel"', html)
        self.assertIn('id="openclaw-chat-form"', html)
        self.assertIn('id="openclaw-admin-link"', html)
        self.assertIn("initOpenClawChat", js)
        self.assertIn("openDelivery_openclaw_history_v1", js)
        self.assertIn("history.slice(-50)", js)
        self.assertIn("收到，正在执行中。", js)
        self.assertIn("task_progress", js)
        self.assertIn("/api/assistant/chat", js)
        self.assertIn('path == "/api/assistant/chat"', server_py)
        self.assertIn(".openclaw-chat-trigger", css)
        self.assertGreater(html.index('id="standalone-map-editor"'), html.index('class="monitor-teleop-rail"'))
        self.assertIn('data-teleop="forward"', html)
        self.assertIn("syncOnlineRobotSelect", js)
        self.assertIn("syncGazeboModelSelect", js)
        self.assertIn("collectOnlineRobotOptions", js)
        self.assertIn('mergePresenceRows().filter((row) => row.online)', js)
        self.assertIn('<select id="gazebo-model-name" disabled>', html)
        self.assertNotIn('<input id="gazebo-model-name"', html)
        self.assertIn('moveButton.disabled = true', js)
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
        self.assertIn("teleopPointerId", js)
        self.assertNotIn('["pointerup", "pointercancel", "lostpointercapture"]', js)
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
        self.assertNotIn("content--gazebo", js)
        self.assertNotIn(".content.content--gazebo", css)
        self.assertIn(
            "grid-template-columns: minmax(0, 1.5fr) minmax(320px, 420px)",
            css,
        )
        self.assertIn("grid-template-columns: minmax(0, 1fr) 220px", css)
        self.assertIn(".gazebo-panel--move {\n  position: sticky;", css)
        gazebo_section = html[html.index('id="view-gazebo"'):]
        monitor_section = html[html.index('id="view-monitor"'):html.index('id="view-settings"')]
        self.assertIn('class="view-header"', gazebo_section)
        self.assertNotIn("gazebo-view-header", gazebo_section)
        self.assertNotIn("gazebo-view-header", monitor_section)

    def test_fresh_teleop_preempts_navigation_without_heartbeat_race(self):
        source = (
            ROOT
            / "src"
            / "driver"
            / "chassis_state_machine"
            / "src"
            / "chassis_state_machine_node.cpp"
        ).read_text(encoding="utf-8")
        teleop_branch = 'if (fresh(advance_received_, advance_stamp_, current))'
        navigation_branch = 'control_status_ == "AUTO" && fresh(navigation_received_'
        self.assertIn(teleop_branch, source)
        self.assertIn(navigation_branch, source)
        self.assertLess(source.index(teleop_branch), source.index(navigation_branch))

    def test_web_console_supports_chinese_and_english(self):
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        i18n = (ROOT / "web" / "i18n.js").read_text(encoding="utf-8")
        css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")

        self.assertEqual(html.count('id="language-select"'), 1)
        self.assertIn('<option value="zh-CN">中文</option>', html)
        self.assertIn('<option value="en">English</option>', html)
        self.assertIn('src="./i18n.js?', html)
        self.assertLess(html.index('src="./i18n.js?'), html.index('src="./app.js?'))
        self.assertIn('const DEFAULT_LOCALE = "zh-CN"', i18n)
        self.assertIn('openDelivery_language_v1', i18n)
        self.assertIn('document.documentElement.lang = locale', i18n)
        self.assertIn('document.title = locale === DEFAULT_LOCALE', i18n)
        self.assertIn('MutationObserver', i18n)
        self.assertIn('wrapCanvasText("fillText")', i18n)
        self.assertIn('window.OpenDeliveryI18n', i18n)
        self.assertIn('.language-picker', css)

    def test_log_bag_player_is_offline_and_complete(self):
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        js = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
        css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
        server_py = (ROOT / "backend" / "server.py").read_text(encoding="utf-8")
        replay_py = (ROOT / "backend" / "bag_replay.py").read_text(encoding="utf-8")
        recorder_cpp = (ROOT / "src" / "system" / "log_bag" / "src" / "robot_log_recorder.cpp").read_text(encoding="utf-8")
        recording_topics_h = (
            ROOT / "src" / "system" / "log_bag" / "include" / "log_bag" / "recording_topics.hpp"
        ).read_text(encoding="utf-8")
        log_bag_launch = (
            ROOT / "params" / "launch" / "log_bag" / "log_bag.launch.py"
        ).read_text(encoding="utf-8")
        startup_launch = (
            ROOT / "params" / "launch" / "system" / "startup.launch.py"
        ).read_text(encoding="utf-8")
        sim_bringup = (
            ROOT / "src" / "system" / "system" / "scripts" / "sim_bringup.sh"
        ).read_text(encoding="utf-8")

        for element_id in (
            "log-bag-select-all",
            "btn-play-log-bag",
            "btn-delete-log-bag",
            "bag-replay-dialog",
            "bag-replay-canvas",
            "bag-replay-progress",
            "bag-replay-speed",
            "bag-replay-semantic-toggle",
            "bag-replay-points-toggle",
            "bag-replay-robot-status",
            "bag-replay-task-id",
            "bag-replay-current-bag",
            "bag-replay-front-camera",
            "bag-replay-front-down-camera",
            "bag-replay-map-follow",
        ):
            self.assertIn('id="' + element_id + '"', html)
        self.assertIn('API_BASE_URL + "/api/log_bag/replay"', js)
        self.assertIn("/api/log_bag/delete", js)
        self.assertIn('path == "/api/log_bag/delete"', server_py)
        self.assertIn("selectedLogBagsForReplay", js)
        self.assertIn("toggleAllLogBags", js)
        self.assertIn("updateLogBagSelectAllState", js)
        self.assertIn("MAX_RENDERED_LOG_BAGS", js)
        self.assertIn('self.send_header("Content-Encoding", "gzip")', server_py)
        self.assertIn("JSON.stringify({ bags:", js)
        self.assertIn("bagReplaySegmentAt", js)
        self.assertNotIn("selectedLogBagIndices.size !== 1", js)
        self.assertIn("requestAnimationFrame(tickBagReplay)", js)
        self.assertIn("loadBagReplayMap", js)
        self.assertIn("bagReplayCameraAt", js)
        self.assertIn("updateBagReplayCamera", js)
        self.assertIn('data.get("bags")', server_py)
        self.assertIn("bag_replay.merge_replays(replays)", server_py)
        self.assertIn(".bag-replay-state", css)
        self.assertIn(".bag-replay-cameras", css)
        self.assertIn('path == "/api/log_bag/replay"', server_py)
        self.assertIn('out["available_maps"] = list_floors()', server_py)
        self.assertIn("?mode=ro", replay_py)
        self.assertIn("PRAGMA query_only=ON", replay_py)
        self.assertNotIn("import rclpy", replay_py)
        self.assertNotIn("subprocess", replay_py)
        self.assertIn("/front_camera/image_raw", recording_topics_h)
        self.assertIn("/front_down_camera/image_raw", recording_topics_h)
        self.assertIn('"/tf"', recording_topics_h)
        self.assertIn('"/scan_2d"', recording_topics_h)
        self.assertNotIn('"/rosout"', recording_topics_h)
        self.assertNotIn("local_costmap", recording_topics_h)
        self.assertIn('"sensor_msgs/msg/Image": 90', replay_py)
        self.assertNotIn('id="bag-replay-map-select"', html)
        self.assertIn("formatLogBagTimestamp", js)
        self.assertIn("formatLogBagReason", js)
        self.assertIn('timeZone: "Asia/Shanghai"', js)
        self.assertIn(" CST`;", js)
        self.assertIn("bagReplayMapAt", js)
        self.assertIn("bagReplayPathAt", js)
        self.assertIn("bagReplaySegmentsCompatible", js)
        self.assertIn("bagReplayRecordedMapAt", js)
        self.assertIn('endsWith("/received_global_plan")', js)
        self.assertIn("syncBagReplayMapToCurrentTime", js)
        self.assertIn("timeline.maps", js)
        self.assertIn('const stride = Math.max(1, Math.ceil(poses.length / 1500));', js)
        self.assertIn('coordinates === "map"', js)
        self.assertIn("setBagReplayPlaying(duration > 0)", js)
        self.assertIn('"maps": map_changes', replay_py)
        self.assertIn('"initial_map_name": initial_map_name', replay_py)
        self.assertIn("width: auto", css)
        self.assertIn("height: auto", css)
        self.assertIn("aspect-ratio: auto", css)
        self.assertIn("append_unique(current_tags_, task_id)", recorder_cpp)
        self.assertIn("write_bag_tags_marker(current_bag_path_, current_tags_)", recorder_cpp)
        self.assertIn("log_bag::cst_iso8601", recorder_cpp)
        self.assertNotIn("task_boundary_reason_", recorder_cpp)
        self.assertNotIn('stop_current_bag("task_', recorder_cpp)
        self.assertIn("sqlite_logical_bag_size", recorder_cpp)
        self.assertIn("find_custom_msgs_prefix", recorder_cpp)
        self.assertIn("sqlite_topic_message_count", recorder_cpp)
        self.assertNotIn("continuing with recorder sidecar fallback", recorder_cpp)
        self.assertIn("restart_unhealthy_bag", recorder_cpp)
        self.assertIn("stalled_robot_status", recorder_cpp)
        self.assertIn("robot_status heartbeat stalled", recorder_cpp)
        self.assertIn("stop_ros_gazebo_runtime", (ROOT / "start_web_stack.sh").read_text(encoding="utf-8"))
        self.assertIn("50U * 1024U * 1024U", recorder_cpp)
        self.assertIn("1024U * 1024U * 1024U", recorder_cpp)
        self.assertIn("500U * 1024U * 1024U", recorder_cpp)
        self.assertIn("storage cap removed oldest bag", recorder_cpp)
        self.assertIn("recorder_->active() && current_bag_health_verified_ && prune_pending_", recorder_cpp)
        self.assertIn("kStorageCheckInterval", recorder_cpp)
        self.assertIn("active_bytes", recorder_cpp)
        for launcher in (log_bag_launch, startup_launch):
            self.assertIn("respawn=True", launcher)
            self.assertIn("respawn_delay=5.0", launcher)
            self.assertIn('default_value="52428800"', launcher)
            self.assertIn('default_value="1073741824"', launcher)
            self.assertIn('default_value="524288000"', launcher)
            self.assertIn('"--max-robot-bytes"', launcher)
            self.assertIn('"--prune-target-bytes"', launcher)
        self.assertIn('SIM_BRINGUP_MAX_BAG_BYTES:-52428800', sim_bringup)
        self.assertIn('SIM_BRINGUP_MAX_ROBOT_BYTES:-1073741824', sim_bringup)
        self.assertIn('SIM_BRINGUP_PRUNE_TARGET_BYTES:-524288000', sim_bringup)
        self.assertIn("write_robot_status_sidecar", recorder_cpp)
        self.assertIn("critical_status_topic", recorder_cpp)

    def test_openclaw_chat_panel_prevents_horizontal_overflow(self):
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        js = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
        css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
        self.assertIn('id="openclaw-chat-panel"', html)
        self.assertIn("grid-template-columns: minmax(0, 1fr) auto auto", css)
        self.assertIn("overflow-x: hidden", css)
        self.assertIn("grid-template-columns: minmax(0, 1fr) auto", css)
        self.assertIn("Array.isArray(payload?.items)", js)

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

    def test_topdown_camera_uses_latest_pose_and_jpeg_frames(self):
        js = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        css = (ROOT / "web" / "styles.css").read_text(encoding="utf-8")
        server_py = (ROOT / "backend" / "server.py").read_text(encoding="utf-8")
        bridge_py = (ROOT / "backend" / "ros_tf_bridge.py").read_text(encoding="utf-8")
        world = (
            ROOT / "src" / "simulate" / "simulate" / "worlds" / "drawn_model.world"
        ).read_text(encoding="utf-8")

        self.assertIn("CAMERA_DRIVE_SEND_INTERVAL_MS = 66", js)
        self.assertIn("cameraDriveRequestInFlight", js)
        self.assertIn("pendingCameraDriveBody", js)
        self.assertIn("cameraScreenDriveVector(screenRight, screenUp)", js)
        self.assertIn(
            "rotateVectorByQuaternion({ x: 0, y: -1, z: 0 }, orientation)", js
        )
        self.assertIn(
            "rotateVectorByQuaternion({ x: 0, y: 0, z: 1 }, orientation)", js
        )
        self.assertIn("revisionAtStart !== cameraPoseRevision", js)
        self.assertIn("handleTopCameraPointerMove", js)
        self.assertIn("handleTopCameraWheel", js)
        self.assertIn("beginTopCameraPinch", js)
        self.assertIn("cameraPointers.size >= 2", js)
        self.assertIn("TOP_CAMERA_VIEW_SCALE_MAX", js)
        self.assertIn("renderTopCameraLocalView", js)
        self.assertIn("latestTopCameraBitmap = bitmap", js)
        self.assertIn("previousBitmap?.close?.()", js)
        self.assertIn(
            "(topCameraViewCenterX + (canvasPoint.x - 0.5) / topCameraViewScale)",
            js,
        )
        gesture_js = js[
            js.index("function beginTopCameraPinch"):
            js.index("async function refreshTopCameraFrame")
        ]
        self.assertNotIn("cameraModel.", gesture_js)
        self.assertNotIn("cameraPoseRevision", gesture_js)
        self.assertNotIn("postTopdownCameraPoseQuiet", gesture_js)
        camera_control_js = js[
            js.index("function nudgeTopdownCamera"):
            js.index("const CAMERA_DRIVE_SEND_INTERVAL_MS")
        ]
        self.assertIn("cameraModel.x +=", camera_control_js)
        self.assertIn("cameraModel.z =", camera_control_js)
        self.assertIn("postTopdownCameraPoseQuiet(true)", camera_control_js)
        self.assertIn("nudgeTopdownCamera(screenRight, screenUp)", js)
        self.assertIn("heldSec * 1.2", js)
        self.assertIn("btnGazeboCamUpLeft", js)
        self.assertIn('type="range" step="0.5" min="0.5" max="16"', html)
        self.assertIn("拖拽、滚轮和双指手势只调整本地图片视图", html)
        self.assertIn("不会移动俯瞰相机", html)
        self.assertIn("touch-action: none", css)
        self.assertIn("/api/gazebo/top_camera.jpg?frame_seq=", js)
        self.assertIn("frameSeq !== lastRenderedTopCameraFrameSeq", js)
        self.assertNotIn("topCameraImageSkip", js)
        self.assertIn('path == "/api/gazebo/top_camera.jpg"', server_py)
        self.assertIn("get_topdown_image_status()", server_py)
        self.assertIn("depth=1", bridge_py)
        self.assertIn('format="JPEG", quality=82', bridge_py)
        self.assertIn("<update_rate>15</update_rate>", world)

if __name__ == "__main__":
    unittest.main()
