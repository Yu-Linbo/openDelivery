#!/usr/bin/env python3
"""
Per-robot stack orchestration for the web console.

**Component model** (see ``_component_catalog``; one row per logical slice):

- **仿真上线**：仅启动一个托管进程（id = ``robot_id``），运行 ``sim_bringup.sh``；脚本内依次拉起
  Gazebo/仿真、heartbeat、manager（health_monitor + task_manager）、定位、（按磁盘缓存可选）建图、导航，
  并对 heartbeat / slam lifecycle 做 configure/activate。
- **仿真离线**：向 ``/<robot>/set_heartbeat_params`` 写入 ``robot_status=SHUTDOWN``（``RobotStatus.msg`` 中
  ``ROBOT_STATUS_SHUTDOWN=4``），供各节点订阅 ``/<robot>/robot_status`` 后自行收尾；再对 navigation / heartbeat
  做 best-effort lifecycle shutdown，最后 ``pause`` 各托管项（已废弃 ``sim_shutdown.sh``）。
- ``heartbeat`` / ``navigation``：ROS 2 lifecycle；``localization`` / ``mapping``：脚本内 ``ros2 launch``，
  Web 侧另注册同名托管项（不二次 start）以便展示与 start/pause。
- **建图**：是否启动由 ``sim_bringup.sh`` 读取 ``backend/data/robot_status_last.json`` 决定（与历史
  ``robot_status == mapping`` 一致）；未自动建图时可在 ROS 页对 mapping 点 ``start``。
"""

import os
import re
import shlex
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


LIFECYCLE_TRANSITIONS = {"configure", "activate", "deactivate", "cleanup", "shutdown"}
LIFECYCLE_STATE_RE = re.compile(r"state:\s*\[\s*(\d+)\s*\]\s*([^\r\n]+)")


def _persisted_auto_mapping(last: Optional[Dict[str, Any]]) -> bool:
    if not last:
        return False
    if str(last.get("task_status") or "").strip().lower() == "mapping":
        return True
    return str(last.get("robot_status") or "").strip().lower() == "mapping"


class RobotLifecycleOrchestrator:
    """Backend orchestration layer for lifecycle-style robot stack control."""

    def __init__(self, root_dir: Path, ros_node_manager):
        self._root_dir = Path(root_dir)
        self._ros_node_manager = ros_node_manager
        self._lock = threading.Lock()
        self._state = {}
        self._last_error = ""

    def _bash_prefix(self):
        ros_distro = (os.environ.get("ROS_DISTRO") or "foxy").strip()
        install_setup = self._root_dir / "install" / "setup.bash"
        install_src = f'source "{install_setup}"' if install_setup.is_file() else "true"
        return (
            f'set -eo pipefail; source "/opt/ros/{ros_distro}/setup.bash"; '
            f'cd "{self._root_dir}" && {install_src}; '
        )

    def _run_shell(self, cmd: str, timeout: float = 10.0):
        full_cmd = self._bash_prefix() + cmd
        return subprocess.run(
            ["bash", "-lc", full_cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
            env=os.environ.copy(),
        )

    def _component_catalog(self, robot_id: str):
        rid = str(robot_id or "").strip()
        slam_ns = f"/{rid}/slam_bringup"
        return [
            {
                "id": "simulate",
                "label_zh": "仿真 / Gazebo",
                "type": "process_wrapper",
                "managed_node_id": rid,
                "expected_node": "/gazebo",
                "transitions": ["start", "shutdown"],
            },
            {
                "id": "heartbeat",
                "label_zh": "心跳",
                "type": "lifecycle",
                "node": f"/{rid}/heartbeat",
                "transitions": ["configure", "activate", "deactivate", "cleanup", "shutdown"],
            },
            {
                "id": "localization",
                "label_zh": "定位",
                "type": "process_wrapper",
                "managed_node_id": f"localization_{rid}",
                "expected_node": f"{slam_ns}/localization",
                "transitions": ["start", "shutdown"],
            },
            {
                "id": "mapping",
                "label_zh": "建图",
                "type": "process_wrapper",
                "managed_node_id": f"mapping_{rid}",
                "expected_node": f"{slam_ns}/mapping",
                "transitions": ["start", "shutdown"],
            },
            {
                "id": "navigation",
                "label_zh": "导航",
                "type": "group",
                "node": f"/{rid}/lifecycle_manager_navigation",
                "transitions": ["configure", "activate", "deactivate", "cleanup", "shutdown"],
            },
        ]

    def _robot_from_state(self):
        with self._lock:
            ids = set(self._state.keys())
        try:
            import ros_robot_status_store

            for item in ros_robot_status_store.list_all_last_status():
                rid = str((item or {}).get("robot_id") or "").strip()
                if rid:
                    ids.add(rid)
        except Exception:
            pass
        try:
            from ros_tf_bridge import _load_robot_specs

            for spec in _load_robot_specs():
                rid = str((spec or {}).get("id") or "").strip()
                if rid:
                    ids.add(rid)
        except Exception:
            pass
        return sorted(ids)

    def _display_name_for_robot(
        self, rid: str, persisted: Optional[Dict[str, Any]]
    ) -> str:
        if persisted:
            name = str(persisted.get("robot_name") or "").strip()
            if name:
                return name
        try:
            from ros_tf_bridge import _load_robot_specs

            for spec in _load_robot_specs():
                if str((spec or {}).get("id") or "").strip() == rid:
                    n = str((spec or {}).get("name") or "").strip()
                    if n:
                        return n
        except Exception:
            pass
        return rid

    def _ensure_robot(self, robot_id: str):
        rid = str(robot_id or "").strip()
        if not rid:
            raise ValueError("robot_id is required")
        with self._lock:
            self._state.setdefault(
                rid,
                {
                    "simulate_started": False,
                    "heartbeat_started": False,
                    "localization_started": False,
                    "mapping_started": False,
                    "navigation_started": False,
                    "sim_mode": "sim",
                    "updated_at": time.time(),
                },
            )
        return rid

    def _lifecycle_get(self, node_name: str):
        # Keep status endpoint responsive even when lifecycle service is missing.
        try:
            proc = self._run_shell(f"ros2 lifecycle get {node_name}", timeout=1.2)
        except subprocess.TimeoutExpired:
            return {"available": False, "state": "missing", "raw": "timeout"}
        if proc.returncode != 0:
            return {"available": False, "state": "missing", "raw": (proc.stderr or proc.stdout or "").strip()}
        txt = (proc.stdout or "").strip()
        m = LIFECYCLE_STATE_RE.search(txt)
        if not m:
            return {"available": True, "state": "unknown", "raw": txt}
        return {"available": True, "state": m.group(2).strip(), "id": int(m.group(1)), "raw": txt}

    def _lifecycle_set(self, node_name: str, transition: str):
        t = str(transition or "").strip().lower()
        if t not in LIFECYCLE_TRANSITIONS:
            raise ValueError("transition must be configure|activate|deactivate|cleanup|shutdown")
        proc = self._run_shell(f"ros2 lifecycle set {node_name} {t}", timeout=6.0)
        if proc.returncode != 0:
            msg = (proc.stderr or proc.stdout or "").strip() or "lifecycle set failed"
            raise RuntimeError(msg)
        return {"ok": True, "raw": (proc.stdout or "").strip()}

    def _lifecycle_try(self, node_name: str, transition: str) -> None:
        try:
            self._lifecycle_set(node_name, transition)
        except Exception:
            pass

    def _sim_bringup_script_path(self) -> Path:
        """Source tree (dev) or colcon install prefix."""
        candidates = [
            self._root_dir
            / "src"
            / "system"
            / "system"
            / "scripts"
            / "sim_bringup.sh",
            self._root_dir
            / "install"
            / "system"
            / "lib"
            / "system"
            / "sim_bringup.sh",
        ]
        for p in candidates:
            if p.is_file():
                return p
        raise RuntimeError(
            "sim_bringup.sh not found; build/install ROS package system."
        )

    def _signal_shutdown_via_heartbeat(self, rid: str) -> None:
        """Publish SHUTDOWN by updating heartbeat params (``robot_status=4``)."""
        # SetHeartbeatParams: empty strings leave name/map; 255 leaves task_status; rate_hz<=0 unchanged.
        yaml_req = '{robot_name: "", current_map: "", robot_status: 4, task_status: 255, rate_hz: 0.0}'
        cmd = (
            f"ros2 service call /{rid}/set_heartbeat_params "
            f"custom_msgs_srvs/srv/SetHeartbeatParams {shlex.quote(yaml_req)}"
        )
        try:
            proc = self._run_shell(cmd, timeout=15.0)
            if proc.returncode != 0:
                err = (proc.stderr or proc.stdout or "").strip()
                if err:
                    # Non-fatal: stack may already be down.
                    pass
        except Exception:
            pass

    def _start_if_needed(
        self,
        node_id: str,
        cmd: str,
        *,
        stop_cmd: Optional[str] = None,
        match: Optional[str] = None,
        note: str = "lifecycle orchestrator managed",
        autostart: bool = True,
    ):
        status = self._ros_node_manager.status()
        managed = status.get("managed_nodes") or []
        cur = next((m for m in managed if m.get("id") == node_id), None)
        if cur and cur.get("running"):
            return
        if not any((m.get("id") == node_id) for m in managed):
            with self._ros_node_manager._lock:
                self._ros_node_manager._managed_nodes.append(
                    {
                        "id": node_id,
                        "name": node_id,
                        "persistent": True,
                        "start_cmd": cmd,
                        "stop_cmd": stop_cmd or f"pkill -f '{cmd}' || true",
                        "match": match or (cmd.split()[0] if cmd else ""),
                        "note": note,
                    }
                )
        if autostart:
            self._ros_node_manager.control(node_id, "start")

    def startup_selected_robot(self, robot_id: str, sim_mode: str = "sim"):
        rid = self._ensure_robot(robot_id)
        sim_mode = (sim_mode or "sim").strip() or "sim"
        last: Dict[str, Any] = {}
        try:
            import ros_robot_status_store

            last = ros_robot_status_store.get_last_status(rid) or {}
        except Exception:
            last = {}
        last_rs = str(last.get("robot_status") or "").strip().lower()
        auto_mapping = _persisted_auto_mapping(last)
        try:
            # 整栈仅在 sim_bringup.sh 内拉起；此处只启动该脚本（托管 id = robot id）
            script = self._sim_bringup_script_path()
            root_q = shlex.quote(str(self._root_dir.resolve()))
            script_q = shlex.quote(str(script.resolve()))
            rid_q = shlex.quote(rid)
            sim_q = shlex.quote(sim_mode)
            sim_cmd = f"SIM_MODE={sim_q} OPEN_DELIVERY_ROOT={root_q} bash {script_q} {rid_q}"
            stop_all = (
                f"pkill -f 'ros2 launch nav_bringup stack.launch.py.*robot_name:={rid}' || true; "
                f"pkill -f 'ros2 launch slam_bringup mapping.launch.py.*robot_name:={rid}' || true; "
                f"pkill -f 'ros2 launch slam_bringup localization.launch.py.*robot_name:={rid}' || true; "
                f"pkill -f 'ros2 launch heartbeat heartbeat.launch.py.*namespace:={rid}' || true; "
                f"pkill -f 'ros2 launch simulate simulate.launch.py.*robot_name:={rid}' || true"
            )
            self._start_if_needed(
                rid,
                sim_cmd,
                stop_cmd=stop_all,
                match=f"namespace:={rid}",
                note="full sim stack via sim_bringup.sh (managed id = robot id)",
            )
            # 脚本已拉起下列进程；仅注册托管项（不二次 start），便于 Web 用 ps 匹配显示与 start/pause
            loc_cmd = (
                f"ros2 launch slam_bringup localization.launch.py "
                f"robot_name:={rid} namespace:={rid}"
            )
            self._start_if_needed(
                f"localization_{rid}",
                loc_cmd,
                stop_cmd=(
                    f"pkill -f 'ros2 launch slam_bringup localization.launch.py "
                    f"robot_name:={rid} namespace:={rid}' || true"
                ),
                match=f"localization.launch.py robot_name:={rid}",
                autostart=False,
                note="registered for status (started by sim_bringup.sh)",
            )
            map_cmd = (
                f"ros2 launch slam_bringup mapping.launch.py "
                f"robot_name:={rid} namespace:={rid}"
            )
            self._start_if_needed(
                f"mapping_{rid}",
                map_cmd,
                stop_cmd=(
                    f"pkill -f 'ros2 launch slam_bringup mapping.launch.py "
                    f"robot_name:={rid} namespace:={rid}' || true"
                ),
                match=f"mapping.launch.py robot_name:={rid}",
                autostart=False,
                note="registered for status (started by sim_bringup.sh if auto)",
            )
            nav_cmd = (
                f"ros2 launch nav_bringup stack.launch.py "
                f"robot_name:={rid} grid_mode:=localize autostart:=false"
            )
            self._start_if_needed(
                f"navigation_{rid}",
                nav_cmd,
                stop_cmd=(
                    f"pkill -f 'ros2 launch nav_bringup stack.launch.py.*robot_name:={rid}' || true"
                ),
                match=f"stack.launch.py robot_name:={rid}",
                autostart=False,
                note="registered for status (started by sim_bringup.sh)",
            )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            raise

        with self._lock:
            self._state[rid].update(
                {
                    "simulate_started": True,
                    "heartbeat_started": True,
                    "localization_started": True,
                    "mapping_started": auto_mapping,
                    "navigation_started": True,
                    "localization_state": "inactive",
                    "mapping_state": "unconfigured" if auto_mapping else None,
                    "sim_mode": sim_mode,
                    "last_persisted_robot_status": last_rs or None,
                    "updated_at": time.time(),
                }
            )
            self._last_error = ""
        return self.status(rid)

    def shutdown_selected_robot(self, robot_id: str):
        """Stop per-robot orchestrated stack (including simulate launch registered under robot id)."""
        rid = self._ensure_robot(robot_id)
        hb = f"/{rid}/heartbeat"
        nav_mgr = f"/{rid}/lifecycle_manager_navigation"
        # Let subscribers see SHUTDOWN on /{rid}/robot_status before tearing down lifecycle nodes.
        self._signal_shutdown_via_heartbeat(rid)
        time.sleep(0.5)
        for t in ("deactivate", "cleanup", "shutdown"):
            self._lifecycle_try(nav_mgr, t)
        for t in ("deactivate", "cleanup", "shutdown"):
            self._lifecycle_try(hb, t)
        for node_id in (
            f"navigation_{rid}",
            f"mapping_{rid}",
            f"localization_{rid}",
            f"heartbeat_{rid}",
            rid,
        ):
            try:
                self._ros_node_manager.control(node_id, "pause")
            except Exception:
                pass
        with self._lock:
            st = self._state.setdefault(rid, {})
            st.pop("localization_state", None)
            st.pop("mapping_state", None)
            st["updated_at"] = time.time()
            self._last_error = ""
        return self.status(rid)

    def transition(self, robot_id: str, component_id: str, transition: str):
        rid = self._ensure_robot(robot_id)
        cid = str(component_id or "").strip()
        if not cid:
            raise ValueError("component is required")
        if cid == "simulate":
            action = "start" if transition in ("configure", "activate", "start") else "pause"
            self._ros_node_manager.control(rid, action)
            return self.status(rid)
        if cid in ("localization", "mapping"):
            node_id = f"{cid}_{rid}"
            action = "start" if transition in ("configure", "activate", "start") else "pause"
            self._ros_node_manager.control(node_id, action)
            with self._lock:
                self._state.setdefault(rid, {})
                if cid == "localization":
                    self._state[rid]["localization_state"] = "active" if action == "start" else "unconfigured"
                if cid == "mapping":
                    self._state[rid]["mapping_state"] = "inactive" if action == "start" else "unconfigured"
            return self.status(rid)
        comp = next((c for c in self._component_catalog(rid) if c["id"] == cid), None)
        if not comp:
            raise ValueError(f"unknown component: {cid}")
        node = comp.get("node")
        if not node:
            raise ValueError(f"component {cid} is not lifecycle-addressable")
        self._lifecycle_set(node, transition)
        return self.status(rid)

    def status(self, robot_id=None):
        robot_ids = [self._ensure_robot(robot_id)] if robot_id else self._robot_from_state()
        if not robot_ids:
            robot_ids = []
        discovered = self._ros_node_manager.list_ros_nodes()

        def _norm_ros_name(n: Optional[str]) -> str:
            s = (n or "").strip()
            if not s:
                return ""
            return s if s.startswith("/") else f"/{s}"

        discovered_names = {
            _norm_ros_name(str(n.get("name"))) for n in discovered if isinstance(n, dict)
        }
        discovered_names.discard("")
        managed = self._ros_node_manager.status().get("managed_nodes") or []
        managed_by_id = {str(m.get("id")): m for m in managed if isinstance(m, dict)}
        robots = []
        for rid in robot_ids:
            self._ensure_robot(rid)
            persisted: Optional[Dict[str, Any]] = None
            try:
                import ros_robot_status_store

                persisted = ros_robot_status_store.get_last_status(rid)
            except Exception:
                persisted = None
            will_auto_mapping = _persisted_auto_mapping(persisted)
            components: List[Dict[str, Any]] = []
            for comp in self._component_catalog(rid):
                node = comp.get("node")
                ctype = comp.get("type")
                running = False
                lifecycle_state = "unknown"
                if ctype == "process_wrapper":
                    mn = managed_by_id.get(comp.get("managed_node_id"))
                    running = bool(mn and mn.get("running"))
                    lifecycle_state = "active" if running else "unconfigured"
                    if comp["id"] in ("localization", "mapping"):
                        with self._lock:
                            virtual = self._state.get(rid, {}).get(f"{comp['id']}_state")
                        if virtual:
                            lifecycle_state = virtual
                elif node:
                    nn = _norm_ros_name(str(node))
                    running = nn in discovered_names
                    if ctype in ("lifecycle", "group"):
                        info = self._lifecycle_get(node)
                        lifecycle_state = info.get("state") or "unknown"
                        if running and lifecycle_state == "missing":
                            lifecycle_state = "active"
                row: Dict[str, Any] = {
                    "id": comp["id"],
                    "label_zh": comp.get("label_zh") or comp["id"],
                    "type": ctype,
                    "node": node or comp.get("expected_node"),
                    "running": running,
                    "lifecycle_state": lifecycle_state,
                    "transitions": comp.get("transitions") or [],
                    "expected": True,
                }
                if comp["id"] == "mapping":
                    row["sim_bringup_autostart_mapping"] = will_auto_mapping
                components.append(row)
            with self._lock:
                online_state = self._state.get(rid, {})
            prefix = f"/{rid}/"
            discovered_under = sorted(
                n for n in discovered_names if n.startswith(prefix) or n == f"/{rid}"
            )
            robots.append(
                {
                    "robot_id": rid,
                    "display_name": self._display_name_for_robot(rid, persisted),
                    "sim_mode": online_state.get("sim_mode", "sim"),
                    "persisted_from_store": persisted,
                    "will_auto_start_mapping_on_sim_bringup": will_auto_mapping,
                    "last_persisted_robot_status_at_bringup": online_state.get(
                        "last_persisted_robot_status"
                    ),
                    "session_mapping_process_started": bool(
                        online_state.get("mapping_started")
                    ),
                    "discovered_nodes": [{"name": n} for n in discovered_under],
                    "components": components,
                }
            )
        with self._lock:
            err = self._last_error
        return {"robots": robots, "last_error": err, "timestamp": time.time()}
