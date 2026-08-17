#!/usr/bin/env python3
"""
Per-robot stack orchestration for the web console.

**Component model** (see ``_component_catalog``; one row per logical slice):

- **仿真上线**：仅启动一个托管进程（id = ``robot_id``），运行 ``sim_bringup.sh``；脚本内依次拉起
  Gazebo/仿真、heartbeat、manager（health_monitor + task_manager）、定位、（按磁盘缓存可选）建图、导航，
  并对 heartbeat / slam lifecycle 做 configure/activate。
- **仿真离线**：向 ``/<robot>/set_heartbeat_params`` 写入 ``robot_status=shutdown``，供各节点订阅
  ``/<robot>/robot_status`` 后自行收尾；再对 navigation / heartbeat 做 best-effort lifecycle shutdown，
  最后 ``pause`` 各托管项（已废弃 ``sim_shutdown.sh``）。
- ``heartbeat`` / ``navigation``：经生命周期接口管理；
  ``slam``：由 ``/<robot>/slam/lifecycle_manager`` 在 ``mapping`` 和 ``localizing`` 两个 Lifecycle 节点之间切换。
"""

import json
import os
import re
import signal
import shlex
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


ROBOT_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
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
        self._infrastructure_lock = threading.Lock()
        self._operation_lock = threading.Lock()
        self._spawn_slots_path = self._root_dir / "backend" / "data" / "sim_spawn_slots.json"
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

    _SPAWN_POSES = (
        (-13.703, 12.825, 0.05, 0.0),
        (-12.703, 12.825, 0.05, 0.0),
        (-13.703, 11.825, 0.05, 0.0),
        (-12.703, 11.825, 0.05, 0.0),
        (-11.703, 12.825, 0.05, 0.0),
        (-13.703, 10.825, 0.05, 0.0),
        (-11.703, 11.825, 0.05, 0.0),
        (-12.703, 10.825, 0.05, 0.0),
        (-10.703, 12.825, 0.05, 0.0),
        (-13.703, 9.825, 0.05, 0.0),
        (-10.703, 11.825, 0.05, 0.0),
        (-11.703, 10.825, 0.05, 0.0),
        (-12.703, 9.825, 0.05, 0.0),
        (-10.703, 10.825, 0.05, 0.0),
        (-11.703, 9.825, 0.05, 0.0),
        (-10.703, 9.825, 0.05, 0.0),
    )

    def _read_spawn_slots(self) -> Dict[str, int]:
        try:
            with open(self._spawn_slots_path, encoding="utf-8") as stream:
                raw = json.load(stream)
        except (FileNotFoundError, OSError, ValueError):
            return {}
        result = {}
        for robot_id, slot in (raw.items() if isinstance(raw, dict) else []):
            if isinstance(robot_id, str) and isinstance(slot, int) and 0 <= slot < len(self._SPAWN_POSES):
                result[robot_id] = slot
        return result

    def _write_spawn_slots(self, slots: Dict[str, int]) -> None:
        self._spawn_slots_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=str(self._spawn_slots_path.parent),
                prefix=f".{self._spawn_slots_path.name}.", suffix=".tmp", delete=False,
            ) as stream:
                json.dump(slots, stream, ensure_ascii=False, indent=2, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
                tmp_path = Path(stream.name)
            os.replace(tmp_path, self._spawn_slots_path)
        finally:
            if tmp_path is not None and tmp_path.exists():
                try:
                    tmp_path.unlink()
                except OSError:
                    pass

    def _spawn_pose_for_robot(self, robot_id: str):
        rid = self._ensure_robot(robot_id)
        with self._infrastructure_lock:
            slots = self._read_spawn_slots()
            slot = slots.get(rid)
            if slot is None:
                used = set(slots.values())
                candidates = range(len(self._SPAWN_POSES)) if rid == "robot2" else range(1, len(self._SPAWN_POSES))
                slot = next((index for index in candidates if index not in used), None)
                if slot is None:
                    raise RuntimeError(
                        f"no free simulation spawn slot; maximum is {len(self._SPAWN_POSES)} robots"
                    )
                slots[rid] = slot
                self._write_spawn_slots(slots)
            return slot, self._SPAWN_POSES[slot]

    def _run_shell(self, cmd: str, timeout: float = 10.0):
        full_cmd = self._bash_prefix() + cmd
        return subprocess.run(
            ["bash", "-lc", full_cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
            env=os.environ.copy(),
        )

    def _gazebo_services_ready(self) -> bool:
        try:
            proc = self._run_shell("ros2 service list", timeout=4.0)
        except Exception:
            return False
        services = set((proc.stdout or "").split()) if proc.returncode == 0 else set()
        return {"/spawn_entity", "/delete_entity"}.issubset(services)

    def _gazebo_transport_ready(self) -> bool:
        """Return whether Gazebo's native transport answers, not merely advertises ROS services."""
        try:
            proc = subprocess.run(
                # drawn_model is part of the shared world and always exists.  A
                # pose query exercises the request/reply path; unsupported `-l`
                # exits successfully even when gzserver is wedged.
                ["gz", "model", "-m", "drawn_model", "-p"],
                cwd=str(self._root_dir),
                capture_output=True,
                text=True,
                timeout=5.0,
                env=os.environ.copy(),
            )
        except (OSError, subprocess.SubprocessError):
            return False
        if proc.returncode != 0:
            return False
        try:
            values = [float(value) for value in (proc.stdout or "").strip().split()[-6:]]
        except ValueError:
            return False
        return len(values) == 6

    def _wait_for_gazebo(self, timeout_sec: float = 60.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                proc = self._run_shell("ros2 service list", timeout=4.0)
            except Exception:
                proc = None
            services = set((proc.stdout or "").split()) if proc and proc.returncode == 0 else set()
            if "/spawn_entity" in services and "/delete_entity" in services:
                return
            time.sleep(0.5)
        raise RuntimeError("Gazebo infrastructure did not expose spawn/delete services")

    def _ensure_simulation_world(self) -> None:
        # ThreadingHTTPServer may receive two sim-up requests concurrently. Keep the
        # host-level Gazebo/Xvfb world a singleton; robot stacks never own it.
        with self._infrastructure_lock:
            world_cmd = (
                    "ros2 launch simulate simulate.launch.py "
                    "robot_name:=simulation_world namespace:=simulation_world "
                    "start_gazebo:=true spawn_robot:=false use_sim_time:=true"
                )
            world_running = self._gazebo_services_ready() and self._gazebo_transport_ready()
            if not world_running:
                self._terminate_stale_world_processes()
            self._start_if_needed(
                "simulation_world", world_cmd,
                    stop_cmd=(
                        "pkill -f 'ros2 launch simulate simulate.launch.py.*"
                        "robot_name:=simulation_world' || true; "
                        "pkill -f 'gzserver .*openDelivery/install/simulate/share/simulate/worlds/"
                        "drawn_model.world' || true"
                    ),
                    match="robot_name:=simulation_world",
                note="shared Gazebo world; independent from every robot stack",
                autostart=not world_running,
                force=not world_running,
            )
            self._wait_for_gazebo()

    def _restart_simulation_world(self) -> None:
        """Recover a Gazebo server whose advertised services no longer answer."""
        with self._infrastructure_lock:
            try:
                self._ros_node_manager.control("simulation_world", "pause")
            except Exception:
                pass
            self._terminate_stale_world_processes()
            time.sleep(1.0)
            world_cmd = (
                "ros2 launch simulate simulate.launch.py "
                "robot_name:=simulation_world namespace:=simulation_world "
                "start_gazebo:=true spawn_robot:=false use_sim_time:=true"
            )
            self._start_if_needed(
                "simulation_world",
                world_cmd,
                stop_cmd=(
                    "pkill -f 'ros2 launch simulate simulate.launch.py.*"
                    "robot_name:=simulation_world' || true; "
                    "pkill -f 'gzserver .*openDelivery/install/simulate/share/simulate/worlds/"
                    "drawn_model.world' || true"
                ),
                match="robot_name:=simulation_world",
                note="shared Gazebo world; automatic recovery after transport failure",
                autostart=True,
                force=True,
            )
            self._wait_for_gazebo()

    def _terminate_stale_world_processes(self) -> None:
        """Terminate only this project's orphaned shared-world launch/gzserver processes."""
        world_path = str(
            self._root_dir
            / "install"
            / "simulate"
            / "share"
            / "simulate"
            / "worlds"
            / "drawn_model.world"
        )
        targets = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                args = entry.joinpath("cmdline").read_bytes().split(b"\0")
                argv = [part.decode(errors="replace") for part in args if part]
            except (OSError, ProcessLookupError):
                continue
            if not argv:
                continue
            is_world_server = Path(argv[0]).name == "gzserver" and world_path in argv
            joined = " ".join(argv)
            is_world_launch = (
                "ros2 launch simulate simulate.launch.py" in joined
                and "robot_name:=simulation_world" in joined
            )
            if is_world_server or is_world_launch:
                targets.append(int(entry.name))
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pid in targets:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    continue
            if sig == signal.SIGTERM:
                time.sleep(1.0)

    def _terminate_stale_robot_processes(self, robot_id: str) -> None:
        """Remove orphaned descendants left after a backend/process-manager restart."""
        rid = self._ensure_robot(robot_id)
        namespace_token = f"__ns:=/{rid}"
        robot_arg = f"robot_name:={rid}"
        bringup_pattern = re.compile(rf"sim_bringup\.sh\s+{re.escape(rid)}(?:\s|$)")
        targets = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                args = entry.joinpath("cmdline").read_bytes().split(b"\0")
                argv = [part.decode(errors="replace") for part in args if part]
            except (OSError, ProcessLookupError):
                continue
            joined = " ".join(argv)
            namespaced = any(
                arg == namespace_token or arg.startswith(namespace_token + "/")
                for arg in argv
            )
            if (
                namespaced
                or robot_arg in argv
                or bringup_pattern.search(joined)
            ):
                targets.append(int(entry.name))
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pid in targets:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    continue
            if sig == signal.SIGTERM:
                time.sleep(1.0)

    def _simulation_entity_present(self, robot_id: str) -> Optional[bool]:
        """Return entity presence from the fresh Web bridge cache, or None if unknown."""
        rid = self._ensure_robot(robot_id)
        try:
            from ros_sensor_store import get_gazebo_models

            snapshot = get_gazebo_models() or {}
            cached_at = float(snapshot.get("_cached_at") or snapshot.get("stamp") or 0.0)
            if not snapshot.get("available") or time.time() - cached_at > 3.0:
                return self._native_simulation_entity_present(rid)
            names = {
                str(item.get("name") or "").strip()
                for item in snapshot.get("models") or []
                if isinstance(item, dict)
            }
            return rid in names
        except Exception:
            return self._native_simulation_entity_present(rid)

    def _native_simulation_entity_present(self, robot_id: str) -> Optional[bool]:
        """Query Gazebo transport directly when the Web model cache is unavailable."""
        rid = self._ensure_robot(robot_id)
        try:
            proc = subprocess.run(
                ["gz", "model", "-m", rid, "-i"],
                cwd=str(self._root_dir),
                capture_output=True,
                text=True,
                timeout=5.0,
                env=os.environ.copy(),
            )
        except (OSError, subprocess.SubprocessError):
            return None
        text = ((proc.stdout or "") + "\n" + (proc.stderr or "")).lower()
        if proc.returncode == 0:
            return True
        if "does not exist" in text or "unable to find" in text or "not found" in text:
            return False
        return None

    def _delete_simulation_entity(self, robot_id: str) -> bool:
        rid = self._ensure_robot(robot_id)
        request = shlex.quote(f"{{name: {rid}}}")
        service_deleted = False
        try:
            proc = self._run_shell(
                f"ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity {request}",
                timeout=15.0,
            )
            text = (proc.stdout or "") + "\n" + (proc.stderr or "")
            service_deleted = proc.returncode == 0 and (
                "success: True" in text or "success=true" in text
            )
        except Exception:
            pass
        # A successful ROS response can still race with Gazebo's actual removal.
        # Verify through native transport and use native deletion as fallback.
        for _ in range(10 if service_deleted else 1):
            present = self._native_simulation_entity_present(rid)
            if present is False:
                return True
            if present is None:
                break
            time.sleep(0.1)
        try:
            native = subprocess.run(
                ["gz", "model", "-m", rid, "-d"],
                cwd=str(self._root_dir),
                capture_output=True,
                text=True,
                timeout=8.0,
                env=os.environ.copy(),
            )
        except (OSError, subprocess.SubprocessError):
            return service_deleted
        if native.returncode != 0:
            return service_deleted
        for _ in range(20):
            present = self._native_simulation_entity_present(rid)
            if present is False:
                return True
            if present is None:
                return True
            time.sleep(0.1)
        return False

    def _stack_lifecycle_transition(self, robot_id: str, node_name: str, transition: str):
        rid = str(robot_id or "").strip()
        nn = str(node_name or "").strip()
        tr = str(transition or "").strip().lower()
        if not rid or not nn or not tr:
            raise ValueError("robot_id, node_name and transition are required")
        cmd = (
            f"ros2 service call /{rid}/slam/set_stack_lifecycle_transition "
            f"custom_msgs_srvs/srv/SetStackLifecycleTransition "
            f"'{{node_name: \"{nn}\", transition: \"{tr}\"}}'"
        )
        proc = self._run_shell(cmd, timeout=20.0)
        if proc.returncode != 0:
            detail = (proc.stderr or proc.stdout or "").strip()
            raise RuntimeError(detail or f"stack lifecycle transition failed: {nn} {tr}")

    def _component_catalog(self, robot_id: str):
        rid = str(robot_id or "").strip()
        return [
            {
                "id": "simulate",
                "label_zh": "仿真 / Gazebo",
                "type": "process_wrapper",
                "managed_node_id": rid,
                "expected_node": f"/{rid}/simulate/robot_state_publisher",
                "transitions": ["start", "shutdown"],
            },
            {
                "id": "slam_lifecycle_manager",
                "label_zh": "SLAM 生命周期",
                "type": "stack_manager",
                "node": f"/{rid}/slam/lifecycle_manager",
                "transitions": [],
            },
            {
                "id": "heartbeat",
                "label_zh": "心跳",
                "type": "lifecycle",
                "node": f"/{rid}/heartbeat",
                "transitions": ["configure", "activate", "deactivate", "cleanup", "shutdown"],
            },
            {
                "id": "slam",
                "label_zh": "SLAM",
                "type": "slam_mode",
                "node": f"/{rid}/slam/lifecycle_manager",
                "transitions": ["mapping", "localize", "inactive"],
            },
            {
                "id": "navigation",
                "label_zh": "导航",
                "type": "group",
                "node": f"/{rid}/navigation/lifecycle_manager",
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
        if not ROBOT_ID_RE.fullmatch(rid):
            raise ValueError("robot_id invalid: use [A-Za-z0-9_-], 1-64 chars")
        with self._lock:
            self._state.setdefault(
                rid,
                {
                    "simulate_started": False,
                    "heartbeat_started": False,
                    "slam_mode": "inactive",
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
        """Publish shutdown by updating heartbeat params (``robot_status=shutdown``)."""
        # SetHeartbeatParams: empty strings leave name/map/task_status unchanged; rate_hz<=0 unchanged.
        yaml_req = (
            '{robot_name: "", robot_model: "", current_map: "", current_position: "", '
            'robot_status: "shutdown", task_status: "", rate_hz: 0.0, task_progress: -1.0}'
        )
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
        match_regex: bool = False,
        note: str = "lifecycle orchestrator managed",
        autostart: bool = True,
        force: bool = False,
    ):
        status = self._ros_node_manager.status()
        managed = status.get("managed_nodes") or []
        cur = next((m for m in managed if m.get("id") == node_id), None)
        if cur and cur.get("running") and not force:
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
                        "match_regex": match_regex,
                        "note": note,
                    }
                )
        else:
            with self._ros_node_manager._lock:
                for spec in self._ros_node_manager._managed_nodes:
                    if spec.get("id") == node_id:
                        spec["start_cmd"] = cmd
                        if stop_cmd:
                            spec["stop_cmd"] = stop_cmd
                        if match:
                            spec["match"] = match
                        spec["match_regex"] = match_regex
                        spec["note"] = note
                        break
        if autostart:
            if cur and cur.get("running") and force:
                self._ros_node_manager.control(node_id, "restart")
            else:
                self._ros_node_manager.control(node_id, "start")

    def _robot_process_spec(self, rid: str, sim_mode: str, pose, slot: int = 0) -> Dict[str, str]:
        script = self._sim_bringup_script_path()
        root_q = shlex.quote(str(self._root_dir.resolve()))
        script_q = shlex.quote(str(script.resolve()))
        rid_q = shlex.quote(rid)
        sim_q = shlex.quote(sim_mode)
        x, y, z, yaw = pose
        collision_bit = 1 << (int(slot) + 2)
        env = (
            f"SIM_MODE={sim_q} OPEN_DELIVERY_ROOT={root_q} "
            f"SIM_SPAWN_X={x} SIM_SPAWN_Y={y} SIM_SPAWN_Z={z} SIM_SPAWN_YAW={yaw} "
            f"SIM_COLLISION_BIT={collision_bit}"
        )
        return {
            "cmd": f"{env} bash {script_q} {rid_q}",
            "stop_cmd": (
                f"pkill -f 'sim_bringup.sh {rid}([[:space:]]|$)' || true; "
                f"pkill -f 'ros2 launch system startup.launch.py.*robot_name:={rid}([[:space:]]|$)' || true; "
                f"pkill -f 'ros2 launch nav_bringup stack.launch.py.*robot_name:={rid}([[:space:]]|$)' || true; "
                f"pkill -f 'ros2 launch manager manager.launch.py.*namespace:={rid}([[:space:]]|$)' || true; "
                f"pkill -f 'ros2 launch heartbeat heartbeat.launch.py.*namespace:={rid}([[:space:]]|$)' || true; "
                f"pkill -f 'ros2 launch log_bag log_bag.launch.py.*robot_name:={rid}([[:space:]]|$)' || true; "
                f"pkill -f 'robot_log_recorder.*--robot-name {rid}([[:space:]]|$)' || true; "
                f"pkill -f 'ros2 launch simulate simulate.launch.py.*robot_name:={rid}([[:space:]]|$)' || true"
            ),
        }

    def _ensure_robot_specs(self, rid: str, sim_mode: str = "sim", pose=None) -> None:
        if pose is None:
            with self._infrastructure_lock:
                slot = self._read_spawn_slots().get(rid)
            pose = self._SPAWN_POSES[slot] if slot is not None else self._SPAWN_POSES[0]
        with self._infrastructure_lock:
            slot = self._read_spawn_slots().get(rid, 0)
        spec = self._robot_process_spec(rid, sim_mode, pose, slot)
        self._start_if_needed(
            rid, spec["cmd"], stop_cmd=spec["stop_cmd"],
            match=rf"sim_bringup\.sh\s+{re.escape(rid)}(?![A-Za-z0-9_-])",
            match_regex=True, autostart=False,
            note="per-robot stack; shared Gazebo is managed separately",
        )
        nav_cmd = (
            f"ros2 launch nav_bringup stack.launch.py "
            f"robot_name:={rid} grid_mode:=localize autostart:=false"
        )
        self._start_if_needed(
            f"navigation_{rid}", nav_cmd,
            stop_cmd=(
                f"pkill -f 'ros2 launch nav_bringup stack.launch.py.*robot_name:={rid}([[:space:]]|$)' || true"
            ),
            match=rf"stack\.launch\.py.*robot_name:={re.escape(rid)}(?![A-Za-z0-9_-])",
            match_regex=True, autostart=False,
            note="registered for status (started by per-robot stack)",
        )

    def _sim_managed_running(self, rid: str) -> bool:
        status = self._ros_node_manager.status()
        managed = status.get("managed_nodes") or []
        cur = next((m for m in managed if m.get("id") == rid), None)
        return bool(cur and cur.get("running"))

    def startup_selected_robot(
        self,
        robot_id: str,
        sim_mode: str = "sim",
        *,
        is_online=None,
        boot_grace_sec: float = 90.0,
        starting_age_sec: Optional[float] = None,
    ):
        with self._operation_lock:
            return self._startup_selected_robot_locked(
                robot_id, sim_mode, is_online=is_online,
                boot_grace_sec=boot_grace_sec, starting_age_sec=starting_age_sec,
            )

    def _startup_selected_robot_locked(
        self,
        robot_id: str,
        sim_mode: str = "sim",
        *,
        is_online=None,
        boot_grace_sec: float = 90.0,
        starting_age_sec: Optional[float] = None,
    ):
        rid = self._ensure_robot(robot_id)
        sim_mode = (sim_mode or "sim").strip() or "sim"
        online = bool(is_online(rid)) if callable(is_online) else False
        world_ready = self._gazebo_services_ready() and self._gazebo_transport_ready()
        entity_present = self._simulation_entity_present(rid) if world_ready else False
        recovery_needed = online and (not world_ready or entity_present is False)
        if online and not recovery_needed:
            with self._lock:
                self._last_error = ""
            return self.status(rid)

        sim_running = self._sim_managed_running(rid)
        boot_age = starting_age_sec if starting_age_sec is not None else 0.0
        if sim_running and not recovery_needed and boot_age < boot_grace_sec:
            # sim_bringup.sh still running within grace window — idempotent wait.
            with self._lock:
                self._last_error = ""
            return self.status(rid)

        force_restart = sim_running and (recovery_needed or boot_age >= boot_grace_sec)
        last: Dict[str, Any] = {}
        try:
            import ros_robot_status_store

            last = ros_robot_status_store.get_last_status(rid) or {}
        except Exception:
            last = {}
        last_rs = str(last.get("robot_status") or "").strip().lower()
        auto_mapping = _persisted_auto_mapping(last)
        try:
            slot, spawn_pose = self._spawn_pose_for_robot(rid)
            self._ensure_simulation_world()
            self._ensure_robot_specs(rid, sim_mode, spawn_pose)
            if force_restart:
                try:
                    self._ros_node_manager.control(rid, "pause")
                except Exception:
                    pass
                if self._gazebo_services_ready():
                    existing = self._simulation_entity_present(rid)
                    deleted = self._delete_simulation_entity(rid)
                    if existing is True and not deleted:
                        self._restart_simulation_world()
                time.sleep(0.8)
            elif not sim_running:
                # Remove a stale entity left by an interrupted backend/robot stack.
                existing = self._simulation_entity_present(rid)
                deleted = self._delete_simulation_entity(rid)
                if existing is True and not deleted:
                    self._restart_simulation_world()
            if force_restart or not sim_running:
                self._terminate_stale_robot_processes(rid)
            spec = self._robot_process_spec(rid, sim_mode, spawn_pose, slot)
            self._start_if_needed(
                rid, spec["cmd"], stop_cmd=spec["stop_cmd"],
                match=rf"sim_bringup\.sh\s+{re.escape(rid)}(?![A-Za-z0-9_-])",
                match_regex=True,
                note=(
                    f"per-robot stack; shared Gazebo; spawn_slot={slot} "
                    f"pose={spawn_pose}"
                ),
                force=force_restart,
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
                    "slam_mode": "mapping" if auto_mapping else "localize",
                    "navigation_started": True,
                    "sim_mode": sim_mode,
                    "last_persisted_robot_status": last_rs or None,
                    "updated_at": time.time(),
                }
            )
            self._last_error = ""
        return self.status(rid)

    def shutdown_selected_robot(self, robot_id: str):
        with self._operation_lock:
            return self._shutdown_selected_robot_locked(robot_id)

    def _shutdown_selected_robot_locked(self, robot_id: str):
        """Stop per-robot orchestrated stack (including simulate launch registered under robot id)."""
        rid = self._ensure_robot(robot_id)
        self._ensure_robot_specs(rid)
        hb = f"/{rid}/heartbeat"
        nav_mgr = f"/{rid}/navigation/lifecycle_manager"
        # Let subscribers see SHUTDOWN on /{rid}/robot_status before tearing down lifecycle nodes.
        self._signal_shutdown_via_heartbeat(rid)
        time.sleep(0.5)
        for t in ("deactivate", "cleanup", "shutdown"):
            self._lifecycle_try(nav_mgr, t)
        for t in ("deactivate", "cleanup", "shutdown"):
            self._lifecycle_try(hb, t)
        for node_id in (f"navigation_{rid}", rid):
            try:
                self._ros_node_manager.control(node_id, "pause")
            except Exception:
                pass
        # Gazebo belongs to the host, not this robot. Delete only this entity.
        if self._gazebo_services_ready():
            existing = self._simulation_entity_present(rid)
            deleted = self._delete_simulation_entity(rid)
            if existing is True and not deleted:
                raise RuntimeError(f"failed to delete Gazebo entity: {rid}")
        with self._lock:
            st = self._state.setdefault(rid, {})
            st.pop("slam_mode", None)
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
        if cid in ("slam", "mapping", "localization"):
            if cid == "mapping":
                tr = "mapping" if transition in ("configure", "activate", "start", "mapping") else "inactive"
            elif cid == "localization":
                tr = "localize" if transition in ("configure", "activate", "start", "localize") else "inactive"
            else:
                tr = transition
            self._stack_lifecycle_transition(rid, "slam", tr)
            with self._lock:
                self._state.setdefault(rid, {})["slam_mode"] = tr
            return self.status(rid)
        comp = next((c for c in self._component_catalog(rid) if c["id"] == cid), None)
        if not comp:
            raise ValueError(f"unknown component: {cid}")
        node = comp.get("node")
        if not node:
            raise ValueError(f"component {cid} is not lifecycle-addressable")
        target = "navigation/lifecycle_manager" if cid == "navigation" else cid
        self._stack_lifecycle_transition(rid, target, transition)
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
                elif ctype == "slam_mode":
                    slam_ns = f"/{rid}/slam"
                    running = any(
                        n in discovered_names
                        for n in (
                            f"{slam_ns}/mapping",
                            f"{slam_ns}/localizing",
                        )
                    )
                    with self._lock:
                        lifecycle_state = str(
                            self._state.get(rid, {}).get("slam_mode") or "inactive"
                        )
                    if running:
                        mapping_present = f"{slam_ns}/mapping" in discovered_names
                        localizing_state = self._lifecycle_get(f"{slam_ns}/localizing").get("state")
                        if mapping_present:
                            lifecycle_state = "mapping"
                        elif localizing_state == "active":
                            lifecycle_state = "localize"
                        elif lifecycle_state not in ("mapping", "localize"):
                            lifecycle_state = "inactive"
                elif ctype == "stack_manager":
                    running = f"/{rid}/slam/lifecycle_manager" in discovered_names
                    lifecycle_state = "active" if running else "missing"
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
                if comp["id"] == "slam":
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
                    "session_mapping_process_started": str(
                        online_state.get("slam_mode") or ""
                    ).lower() == "mapping",
                    "discovered_nodes": [{"name": n} for n in discovered_under],
                    "components": components,
                }
            )
        with self._lock:
            err = self._last_error
        return {"robots": robots, "last_error": err, "timestamp": time.time()}
