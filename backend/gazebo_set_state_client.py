"""Persistent ROS 2 client for /gazebo/set_model_state (avoids per-request ros2 CLI subprocess)."""

from __future__ import annotations

from collections import OrderedDict
import threading
import time
from typing import Any, Dict, Optional, Tuple

_worker: Optional["_GazeboSetStateWorker"] = None
_worker_lock = threading.Lock()


class _GazeboSetStateWorker(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True)
        self._pending: "OrderedDict[str, Tuple[Any, Dict[str, Any], threading.Event]]" = (
            OrderedDict()
        )
        self._pending_cv = threading.Condition()
        self._ready = threading.Event()
        self._init_error: str = ""
        self._shutdown = False

    def run(self) -> None:
        try:
            import rclpy
            from gazebo_msgs.srv import SetModelState
            from geometry_msgs.msg import Pose, Point, Quaternion, Twist
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as exc:
            self._init_error = str(exc)
            self._ready.set()
            return

        try:
            if not rclpy.ok():
                rclpy.init()
        except Exception as exc:  # noqa: BLE001
            self._init_error = f"rclpy.init: {exc}"
            self._ready.set()
            return

        node = rclpy.create_node("open_delivery_gazebo_set_state")
        client = node.create_client(SetModelState, "/gazebo/set_model_state")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        self._ready.set()

        while rclpy.ok() and not self._shutdown:
            work = None
            with self._pending_cv:
                if not self._pending and not self._shutdown:
                    self._pending_cv.wait(timeout=0.05)
                if self._shutdown:
                    break
                if self._pending:
                    _, work = self._pending.popitem(last=False)
            if work is None:
                executor.spin_once(timeout_sec=0.05)
                continue
            req, result_box, evt = work
            try:
                if not client.wait_for_service(timeout_sec=1.5):
                    result_box["ok"] = False
                    result_box["err"] = "set_model_state service unavailable"
                    evt.set()
                    continue
                future = client.call_async(req)
                t0 = time.monotonic()
                while not future.done() and (time.monotonic() - t0) < 5.0:
                    executor.spin_once(timeout_sec=0.05)
                if not future.done():
                    result_box["ok"] = False
                    result_box["err"] = "set_model_state call timeout"
                else:
                    resp = future.result()
                    result_box["ok"] = bool(getattr(resp, "success", False))
                    result_box["err"] = str(getattr(resp, "status_message", "") or "")
            except Exception as exc:  # noqa: BLE001
                result_box["ok"] = False
                result_box["err"] = str(exc)
            evt.set()

        try:
            executor.remove_node(node)
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

    def _enqueue_latest(
        self, request: Any, result_box: Dict[str, Any], evt: threading.Event
    ) -> None:
        model_state = getattr(request, "model_state", None)
        key = str(getattr(model_state, "model_name", "") or "__default__")
        with self._pending_cv:
            superseded = self._pending.pop(key, None)
            if superseded is not None:
                _, old_result, old_evt = superseded
                old_result["ok"] = True
                old_result["err"] = "superseded by newer request"
                old_evt.set()
            self._pending[key] = (request, result_box, evt)
            self._pending_cv.notify()

    def submit(self, request: Any, timeout_sec: float = 4.0) -> Tuple[bool, str]:
        if not self._ready.wait(timeout=5.0):
            return False, "gazebo client worker not ready"
        if self._init_error:
            return False, self._init_error
        result_box: Dict[str, Any] = {}
        evt = threading.Event()
        self._enqueue_latest(request, result_box, evt)
        if not evt.wait(timeout=timeout_sec):
            return False, "set_model_state wait timeout"
        return bool(result_box.get("ok")), str(result_box.get("err") or "")

    def stop(self) -> None:
        with self._pending_cv:
            self._shutdown = True
            for _, result_box, evt in self._pending.values():
                result_box["ok"] = False
                result_box["err"] = "gazebo client worker stopped"
                evt.set()
            self._pending.clear()
            self._pending_cv.notify_all()


def _get_worker() -> Optional[_GazeboSetStateWorker]:
    global _worker
    with _worker_lock:
        if _worker is not None and _worker.is_alive():
            return _worker
        w = _GazeboSetStateWorker()
        w.start()
        _worker = w
        return _worker


def try_set_model_state_fast(
    model_name: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    reference_frame: str,
    orientation: Optional[Tuple[float, float, float, float]] = None,
) -> Optional[Dict[str, Any]]:
    """Return ``{"ok": True, "output": "rclpy_client"}`` on success, ``None`` to use slow path."""
    try:
        from gazebo_msgs.srv import SetModelState
        from geometry_msgs.msg import Pose, Point, Quaternion, Twist
    except ImportError:
        return None

    w = _get_worker()
    if w is None:
        return None

    pose = Pose()
    pose.position = Point(x=float(x), y=float(y), z=float(z))
    if orientation is not None:
        ox, oy, oz, ow = orientation
        pose.orientation = Quaternion(x=float(ox), y=float(oy), z=float(oz), w=float(ow))
    else:
        import math

        pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=float(math.sin(float(yaw) / 2.0)),
            w=float(math.cos(float(yaw) / 2.0)),
        )

    req = SetModelState.Request()
    req.model_state.model_name = str(model_name)
    req.model_state.pose = pose
    req.model_state.twist = Twist()
    req.model_state.reference_frame = str(reference_frame)

    ok, err = w.submit(req)
    if ok:
        return {"ok": True, "output": "rclpy_client"}
    low = (err or "").lower()
    if any(
        # Timeouts may still complete in the worker. Retrying them through the
        # subprocess fallback would apply an old pose twice.
        s in low
        for s in (
            "unavailable",
            "not ready",
            "import",
            "init",
            "worker not ready",
            "gazebo client worker not ready",
        )
    ):
        return None
    raise RuntimeError(err or "set_model_state failed (fast client)")


def gazebo_fast_client_ready() -> bool:
    w = _get_worker()
    if w is None or not w.is_alive():
        return False
    return w._ready.is_set() and not w._init_error  # noqa: SLF001
