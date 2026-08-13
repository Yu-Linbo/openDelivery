"""Validated persistence for editable occupancy, semantic, and point map assets."""

import base64
import json
import os
import re
import tempfile
from io import BytesIO
from pathlib import Path
from typing import Any, Dict, List

from PIL import Image


_MAP_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
_POINT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
_POINT_TYPES = {"elevator", "standby", "custom"}


def _floor_dir(map_dir: Path, floor: str) -> Path:
    name = str(floor or "").strip()
    if not _MAP_NAME.fullmatch(name):
        raise ValueError("invalid floor name")
    path = map_dir / name
    if not path.is_dir():
        raise FileNotFoundError(f"floor not found: {name}")
    return path


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, path)
    finally:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass


def _decode_data_url(raw: str, expected: str) -> bytes:
    text = str(raw or "").strip()
    prefix = f"data:{expected};base64,"
    if not text.startswith(prefix):
        raise ValueError(f"expected {expected} base64 data URL")
    try:
        return base64.b64decode(text[len(prefix) :], validate=True)
    except Exception as exc:
        raise ValueError("invalid base64 image") from exc


def _image_data_url(path: Path, mime: str) -> str:
    if not path.is_file():
        return ""
    return f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode("ascii")


def _points_path(folder: Path, floor: str) -> Path:
    return folder / f"{floor}_points.json"


def _normalize_point(raw: Dict[str, Any], used: set) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise ValueError("point must be an object")
    point_id = str(raw.get("id") or "").strip()
    if not _POINT_ID.fullmatch(point_id) or point_id in used:
        raise ValueError("point id must be unique and use [A-Za-z0-9_-]")
    point_type = str(raw.get("type") or "custom").strip().lower()
    if point_type not in _POINT_TYPES:
        raise ValueError("point type must be elevator, standby, or custom")
    name = str(raw.get("name") or point_id).strip()
    if not name or len(name) > 80:
        raise ValueError("point name must be 1-80 chars")
    try:
        x = float(raw.get("x"))
        y = float(raw.get("y"))
        yaw = float(raw.get("yaw", 0.0))
    except (TypeError, ValueError) as exc:
        raise ValueError("point x, y, yaw must be numeric") from exc
    used.add(point_id)
    return {"id": point_id, "name": name, "type": point_type, "x": x, "y": y, "yaw": yaw}


def load_points(map_dir: Path, floor: str) -> List[Dict[str, Any]]:
    folder = _floor_dir(map_dir, floor)
    path = _points_path(folder, floor)
    if not path.is_file():
        return []
    raw = json.loads(path.read_text(encoding="utf-8"))
    rows = raw.get("points", []) if isinstance(raw, dict) else raw
    if not isinstance(rows, list):
        raise ValueError("points file must contain a points array")
    used = set()
    return [_normalize_point(item, used) for item in rows]


def save_points(map_dir: Path, floor: str, points: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    folder = _floor_dir(map_dir, floor)
    if not isinstance(points, list) or len(points) > 1000:
        raise ValueError("points must be an array with at most 1000 items")
    used = set()
    clean = [_normalize_point(item, used) for item in points]
    payload = {"version": 1, "map": floor, "points": clean}
    _atomic_write(
        _points_path(folder, floor),
        (json.dumps(payload, ensure_ascii=False, indent=2) + "\n").encode("utf-8"),
    )
    return clean


def load_assets(map_dir: Path, floor: str) -> Dict[str, Any]:
    folder = _floor_dir(map_dir, floor)
    semantic = folder / f"{floor}_semantic.png"
    legend = folder / "semantic_legend.json"
    legend_data = None
    if legend.is_file():
        legend_data = json.loads(legend.read_text(encoding="utf-8"))
    return {
        "floor": floor,
        "semantic_png": _image_data_url(semantic, "image/png"),
        "semantic_available": semantic.is_file(),
        "semantic_legend": legend_data,
        "points": load_points(map_dir, floor),
    }


def save_raster(map_dir: Path, floor: str, pgm_data_url: str) -> Dict[str, Any]:
    folder = _floor_dir(map_dir, floor)
    raw = _decode_data_url(pgm_data_url, "image/x-portable-graymap")
    if len(raw) > 64 * 1024 * 1024:
        raise ValueError("PGM image too large")
    try:
        with Image.open(BytesIO(raw)) as image:
            image.verify()
            size = image.size
    except Exception as exc:
        raise ValueError("invalid PGM image") from exc
    _atomic_write(folder / f"{floor}.pgm", raw)
    return {"ok": True, "floor": floor, "width": size[0], "height": size[1]}


def save_semantic(map_dir: Path, floor: str, png_data_url: str) -> Dict[str, Any]:
    folder = _floor_dir(map_dir, floor)
    raw = _decode_data_url(png_data_url, "image/png")
    if len(raw) > 64 * 1024 * 1024:
        raise ValueError("semantic PNG too large")
    try:
        with Image.open(BytesIO(raw)) as image:
            image.verify()
            size = image.size
    except Exception as exc:
        raise ValueError("invalid semantic PNG") from exc
    with Image.open(folder / f"{floor}.pgm") as raster:
        raster_size = raster.size
    if size != raster_size:
        raise ValueError(f"semantic image size {size} must match raster size {raster_size}")
    _atomic_write(folder / f"{floor}_semantic.png", raw)
    return {"ok": True, "floor": floor, "width": size[0], "height": size[1]}
