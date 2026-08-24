#!/usr/bin/env python3
"""Planar transforms and map/world metadata used by Gazebo truth localization."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


def normalize_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def compose(left, right):
    """Return ``left_T_right`` for planar ``(x, y, yaw)`` transforms."""
    lx, ly, lyaw = left
    rx, ry, ryaw = right
    c = math.cos(lyaw)
    s = math.sin(lyaw)
    return (
        lx + c * rx - s * ry,
        ly + s * rx + c * ry,
        normalize_angle(lyaw + ryaw),
    )


def inverse(pose):
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, normalize_angle(-yaw))


def yaw_from_quaternion(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def pose_error(true_pose, requested_pose):
    """Return the map-coordinate bias that makes ``true_pose`` look requested."""
    return (
        requested_pose[0] - true_pose[0],
        requested_pose[1] - true_pose[1],
        normalize_angle(requested_pose[2] - true_pose[2]),
    )


def apply_pose_error(true_pose, error):
    """Apply a map-coordinate localization bias to an absolute pose."""
    return (
        true_pose[0] + error[0],
        true_pose[1] + error[1],
        normalize_angle(true_pose[2] + error[2]),
    )


def decay_pose_error(error, linear_speed, angular_speed, dt):
    """Move a planar localization bias toward zero at bounded rates."""
    if dt <= 0.0:
        return error

    x, y, yaw = error
    distance = math.hypot(x, y)
    linear_step = max(0.0, linear_speed) * dt
    if distance <= linear_step:
        x = 0.0
        y = 0.0
    elif distance > 0.0 and linear_step > 0.0:
        scale = (distance - linear_step) / distance
        x *= scale
        y *= scale

    angular_step = max(0.0, angular_speed) * dt
    if abs(yaw) <= angular_step:
        yaw = 0.0
    elif angular_step > 0.0:
        yaw -= math.copysign(angular_step, yaw)

    return (x, y, normalize_angle(yaw))


def bounded_time_step(previous_ns, current_ns, max_dt):
    """Return a non-negative, capped time step for a possibly-reset clock."""
    if previous_ns is None or current_ns <= previous_ns:
        return 0.0
    return min((current_ns - previous_ns) / 1.0e9, max(0.0, max_dt))


def _pgm_size(path):
    tokens = []
    with Path(path).open("rb") as stream:
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                break
            tokens.extend(line.split(b"#", 1)[0].split())
    if len(tokens) < 4 or tokens[0] not in (b"P2", b"P5"):
        raise RuntimeError("invalid PGM map: %s" % path)
    width, height = int(tokens[1]), int(tokens[2])
    if width <= 0 or height <= 0:
        raise RuntimeError("invalid PGM dimensions: %s" % path)
    return width, height


def _map_metadata(yaml_path):
    yaml_path = Path(yaml_path)
    values = {}
    for raw in yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    try:
        resolution = float(values["resolution"])
        origin = tuple(
            float(value.strip()) for value in values["origin"].strip("[]").split(",")
        )
        image = values["image"].strip("'\"")
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("invalid map metadata: %s" % yaml_path) from exc
    if resolution <= 0.0 or len(origin) < 3:
        raise RuntimeError("invalid map metadata: %s" % yaml_path)
    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    width, height = _pgm_size(image_path)
    return resolution, origin[:3], width, height


def map_yaml_for_name(initial_map_yaml, map_name):
    """Resolve a sibling map YAML from a trusted initial map path."""
    name = str(map_name or "").strip()
    if not name or Path(name).name != name or name in (".", ".."):
        raise RuntimeError("invalid map name: %r" % map_name)
    initial_path = Path(initial_map_yaml)
    candidate = initial_path.parent.parent / name / ("%s.yaml" % name)
    if not candidate.is_file():
        raise RuntimeError("map YAML not found: %s" % candidate)
    return candidate


def _floor_world_pose(world_path, floor):
    try:
        root = ET.parse(str(world_path)).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError("invalid Gazebo world file %s: %s" % (world_path, exc)) from exc
    matches = []
    for include in root.findall(".//include"):
        name = (include.findtext("name") or "").strip()
        uri = (include.findtext("uri") or "").strip()
        if name == floor or uri == "model://%s" % floor:
            raw = (include.findtext("pose") or "0 0 0 0 0 0").split()
            if len(raw) != 6:
                raise RuntimeError("invalid Gazebo pose for floor %s" % floor)
            try:
                matches.append(tuple(float(value) for value in raw))
            except ValueError as exc:
                raise RuntimeError("invalid Gazebo pose for floor %s" % floor) from exc
    if len(matches) != 1:
        raise RuntimeError(
            "Gazebo world requires exactly one %s model, found %d" % (floor, len(matches))
        )
    return matches[0]


def map_to_world_from_files(map_yaml, world_path):
    """Return ``world_T_map`` for the project's centered-STL floor contract."""
    map_yaml = Path(map_yaml)
    floor = map_yaml.stem
    resolution, origin, width, height = _map_metadata(map_yaml)
    floor_x, floor_y, _z, _roll, _pitch, floor_yaw = _floor_world_pose(
        Path(world_path), floor
    )
    map_yaw = origin[2]
    center_x = origin[0] + math.cos(map_yaw) * width * resolution / 2.0 \
        - math.sin(map_yaw) * height * resolution / 2.0
    center_y = origin[1] + math.sin(map_yaw) * width * resolution / 2.0 \
        + math.cos(map_yaw) * height * resolution / 2.0
    relative_yaw = floor_yaw - map_yaw
    rotated_center = compose((0.0, 0.0, relative_yaw), (center_x, center_y, 0.0))
    return (
        floor_x - rotated_center[0],
        floor_y - rotated_center[1],
        normalize_angle(relative_yaw),
    )
