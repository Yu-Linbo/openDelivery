"""Validate the expanded Gazebo geometry, not Xacro string fragments."""
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

MODEL = Path(__file__).resolve().parents[1] / 'urdf/simple_2d_robot.urdf.xacro'


def transform(text):
    x, y, z, r, p, yaw = map(float, text.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    return np.array([x, y, z]), rotation


@pytest.fixture(scope='module')
def model(tmp_path_factory):
    urdf = tmp_path_factory.mktemp('robot') / 'robot.urdf'
    urdf.write_bytes(subprocess.check_output(['xacro', str(MODEL), 'frame_prefix:=test/']))
    return ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(urdf)])).find('model')


def half_extents(geometry):
    if geometry.find('box') is not None:
        return np.array(list(map(float, geometry.findtext('box/size').split()))) / 2
    if geometry.find('sphere') is not None:
        return np.full(3, float(geometry.findtext('sphere/radius')))
    return np.array([float(geometry.findtext('cylinder/radius'))]*2 +
                    [float(geometry.findtext('cylinder/length'))/2])


def test_camera_frusta_clear_all_body_visuals(model):
    body = model.find("link[@name='test/base_footprint']")
    # Test every output pixel against conservative bounding boxes of all fixed
    # body visuals. Wheel bounds are below and behind both camera origins.
    for sensor in body.findall("sensor[@type='camera']"):
        camera = sensor.find('camera')
        width, height = int(camera.findtext('image/width')), int(camera.findtext('image/height'))
        tangent = math.tan(float(camera.findtext('horizontal_fov')) / 2)
        ys, zs = np.meshgrid(np.linspace(-tangent, tangent, width),
                            np.linspace(-tangent*height/width, tangent*height/width, height))
        directions = np.stack([np.ones_like(ys), ys, zs], axis=-1).reshape(-1, 3)
        origin, rotation = transform(sensor.findtext('pose'))
        directions = directions @ rotation.T
        for visual in body.findall('visual'):
            center, visual_rotation = transform(visual.findtext('pose', '0 0 0 0 0 0'))
            local_origin = (origin-center) @ visual_rotation
            local_directions = directions @ visual_rotation
            extents = half_extents(visual.find('geometry'))
            with np.errstate(divide='ignore', invalid='ignore'):
                t1 = (-extents-local_origin) / local_directions
                t2 = (extents-local_origin) / local_directions
            entry = np.max(np.minimum(t1, t2), axis=1)
            exit_ = np.min(np.maximum(t1, t2), axis=1)
            hits = (exit_ >= np.maximum(entry, float(camera.findtext('clip/near'))))
            assert not np.any(hits), (sensor.get('name'), visual.get('name'), int(hits.sum()))


def test_ground_clearance_support_and_lidar_height(model):
    body = model.find("link[@name='test/base_footprint']")
    for collision in body.findall('collision'):
        center, _ = transform(collision.findtext('pose'))
        bottom = center[2] - half_extents(collision.find('geometry'))[2]
        if 'support' in collision.get('name'):
            assert abs(bottom) < 1e-6
        else:
            assert bottom >= .034
    shell = next(c for c in body.findall('collision') if 'outer_shell' in c.get('name'))
    center, _ = transform(shell.findtext('pose'))
    extents = half_extents(shell.find('geometry'))
    lidar, _ = transform(body.find("sensor[@type='ray']/pose").text)
    assert center[2]-extents[2] < lidar[2] < center[2]+extents[2]
    assert math.hypot(extents[0], extents[1]) < .28
    # Wheels have a 5 mm lateral gap from the chassis and cannot rub the shell.
    for link in model.findall('link'):
        if 'wheel_link' not in link.get('name'):
            continue
        joint = next(j for j in model.findall('joint') if j.findtext('child') == link.get('name'))
        wheel_center, _ = transform(joint.findtext('pose'))
        collision = link.find('collision')
        radius = float(collision.findtext('geometry/cylinder/radius'))
        assert abs(wheel_center[2]-radius) < 1e-6
        assert wheel_center[2]+radius < center[2]-extents[2]
