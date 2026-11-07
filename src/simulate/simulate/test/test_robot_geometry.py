"""Validate the expanded Gazebo geometry, not Xacro string fragments."""
import math
import json
import re
import subprocess
import shlex
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

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


@pytest.fixture(scope='module')
def mesh_bounds(model, tmp_path_factory):
    paths = sorted({node.text for node in model.findall('.//mesh/uri')})
    assert len(paths) == 7
    assert all(Path(path).is_file() for path in paths), 'Build simulate to install the arm meshes'
    executable = tmp_path_factory.mktemp('mesh') / 'bounds'
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'gazebo'], text=True))
    subprocess.run(['c++', str(MODEL.parent.parent / 'test/mesh_bounds.cpp'),
                    '-o', str(executable), *flags, '-std=c++17'], check=True)
    lines = subprocess.check_output([str(executable), *paths], text=True).splitlines()
    return {path: np.fromstring(line, sep=' ').reshape(2, 3)
            for path, line in zip(paths, lines)}


def geometry_bounds(geometry, meshes):
    mesh = geometry.find('mesh')
    if mesh is not None:
        scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
        bounds = meshes[mesh.findtext('uri')] * scale
        return bounds.mean(axis=0), (bounds[1]-bounds[0])/2
    return np.zeros(3), half_extents(geometry)


def half_extents(geometry):
    if geometry.find('box') is not None:
        return np.array(list(map(float, geometry.findtext('box/size').split()))) / 2
    if geometry.find('sphere') is not None:
        return np.full(3, float(geometry.findtext('sphere/radius')))
    return np.array([float(geometry.findtext('cylinder/radius'))]*2 +
                    [float(geometry.findtext('cylinder/length'))/2])


def test_camera_frusta_clear_all_body_visuals(model, mesh_bounds):
    body = model.find("link[@name='test/base_footprint']")
    # Test every output pixel against conservative bounding boxes of all fixed
    # body visuals. Wheel bounds are below and behind both camera origins.
    for sensor in body.findall("sensor[@type='camera']"):
        if sensor.get('name') == 'wrist_camera':
            continue  # Wrist view can include the gripper; this contract covers the navigation cameras.
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
            local_center, extents = geometry_bounds(visual.find('geometry'), mesh_bounds)
            local_origin -= local_center
            with np.errstate(divide='ignore', invalid='ignore'):
                t1 = (-extents-local_origin) / local_directions
                t2 = (extents-local_origin) / local_directions
            entry = np.max(np.minimum(t1, t2), axis=1)
            exit_ = np.min(np.maximum(t1, t2), axis=1)
            hits = (exit_ >= np.maximum(entry, float(camera.findtext('clip/near'))))
            assert not np.any(hits), (sensor.get('name'), visual.get('name'), int(hits.sum()))


def test_ground_clearance_support_and_lidar_height(model, mesh_bounds):
    body = model.find("link[@name='test/base_footprint']")
    for collision in body.findall('collision'):
        center, rotation = transform(collision.findtext('pose'))
        local_center, extents = geometry_bounds(collision.find('geometry'), mesh_bounds)
        center += rotation @ local_center
        bottom = center[2] - (np.abs(rotation) @ extents)[2]
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


def test_navigation_footprint_covers_all_gazebo_collisions(model, mesh_bounds):
    config = MODEL.parents[3] / 'navigation/nav_bringup/config/nav2_params.yaml'
    text = config.read_text()
    footprints = re.findall(r'^\s+footprint: (.+)$', text, re.MULTILINE)
    assert len(footprints) == 2  # global + local
    assert footprints[0] == footprints[1]
    polygon = np.array(json.loads(yaml.safe_load(footprints[0])))
    minimum, maximum = polygon.min(axis=0), polygon.max(axis=0)
    # Include all fixed body parts and wheels after Gazebo's URDF conversion;
    # a narrower navigation envelope must not discard camera/shell collisions.
    for link in model.findall('link'):
        link_position, link_rotation = transform(link.findtext('pose', '0 0 0 0 0 0'))
        for collision in link.findall('collision'):
            center, rotation = transform(collision.findtext('pose', '0 0 0 0 0 0'))
            local_center, half = geometry_bounds(collision.find('geometry'), mesh_bounds)
            corners = np.array([[x,y,z] for x in (-half[0],half[0]) for y in (-half[1],half[1]) for z in (-half[2],half[2])])
            corners = ((corners + local_center) @ rotation.T + center) @ link_rotation.T + link_position
            assert np.all(corners[:,:2] >= minimum - 1e-6), collision.get('name')
            assert np.all(corners[:,:2] <= maximum + 1e-6), collision.get('name')


def test_smolvla_wrist_pose_and_camera_contract(model):
    body = model.find("link[@name='test/base_footprint']")
    sensors = {s.get('name'): s for s in body.findall("sensor[@type='camera']")}
    assert set(sensors) == {'front_camera', 'front_down_camera', 'wrist_camera'}
    sensor = sensors['wrist_camera']
    position, rotation = transform(sensor.findtext('pose'))
    # Independent reference from SmolVLA build_robot('vla') loaded in PyBullet,
    # with the six Simulation.FOLD_Q angles and both fingers at 0.028 m.
    assert np.allclose(position, [-.0384382606, .0107450234, .6310049295], atol=2e-6)
    assert rotation[0, 0] > .98  # Optical +Z / Gazebo +X points forward.
    assert rotation[2, 0] < -.14  # ...and down, as in the source demo.
    assert sensor.findtext('camera/image/width') == '320'
    assert sensor.findtext('camera/image/height') == '240'
    horizontal = float(sensor.findtext('camera/horizontal_fov'))
    vertical = 2*math.atan(math.tan(horizontal/2)*3/4)
    assert math.isclose(vertical, math.radians(75), abs_tol=1e-5)
    plugin = sensor.find('plugin')
    assert plugin.findtext('frame_name') == 'test/wrist_camera_optical_frame'
    assert plugin.findtext('camera_name') == 'wrist_camera'
    assert plugin.find('ros/remapping').text == 'wrist_camera/image_raw:=camera/wrist/image_raw'


@pytest.mark.parametrize('prefix', ['', 'robot1/', 'robot2/'])
def test_arm_frames_and_assets_are_self_contained(prefix):
    root = ET.fromstring(subprocess.check_output(['xacro', str(MODEL), 'frame_prefix:='+prefix]))
    links = {link.get('name') for link in root.findall('link')}
    for name in ('g_base', 'joint6_flange', 'gripper_palm', 'left_finger',
                 'right_finger', 'tool_tip', 'wrist_camera_optical_frame'):
        assert prefix+name in links
    for name in ('arm_mount', 'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
                 'left_finger_joint', 'right_finger_joint'):
        joint = root.find("joint[@name='%s']" % (prefix+name))
        assert joint is not None and joint.get('type') == 'fixed'
        assert joint.find('parent').get('link') in links
        assert joint.find('child').get('link') in links
    for mesh in root.iter('mesh'):
        path = Path(mesh.get('filename'))
        assert path.is_file()
        assert path.with_suffix('.png').is_file()
