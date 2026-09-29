#!/usr/bin/env python3
"""Manual Gazebo Classic integration check; needs a working DISPLAY and ROS setup.

Uses a separate Gazebo master, no ROS nodes, and leaves reports/screenshots in a
new temporary directory. Build simulate first. Runtime timeout: 120 seconds.
"""
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


FIXTURE = Path(__file__).resolve().parent
PROJECT = FIXTURE.parents[3]


def main():
    output = Path(tempfile.mkdtemp(prefix='delivery-model-check-'))
    print(f'Artifacts: {output}', flush=True)
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'gazebo'], text=True))
    fixture_plugin = output / 'librobot_model_validation.so'
    subprocess.run(['c++', '-shared', '-fPIC', str(FIXTURE / 'robot_model_validation.cpp'),
                    '-o', str(fixture_plugin), *flags, '-std=c++17'], check=True)
    world = ET.fromstring('''<sdf version="1.6"><world name="default">
      <gravity>0 0 -9.81</gravity>
      <physics type="ode"><max_step_size>0.001</max_step_size>
        <real_time_update_rate>1000</real_time_update_rate></physics>
      <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.8 0.9 1</background></scene>
      <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 0.2 -1</direction></light>
      <model name="ground"><static>true</static><link name="ground">
        <collision name="ground"><geometry><plane><normal>0 0 1</normal>
          <size>100 100</size></plane></geometry></collision>
        <visual name="ground"><geometry><plane><normal>0 0 1</normal>
          <size>100 100</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link></model></world></sdf>''')
    root = world.find('world')
    plugin = ET.SubElement(root, 'plugin', name='validation', filename=str(fixture_plugin))
    ET.SubElement(plugin, 'output_dir').text = str(output)
    model_path = FIXTURE.parent / 'urdf/simple_2d_robot.urdf.xacro'
    filter_plugin = PROJECT / 'install/simulate/lib/libray_collision_filter_plugin.so'
    assert filter_plugin.exists(), 'Build simulate first'
    for name, bit, pose in [('probe', 4, '0 0 0.01 0 0 0'),
                            ('peer', 8, '2 0 0 0 0 0'),
                            ('reverse_probe', 16, '0 3 0.01 0 0 0')]:
        urdf = output / f'{name}.urdf'
        urdf.write_bytes(subprocess.check_output([
            'xacro', str(model_path), f'robot_namespace:={name}', f'frame_prefix:={name}/',
            f'collision_bit:={bit}', f'collision_filter_plugin:={filter_plugin}']))
        model = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(urdf)])).find('model')
        model.set('name', name)
        ET.SubElement(model, 'pose').text = pose
        if name == 'peer':
            ET.SubElement(model, 'static').text = 'true'
        if name == 'reverse_probe':
            model.find(".//sensor[@type='ray']/pose").text = '0.23 0 0.17 0 0 3.141592653589793'
        # Use the production geometry/filter but isolate ROS and drive wheels
        # directly in the fixture. This must not publish into the live ROS graph.
        for node in [model, *model.findall('.//sensor')]:
            for child in node.findall('plugin'):
                if child.get('name') != 'ray_collision_filter':
                    node.remove(child)
        root.append(model)
    root.append(ET.fromstring('''<model name="inspection"><static>true</static>
      <pose>0.8 -0.8 0.70 0 0.474 2.35619449</pose><link name="camera">
      <sensor name="inspection" type="camera"><always_on>true</always_on><update_rate>5</update_rate>
        <camera><horizontal_fov>0.85</horizontal_fov><image><width>800</width><height>600</height>
          <format>R8G8B8</format></image><clip><near>0.01</near><far>30</far></clip></camera>
      </sensor></link></model>'''))
    world_file = output / 'validation.world'
    ET.ElementTree(world).write(world_file)
    with socket.socket() as free_port:
        free_port.bind(('127.0.0.1', 0))
        port = free_port.getsockname()[1]
    env = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}', LIBGL_ALWAYS_SOFTWARE='1')
    with (output / 'gazebo.log').open('w') as log:
        process = subprocess.Popen(['gzserver', '--verbose', str(world_file)], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 120
            report_file = output / 'report.json'
            while time.monotonic() < deadline and process.poll() is None:
                if report_file.exists():
                    try:
                        report = json.loads(report_file.read_text())
                    except json.JSONDecodeError:
                        time.sleep(0.1)
                        continue
                    print(json.dumps(report, indent=2), flush=True)
                    assert report['passed'], f'Validation failed; see {output}'
                    return
                time.sleep(0.25)
            raise RuntimeError(f'No complete report; see {output}/gazebo.log')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == '__main__':
    main()
