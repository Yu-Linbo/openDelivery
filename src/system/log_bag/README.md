# Robot log recording

The recorder waits for `/ROBOT/robot_status.robot_status` to become
`localization_lost` or `ready` before opening its first bag. The health monitor
uses `localization_lost` after the configured startup nodes are reachable;
`ready` also permits attaching to a robot that has already localized.
`initializing` and `localizing` alone do not start recording. Once recording
has started, later localization transitions remain recorded. `shutdown` or a
new `initializing` state closes the bag and resets the startup gate.

Recording uses serialized subscriptions on the recorder's existing ROS node.
Size rotation closes and opens rosbag2 SQLite writers without restarting ROS
participants or subscriptions. Late topics are discovered once per second.
Transient-local samples (including static TF) are carried into each new bag.
Empty bags are discarded even when a task tag exists. The existing heartbeat
watchdog, task indexing, and storage retention remain in effect.

This replaces the former `ros2 bag record` subprocess per file. In the observed
failure, 2,257 subprocess starts preceded an exit with status 139 and repeated
empty recordings, while the parent still received robot status. Keeping the
working participant avoids repeating that child discovery path; it does not
prove the underlying DDS crash cause.

## Verification

With ROS Foxy and the workspace environment sourced:

```bash
colcon build --packages-select log_bag --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select log_bag --event-handlers console_direct+
python3 src/system/log_bag/test/test_recording_integration.py \
  install/log_bag/lib/log_bag/robot_log_recorder
```

The integration test uses ROS domain 187, a synthetic `test_robot`, and temporary
storage. It checks the startup gate, rotation, latched static TF, shutdown and
restart gating, SQLite integrity, and the backend's actual replay decoder.
