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

## Camera frames and task tags

The recorder subscribes to both raw cameras but keeps only the latest frame
from each in memory. When a root `TaskStatus` first activates a task ID, or
first reports `Finished`, `Failed`, or `Terminated` for a task already tagged
in that bag, the recorder writes one cached frame per camera into the current
bag. An unrelated retained terminal status is ignored. Both frames receive the same
wall-clock event timestamp. Repeated status messages do not create more frames.
A camera frame older than two seconds, an undiscovered camera, or a task event
while no bag is open produces no frame for that camera. Rotation alone does not
add a frame; a task still active at rotation is inherited by the new bag.

Only one root task ID remains active at a time. A new ID supersedes the old one
if the task manager did not publish a terminal status. An empty task ID or robot
shutdown ends the old task; a new ID that immediately fails validation also
ends the old one. These implicit endings each record one camera snapshot group
when a bag and fresh frames are available. A new `Waiting` status may start a
new generation of a previously completed task ID. Future bags inherit only the
current active ID. `tags` still lists every task that occurred within an
individual bag: two sequential tasks can legitimately share one bag even
though they were not active together. Retained `TaskStatus` messages from a
former publisher are not copied into bags after the task manager restarts.
Older bags retain their original tags and may contain continuously recorded
images; replay must accept both forms.

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
