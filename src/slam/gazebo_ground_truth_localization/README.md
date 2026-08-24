# Gazebo ground-truth localization

This simulation-only localization backend publishes `map -> odom` from Gazebo
model state. It converts the shared Gazebo world pose into the selected
occupancy-map coordinates using the floor include pose in the world file and
the map YAML/PGM dimensions. With no artificial relocalization error, the
published pose is the exact Gazebo truth in `map`.

The node also consumes the robot's `initial` pose topic. An initial-pose command
is published immediately without teleporting the Gazebo model, then its
map-coordinate error is gradually corrected back to the immutable Gazebo truth.
Translation and yaw converge independently at bounded rates while normal robot
motion continues to track the current Gazebo model state.

The node follows `/<robot>/robot_status.current_map` and reloads the matching
`<map-root>/<map>/<map>.yaml` transform. This keeps its Gazebo truth aligned
with dynamic floor switches without restarting the localization process.

Optional node parameters control the convergence rates. Existing launch files
need no changes:

- `linear_correction_speed` (default `0.10` m/s)
- `angular_correction_speed` (default `0.0872665` rad/s, approximately 5 deg/s)

Select it with `localization_method:=gazebo_ground_truth`. It is intended only
for simulation and must never be used as a production localization source.
