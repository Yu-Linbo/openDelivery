# Gazebo ground-truth localization

This simulation-only localization backend publishes exact `map -> odom` from
Gazebo model state. It converts the shared Gazebo world pose into the selected
occupancy-map coordinates using the floor include pose in the world file and
the map YAML/PGM dimensions.

The node also consumes the robot's `initial` pose topic. An initial-pose command
realigns `map -> world` without teleporting the Gazebo model, so the normal Web
relocalization flow works consistently for `slam_toolbox`, legacy AMCL, and this
truth backend.

Select it with `localization_method:=gazebo_ground_truth`. It is intended only
for simulation and must never be used as a production localization source.
