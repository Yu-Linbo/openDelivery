# Relocalization

Per-robot scan-assisted relocalization. Launching `manager.launch.py` places the node in the
robot namespace, for example `/robot2/relocalization`.

## Interfaces

- Subscribes `robot_status` (`custom_msgs_srvs/RobotStatus`) to select the current map and
  inspect localization state.
- Subscribes `scan_2d` and the transient-local static `map`.
- Provides `record_relocalization` (`RecordRelocalization`): while status is `ready`, captures
  the current `map -> base_footprint` pose, laser extrinsic and one scan frame. The atomic binary
  record is stored at `<map_root>/<current_map>/relocalization/<record_id>.rloc`.
- Provides `relocalize` (`Relocalize`) and publishes successful results to `initial` for AMCL.

## Modes

- Mode 0 (`MODE_HISTORY`): lazily loads all records belonging to the current map, searches around
  every recorded pose, and ranks candidates using occupancy-map endpoint hits plus current-to-saved
  scan correlation.
- Mode 1 (`MODE_POSE_FIRST`): searches around the supplied map-frame pose using occupancy-map
  matching. If its score is below `pose_first_threshold`, it automatically runs mode 0.

Expected workflow:

1. With localization known to be accurate (`robot_status=ready`), an external application calls
   the record service with a stable ID.
2. After a map switch it calls mode 1 with the pre-switch pose. On first boot, it calls mode 0.

Records are never carried across maps: the current map name from `RobotStatus` selects both the
storage directory and the records eligible for matching. A map change invalidates the cached grid
until a new `OccupancyGrid` is received.

The monitor's **记录重定位点** action calls the record service and, only after a successful capture,
adds a `relocalization` point with the same ID to `<map>_points.json`. History loading reconciles
the binary records against that point file: deleting the point through the existing map point UI
also causes the unreferenced `.rloc` record to be removed on the next mode-0/fallback load. Invalid
point JSON is left untouched so a damaged metadata file cannot erase all scan records.
