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
- Provides `relocalize` (`Relocalize`). Same-map results are published to `initial`; a
  different-map result is sent through `localize_nav_command` so `task_manager` applies the
  existing map/status/initial-pose transition.

## Modes

- Mode 0 (`MODE_HISTORY`): searches every record on the current map first. If its best score is
  below `history_match_threshold`, it discovers other maps with saved records, loads their
  YAML/PGM files without changing the live map, and searches all of their records. A result on
  another map requests a real map switch before localization continues.
- Mode 1 (`MODE_POSE_FIRST`): searches around the supplied map-frame pose using occupancy-map
  matching. If its score is below `pose_first_threshold`, it searches historical records on the
  current target map only, so an intentional elevator map switch cannot be reversed.

Expected workflow:

1. With localization known to be accurate (`robot_status=ready`), an external application calls
   the record service with a stable ID.
2. After a map switch it calls mode 1 with the pre-switch pose.
3. On first startup transition from `localizing` to `localization_lost`, the node automatically
   retries mode 0 until inputs are ready. This can be disabled with
   `auto_relocalize_on_startup=false`.

Records remain owned by their map directory. Cross-map scoring uses each candidate map's own
YAML/PGM occupancy data. A live map change invalidates the cached grid until a new
`OccupancyGrid` is received.

Relevant optional parameters are `history_match_threshold=0.55`,
`auto_relocalize_on_startup=true`, `auto_relocalize_retry_limit=30`, and
`auto_relocalize_retry_period_sec=0.5`.

The monitor's **记录重定位点** action calls the record service and, only after a successful capture,
adds a `relocalization` point with the same ID to `<map>_points.json`. History loading reconciles
the binary records against that point file: deleting the point through the existing map point UI
also causes the unreferenced `.rloc` record to be removed on the next mode-0/fallback load. Invalid
point JSON is left untouched so a damaged metadata file cannot erase all scan records.
