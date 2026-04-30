# local_planner — ROS2 Code Explanation

## 1. Overview

`local_planner` provides real-time collision-avoidance and waypoint-following for the Unitree Go2 quadruped. It contains two cooperating nodes: **`localPlanner`** selects a collision-free motion direction from a precomputed set of candidate paths, and **`pathFollower`** converts that chosen path into velocity commands sent directly to the robot. Together they form the inner loop of the autonomy stack, sitting between the SLAM/terrain-analysis layer and the hardware drive interface.

---

## 2. High-Level Architecture

```
           ┌─────────────────────────────────────────────────────┐
           │                   local_planner package              │
           │                                                       │
  /state_estimation ──►┐                                          │
  /registered_scan ──►│  localPlanner node                        │
  /terrain_map ────►│  (localPlanner.cpp)                        │
  /joy ────────────►│  • Crops point cloud to adjacentRange       │
  /way_point ──────►│  • Rotates cloud to vehicle frame           │
  /speed ──────────►│  • Scores 343 candidate paths × 36 headings │
  /navigation_boundary►│  • Selects best collision-free group     │──► /path
  /added_obstacles ─►│  • Outputs path in "vehicle" frame         │──► /free_paths
  /check_obstacle ──►┘                                            │
                                                                   │
  /state_estimation ──►┐                                          │
  /path ──────────────►│  pathFollower node                       │
  /joy ───────────────►│  (pathFollower.cpp)                      │
  /speed ─────────────►│  • Pure-pursuit look-ahead               │──► /cmd_vel
  /stop ──────────────►│  • Ramp acceleration/deceleration        │──► /api/sport/request
                        └┘  • Omni-directional TwistStamped output │
                                                                   │
           └─────────────────────────────────────────────────────┘
```

**Key design pattern — precomputed path library:** At startup `localPlanner` reads four binary data files from `paths/`: 343 candidate paths grouped into 7 coarse groups, plus a correspondence table that maps a 2-D voxel grid cell to every path that passes through it. At runtime, each obstacle point is mapped into a voxel, and all paths intersecting that voxel are marked blocked — avoiding any per-path geometric sweep.

---

## 3. Subscribed Topics (Inputs)

### localPlanner node

| Topic | Message Type | Description |
|---|---|---|
| `/state_estimation` | `nav_msgs/Odometry` | Vehicle pose (position + quaternion orientation). Position is offset by `sensorOffsetX/Y` to account for the LiDAR not being at the vehicle center. |
| `/registered_scan` | `sensor_msgs/PointCloud2` | Raw or registered LiDAR point cloud. Used when `useTerrainAnalysis = false`. Each point is treated as an obstacle if its height is within `[minRelZ, maxRelZ]` relative to the vehicle. |
| `/terrain_map` | `sensor_msgs/PointCloud2` | Traversability-annotated cloud from `terrain_analysis`. Used when `useTerrainAnalysis = true`. **Point intensity encodes obstacle height** above ground (see §8). |
| `/joy` | `sensor_msgs/Joy` | Joystick input. Axes[3]/[4] = desired direction/speed; Axes[2] = autonomy mode toggle (< −0.1 enables); Axes[5] = obstacle-check disable (< −0.1 disables). |
| `/way_point` | `geometry_msgs/PointStamped` | Current navigation goal in the map frame. Used to compute direction-to-goal in autonomy mode. |
| `/speed` | `std_msgs/Float32` | Speed command from a higher-level planner (e.g. route planner). Accepted only when `autonomyMode = true` and no recent joystick input (see `joyToSpeedDelay`). |
| `/navigation_boundary` | `geometry_msgs/PolygonStamped` | Polygon boundary. Edges are sampled into obstacle points so the robot stays inside the boundary. Points at the same Z level form a boundary segment. |
| `/added_obstacles` | `sensor_msgs/PointCloud2` | Extra obstacle points injected externally (intensity is overwritten to 200.0 to ensure they always count as hard obstacles). |
| `/check_obstacle` | `std_msgs/Bool` | External toggle to enable/disable obstacle checking. Accepted only in autonomy mode and after `joyToCheckObstacleDelay` seconds since last joystick input. |

### pathFollower node

| Topic | Message Type | Description |
|---|---|---|
| `/state_estimation` | `nav_msgs/Odometry` | Vehicle pose. Also reads `twist.angular.x/y` for inclination-rate safety stop. |
| `/path` | `nav_msgs/Path` | Selected path from `localPlanner`, expressed in the `vehicle` frame at the moment it was published. Poses contain only `position.x/y/z`; orientation fields are unused. |
| `/joy` | `sensor_msgs/Joy` | Joystick override. Axes[5] < −0.1 activates `manualMode` (direct Axes[0/3/4] → yaw/forward/lateral). |
| `/speed` | `std_msgs/Float32` | Desired speed from higher-level planner. Applied when `autonomyMode = true` and no recent joystick input. |
| `/stop` | `std_msgs/Int8` | Safety stop bitmask. Bit 0: block forward; Bit 1: block backward; Bit 2: block CCW yaw; Bit 3: block CW yaw. |

---

## 4. Published Topics (Outputs)

### localPlanner node

| Topic | Message Type | Description |
|---|---|---|
| `/path` | `nav_msgs/Path` | The selected collision-free path as a sequence of 3-D waypoints in the `vehicle` frame. Stamped with the latest odometry timestamp. If no path is found, publishes a single zero-pose (stop command). |
| `/free_paths` | `sensor_msgs/PointCloud2` | Visualization of all collision-free candidate paths for RViz. Only published when `PLOTPATHSET = 1` (compile-time flag, always on). Frame: `vehicle`. |

### pathFollower node

| Topic | Message Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Drive command in the `vehicle` frame. `linear.x` = forward speed (m/s), `linear.y` = lateral speed (m/s), `angular.z` = yaw rate (rad/s). |
| `/api/sport/request` | `unitree_api/Request` | Real-robot Unitree API motion request. Published only when `is_real_robot = true`. Calls `SportClient::Move()` or `SportClient::StopMove()`. |

---

## 5. Services & Actions

None.

---

## 6. Parameters

### localPlanner node

| Parameter | Type | Default (launch) | Effect |
|---|---|---|---|
| `pathFolder` | `string` | `$(find-pkg-share local_planner)/paths` | Directory containing `startPaths.ply`, `paths.ply`, `pathList.ply`, `correspondences.txt`. Must be set correctly or node exits. |
| `vehicleLength` | `double` | `0.3` m | Robot body length. Used to compute the bounding-circle diameter for rotation-obstacle check. |
| `vehicleWidth` | `double` | `0.7` m | Robot body width. Same use as above. |
| `sensorOffsetX/Y` | `double` | `0.0` m | Displacement of LiDAR from robot center in forward/lateral direction. Shifts reported odometry position to match sensor location. |
| `twoWayDrive` | `bool` | `false` | Allow backward motion. If `false`, goal directions behind the robot are clamped to ±90°. |
| `laserVoxelSize` | `double` | `0.05` m | Voxel filter leaf size for `/registered_scan`. Larger values reduce CPU at the cost of resolution. |
| `terrainVoxelSize` | `double` | `0.2` m | Voxel filter leaf size for `/terrain_map`. |
| `useTerrainAnalysis` | `bool` | `true` | `true` = consume `/terrain_map` (intensity = obstacle height); `false` = consume `/registered_scan` (raw height filter). |
| `checkObstacle` | `bool` | `true` | Master switch for obstacle avoidance. Disable for debugging. |
| `checkRotObstacle` | `bool` | `false` | Check obstacles inside the rotation footprint to restrict in-place yaw. Rarely needed, adds cost. |
| `adjacentRange` | `double` | `3.0` m | Radius of point cloud crop around the vehicle. All path scoring happens within this sphere. Smaller = faster but shorter lookahead. |
| `obstacleHeightThre` | `double` | `0.3` m | Minimum intensity (= obstacle height) in the terrain map for a point to block a path. **Key tuning knob for UTLidar noise** — matches the 0.3 m note in CLAUDE.md. |
| `groundHeightThre` | `double` | `0.1` m | Minimum height to apply a cost penalty (soft obstacle). Used when `useCost = true`. |
| `costHeightThre` | `double` | `0.1` m | Height at which cost penalty reaches maximum (penalty = 1 − h/costHeightThre). |
| `costScore` | `double` | `0.02` | Minimum penalty score floor (prevents zero score for passable terrain). |
| `useCost` | `bool` | `false` | Enable soft cost for low-height obstacles instead of hard blocking. |
| `pointPerPathThre` | `int` | `2` | Number of obstacle points that must fall on a path to consider it blocked. Increase to 3–4 on noisy sensors to reduce false blockages. |
| `minRelZ` / `maxRelZ` | `double` | `−0.5` / `0.25` m | Height range for obstacle points in the raw-scan mode (relative to vehicle Z). Adjust `maxRelZ` upward if tall obstacles are missed. |
| `maxSpeed` | `double` | `0.7` m/s | Speed normalization denominator. Used to scale `joySpeed` ∈ [0, 1]. |
| `dirWeight` | `double` | `0.02` | Weight multiplier on angular deviation penalty in path scoring. Smaller = stronger directional bias. |
| `dirThre` | `double` | `90.0` deg | Maximum angular deviation from the desired heading to consider a path. Paths beyond this angle are skipped entirely. |
| `dirToVehicle` | `bool` | `false` | If `true`, `dirThre` is applied relative to the vehicle's forward axis instead of the goal direction. |
| `pathScale` | `double` | `0.75` | Scale factor applied to precomputed path geometry. 1.0 = full size (~3.2 m reach); 0.75 = tighter paths for Go2. |
| `minPathScale` | `double` | `0.5` | Minimum scale the planner will fall back to when no path is found at larger scale. |
| `pathScaleStep` | `double` | `0.25` | Step size for reducing `pathScale` during fallback search. |
| `pathScaleBySpeed` | `bool` | `true` | Automatically reduce path scale proportional to speed (shorter paths at low speed). |
| `minPathRange` | `double` | `1.0` m | Minimum forward range after all scale fallbacks. Node publishes a stop pose if even this fails. |
| `pathRangeStep` | `double` | `0.5` m | Step for reducing path range in the secondary fallback. |
| `pathRangeBySpeed` | `bool` | `true` | Automatically reduce path range proportional to speed. |
| `pathCropByGoal` | `bool` | `true` | Crop paths at the goal distance (+ `goalClearRange`). Prevents planning past the goal. |
| `autonomyMode` | `bool` | `false` | Start in autonomy mode without joystick. |
| `autonomySpeed` | `double` | `0.5` m/s | Default speed in autonomy mode. |
| `joyToSpeedDelay` | `double` | `2.0` s | Time after last joystick input before `/speed` commands are accepted. |
| `joyToCheckObstacleDelay` | `double` | `5.0` s | Time after last joystick input before `/check_obstacle` commands are accepted. |
| `goalClearRange` | `double` | `0.5` m | Extra forward margin beyond goal distance when cropping paths. |
| `goalX` / `goalY` | `double` | `0.0` | Static initial goal position (map frame). Can be overridden at runtime by `/way_point`. |

### pathFollower node

| Parameter | Type | Default (launch) | Effect |
|---|---|---|---|
| `sensorOffsetX/Y` | `double` | `0.0` m | Same meaning as in localPlanner — must be identical. |
| `pubSkipNum` | `int` | `1` | Publish `/cmd_vel` every `pubSkipNum + 1` control loop iterations. 1 = publish every other loop (50 Hz effective at 100 Hz loop). |
| `twoWayDrive` | `bool` | `false` | Enable backward motion; must match localPlanner setting. |
| `lookAheadDis` | `double` | `0.5` m | Pure-pursuit look-ahead distance. Larger = smoother but more overshooting on tight turns. |
| `yawRateGain` | `double` | `1.5` | Proportional gain for yaw-rate control when moving. Increase for snappier heading correction. |
| `stopYawRateGain` | `double` | `1.5` | Yaw-rate gain when nearly stopped. Separate from `yawRateGain` for stability at low speed. |
| `maxYawRate` | `double` | `80.0` deg/s | Yaw-rate saturation limit. |
| `maxSpeed` | `double` | `0.7` m/s | Maximum forward speed. Must match localPlanner's `maxSpeed`. |
| `maxAccel` | `double` | `2.0` m/s² | Acceleration ramp rate. Applied at 100 Hz, so effective delta per step = `maxAccel / 100`. |
| `switchTimeThre` | `double` | `1.0` s | Minimum time between forward/backward direction switches in `twoWayDrive` mode. Prevents rapid oscillation. |
| `dirDiffThre` | `double` | `0.4` rad | Heading error below which the robot starts accelerating forward. Above this, the robot rotates in place. |
| `omniDirDiffThre` | `double` | `1.5` rad | Heading error threshold used near the goal (`dis < goalCloseDis`). Larger value allows omni-directional final approach. |
| `noRotSpeed` | `double` | `10.0` m/s | Speed above which yaw rate is forced to zero (effectively never reached; safety guard). |
| `stopDisThre` | `double` | `0.3` m | Distance to path end at which speed is zeroed (arrival detection). |
| `slowDwnDisThre` | `double` | `0.75` m | Distance from path end at which deceleration begins. Approach speed = `joySpeed × (endDis / slowDwnDisThre)`. |
| `useInclRateToSlow` | `bool` | `false` | Slow down for `slowTime1 + slowTime2` when roll/pitch rate exceeds `inclRateThre`. |
| `inclRateThre` | `double` | `120.0` deg/s | Angular-rate threshold for inclination-rate slow-down. |
| `slowRate1/2` | `double` | `0.25` / `0.5` | Speed multipliers during the two slow-down phases. |
| `slowTime1/2` | `double` | `2.0` / `2.0` s | Duration of each slow-down phase. |
| `useInclToStop` | `bool` | `false` | Emergency stop when roll or pitch exceeds `inclThre`. |
| `inclThre` | `double` | `45.0` deg | Roll/pitch limit before emergency stop. |
| `stopTime` | `double` | `5.0` s | Duration of emergency stop. |
| `noRotAtStop` | `bool` | `false` | Zero yaw command when joystick speed is zero. |
| `noRotAtGoal` | `bool` | `true` | Zero yaw rate once the robot is within `stopDisThre` of the path end. |
| `autonomyMode` | `bool` | `false` | Must match localPlanner setting. |
| `autonomySpeed` | `double` | `0.5` m/s | Must match localPlanner setting. |
| `joyToSpeedDelay` | `double` | `2.0` s | Must match localPlanner setting. |
| `goalCloseDis` | `double` | `0.4` m | Distance threshold for switching to the `omniDirDiffThre` heading tolerance. |
| `is_real_robot` | `bool` | `true` | When `true`, also publishes Unitree API motion requests to `/api/sport/request`. |

---

## 7. Configuration Files

There is no YAML config file. All parameters are set in the launch file:

**`launch/local_planner.launch`**

This is a ROS1-style XML launch file (not a Python launch file). Key groups of arguments:

| Launch Argument | Default | Notes |
|---|---|---|
| `maxSpeed` | `0.7` | Passed to both nodes; must be consistent. |
| `autonomyMode` | `false` | Set to `true` to start without joystick. |
| `autonomySpeed` | `0.5` | Effective only when `autonomyMode = true`. |
| `twoWayDrive` | `false` | Enable for bidirectional navigation. |
| `goalCloseDis` | `0.4` | Arrival radius for pathFollower. |
| `is_real_robot` | `true` | Set to `false` in simulation. |
| `sensorOffsetX/Y` | `0.0` | Adjust if LiDAR is offset from robot center. |

The `pathFolder` is resolved to the package's installed `share/local_planner/paths` directory automatically.

---

## 8. Required Data Formats

### `/terrain_map` (PointCloud2, `sensor_msgs/PointCloud2`)
- **Field layout:** `x`, `y`, `z` (float32) — 3-D position in the global map frame.
- **`intensity` field (float32) encodes obstacle height above ground** — this is the critical convention:
  - `intensity > obstacleHeightThre` (default 0.3 m): hard obstacle, blocks path.
  - `groundHeightThre < intensity ≤ obstacleHeightThre`: soft obstacle, contributes cost penalty when `useCost = true`.
  - `intensity ≤ groundHeightThre`: flat ground, ignored.
- This encoding is produced by `terrain_analysis` and `terrain_analysis_ext`. Do not substitute a raw LiDAR scan here.

### `/registered_scan` (PointCloud2)
- Standard `x`, `y`, `z` float32 fields. Intensity is not used.
- Used only when `useTerrainAnalysis = false`.
- Points are filtered to `[minRelZ, maxRelZ]` relative to the vehicle — there is no intensity-based obstacle classification.

### `/path` (nav_msgs/Path) — internal between the two nodes
- All poses are in the `vehicle` frame as it was at the moment `localPlanner` computed the path.
- Only `pose.position.x/y/z` are used; orientation quaternions are not set.
- A single zero-pose (`x=y=z=0`) signals "no path found — stop."

### `/cmd_vel` (geometry_msgs/TwistStamped)
- Frame: `vehicle`.
- `linear.x`: forward speed (m/s), positive = forward.
- `linear.y`: lateral speed (m/s), positive = left.
- `angular.z`: yaw rate (rad/s), positive = CCW.

### `/navigation_boundary` (geometry_msgs/PolygonStamped)
- Polygon vertices define boundary edges. Only edges where consecutive vertices share the **same Z value** are treated as active boundary segments.
- Edges are sampled into obstacle points at `terrainVoxelSize` spacing with `intensity = 100.0` and repeated `pointPerPathThre` times to guarantee they block paths.

### `/stop` (std_msgs/Int8) — bitmask
| Bit | Value | Effect |
|---|---|---|
| 0 | 1 | Block forward motion (`vehicleSpeed > 0 → 0`) |
| 1 | 2 | Block backward motion |
| 2 | 4 | Block positive (CCW) yaw rate |
| 3 | 8 | Block negative (CW) yaw rate |

### Precomputed path data files (`paths/`)
- **`startPaths.ply`**: PLY binary file, fields `x y z groupID`. Each point is a waypoint; `groupID ∈ [0, 6]` assigns it to one of 7 motion groups (straight, slight left/right, large left/right, etc.).
- **`paths.ply`**: PLY binary file, fields `x y z pathID intensity`. Full-resolution path geometry for visualization only (1 in 30 points are loaded). 343 paths total.
- **`pathList.ply`**: PLY binary file, one entry per path. Fields `endX endY endZ pathID groupID`. Stores the end-point direction used for goal-alignment scoring.
- **`correspondences.txt`**: Text file. Each line starts with a grid voxel ID followed by path IDs that pass through that voxel, terminated by `-1`. Grid is 161 × 451 cells, cell size 0.02 m, covering a 3.2 m × 4.5 m fan in front of the vehicle.

---

## 9. Launch Integration

```bash
# Standalone (rarely used directly):
ros2 launch local_planner local_planner.launch

# Typical system launch (called from vehicle_simulator):
./system_real_robot.sh        # includes local_planner with real-robot defaults
./system_simulation.sh        # is_real_robot:=false
```

**Override examples:**

```bash
ros2 launch local_planner local_planner.launch \
  maxSpeed:=0.5 \
  autonomyMode:=true \
  autonomySpeed:=0.3 \
  twoWayDrive:=true
```

**Required nodes that must be running first:**

| Dependency | Why |
|---|---|
| SLAM (`point_lio_unilidar`) | Provides `/state_estimation` odometry |
| `terrain_analysis` | Provides `/terrain_map` when `useTerrainAnalysis = true` |
| `sensor_scan_generation` | Provides `/registered_scan` when `useTerrainAnalysis = false` |
| Static TF (`transform_sensors`) | Publishes `sensor` → `vehicle` and `sensor` → `camera` frames |
| `unitree_api` (real robot only) | Required for `/api/sport/request` to reach the robot |

**TF frames used:**
- `vehicle` — body-fixed frame, origin at robot center (offset by `sensorOffsetX/Y` from sensor).
- `sensor` — LiDAR frame (parent of `vehicle`).
- The static transform `sensor` → `vehicle` is published inside this launch file itself.

---

## 10. Known Limitations & Tuning Tips

- **High noise from UTLidar** — the launch file sets `obstacleHeightThre = 0.3` (matching the CLAUDE.md note that obstacles must be > 0.3 m above ground to be detected). Lowering this value will cause false blockages from ground noise.

- **`pointPerPathThre` vs. false stops** — the default is 2, meaning a single outlier point will not block a path. On very noisy sensors, raise to 3–4. On clean sensors, 1 gives faster reaction.

- **`pathScale` fallback loop** — if no path is found at the nominal scale, the planner shrinks the scale by `pathScaleStep` and retries, down to `minPathScale`. If still blocked, it reduces `pathRange`. This can cause brief hesitation in tight spaces. To reduce hesitation: lower `minPathScale` or lower `obstacleHeightThre` slightly.

- **`dirThre = 90°`** — only paths within ±90° of the desired heading are ever considered. In a dead-end scenario the robot will stop (publish zero pose) rather than turn around, unless `twoWayDrive = true`.

- **`is_real_robot` must be `false` in simulation** — if left `true` in simulation the node will publish to `/api/sport/request` which may confuse the Unity bridge.

- **Path library is fixed at 343 paths** — this is a compile-time constant (`pathNum = 343`). Regenerating the path library requires re-running `path_generator.m` (MATLAB) in `paths/`. The grid geometry constants (`gridVoxelOffsetX = 3.2`, `gridVoxelOffsetY = 4.5`, `searchRadius = 0.55`) must match the values used when the correspondence table was generated.

- **`pubSkipNum = 1`** — pathFollower publishes at 50 Hz (every other loop at 100 Hz). Lowering to 0 doubles output to 100 Hz for more responsive control at high speed.

- **Inclination safety (`useInclToStop`, `useInclRateToSlow`)** — both are `false` by default. Enable `useInclRateToSlow` for terrain with sudden drops; enable `useInclToStop` as a hard safety guard on steep slopes.
