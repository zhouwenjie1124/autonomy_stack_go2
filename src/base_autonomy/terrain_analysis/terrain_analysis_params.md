# Terrain Analysis Parameter Guide

Source: `terrain_analysis/src/terrainAnalysis.cpp`

The node runs at 100 Hz, processes `/registered_scan` (map-frame point cloud from SLAM), and publishes `/terrain_map` — a point cloud where each point's `intensity` encodes height above the estimated ground. The local planner uses this to classify traversable vs. obstacle cells.

---

## Fixed Internal Geometry

These are hardcoded — not exposed as parameters — but they determine what the tunable parameters mean.

| Constant | Value | Meaning |
|---|---|---|
| `terrainVoxelSize` | 1.0 m | Side length of each terrain accumulation cell |
| `terrainVoxelWidth` | 21 | Grid is 21×21 cells → covers ±10 m around the robot |
| `planarVoxelSize` | 0.2 m | Resolution of the ground-estimation and obstacle grid |
| `planarVoxelWidth` | 51 | Planar grid is 51×51 → covers ±5 m at 0.2 m resolution |

The terrain voxel grid slides (rolls over) as the robot moves, so old scans are kept until they decay.

---

## Point Cloud Preprocessing

### `scanVoxelSize` (default 0.05 m)
Leaf size for the voxel-grid downsampler applied **per terrain voxel cell** before accumulation.

- Smaller → denser map, higher CPU cost.
- Larger → faster but less detail; very large values (>0.2) may cause poor ground estimation on rough terrain.
- **Go2 / MID360 recommendation:** 0.05–0.1 m is a good range. At 0.1 the map is still dense enough and CPU load drops noticeably.

### `minRelZ` (default −1.5 m) and `maxRelZ` (default 0.5 m)
Height window relative to the vehicle for **ingesting points**. Points outside `[vehicleZ + minRelZ, vehicleZ + maxRelZ]` are discarded immediately in the scan callback and are never accumulated.

- `minRelZ`: how far below the robot to accept ground points. −1.5 is generous for slopes; tighten to −0.5 if underground reflections are polluting the map.
- `maxRelZ`: how far above the robot to keep points. 0.5 m means objects taller than 0.5 m above the robot body are clipped. Increase if the robot needs to detect overhead obstacles; keep low to ignore ceiling/canopy.

### `disRatioZ` (default 0.2)
The effective Z window **expands with distance** by `disRatioZ × horizontalDistance`. This compensates for LiDAR tilt on slopes: a point 5 m away gets ±1 m extra Z tolerance.

- Increase on hilly terrain to avoid clipping legitimate far-away ground returns.
- Decrease in flat indoor environments to keep the window tight.

---

## Map Memory and Decay

### `decayTime` (default 2.0 s)
Points older than `decayTime` seconds are **removed from the accumulated map** unless they are within `noDecayDis` of the robot. This prevents stale obstacles from blocking the robot after the environment changes.

- Shorter → the map forgets obstacles quickly; good for dynamic scenes.
- Longer → more stable map; better on slow hardware or with a noisy LiDAR.
- **Typical range:** 1–5 s. For Go2 indoors at moderate speed, 2 s is reasonable.

### `noDecayDis` (default 4.0 m, overridden to 0.0 in launch)
Points within this radius of the robot **never decay**, regardless of `decayTime`. This keeps the area immediately around the robot well-mapped.

- The launch file sets this to 0.0, which disables the no-decay zone — all points age uniformly.
- Setting it to e.g. 3.0 m ensures the robot's immediate surroundings stay mapped even when standing still for a long time.

### `clearingDis` (default 8.0 m)
When a manual map-clear event is triggered (joystick button 5 or `/map_clearing` topic), all points within this radius are erased.

- Increase if the robot needs a wider reset after a SLAM correction.
- Decrease if you only want to clear the area the robot is standing in.

### `voxelPointUpdateThre` (default 100)
A terrain voxel cell is **reprocessed** (downsampled and old points pruned) when it has accumulated this many new points.

- Lower → more frequent processing, smoother map updates, higher CPU.
- Higher → batch updates, lower CPU, but the map reacts more slowly.

### `voxelTimeUpdateThre` (default 2.0 s)
A terrain voxel cell is also reprocessed if it has not been updated for this many seconds. Acts as a timeout so cells with few points still get cleaned up periodically.

---

## Ground Estimation

### `useSorting` (default true)
Chooses the ground-height estimation strategy for each planar cell:

- `true` → **Quantile method:** sort all point Z-values in the cell, pick the `quantileZ` percentile. More robust to noise and outliers.
- `false` → **Minimum method:** use the lowest Z-value. Simple but sensitive to ground-level noise or LiDAR multi-path returns.

**Recommendation:** keep `true` for outdoor use with the MID360.

### `quantileZ` (default 0.25)
When `useSorting` is true, the ground estimate is the Z-value at this percentile of the sorted points in each planar cell.

- 0.0 → same as minimum (most aggressive, picks lowest point).
- 0.25 → bottom quartile — good balance between filtering noise and tracking true ground.
- 0.5 → median — more conservative, ground estimate is higher, so less of the slope registers as an obstacle.

**Tuning tip:** if the ground is falsely marked as obstacle (terrain map is too "busy"), increase `quantileZ` toward 0.5.

### `limitGroundLift` (default false) and `maxGroundLift` (default 0.15 m)
When `limitGroundLift` is true, the quantile-estimated ground cannot be more than `maxGroundLift` above the **minimum** point in the cell. This prevents the ground estimate from jumping up onto a low obstacle.

- Useful when walls or boxes sit at ground level and would otherwise raise the ground estimate.
- Set `maxGroundLift` to roughly half the robot's step height.

### `considerDrop` (default false)
When false, `disZ = point.z − groundEstimate` and only **positive** disZ (above-ground) points become obstacles.
When true, `disZ = |point.z − groundEstimate|`, so **drops** (holes, stairs down) also contribute to the obstacle height. Enable this if the robot must avoid negative steps.

---

## Obstacle Height Thresholding

### `vehicleHeight` vs `vehicleZ` — key distinction

| | `vehicleZ` | `vehicleHeight` |
|---|---|---|
| **Type** | Position (world frame, meters) | Size threshold (meters) |
| **Changes at runtime** | Yes — updated every odometry message | No — fixed parameter |
| **Meaning** | Where the robot currently is (Z coordinate of IMU in world) | How tall the robot body is |
| **Used for** | Vertical reference: defines the center of all Z crop windows | Output filter: discards points above the robot's clearance |
| **Affects** | Point ingestion, map pruning, ground estimation pool, dynamic obstacle angles | Only the final terrain map output and no-data synthetic obstacles |

In short: `vehicleZ` answers **"where is the robot"**, `vehicleHeight` answers **"how tall is the robot"**. They operate at completely different stages of the pipeline and have no mathematical relationship with each other.

```
world Z axis
│
│  point.z  ──────────────────────────── LiDAR return (world frame)
│               ↑ disZ = point.z − groundLevel
│  groundLevel  ──────────────────────── estimated per cell (planarVoxelElev)
│
│               ← vehicleHeight = 1.5 m → output filter: keep if 0 ≤ disZ < vehicleHeight
│
│  vehicleZ  ──────────────────────────── IMU position (world frame, moves with robot)
│               used to crop points: minRelZ < (point.z − vehicleZ) < maxRelZ
```

### `vehicleHeight` (default 1.5 m)
A point is kept in the output terrain map only if its height above the ground estimate satisfies `0 ≤ disZ < vehicleHeight`. Points above `vehicleHeight` are clipped (they are above the robot and irrelevant). The point's `intensity` in the output cloud equals `disZ`.

- Set to the robot's actual clearance height plus some margin.
- Go2 body height is ~0.28 m; 1.5 m gives generous headroom. Reducing to 0.8–1.0 m is safe and removes ceiling returns indoors.

### `minBlockPointNum` (default 10)
A planar cell must contain at least this many points before any of its points are output as obstacles. Cells with fewer points are treated as having no data.

- Lower → more sensitive, more false positives from sparse returns.
- Higher → more robust but may miss small obstacles.
- For the MID360 (dense scan), 10 is reasonable. For a sparser sensor, lower to 3–5.

---

## Dynamic Obstacle Clearing (`clearDyObs`)

This feature attempts to remove **dynamic obstacles** (people, moving objects) from the map by reasoning about whether a point could have been seen through by the sensor in the current scan.

### `clearDyObs` (default false)
Master switch. When false, all following `DyObs` parameters are irrelevant.

Enable if the robot frequently gets stuck by moving people or objects that the map keeps remembering.

### `minDyObsDis` (default 0.3 m)
Points closer than this distance are always considered dynamic (counted without angle check). Avoids divide-by-zero and handles near-field noise.

### `minDyObsAngle` (default 0.0°)
A point is a dynamic-obstacle candidate only if the elevation angle from the sensor to the point (above `minDyObsRelZ`) exceeds this angle. Positive values exclude near-horizontal rays (which tend to be stable ground returns).

### `minDyObsRelZ` (default −0.3 m)
The vertical reference offset used in the angle calculation. The angle is computed as `atan2(pointZ − vehicleZ − minDyObsRelZ, horizontalDis)`. Adjusting this shifts which elevation angles qualify.

### `absDyObsRelZThre` (default 0.2 m)
If `|pointZ − vehicleZ| < absDyObsRelZThre`, the point is counted as dynamic regardless of the VFOV check. This catches low-height dynamic objects that might otherwise be excluded by the FOV filter.

### `minDyObsVFOV` / `maxDyObsVFOV` (default −16° / +16°)
After rotating the point into the sensor frame (correcting for vehicle roll and pitch), the point is counted as a dynamic-obstacle candidate only if its vertical angle lies within this FOV. Set these to match the **actual vertical FOV** of the sensor so only rays that the sensor physically scanned are used for clearing.

- MID360: ±40° VFOV → could use −40 / +40, but a tighter range reduces false clearing.

### `minDyObsPointNum` (default 1)
A planar cell is marked as a dynamic obstacle (and cleared from the map) if the dynamic-obstacle point count reaches this threshold. Increase to require more evidence before clearing.

---

## No-Data Obstacle (`noDataObstacle`)

This feature marks the **region directly in front of the robot** as an obstacle when there is no scan data — useful to prevent driving into sensor blind spots.

### `noDataObstacle` (default false)
Master switch. Enable if the robot drives into areas with no LiDAR coverage (e.g., the sensor has a forward blind spot or the scan is blocked).

### `noDataAreaMinX` / `noDataAreaMaxX` (default 0.3 / 1.8 m)
### `noDataAreaMinY` / `noDataAreaMaxY` (default −0.9 / +0.9 m)
Bounding box in the **robot-forward frame** (X = forward, Y = left) defining the blind-spot zone to monitor. Only voxels inside this box are checked for missing data.

- Tune these to match the actual sensor blind spot geometry.
- For the Go2 with MID360 mounted on top, the forward blind spot (body occlusion) is roughly 0.3–1.5 m forward and ±0.4 m lateral.

### `maxElevBelowVeh` (default −0.6 m)
A cell inside the no-data zone is treated as missing-data only if its ground estimate is lower than `vehicleZ + maxElevBelowVeh`. This avoids false obstacles on slopes where the ground is legitimately visible but low.

### `noDataBlockSkipNum` (default 0)
Number of voxel layers to **dilate** the no-data obstacle region outward. 0 means no dilation (exact boundary). Increase to add a safety margin around the blind spot.

---

## Current Launch Values vs. Defaults

| Parameter | Default (code) | Launch value | Notes |
|---|---|---|---|
| `scanVoxelSize` | 0.05 | 0.05 | — |
| `decayTime` | 2.0 | 2.0 | — |
| `noDecayDis` | 4.0 | **0.0** | No-decay zone disabled |
| `clearingDis` | 8.0 | 8.0 | — |
| `useSorting` | true | true | — |
| `quantileZ` | 0.25 | 0.25 | — |
| `considerDrop` | false | false | — |
| `limitGroundLift` | false | false | — |
| `maxGroundLift` | 0.15 | 0.15 | — |
| `clearDyObs` | false | false | — |
| `minDyObsRelZ` | −0.5 | **−0.3** | Slightly less aggressive |
| `maxRelZ` | 0.2 | **0.5** | Taller obstacles captured |
| `minRelZ` | −1.5 | −1.5 | — |
| `vehicleHeight` | 1.5 | 1.5 | — |
| `minBlockPointNum` | 10 | 10 | — |

---

## Quick Tuning Reference for Go2 + MID360

| Symptom | Parameter to adjust |
|---|---|
| Ground falsely marked as obstacle | Increase `quantileZ` (0.25 → 0.4) |
| Small obstacles missed | Decrease `minBlockPointNum` (10 → 5) |
| Ghost obstacles from moving people | Enable `clearDyObs`, tune `minDyObsVFOV` to ±40° |
| Map too slow to update | Decrease `voxelPointUpdateThre` (100 → 50) |
| High CPU from terrain analysis | Increase `scanVoxelSize` (0.05 → 0.1) |
| Stale obstacles after map clear | Decrease `decayTime` (2.0 → 1.0) |
| Robot drives into blind spot | Enable `noDataObstacle`, tune bounding box |
| Tall obstacles clipped | Increase `maxRelZ` (0.5 → 1.0) |
| Underground noise pollutes map | Tighten `minRelZ` (−1.5 → −0.5) |
