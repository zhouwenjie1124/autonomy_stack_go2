# sensorScanGeneration.cpp — ROS2 Code Explanation

## 1. Overview

This node solves a specific coordinate-frame mismatch problem in the autonomy stack. SLAM (`point_lio_unilidar`) outputs the registered point cloud in the **map frame** (`/registered_scan`), but the terrain analysis and local planner nodes expect the point cloud to be in the **sensor frame at the moment of the scan**. This node time-synchronizes the registered scan with the SLAM odometry, inverts the map→sensor transform, re-expresses every point back into the sensor frame, and republishes both the cloud and an odometry message stamped to the exact scan time.

## 2. High-Level Architecture

Single node (`sensor_scan`), single callback triggered by time-synchronized pairs of odometry + point cloud.

```
/state_estimation  (Odometry)  ──┐
                                  ├─ ApproximateTime sync ──> laserCloudAndOdometryHandler
/registered_scan   (PointCloud2) ─┘
                                          │
                          ┌───────────────┼───────────────────┐
                          ▼               ▼                   ▼
             /state_estimation_at_scan  TF broadcast     /sensor_scan
               (Odometry, re-stamped)  map→sensor_at_scan  (PointCloud2 in
                                                           sensor_at_scan frame)
```

**Core transform logic:**  
For each incoming map-frame point `p`, compute `p_sensor = T_map_sensor⁻¹ * p`, where `T_map_sensor` is built directly from the synchronized odometry pose.

## 3. Subscribed Topics (Inputs)

| Topic | Message Type | Description |
|---|---|---|
| `/state_estimation` | `nav_msgs/Odometry` | SLAM pose output (map frame). Provides the transform used to invert points back to sensor frame. |
| `/registered_scan` | `sensor_msgs/PointCloud2` | LiDAR scan already registered to the map by SLAM. Points are in the map frame (`pcl::PointXYZ`). |

Both subscriptions use `BEST_EFFORT` reliability, `KEEP_LAST` history depth 1 (sensor QoS), synchronized with `ApproximateTime` queue depth 100.

## 4. Published Topics (Outputs)

| Topic | Message Type | Description |
|---|---|---|
| `/state_estimation_at_scan` | `nav_msgs/Odometry` | Same pose as input but re-stamped to the LiDAR scan time, with `frame_id="map"` and `child_frame_id="sensor_at_scan"`. |
| `/sensor_scan` | `sensor_msgs/PointCloud2` | Point cloud with every point transformed back to the sensor frame at scan time. `frame_id="sensor_at_scan"`. |

## 5. Services & Actions

None.

## 6. Parameters

None declared. All topic names are hardcoded.

## 7. Configuration Files

None.

## 8. Required Data Formats

- **`/registered_scan`**: Must contain `pcl::PointXYZ`-compatible fields (`x`, `y`, `z`). Points must already be in the map frame (as output by Point-LIO).
- **`/state_estimation`**: Pose must be map→sensor transform expressed as a `geometry_msgs/Pose` inside the Odometry message.
- **TF frame `sensor_at_scan`**: Dynamically broadcast by this node — downstream consumers (terrain_analysis, local_planner) must expect this frame name.
- **Timestamp**: Output cloud and odometry are stamped with the **LiDAR scan timestamp** (`laserCloud2->header.stamp`), not the odometry timestamp. This is intentional — it time-aligns the cloud with when the scan actually occurred.

## 9. Launch Integration

Launched from `vehicle_simulator/launch/` (commented out in `system_real_robot.launch` by default):

```xml
<!-- <include file="$(find-pkg-share sensor_scan_generation)/launch/sensor_scan_generation.launch" /> -->
```

**Dependencies that must be running first:**
- `point_lio_unilidar` — must be publishing both `/state_estimation` and `/registered_scan`

This node is **optional** in the real-robot stack. Enable it only when a downstream node requires `/sensor_scan` or `/state_estimation_at_scan` (e.g. `terrain_analysis` when configured to use the sensor-frame cloud).

## 10. Known Limitations & Tuning Tips

- **ApproximateTime queue depth is 100** — large queue prevents drop but increases latency if odometry and scan rates diverge significantly.
- **No parameter interface** — topic names are hardcoded; changing them requires recompiling.
- **Global state** — point clouds and transform are stored as global variables, not class members. This is safe for a single-threaded `rclcpp::spin` but would be unsafe with a multi-threaded executor.
- **`newTransformToMap` flag declared but never used** — dead code, has no effect.
