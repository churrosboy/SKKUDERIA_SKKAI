# SKKUDERIA_SKKAI

ROS 2 (Humble) autonomous racing stack for an F1TENTH car equipped with a
SOSLAB GL-5 2D lidar. The stack is based on the ForzaETH race stack; this
document covers the lidar setup and the algorithms that consume lidar data.

## 1. SOSLAB GL-5 lidar setup

### Sensor

| Item | Value |
|---|---|
| Model | SOSLAB GL-5 (2D, Ethernet) |
| Field of view | 270 deg (+/- 135 deg), 90 deg blind sector behind the car |
| Angular resolution | 0.18 deg per beam (about 1500 beams per scan) |
| Scan rate | 40 Hz |
| Range | 0.08 m to 9 m |
| Per-point data | x, y, z (meters) and intensity (raw pulse width) |

### Driver package: `sensors/soslab_gl5_driver_prebuilt`

The ROS 2 driver is a thin node built on the official SOSLAB SDK
(`src/gl5_node.cpp`). It connects to the sensor over Ethernet, starts the
stream, and republishes every SDK frame as a `sensor_msgs/PointCloud2` with
four packed `FLOAT32` fields (`x`, `y`, `z`, `intensity`, 16 bytes per point).
The SDK reports millimeters; the node converts to meters.

The package ships a prebuilt aarch64 binary (`prebuilt/soslab_gl5_driver_node`)
together with the SDK shared library (`prebuilt/libLidar_x64_release.so`). The
binary has `RUNPATH=$ORIGIN:$ORIGIN/..:/opt/ros/humble/lib`, so it finds the
library both under `--symlink-install` and under a regular install
(`lib/soslab_gl5_driver/` and `lib/`). `colcon build` only installs these two
files; no SDK is needed on the build machine.

To rebuild the node from source, place the SDK headers and library under
`sensors/soslab_gl5_driver_prebuilt/_archive_/{include,lib}` and build
`src/CMakeLists.txt` as a standalone CMake project with ROS 2 Humble sourced:

```bash
cd sensors/soslab_gl5_driver_prebuilt/src
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
cp build/soslab_gl5_driver_node ../prebuilt/
```

### Network

The sensor and the onboard computer talk over a dedicated Ethernet link.
Configure the computer's wired interface statically on the same subnet.

| Endpoint | Address | Port |
|---|---|---|
| Lidar (`ip_address_device`, `ip_port_device`) | 10.110.1.2 | 2000 |
| Onboard PC (`ip_address_pc`, `ip_port_pc`) | 10.110.1.3 | 3000 |

### Parameters: `stack_master/config/sensors.yaml`

```yaml
soslab_gl5:
  ros__parameters:
    lidarType: "GL5"          # GL5 or GL3
    ip_address_device: "10.110.1.2"
    ip_port_device: 2000
    ip_address_pc: "10.110.1.3"
    ip_port_pc: 3000
    pc_topic_lidar0: "soslab/pointcloud"
    frame_id: "laser_raw"
    max_intensity: 3000       # kept for config compatibility, unused

pc_to_scan:
  ros__parameters:
    target_frame: "laser"
    transform_tolerance: 0.01
    min_height: -10.0         # GL-5 is 2D, height filter is irrelevant
    max_height:  10.0
    angle_min: -2.356         # must match the real 270 deg FoV
    angle_max:  2.356
    angle_increment: 0.00314
    scan_time: 0.025
    range_min: 0.08
    range_max: 9.0
    use_inf: true
    concurrency_level: 1
```

Two details matter here:

- **Frame orientation.** The SDK cloud is yawed 90 deg relative to the optical
  center (sensor 0 deg points at the car nose, but the cloud x-axis points
  sideways). The driver therefore publishes in `laser_raw`, and the bringup
  launch publishes a static `laser -> laser_raw` transform that un-rotates it.
  `base_link -> laser` is a static transform of 0.27 m forward and 0.11 m up.
- **Scan bounds.** `angle_min`/`angle_max` must stay at +/- 135 deg. Widening
  them to +/- pi fills the blind sector behind the car with `inf`, which
  Cartographer treats as a miss and carves free space through the track wall
  behind the car on every scan.

### Topic pipeline

```
soslab_gl5 (driver)
  └─ /soslab/pointcloud            PointCloud2, frame laser_raw, 40 Hz
       ├─ pc_to_scan (pointcloud_to_laserscan)
       │    └─ /scan                 LaserScan, frame laser  -> Cartographer, FTG, tracking
       └─ scan_intensity_filter (perception/intensity_filter.py)
            └─ /soslab/pointcloud_obs
                 └─ pointcloud_to_laserscan
                      └─ /scan_obs   LaserScan               -> obstacle detection (via remap)
```

The driver, `pc_to_scan`, and the static transforms are started by the
`f1tenth_stack` bringup (`base_system/f1tenth_system`), which
`stack_master/launch/teleop_launch.xml` includes and hands `sensors.yaml`.
On the car:

```bash
ros2 launch stack_master base_system_launch.xml racecar_version:=NUC2 map_name:=<map>
```

### Intensity filter

`scan_intensity_filter` removes weak mid-range returns before obstacle
detection. A point is dropped when

```
r_min < range < r_max   and   intensity < intensity_thresh
```

with defaults `r_min = 3.0 m`, `r_max = 7.0 m`, `intensity_thresh = 12`. Weak
returns in that band are grazing-angle artifacts off smooth walls; they flicker
from scan to scan and would otherwise be clustered into phantom obstacles. Near
returns are always strong and nothing beyond `r_max` is dropped, so long-range
detection is not capped. Only the detection path uses the filtered cloud;
Cartographer keeps the unfiltered `/scan` so localization is never starved of
wall constraints. All three parameters can be changed live with
`ros2 param set /scan_intensity_filter <name> <value>`.

## 2. Algorithms that use the lidar

### 2.1 Localization: Cartographer pure localization

`state_estimation/launch/state_estimation_launch.xml` runs `cartographer_node`
on `/scan` against a frozen `.pbstream` map. The GL-5 specific configuration in
`stack_master/config/<car>/slam/f110_2d_loc.lua`:

- `max_range = 9.0`, `min_range = 0.1` (sensor range)
- `pure_localization_trimmer.max_submaps_to_keep = 5`
- `optimize_every_n_nodes = 20` (fewer pose-graph stalls than the default 5)
- Ceres scan matcher translation/rotation weights scaled by 0.2 so the scan
  match dominates the odometry prior
- odometry enters through `odom_gate`, which drops odometry frames while a
  collision stop is active so spinning wheels cannot drag the pose away

Mapping uses `f110_2d.lua` with the same sensor limits.

### 2.2 Scan-to-map relocalization (`state_estimation`)

`initialpose_to_cartographer.py` bridges RViz "2D Pose Estimate" to
Cartographer's `finish_trajectory`/`start_trajectory` services and refines the
clicked pose against the map with the live scan. The matching math lives in
`scan_matcher.py` (pure NumPy, unit tested in `test/test_initialpose_snap.py`).

1. **Likelihood field.** From the `/map` occupancy grid, keep only occupied
   cells that touch free space (the visible wall surface), take a Euclidean
   distance transform, and build `exp(-d^2 / 2 sigma^2)` per cell
   (`snap_sigma = 0.10 m`). Using the wall surface rather than every occupied
   cell penalizes beams that overshoot a wall as much as beams that fall short.
2. **Correlative search.** Scan endpoints (at most `snap_max_beams = 400`) are
   transformed into `base_link` using the `base_link -> laser` TF and scored at
   every candidate pose in a window of `+/- 0.5 m` and `+/- 15 deg` around the
   click (translation step one map cell, yaw step 1 deg, then a 0.2 deg fine
   pass). The score is the mean likelihood of the endpoints.
3. **Gate.** If the best score is below `snap_min_score = 0.35`, or the laser
   TF is unavailable, the raw click is used instead. Otherwise the trajectory
   is restarted at the refined pose and the scan visibly sits on the map at
   once.

The same matcher drives the optional hand-guided collision recovery
(`collision_reloc_enable`). After a collision stop the node relocalizes at the
pre-crash pose, watches `/scan` to detect that the car has been pushed and set
down (`putdown_still_sec = 1.0 s` without scan change), then runs
`raceline_search`: candidate poses are generated along the raceline within
`+/- 4 m` of the crash position, at lateral offsets of `+/- 0.6 m`, with
headings `+/- 45 deg` around the raceline direction including the reversed
direction. The best candidate is refined with the normal snap, checked against
the track bounds and for ambiguity, and the trajectory is restarted there.

### 2.3 Obstacle detection (`perception`, `detection_core`)

The detector (`detect`, C++ port of the ForzaETH detector) runs at 20 Hz on the
latest scan. It subscribes to `/scan`; remap it to `/scan_obs` to run on the
intensity-filtered scan:

1. Transform scan points into the `map` frame with the scan-stamped TF
   (fallback to the latest TF within `max_tf_staleness = 0.05 s`).
2. Keep only points inside the track. The track boundary mask is eroded with
   an 11-pixel kernel (`filter_kernel_size`, about 0.35 m at 5 cm resolution)
   and inflated by `boundaries_inflation = 0.2 m`, so wall returns and flicker
   near the walls never become obstacles.
3. Adaptive breakpoint clustering along the scan: consecutive points start a
   new cluster when their distance exceeds the range-dependent threshold
   (`lambda = 10 deg` minimum reliable angle, `sigma = 0.03 m` range noise) or
   `new_cluster_threshold_m = 0.4 m`. Clusters with fewer than
   `min_obs_size = 10` points are discarded.
4. L-shape fitting on each cluster (`min_2_points_dist = 0.01 m`) gives a
   center, size and orientation; obstacles outside `min_obs_size_m = 0.1 m`
   to `max_obs_size = 0.5 m` are dropped.
5. Publish `/perception/detection/raw_obstacles` plus RViz markers, with
   `max_viewing_distance = 9.0 m` matching the sensor range.

### 2.4 Opponent tracking (`perception`, `tracking_core`)

The tracker (`tracking`) associates detections to tracks in Frenet
coordinates (`max_dist = 0.5 m`) and runs a Kalman filter per track to estimate
`s, d, v_s, v_d`; tracks are classified static or dynamic from their velocity
history. The lidar scan is used for the visibility check: for each track the
tracker looks up the scan beams at the track's bearing (+/- 4 beams) and, if
the measured range is longer than the expected distance minus
`fov_dist_margin = 0.4 m`, the obstacle should have been seen. Unseen tracks
lose time-to-live (`ttl_dynamic = 40`, `ttl_static = 3` cycles) and are deleted,
while tracks occluded or outside the 270 deg field of view are kept. Confirmed
tracks are published on `/perception/obstacles`.

### 2.5 Follow-the-Gap controller (`controller`, `ftg_core`)

Follow-the-Gap (FTG) is the reactive fallback: it drives in FTG-only sectors
and when the state machine detects the car stuck behind an opponent in
`TRAILING`. It works directly on `/scan` (`controller/ftg.py` and its C++ port
`ftg_core.cpp`):

1. **Crop.** Drop `ftg_range_offset = 375` beams on each side so only the
   forward sector is considered.
2. **Smooth and clip.** 3-beam moving average, ranges clipped to
   `ftg_max_lidar_dist = 5.0 m`, NaN beams treated as blocked.
3. **Safety border.** At every range discontinuity larger than 0.5 m the near
   range is extended over `ftg_safety_radius = 100` beams (about 15 deg) in
   both directions, so the car keeps clearance from edges and corners.
4. **Gap selection.** Beams are binarized against a radius that grows with
   speed, `min(5 m, track_width / 2 + 2 * v / ftg_max_speed)`; the largest run
   of free beams is the gap and the target point is the middle of that gap at
   that radius.
5. **Command.** Steering follows the target bearing; speed is chosen from the
   steering angle in tiers (ultra-straight below 3 deg, straight below 10 deg,
   mild corner below 30 deg, corner otherwise), capped at
   `ftg_max_speed = 4.0 m/s`.

`ftg_track_width = 1.65 m` sets the gap radius scale. All `ftg_*` parameters
live on the controller node and can be changed at runtime.
