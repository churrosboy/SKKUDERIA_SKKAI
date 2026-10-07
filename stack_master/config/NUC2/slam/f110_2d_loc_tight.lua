-- 2026-08-21: "tight" A/B variant of f110_2d_loc.lua for self-correcting drift.
-- Same local scan matcher (ceres x0.2 kept — x0.5 overshot straights on 08-20);
-- instead the POSE-GRAPH correction loop runs more often and accepts only
-- higher-quality constraint matches, so accumulated drift snaps back sooner
-- and wrong corridor matches (longitudinally-slid) are rejected.
-- Select with: loc_lua:=f110_2d_loc_tight.lua   (default lua unchanged)
-- CPU WARNING: constraint matching is the expensive part; only run this once
-- system load has headroom, and keep enable_recovery_state on to smooth the
-- more frequent (smaller) pose corrections.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",  --Change to "odom" for REP105 compliance (but worse performance)
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  publish_to_tf = true,
  use_landmarks = false,
  publish_tracked_pose = true,
  publish_frame_projected_to_2d = true,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 4.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.max_range = 9.0 -- GL-5 spec: 9 m
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80

-- ============ TIGHT deltas vs f110_2d_loc.lua ============
-- 2026-08-21 on-car result (load 12/6 cores): first version (optimize 10 +
-- sampling 0.2) pinned cartographer at 96% CPU and stalled /tracked_pose for
-- up to 0.33 s (117 Hz avg but 0.33 s max gap; /drive gapped 0.29 s) — WORSE
-- than baseline drift. Softened same day to the "lite" values below: keep the
-- free wins (min_score, max_constraint_distance), halve the added match load,
-- and leave optimization cadence at the base 20.
-- POSE_GRAPH.optimize_every_n_nodes = 10  -- v1: doubled blocking-optimization frequency -> pose stalls
POSE_GRAPH.optimize_every_n_nodes = 20

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.05   -- base value
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.2    -- v1: 4x attempts pinned the CPU
-- 0.05 -> 0.1: 2x more constraint attempts, but each attempt is ~3x cheaper
-- thanks to max_constraint_distance 5 below, so net cost ~= baseline.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1

-- default min_score = 0.55 (base file leaves default)
-- 0.55 -> 0.62: "stricter" in the useful sense — reject low-quality corridor
-- matches (the longitudinally-wrong ones on straights); unambiguous corner
-- matches still score well above this.
POSE_GRAPH.constraint_builder.min_score = 0.62

-- default max_constraint_distance = 15
-- 15 -> 5: anti-aliasing guard for the near-symmetric track — never try to
-- match against geometry >5 m from the current pose estimate (drift observed
-- is <2 m, so 5 m is ample slack for snap-back).
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.
-- =========================================================

POSE_GRAPH.global_sampling_ratio = 0.05

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0

-- 2026-08-21: same standstill-snap change as base f110_2d_loc.lua — admit nodes
-- while parked so constraint search keeps correcting pose; no-op above ~0.7 m/s.
-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.  -- cartographer default
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3

return options
