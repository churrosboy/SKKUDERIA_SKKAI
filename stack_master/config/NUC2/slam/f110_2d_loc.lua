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
  odometry_sampling_ratio = 0.5,
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

-- might be able to optimize these parameters
-- see: http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
-- POSE_GRAPH.optimize_every_n_nodes = 5
-- 5 caused a ~200 ms TF stall every 1.5-3 s of driving (pose-graph optimization);
-- 20 makes stalls 4x rarer. In pure localization the scan matcher still tracks continuously.
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.global_sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0
--POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1.25 * POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight

-- 2026-08-21: let pose-graph nodes through at STANDSTILL so constraint search
-- can keep snapping the pose onto the frozen map while parked. While driving
-- this is a no-op: above ~0.7 m/s the 0.2 m distance gate already fires faster
-- than this time gate. Only standstill behavior changes (node rate 0.2 Hz -> ~3 Hz).
-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.  -- cartographer default
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5

return options

