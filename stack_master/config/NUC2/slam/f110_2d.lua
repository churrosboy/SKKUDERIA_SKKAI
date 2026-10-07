include "map_builder.lua"
include "trajectory_builder.lua"

-- 2026-08-20d: mapping config rebuilt as the unicorn-racing-stack equivalent
-- (mapping_2d.lua) after a day of shrink/balloon whipsaw proved our old mix
-- (gyroless front end + tuned back end) has no safe operating point.
-- Deliberate deviations from unicorn, do NOT "fix" them to match:
--   * tracking_frame stays base_link (their vesc_imu_rot sidesteps colocation;
--     our base_link->imu TF has zero translation since 2026-08-20, so base_link
--     works and the rest of the stack expects it)
--   * publish_to_tf stays true (OUR cartographer owns map->base_link; carstate
--     reads the TF. unicorn's EKF owns TF instead -- copying that breaks the stack)
--   * max_range 9.0 (GL-5 spec; unicorn's lidar sees ~50 m)
--   * num_subdivisions_per_laser_scan stays 1 (unicorn 10 = scan unwarping;
--     useless here, pc_to_scan emits time_increment=0, no per-point stamps)
--   * max_constraint_distance 5 kept (unicorn leaves default 15; ours is extra
--     anti-fold safety on the self-similar rectangle, harmless)

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",  --Change to "odom" for REP105 compliance (but worse performance)
  odom_frame = "odom",
  provide_odom_frame = false,
  -- use_odometry = true,
  -- 2026-08-20d unicorn: odom-free front end -- motion prior is IMU rotation +
  -- extrapolator velocity; wheel slip / bad wheel yaw (596 vs 921 deg/lap) no
  -- longer poisons the prior. IMU is REQUIRED below.
  use_odometry = false,
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
  odometry_sampling_ratio = 1,
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
-- MAP_BUILDER.num_background_threads = 3.0
MAP_BUILDER.num_background_threads = 4.0    -- unicorn mapping: 4
-- TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 2026-08-20d unicorn: gyro feeds the pose extrapolator (rotation drift was
-- opening every corner -> map ballooned outward). REQUIRES the zero-translation
-- base_link->imu TF from bringup (2026-08-20) or cartographer crashes on the
-- colocation CHECK, and the imu topic remap on the MAPPING cartographer node
-- in state_estimation_launch.xml.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.max_range = 9.0 -- GL-5 spec: 9 m
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- unicorn
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- unicorn: match ~every scan

-- might be able to optimize these parameters
-- see: http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90   -- unicorn (default)
-- POSE_GRAPH.optimize_every_n_nodes = 20
-- 2026-08-20d unicorn: 100 -- with a low-drift front end, optimize rarely in
-- big well-constrained batches while mapping (loc is the opposite, see loc lua)
POSE_GRAPH.optimize_every_n_nodes = 100

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.2 * TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight
-- 2026-08-20d unicorn: DEFAULT ceres weights (10/40) restored for mapping --
-- the x0.2 weakening was compensation for the untrustworthy wheel-odom prior;
-- with the IMU prior the default anchors are the matched pair. (Do not confuse
-- with unicorn's rotation_weight 0.1 -- that is their LOCALIZATION override.)

-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 40
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 10
-- 2026-08-20d unicorn: odometry fully out of the graph (moot with
-- use_odometry=false, set for clarity; wheel yaw was pulling the graph wrong)
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0

-- ===== loop closure: unicorn gates (matched to the low-drift IMU front end) =====
-- History (2026-08-20): default 7 m window + 0.55 score -> map SCRUNCHED (false
-- closures folding the self-similar rectangle); tightened to 2.0/0.62 -> map
-- BALLOONED (real closures starved, gyroless drift opened the corners);
-- 3.0/0.60 was the gyroless stopgap. With the IMU front end the strict gates
-- below are the stable pair. If the map balloons AGAIN with IMU on, widen the
-- window to 2.5 before touching anything else.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5  -- unicorn
-- 2026-08-20e: user widened 1.5 -> 3.5 (closure headroom over drift; note this exceeds
-- the corridor spacing, so the min_score 0.6 gate + max_constraint_distance 5 are what
-- stand between us and wrong-wall folds now)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.min_score = 0.6                       -- unicorn
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8   -- unicorn
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3                  -- unicorn (= default)
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.          -- OURS, kept (see header)
POSE_GRAPH.global_sampling_ratio = 0.0                              -- unicorn: no whole-map search (teleports on self-similar tracks)
POSE_GRAPH.global_constraint_search_after_n_seconds = 1000.         -- unicorn

return options
