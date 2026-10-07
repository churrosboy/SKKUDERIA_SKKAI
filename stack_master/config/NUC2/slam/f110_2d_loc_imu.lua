-- 2026-08-20 IMU variant of pure localization: identical to f110_2d_loc.lua but feeds
-- the gyro directly into cartographer's pose extrapolator (better rotation prediction
-- than EKF-fused odom, which is also halved by odometry_sampling_ratio 0.5).
-- Select via launch arg: loc_lua:=f110_2d_loc_imu.lua (default stays f110_2d_loc.lua).
-- REQUIRES the base_link->imu static TF to have ZERO translation (bringup_launch.py,
-- changed 2026-08-20) or cartographer dies with "IMU frame must be colocated".
include "f110_2d_loc.lua"

TRAJECTORY_BUILDER_2D.use_imu_data = true

return options
