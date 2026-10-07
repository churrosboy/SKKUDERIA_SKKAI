# Opponent Detection & Tracking
![Perception Architecture](./misc/perception_arch.png)

This directory contains the Opponent Detection & Tracking system of the ForzaETH F110 Team. It is divided into C++ LiDAR detection and Python opponent tracking.

The C++ `detect` node ports Unicorn's adaptive clustering, occupancy-map filtering, and L-shape fitting while retaining SKKUDERIA's topic contract. Its `main()` uses plain `rclcpp::spin`, so scan, waypoint, TF, and timer callbacks execute serially without an application mutex. The former detector remains available as `detect_python` for bag-based comparisons.

Detection publishes `/perception/detection/raw_obstacles`; the existing tracker consumes it, classifies obstacles as static or dynamic, and performs trajectory prediction. This migration does not change the tracker/planner interface.

The perception system is only executed in head-to-head mode. After a successful launch, you should see the following messages:
- `C++ LiDAR detector ready at 40.0 Hz ...`
- `[Opponent Tracking]: Ready`

## Build and run

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
colcon build --symlink-install --packages-up-to perception
source install/setup.bash
ros2 launch perception perception_launch.xml \
  param_file:=$(ros2 pkg prefix stack_master)/share/stack_master/config/opponent_tracker_params.yaml \
  map_name:=hangar_1905_v0
```

Use `detector_exec:=detect_python` only to run the legacy implementation against the same bag. For Jetson validation, enable `measure` under the `detect` node and inspect `/perception/detection/latency_ms`; also verify the 40 Hz output rate, TF warnings, obstacle counts, CPU load, and downstream `/perception/obstacles` behavior before racing.

## Overview
This section provides a short overview of the two subsystems.
### Opponent Detection
This system detects obstacles on the racetrack. For this, the laser scans are segmented into smaller objects, which are then filtered based on their size and position. A rectangle fitting method is applied for better feature extraction. The detected obstacles are then passed to the `Opponent Tracking` algorithm.

### Opponent Tracking
The tracking algorithm keeps track of the detected obstacles and associates them. Furthermore, it classifies the obstacles into the classes `static` and `dynamic` based on their standard deviation. In the last step, a prediction for the obstacles is made. For `static` obstacles, a simple averaging method is applied, while the prediction for `dynamic` obstacles is made with a Kalman Filter.

## Dynamic Reconfigurable Parameters
There are several parameters that can be dynamically reconfigured. Parameters can be changed using `ros2 param set [node] [parameter] [value]`.
A short overview of the reconfigurable parameters:
### Opponent Detection
- `min_obs_size`: Minimum size of the point cloud in `#laser_points`. Is used to filter FP.
- `max_obs_size`: Maximum size of an object in `meters`. Is used to filter FP.
- `max_viewing_distance`:  The maximum range for which obstacles can be detected. Currently, the maximum range allowed at F1TENTH is 10m.
- `boundaries_inflation`: This parameter artificially increases the thickness of the track boarders. Can help to filter FP along the boundaries.

### Opponent Tracking
- `ttl_dynamic`: The time-to-live of the dynamic prediction for the opponent obstacle.
- `ratio_to_glob_path`: The ratio between the target velocity of the opponent obstacle and the global velocity (i.e., the velocity of our car). Defines to which target speed the dynamic prediction will convert. If set to 1, the target velocity is equal to the race line speed.
- `ttl_static`: The time-to-live for static and not-classified obstacles. Be aware that the TTL of static obstacles is only decreased if they are visible. This is not the case if `noMemoryMode` is set to `TRUE`
- `min_nb_meas`: Minimum number of measurements of a given obstacle before it gets classified as either static or dynamic.
- `dist_deletion`: Distance up to which obstacles can be deleted.
- `dist_infromt`: Distance up to which obstacles in front of the car are published.
- `min_std`: Standard deviation of the (s,d) position up to which obstacles are classified as static. Needs to be increased when using SynPF.
- `max_std`: Standard deviation of the (s,d) position above which obstacles are immediately classified as dynamic. Needs to be increased when using SynPF.
- `vs_reset`: Average velocity under which dynamic obstacles get reset to static.
- `aggro_multi`: A multiplier for the association distance for dynamic obstacles. 
- `debug_mode`: Will enable some outputs to the console for debugging.
- `publish_static`: When set to `FALSE`, static obstacles will no longer be published. Can help in races with a lot of FP.
- `noMemoryMode`: When set to `TRUE`, obstacles will not be stored, and only the currently visible obstacles are published. Can help in races. 

## Other Parameters
This is a brief explanation of parameters, which are not dynamically reconfigurable. They can be found in `stack_master/config/opponent_tracker_params.yaml`
### Opponent Detection
- `rate`: The rate at which the node is running.
- `lambda`: The minimum reliable detection angle in `degrees`.
- `sigma`: The standard deviation for the noise of the LiDAR ranges in `meters`.
- `min_2_points_dist`: The minimum distance between two LiDAR points. 

### Opponent Tracking
- `rate`: The rate at which the node is running.
- `P_d`: Kalman Filter proportional gain for the distance in d direction
- `P_vs`: Kalman Filter proportional gain for the velocity in s direction
- `P_vd`: Kalman Filter proportional gain for the velocity in d direction
- `measurment_var_s`: The variance of the measurement noise in the s direction.
- `measurment_var_d`: The variance of the measurement noise in the d direction.
- `measurment_var_vs`: The variance of the measurement noise for the velocity in s direction.
- `measurment_var_vd`: The variance of the measurement noise for the velocity in d direction.
- `process_var_vs`: The variance of the process velocity noise for the velocity in s direction.
- `process_var_vd`: The variance of the process velocity noise for the velocity in d direction.
- `max_dist`: The maximal distance for the association of obstacles.
- `var_pub`: Obstacles that have a higher variance will not be published.
