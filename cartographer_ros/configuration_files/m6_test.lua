-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = true,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

-- close loop closure during scaning set to 0
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 1.
POSE_GRAPH.constraint_builder.max_constraint_distance = 100.
POSE_GRAPH.constraint_builder.min_score = 0.3
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4

POSE_GRAPH.max_num_final_iterations = 1000
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.


-- loop closure weight
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1000
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1000

-- initial guess for optimization
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(30.)


POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30.
-- ceres 3d matcher weight
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 100000.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 100000.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 10

-- scan matcher weight param
POSE_GRAPH.matcher_translation_weight = 100000
POSE_GRAPH.matcher_rotation_weight = 100000
-- imu weight param
POSE_GRAPH.optimization_problem.acceleration_weight = 100000
POSE_GRAPH.optimization_problem.rotation_weight = 100000
-- local slam weight
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 100000
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 100000
-- odometry weight
POSE_GRAPH.optimization_problem.odometry_translation_weight = 100000
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 100000
-- fix frame weight
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 100000
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 100000

-- optimization huber function
POSE_GRAPH.optimization_problem.huber_scale = 10

POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100



-- Trajectory builder 3D
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.
-- ceres scan matcher weight
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 10
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.3
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.04

TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 2000

TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 320
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2
return options
