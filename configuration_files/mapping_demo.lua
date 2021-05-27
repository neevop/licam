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
  use_landmarks = false,
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

-- The proportion of added constraints to potential constraints
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03

-- Poses to be considered near a submap
POSE_GRAPH.constraint_builder.max_constraint_distance = 150

-- Threshold for the scan match score
POSE_GRAPH.constraint_builder.min_score = 0.5

-- Threshold of global localizations
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- Translational component of loop closure constraints
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1

-- Rotational component of loop closure constraints
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1

--  Logs information of loop-closing constraints
POSE_GRAPH.constraint_builder.log_matches = true

-- Scaling parameter for Huber loss function
POSE_GRAPH.optimization_problem.huber_scale = 5e2

-- Scaling parameter for the IMU acceleration term
POSE_GRAPH.optimization_problem.acceleration_weight = 1

-- Scaling parameter for the IMU rotation term
POSE_GRAPH.optimization_problem.rotation_weight = 1

-- Translation between consecutive nodes based on the local SLAM pose
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1

-- Rotation between consecutive nodes based on the local SLAM pose
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1

-- Translation between consecutive nodes based on the odometry
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0

-- Rotation between consecutive nodes based on the odometry
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0

-- Scaling parameter for the FixedFramePose translation
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1

-- Scaling parameter for the FixedFramePose rotation
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1

-- Ceres solver summary will be logged for every optimization
POSE_GRAPH.optimization_problem.log_solver_summary = true

MAP_BUILDER.use_trajectory_builder_2d = false

MAP_BUILDER.use_trajectory_builder_3d = true

MAP_BUILDER.num_background_threads = 8

-- @TODO
MOTION_fILTER.max_time_seconds = 0.1

MOTION_fILTER.max_distance_meters = 150

MOTION_fILTER.max_angle_radians = 3.14


POSE_GRAPH.optimize_every_n_nodes = 420

-- Translational component of non-loop-closure scan matcher constraints
POSE_GRAPH.matcher_translation_weight = 1

-- Rotational component of non-loop-closure scan matcher constraints
POSE_GRAPH.matcher_rotation_weight = 1

--  ‘optimization_problem_options’ for the final optimization
POSE_GRAPH.max_num_final_iterations = 16

-- Sample a single trajectory’s nodes for global localization
POSE_GRAPH.global_sampling_ratio = 1e-2

-- Output histograms for the pose residuals
POSE_GRAPH.log_residual_histograms = true

-- If for the duration specified by this option no global contraint 
-- has been added between two trajectories, loop closure searches will 
-- be performed globally rather than in a smaller search window.
POSE_GRAPH.global_constraint_search_after_n_seconds = 300

-- TRAJECTORY_BUILDER.pure_localization = false

--Rangefinder points outside these ranges will be dropped
TRAJECTORY_BUILDER_3D.min_range = 2

TRAJECTORY_BUILDER_3D.max_range = 100

-- Number of range data to accumulate into one unwarped
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 100

-- Voxel filter that gets applied to the range data immediately after cropping
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

-- Whether to solve the online scan matching first using the correlative scan matcher to generate a good starting point for Ceres.
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true

-- the orientation moving average based on observed gravity via the IMU
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 1

-- Number of histogram buckets for the rotational scan matcher.
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 0.1




TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160 --400
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false --true
TRAJECTORY_BUILDER_3D.min_range = 2
TRAJECTORY_BUILDER_3D.max_range = 100

TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 8

-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 0.01
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 0.02

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8

POSE_GRAPH.optimize_every_n_nodes = 420 --320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20 --10
POSE_GRAPH.constraint_builder.min_score = 0.5 -- 0.62 
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.5 --0.66

POSE_GRAPH.max_num_final_iterations = 20

POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true

return options
