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
  tracking_frame = "lidar0_imu", --lidar0_imu
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0, ---
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1, ---
  lookup_transform_timeout_sec = 0.3,---
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 3e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

}




MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
MAX_2D_RANGE = 120

TRAJECTORY_BUILDER.pure_localization =true
--TRAJECTORY_BUILDER.initial_trajectory_pose = INITIAL_TRAJECTORY_POSE,

TRAJECTORY_BUILDER_2D.max_range = MAX_2D_RANGE
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_z = 0.35
TRAJECTORY_BUILDER_2D.min_z = 0.25
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --24 --180 --defaule is 19 24
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40 --20--40
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400 --40 1e3  4e2 4e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100 --1e3 2e2 10.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  --120
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1


TRAJECTORY_BUILDER_2D.use_imu_data = false --using imu data
--TRAJECTORY_BUILDER_3D.scans_per_accumulation = 130

--new added=================

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-2


TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3


POSE_GRAPH.optimize_every_n_nodes = 1 --2
POSE_GRAPH.global_sampling_ratio = 3e-6 --was commented 3e-6 must be less than 1
POSE_GRAPH.global_constraint_search_after_n_seconds = 0.3 -- was commented

POSE_GRAPH.constraint_builder.min_score =0.7 --0.62
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85--0.68
POSE_GRAPH.constraint_builder.sampling_ratio = 1 --1, 3e-4

POSE_GRAPH.optimization_problem.huber_scale = 1e2 --
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 25

--POSE_GRAPH.max_num_final_iterations = 1e5

--POSE_GRAPH.optimization_problem.acceleration_weight
--POSE_GRAPH.optimization_problem.rotation_weight
--POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 3e1
--POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
--POSE_GRAPH.optimization_problem.log_solver_summary = true




return options
