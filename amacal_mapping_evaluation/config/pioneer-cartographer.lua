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

if toplevel ~= nil then
  old_toplevel = toplevel
  toplevel = false
else
  toplevel = true
end

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = robot_name .. "/base_link",
  published_frame = robot_name .. "/base_link",
  odom_frame = robot_name .. "/odom",
  provide_odom_frame = false,
  use_odometry = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 10,
  pose_publish_period_sec = 30e-3,
  trajectory_publish_period_sec = 100e-3,
  rangefinder_sampling_ratio = 1./2.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.max_range = 29.9
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.01
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30

--SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2
--POSE_GRAPH.global_constraint_search_after_n_seconds = 5.
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
POSE_GRAPH.global_sampling_ratio = 0.01
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimize_every_n_nodes = 15

if toplevel then
  return options
else
  toplevel = old_toplevel
end
