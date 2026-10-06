-- Reuse current sensor and local scan-matching configuration.
include "racecar_2d.lua"
-- This node owns odom -> base_footprint ONLY. AMCL owns map -> odom.
-- No occupancy grid node is launched in saved-map navigation mode.
options.map_frame = "odom"
options.provide_odom_frame = false
-- Keep odometry continuous: no global pose corrections in this frame.
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0
return options
