# Package `histogram_lane_filter` {#lane_filter}

## `histogram_lane_filter_node` {#lane_filter-lane_filter_node}

This package runs the lane filtering during lane control mode. There are two functionalities implemented. The first one is the estimation of the actual position. This position consist of a distance $d$ form the center of the line and an angle $phi$ with respect to the center of the lane.

ROS 2 (`ament_python`) package. The node runs inside the robot namespace (`/ROBOT/...`):

| | name | type |
|---|---|---|
| subscribes | `camera_node/image/compressed` | `sensor_msgs/CompressedImage` |
| subscribes | `camera_node/camera_info` | `sensor_msgs/CameraInfo` |
| subscribes | `left_wheel_encoder_driver_node/tick`, `right_wheel_encoder_driver_node/tick` | `duckietown_msgs/WheelEncoderStamped` |
| publishes | `histogram_lane_filter_node/lane_pose` | `duckietown_msgs/LanePose` |
| publishes | `histogram_lane_filter_node/debug/belief/compressed` | `sensor_msgs/CompressedImage` (belief histogram) |
| publishes | `histogram_lane_filter_node/debug/segments/compressed` | `sensor_msgs/CompressedImage` (detected lines) |
| publishes | `histogram_lane_filter_node/debug/projected_segments/compressed` | `sensor_msgs/CompressedImage` (ground-projected segments) |
| service | `histogram_lane_filter_node/reset` | `std_srvs/Trigger` (belief back to the prior) |

Parameters (live, see `config/histogram_lane_filter_node/default.yaml`): the filter configuration
(`mean_d_0`, `sigma_d_0`, `delta_d`, `d_min`, `d_max`, `lanewidth`, ...), `debug`, `predict_frequency`,
and the robot parameters `wheel_radius`, `wheel_baseline`, `encoder_resolution` (defaults from the robot calibration).

The filter itself is implemented by the student in `packages/solution/histogram_filter.py`
(wrapped by `packages/solution/lane_filter.py`).
