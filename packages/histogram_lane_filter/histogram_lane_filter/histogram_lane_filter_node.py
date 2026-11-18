#!/usr/bin/env python3
"""Histogram lane filter node (ROS 2).

Generates an estimate of the lane pose: the lateral deviation `d` and the heading deviation `phi` from
the center of the lane. The node detects the white/yellow line segments in the camera image, projects
them on the ground plane and feeds them to the student's histogram filter
(`packages/solution/histogram_filter.py`, wrapped by `packages/solution/lane_filter.py`); the wheel
encoders drive the prediction step.

All names are relative to the robot namespace (the launch file sets `namespace=veh`).

Subscribers:
    camera_node/image/compressed (sensor_msgs/CompressedImage): camera images
    camera_node/camera_info (sensor_msgs/CameraInfo): camera intrinsics (used once)
    left_wheel_encoder_driver_node/tick, right_wheel_encoder_driver_node/tick
        (duckietown_msgs/WheelEncoderStamped): wheel encoder ticks

Publishers:
    ~/lane_pose (duckietown_msgs/LanePose): the lane pose estimate
    ~/debug/belief/compressed (sensor_msgs/CompressedImage): the belief histogram as an image
    ~/debug/segments/compressed (sensor_msgs/CompressedImage): the detected line segments
    ~/debug/projected_segments/compressed (sensor_msgs/CompressedImage): the ground-projected segments

Services:
    ~/reset (std_srvs/Trigger): reset the belief to the prior

Parameters (live): the filter configuration (see config/histogram_lane_filter_node/default.yaml),
    `debug`, `predict_frequency`, and the robot parameters `wheel_radius`, `wheel_baseline`,
    `encoder_resolution` (defaults read from the robot calibration). Changing a filter or robot
    parameter rebuilds the filter (the belief goes back to the prior).
"""
import os
import threading
from typing import Optional

import numpy as np
import rclpy
from duckietown_msgs.msg import LanePose, WheelEncoderStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import Trigger

from dt_computer_vision.camera import CameraModel
from dt_computer_vision.camera.homography import Homography, HomographyToolkit
from dt_computer_vision.ground_projection import GroundProjector

from histogram_lane_filter.rendering import plot_belief  # reprep-free port (python 3.12)
from lx_ros2_tools.image import bgr_to_compressed, compressed_to_bgr
from lx_ros2_tools.params import LiveParams
from lx_ros2_tools.robot import get_wheel_params

from solution.lane_filter import LaneFilterHistogram

# filter configuration (defaults also in config/histogram_lane_filter_node/default.yaml)
FILTER_DEFAULTS = {
    "mean_d_0": 0.0,
    "mean_phi_0": 0.0,
    "sigma_d_0": 0.1,
    "sigma_phi_0": 0.1,
    "delta_d": 0.02,
    "delta_phi": 0.1,
    "d_max": 0.3,
    "d_min": -0.3,
    "phi_min": -1.5,
    "phi_max": 1.5,
    "linewidth_white": 0.05,
    "linewidth_yellow": 0.025,
    "lanewidth": 0.23,
    "sigma_d_mask": 1.0,
    "sigma_phi_mask": 2.0,
    "range_min": 0.2,
    "range_est": 0.45,
    "range_max": 0.6,
}
ROBOT_KEYS = ("encoder_resolution", "wheel_radius", "wheel_baseline")
EXTRINSICS_DIR = "/data/config/calibrations/camera_extrinsic"


class HistogramLaneFilterNode(Node):
    def __init__(self):
        super().__init__("histogram_lane_filter_node")
        self.veh = self.get_namespace().strip("/")

        # robot parameters (calibration file, else Duckiebot defaults); overridable as ROS parameters
        wheels = get_wheel_params(self.veh)
        self.get_logger().info(
            f"wheel baseline {wheels.baseline} m, radius {wheels.radius} m, "
            f"encoder resolution {wheels.encoder_resolution} ({wheels.source})"
        )

        # live parameters
        defaults = dict(FILTER_DEFAULTS)
        defaults.update({
            "encoder_resolution": int(wheels.encoder_resolution),
            "wheel_radius": float(wheels.radius),
            "wheel_baseline": float(wheels.baseline),
            "debug": True,
            "predict_frequency": 30.0,
        })
        self.params = LiveParams(self, defaults, on_change=self._on_param)

        # state
        self._lock = threading.Lock()
        self._camera: Optional[CameraModel] = None
        self._projector: Optional[GroundProjector] = None
        self.camera_info_received = False
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0
        self._left_encoder_initialized = False
        self._right_encoder_initialized = False
        self.last_encoder_stamp = None

        # the filter (student code)
        self.filter = self._make_filter()

        # subscribers
        self.sub_image = self.create_subscription(
            CompressedImage, "camera_node/image/compressed", self.cb_image, qos_profile_sensor_data
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo, "camera_node/camera_info", self.cb_camera_info, 10
        )
        self.sub_encoder_left = self.create_subscription(
            WheelEncoderStamped, "left_wheel_encoder_driver_node/tick", self.cb_left_encoder, qos_profile_sensor_data
        )
        self.sub_encoder_right = self.create_subscription(
            WheelEncoderStamped, "right_wheel_encoder_driver_node/tick", self.cb_right_encoder, qos_profile_sensor_data
        )

        # publishers
        self.pub_lane_pose = self.create_publisher(LanePose, "~/lane_pose", 10)
        self.pub_belief_img = self.create_publisher(CompressedImage, "~/debug/belief/compressed", 10)
        self.pub_segments_img = self.create_publisher(CompressedImage, "~/debug/segments/compressed", 10)
        self.pub_projected_segments_img = self.create_publisher(
            CompressedImage, "~/debug/projected_segments/compressed", 10
        )

        # services
        self.srv_reset = self.create_service(Trigger, "~/reset", self.srv_reset_cb)

        # prediction timer (encoder data can come very quickly)
        self._predict_timer = None
        self._start_predict_timer()

        self.get_logger().info("histogram_lane_filter_node started")

    # --- filter / parameters ------------------------------------------------------------------
    def _filter_kwargs(self) -> dict:
        kwargs = {k: float(self.params[k]) for k in FILTER_DEFAULTS}
        kwargs["encoder_resolution"] = int(self.params.encoder_resolution)
        kwargs["wheel_radius"] = float(self.params.wheel_radius)
        kwargs["wheel_baseline"] = float(self.params.wheel_baseline)
        return kwargs

    def _make_filter(self) -> LaneFilterHistogram:
        lane_filter = LaneFilterHistogram(**self._filter_kwargs())
        if self._camera is not None and self._projector is not None:
            lane_filter.initialize_camera(self._camera, self._projector)
        return lane_filter

    def _on_param(self, name, value):
        self.get_logger().info(f"parameter {name} = {value}")
        if name in FILTER_DEFAULTS or name in ROBOT_KEYS:
            # the grid / road / robot specs are fixed at construction: rebuild (belief -> prior)
            with self._lock:
                try:
                    self.filter = self._make_filter()
                except Exception as e:  # noqa: BLE001 - keep the previous filter
                    self.get_logger().error(f"could not rebuild the filter with {name}={value}: {e}")
        elif name == "predict_frequency":
            self._start_predict_timer()

    def _start_predict_timer(self):
        if self._predict_timer is not None:
            self.destroy_timer(self._predict_timer)
            self._predict_timer = None
        freq = float(self.params.predict_frequency)
        if freq > 0.0:
            self._predict_timer = self.create_timer(1.0 / freq, self.cb_predict)
        else:
            self.get_logger().warn("predict_frequency <= 0: encoder prediction disabled")

    # --- camera ---------------------------------------------------------------------------------
    def cb_camera_info(self, msg: CameraInfo):
        if self.camera_info_received:
            return
        self.get_logger().info("Received camera info message")
        H = self.load_extrinsics()
        if H is None:
            return  # keep waiting: the next message retries (the error has been logged)
        crop_top = self.filter._crop_top()
        # the filter works on the image cropped at the top: shift the principal point accordingly
        _K = np.reshape(np.array(msg.k, dtype=float), (3, 3))
        _K[1][2] = _K[1][2] - crop_top
        _P = np.reshape(np.array(msg.p, dtype=float), (3, 4))
        _P[1][2] = _P[1][2] - crop_top
        camera = CameraModel(
            width=msg.width,
            height=msg.height,
            K=_K,
            D=np.reshape(np.array(msg.d, dtype=float), (5,)),
            P=_P,
            H=H,
        )
        projector = GroundProjector(camera)
        with self._lock:
            self._camera, self._projector = camera, projector
            self.filter.initialize_camera(camera, projector)
        self.camera_info_received = True
        self.get_logger().info("Camera model initialized")

    def load_extrinsics(self) -> Optional[Homography]:
        """Load the homography matrix from the extrinsic calibration file of the robot
        (`/data/config/calibrations/camera_extrinsic/<robot>.yaml`, else `default.yaml`)."""
        cali_file = os.path.join(EXTRINSICS_DIR, f"{self.veh}.yaml")
        if not os.path.isfile(cali_file):
            self.get_logger().warn(f"Can't find calibration file: {cali_file}. Using default calibration instead.")
            cali_file = os.path.join(EXTRINSICS_DIR, "default.yaml")
        if not os.path.isfile(cali_file):
            self.get_logger().error(f"Found no extrinsic calibration file in {EXTRINSICS_DIR}: images are ignored")
            return None
        try:
            H: Homography = HomographyToolkit.load_from_disk(cali_file, return_date=False)  # type: ignore
            return H.reshape((3, 3))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Error in parsing calibration file {cali_file}:\n{e}")
            return None

    # --- encoders / prediction ------------------------------------------------------------------
    def cb_left_encoder(self, msg: WheelEncoderStamped):
        if not self._left_encoder_initialized:
            self.left_encoder_ticks = msg.data
            self._left_encoder_initialized = True
            self._check_encoder_resolution(msg)
        self.left_encoder_ticks_delta = msg.data - self.left_encoder_ticks
        self.last_encoder_stamp = msg.header.stamp

    def cb_right_encoder(self, msg: WheelEncoderStamped):
        if not self._right_encoder_initialized:
            self.right_encoder_ticks = msg.data
            self._right_encoder_initialized = True
            self._check_encoder_resolution(msg)
        self.right_encoder_ticks_delta = msg.data - self.right_encoder_ticks
        self.last_encoder_stamp = msg.header.stamp

    def _check_encoder_resolution(self, msg: WheelEncoderStamped):
        # the driver knows the real resolution of the encoders
        if msg.resolution > 0 and int(msg.resolution) != int(self.params.encoder_resolution):
            self.get_logger().info(f"encoder resolution from the driver: {msg.resolution} ticks/revolution")
            self.params.encoder_resolution = int(msg.resolution)

    def cb_predict(self):
        # first let's check if we moved at all, if not abort
        if self.right_encoder_ticks_delta == 0 and self.left_encoder_ticks_delta == 0:
            return
        with self._lock:
            self.filter.predict(self.left_encoder_ticks_delta, self.right_encoder_ticks_delta)
        self.left_encoder_ticks += self.left_encoder_ticks_delta
        self.right_encoder_ticks += self.right_encoder_ticks_delta
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0

    # --- images / update ------------------------------------------------------------------------
    def cb_image(self, msg: CompressedImage):
        if not self.camera_info_received:
            return
        try:
            image = compressed_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(f"Could not decode image: {e}")
            return
        with self._lock:
            cropped_image = image[self.filter.crop_top:, :, :]
            lines = self.filter.detect_lines(cropped_image)
            segments = self.filter.lines_to_projected_segments(lines)
            # update
            self.filter.update(segments)
            # publish
            self.publish_estimate(msg.header.stamp)

    def publish_estimate(self, stamp):
        d_max, phi_max = self.filter.get_estimate()

        lane_pose = LanePose()
        lane_pose.header.stamp = stamp
        lane_pose.d = float(d_max)
        lane_pose.phi = float(phi_max)
        lane_pose.in_lane = True
        lane_pose.status = LanePose.NORMAL
        self.pub_lane_pose.publish(lane_pose)

        if self.params.debug:
            self.publish_debug(lane_pose.header)

    def publish_debug(self, header):
        """Creates and publishes the debug images (only when somebody is listening)."""
        if self.pub_belief_img.get_subscription_count() > 0:
            belief_img = bgr_to_compressed(np.ascontiguousarray(plot_belief(self.filter, dpi=300)), header=header)
            self.pub_belief_img.publish(belief_img)
        if self.pub_segments_img.get_subscription_count() > 0 and getattr(self.filter, "image_w_dets", None) is not None:
            self.pub_segments_img.publish(bgr_to_compressed(self.filter.image_w_dets, header=header))
        if (
            self.pub_projected_segments_img.get_subscription_count() > 0
            and getattr(self.filter, "image_w_segs_rgb", None) is not None
        ):
            self.pub_projected_segments_img.publish(bgr_to_compressed(self.filter.image_w_segs_rgb, header=header))

    # --- services -------------------------------------------------------------------------------
    def srv_reset_cb(self, _req, res):
        with self._lock:
            self.filter.initialize()
        res.success, res.message = True, "belief reset to the prior"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = HistogramLaneFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
