#!/usr/bin/env python3

import argparse
import math
import os
import signal
import threading
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from dobot_integrated_demo.config import ensure_valid_cyclonedds_uri, load_yaml, resolve_path

@dataclass
class CameraIntrinsics:
    """用于将深度像素投影为点云的针孔相机内参。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class DepthFrame:
    """缓存 DDS 最近一帧深度数据，所有 ROS 输出只消费最新帧。"""

    raw_data: bytes
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    scale: float
    frame_id: str
    stamp: Time
    arrival_time: float
    sequence: int


class DepthBridgeNode(Node):
    """将 Dobot DDS 深度图桥接为 ROS 2 话题和障碍物距离。"""

    def __init__(self, config_file: str):
        super().__init__("dobot_depth_bridge_node")
        self._config_file = config_file
        self._config = load_yaml(config_file)

        dds_cfg = self._config.get("dds", {})
        safety_cfg = self._config.get("safety", {})
        viz_cfg = self._config.get("visualization", {})
        log_cfg = self._config.get("logging", {})

        self._front_topic = dds_cfg.get("front_depth_topic", "rt/camera/camera2/image_depth")
        self._back_topic = dds_cfg.get("back_depth_topic", "rt/camera/camera3/image_depth")
        self._dds_config = resolve_path(dds_cfg.get("config_file"), config_file)
        self._dds_domain_id = int(dds_cfg.get("domain_id", 0))
        self._cyclonedds_uri = ensure_valid_cyclonedds_uri(config_file)
        self._frame_stale_timeout = max(0.5, float(dds_cfg.get("stale_frame_timeout_seconds", 1.5)))
        self._restart_on_stale = bool(dds_cfg.get("restart_on_stale", True))
        self._restart_timeout = max(self._frame_stale_timeout, float(dds_cfg.get("restart_timeout_seconds", 8.0)))
        self._restart_cooldown = max(1.0, float(dds_cfg.get("restart_cooldown_seconds", 10.0)))

        self._roi = safety_cfg.get("roi", {})
        self._front_danger = float(safety_cfg.get("front_danger_distance_m", 0.5))
        self._back_danger = float(safety_cfg.get("back_danger_distance_m", 0.5))
        self._min_depth = float(safety_cfg.get("min_valid_depth_m", 0.1))
        self._max_depth = float(safety_cfg.get("max_valid_depth_m", 3.0))
        self._method = str(safety_cfg.get("distance_method", "percentile")).lower()
        self._percentile = float(safety_cfg.get("distance_percentile", 10.0))
        self._distance_sample_step = max(1, int(safety_cfg.get("distance_sample_step", 2)))
        self._distance_publish_period = max(0.0, float(safety_cfg.get("distance_publish_period_seconds", 0.05)))
        self._log_period = float(log_cfg.get("distance_log_period_seconds", 1.0))
        self._last_log_time = 0.0
        self._last_diag_time = 0.0
        self._last_error_status_time = 0.0

        intr_cfg = viz_cfg.get("camera_intrinsics", {})
        self._intrinsics = CameraIntrinsics(
            width=int(intr_cfg.get("width", 640)),
            height=int(intr_cfg.get("height", 480)),
            fx=float(intr_cfg.get("fx", 386.0)),
            fy=float(intr_cfg.get("fy", 386.0)),
            cx=float(intr_cfg.get("cx", 320.0)),
            cy=float(intr_cfg.get("cy", 240.0)),
        )
        pc_cfg = viz_cfg.get("pointcloud", {})
        img_cfg = viz_cfg.get("depth_image", {})
        marker_cfg = viz_cfg.get("markers", {})
        self._sample_step = max(1, int(pc_cfg.get("sample_step", 8)))
        self._max_points = max(100, int(pc_cfg.get("max_points", 12000)))
        self._pc_publish_period = max(0.0, float(pc_cfg.get("publish_period_seconds", 1.0)))
        self._pointcloud_roi_only = bool(pc_cfg.get("use_roi_only", True))
        self._image_publish_period = max(0.0, float(img_cfg.get("publish_period_seconds", 0.3)))
        self._marker_publish_period = max(0.0, float(marker_cfg.get("publish_period_seconds", 0.2)))
        self._publish_depth = bool(viz_cfg.get("publish_depth_image", True))
        self._publish_pc = bool(viz_cfg.get("publish_point_cloud", True))
        self._publish_markers = bool(viz_cfg.get("publish_markers", True))
        self._pointcloud_in_base = bool(viz_cfg.get("pointcloud_in_base_frame", True))
        self._front_frame = str(viz_cfg.get("front_frame_id", "front_depth_camera"))
        self._back_frame = str(viz_cfg.get("back_frame_id", "back_depth_camera"))
        self._marker_frame = str(viz_cfg.get("marker_frame_id", "safety_guard_base"))
        self._last_distance_publish = {"front": 0.0, "back": 0.0}
        self._last_image_publish = {"front": 0.0, "back": 0.0}
        self._last_image_sequence = {"front": -1, "back": -1}
        self._last_marker_publish = {"front": 0.0, "back": 0.0}
        self._latest_frames: dict[str, DepthFrame | None] = {"front": None, "back": None}
        self._latest_distances = {"front": float("nan"), "back": float("nan")}
        self._frame_sequences = {"front": 0, "back": 0}
        self._received_frames = {"front": 0, "back": 0}
        self._published_images = {"front": 0, "back": 0}
        self._published_distances = {"front": 0, "back": 0}
        self._published_pointclouds = 0
        self._dropped_frames = {"front": 0, "back": 0}
        self._processing_errors = 0
        self._latest_latency_ms = {"front": float("nan"), "back": float("nan")}
        self._stale_published = {"front": False, "back": False}
        self._last_empty_cloud_publish = 0.0
        self._last_restart_time = 0.0
        self._restart_count = 0
        self._frame_lock = threading.Lock()
        self._dds_lock = threading.Lock()
        self._worker_stop = threading.Event()
        self._main_thread = None
        self._pc_thread_stop = threading.Event()
        self._pc_thread = None
        self._watchdog_stop = threading.Event()
        self._watchdog_thread = None
        self._last_pc_sequences = {"front": -1, "back": -1}
        self._sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._front_distance_pub = self.create_publisher(Float32, "/safety_guard/front/distance", self._sensor_qos)
        self._back_distance_pub = self.create_publisher(Float32, "/safety_guard/back/distance", self._sensor_qos)
        self._status_pub = self.create_publisher(String, "/safety_guard/depth_status", 10)
        self._markers_pub = self.create_publisher(MarkerArray, "/safety_guard/markers", self._sensor_qos)
        self._front_image_pub = self.create_publisher(Image, "/safety_guard/front/depth/image_raw", self._sensor_qos)
        self._back_image_pub = self.create_publisher(Image, "/safety_guard/back/depth/image_raw", self._sensor_qos)
        self._points_pub = self.create_publisher(PointCloud2, "/safety_guard/points", self._sensor_qos)
        if self._publish_pc:
            self._pc_thread = threading.Thread(target=self._point_cloud_worker, daemon=True)
            self._pc_thread.start()

        timer_periods = [self._distance_publish_period]
        if self._publish_depth:
            timer_periods.append(self._image_publish_period)
        if self._publish_markers:
            timer_periods.append(self._marker_publish_period)
        active_periods = [period for period in timer_periods if period > 0.0]
        self._main_timer_period = min(active_periods) if active_periods else 0.02
        self._main_thread = threading.Thread(target=self._main_worker, daemon=True)
        self._main_thread.start()
        self._watchdog_thread = threading.Thread(target=self._watchdog_worker, daemon=True)
        self._watchdog_thread.start()

        self._middleware = None
        self._init_dds()

        self.get_logger().info("前向深度 DDS 话题: %s" % self._front_topic)
        self.get_logger().info("后向深度 DDS 话题: %s" % self._back_topic)
        self.get_logger().info("CYCLONEDDS_URI: %s" % (self._cyclonedds_uri or "<unset>"))
        self.get_logger().info("DDS 配置: %s" % (self._dds_config or f"domain_id={self._dds_domain_id}"))

    def _init_dds(self):
        """创建前后深度图 DDS 订阅。"""
        import dds_middleware_python as dds

        with self._dds_lock:
            if self._dds_config:
                self._middleware = dds.PyDDSMiddleware(self._dds_config)
            else:
                self._middleware = dds.PyDDSMiddleware(self._dds_domain_id)

            qos_config = {
                # 深度图是连续数据流，实时新帧比重传旧帧更重要。
                "reliability": "best_effort",
                "history_kind": "keep_last",
                "history_depth": 1,
                "durability": "volatile",
            }
            self._middleware.subscribeImage(
                self._front_topic,
                lambda msg: self._handle_depth("front", msg),
                qos_config,
            )
            self._middleware.subscribeImage(
                self._back_topic,
                lambda msg: self._handle_depth("back", msg),
                qos_config,
            )
        self.get_logger().info("点云发布: %s" % self._publish_pc)
        self.get_logger().info(
            "ROI: x=[%.2f, %.2f], y=[%.2f, %.2f], method=%s, percentile=%.1f"
            % (
                float(self._roi.get("x_min", 0.15)),
                float(self._roi.get("x_max", 0.85)),
                float(self._roi.get("y_min", 0.15)),
                float(self._roi.get("y_max", 0.85)),
                self._method,
                self._percentile,
            )
        )

    def _handle_depth(self, side: str, depth_msg):
        """DDS 回调只缓存最新帧，避免任何旧帧在 Python 或 RViz 链路中排队。"""
        try:
            encoding = str(depth_msg.encoding())
            scale = self._scale_for_encoding(encoding)
            frame_id = self._front_frame if side == "front" else self._back_frame
            raw_data = bytes(depth_msg.data())
            height = int(depth_msg.height())
            width = int(depth_msg.width())
            step = int(depth_msg.step())
            if not self._is_valid_raw_frame(raw_data, height, width, step, encoding):
                self._dropped_frames[side] += 1
                return
            arrival_time = time.time()
            with self._frame_lock:
                self._frame_sequences[side] += 1
                sequence = self._frame_sequences[side]
                self._latest_frames[side] = DepthFrame(
                    raw_data=raw_data,
                    height=height,
                    width=width,
                    encoding=encoding,
                    is_bigendian=int(depth_msg.is_bigendian()),
                    step=step,
                    scale=scale,
                    frame_id=frame_id,
                    stamp=self._stamp_from_depth_msg(depth_msg),
                    arrival_time=arrival_time,
                    sequence=sequence,
                )
                self._received_frames[side] += 1
                self._stale_published[side] = False

        except Exception as exc:
            status = String()
            status.data = f"{side} depth cache failed: {exc}"
            self._status_pub.publish(status)
            self.get_logger().error(status.data)

    def _main_worker(self):
        """常驻实时发布线程；单次异常只记录，不允许发布链路永久停止。"""
        period = self._main_timer_period if self._main_timer_period > 0.0 else 0.02
        while not self._worker_stop.is_set():
            started = time.time()
            try:
                self._publish_periodic_outputs()
            except Exception as exc:
                self._processing_errors += 1
                self._publish_status(f"main publish loop recovered from error: {exc}", error=True)
            elapsed = time.time() - started
            self._worker_stop.wait(max(0.001, period - elapsed))

    def _watchdog_worker(self):
        while not self._watchdog_stop.is_set():
            self._watchdog_stop.wait(2.0)
            if self._watchdog_stop.is_set() or not self._restart_on_stale:
                break
            now = time.time()
            frames = self._snapshot_frames()
            stale_sides = [
                side
                for side, frame in frames.items()
                if frame is not None and now - frame.arrival_time > self._restart_timeout
            ]
            if not stale_sides:
                continue
            if now - self._last_restart_time < self._restart_cooldown:
                continue
            self._restart_dds(stale_sides, now)

    def _restart_dds(self, stale_sides: list[str], now: float):
        self._last_restart_time = now
        self._restart_count += 1
        self._publish_status(
            "depth stream stale on %s for > %.1fs, restarting DDS subscriptions (count=%d)"
            % (",".join(stale_sides), self._restart_timeout, self._restart_count),
            error=True,
        )
        with self._frame_lock:
            for side in stale_sides:
                self._latest_frames[side] = None
                self._latest_distances[side] = float("nan")
                self._latest_latency_ms[side] = float("nan")
                self._stale_published[side] = False
        try:
            self._middleware = None
            self._init_dds()
        except Exception as exc:
            self._processing_errors += 1
            self._publish_status("DDS restart failed: %s" % exc, error=True)

    def _publish_periodic_outputs(self):
        now = time.time()
        frames = self._snapshot_frames()
        for side, frame in frames.items():
            if frame is None:
                if self._should_publish(self._last_distance_publish, side, now, self._distance_publish_period):
                    self._publish_stale_distance(side, now)
                continue
            try:
                if self._is_frame_stale(frame, now):
                    self._publish_stale_outputs(side, frame, now)
                    continue
                if self._should_publish(self._last_distance_publish, side, now, self._distance_publish_period):
                    self._publish_distance(side, frame)
                if self._publish_depth and self._should_publish_image(side, frame):
                    self._publish_depth_image(side, frame)
                if self._publish_markers and self._should_publish(self._last_marker_publish, side, now, self._marker_publish_period):
                    self._publish_marker(side, self._latest_distances[side], frame.stamp)
            except Exception as exc:
                self._processing_errors += 1
                self._publish_status(f"{side} publish recovered from error: {exc}", error=True)
        self._publish_diagnostics(now)

    def _snapshot_frames(self) -> dict[str, DepthFrame | None]:
        with self._frame_lock:
            return dict(self._latest_frames)

    def _is_frame_stale(self, frame: DepthFrame, now: float | None = None) -> bool:
        now = time.time() if now is None else now
        return now - frame.arrival_time > self._frame_stale_timeout

    def _publish_stale_outputs(self, side: str, frame: DepthFrame, now: float):
        if self._should_publish(self._last_distance_publish, side, now, self._distance_publish_period):
            self._publish_stale_distance(side, now)
        if not self._stale_published.get(side, False):
            self._publish_marker(side, float("nan"), self.get_clock().now().to_msg())
            self._stale_published[side] = True

    def _publish_stale_distance(self, side: str, now: float):
        self._latest_distances[side] = float("nan")
        self._latest_latency_ms[side] = float("nan")
        msg = Float32()
        msg.data = float("nan")
        if side == "front":
            self._front_distance_pub.publish(msg)
        else:
            self._back_distance_pub.publish(msg)
        self._published_distances[side] += 1

        if self._log_period > 0.0 and now - self._last_log_time >= self._log_period:
            self._last_log_time = now
            self.get_logger().warning(
                "distance stale: front=%.3fm, back=%.3fm, latency_ms(front/back)=%.1f/%.1f"
                % (
                    self._latest_distances["front"],
                    self._latest_distances["back"],
                    self._latest_latency_ms["front"],
                    self._latest_latency_ms["back"],
                )
            )

    def _should_publish_image(self, side: str, frame: DepthFrame) -> bool:
        now = time.time()
        if not self._should_publish(self._last_image_publish, side, now, self._image_publish_period):
            return False
        return frame.sequence != self._last_image_sequence.get(side, -1)

    def _publish_distance(self, side: str, frame: DepthFrame):
        depth = self._frame_to_depth(frame)
        distance = self._compute_roi_distance(depth, frame.scale)
        self._latest_distances[side] = distance
        self._latest_latency_ms[side] = max(0.0, (time.time() - frame.arrival_time) * 1000.0)
        msg = Float32()
        msg.data = float(distance) if math.isfinite(distance) else float("nan")
        if side == "front":
            self._front_distance_pub.publish(msg)
        else:
            self._back_distance_pub.publish(msg)
        self._published_distances[side] += 1

        now = time.time()
        if self._log_period > 0.0 and now - self._last_log_time >= self._log_period:
            self._last_log_time = now
            self.get_logger().info(
                "distance: front=%.3fm, back=%.3fm, latency_ms(front/back)=%.1f/%.1f"
                % (
                    self._latest_distances["front"],
                    self._latest_distances["back"],
                    self._latest_latency_ms["front"],
                    self._latest_latency_ms["back"],
                )
            )

    def _publish_diagnostics(self, now: float):
        if now - self._last_diag_time < 5.0:
            return
        self._last_diag_time = now
        status = String()
        status.data = (
            "depth_bridge stats: "
            f"rx(front/back)={self._received_frames['front']}/{self._received_frames['back']}, "
            f"drop(front/back)={self._dropped_frames['front']}/{self._dropped_frames['back']}, "
            f"img(front/back)={self._published_images['front']}/{self._published_images['back']}, "
            f"dist(front/back)={self._published_distances['front']}/{self._published_distances['back']}, "
            f"pc={self._published_pointclouds}, "
            f"errors={self._processing_errors}, "
            f"restarts={self._restart_count}, "
            f"latency_ms(front/back)={self._latest_latency_ms['front']:.1f}/{self._latest_latency_ms['back']:.1f}"
        )
        self._status_pub.publish(status)

    def _publish_status(self, text: str, error: bool = False):
        if error:
            now = time.time()
            if now - self._last_error_status_time < 1.0:
                return
            self._last_error_status_time = now
        status = String()
        status.data = text
        self._status_pub.publish(status)
        if error:
            self.get_logger().error(text)

    def _should_publish(self, last_publish: dict[str, float], side: str, now: float, period: float) -> bool:
        if period <= 0.0 or now - last_publish.get(side, 0.0) >= period:
            last_publish[side] = now
            return True
        return False

    def _scale_for_encoding(self, encoding: str) -> float:
        encoding = encoding.upper()
        if "16UC1" in encoding or "MONO16" in encoding:
            return 0.001
        elif "32FC1" in encoding:
            return 1.0
        raise ValueError(f"Unsupported depth encoding: {encoding}")

    def _bytes_per_pixel(self, encoding: str) -> int:
        encoding = encoding.upper()
        if "16UC1" in encoding or "MONO16" in encoding:
            return 2
        if "32FC1" in encoding:
            return 4
        raise ValueError(f"Unsupported depth encoding: {encoding}")

    def _is_valid_raw_frame(self, raw_data: bytes, height: int, width: int, step: int, encoding: str) -> bool:
        try:
            bytes_per_pixel = self._bytes_per_pixel(encoding)
        except ValueError as exc:
            self._publish_status(f"drop frame: {exc}", error=True)
            return False
        if height <= 0 or width <= 0 or step <= 0:
            self._publish_status(f"drop frame: invalid shape {width}x{height}, step={step}", error=True)
            return False
        if step < width * bytes_per_pixel:
            self._publish_status(
                f"drop frame: step {step} smaller than width bytes {width * bytes_per_pixel}",
                error=True,
            )
            return False
        required = height * step
        if len(raw_data) < required:
            self._publish_status(
                f"drop frame: data too short {len(raw_data)} < required {required}",
                error=True,
            )
            return False
        return True

    def _frame_to_depth(self, frame: DepthFrame) -> np.ndarray:
        """将缓存的原始 bytes 视图转换为深度矩阵，不额外复制像素数据。"""
        encoding = frame.encoding.upper()
        if "16UC1" in encoding or "MONO16" in encoding:
            row_values = frame.step // 2
            raw = np.frombuffer(frame.raw_data, dtype=np.uint16, count=frame.height * row_values)
            return raw.reshape((frame.height, row_values))[:, : frame.width]
        if "32FC1" in encoding:
            row_values = frame.step // 4
            raw = np.frombuffer(frame.raw_data, dtype=np.float32, count=frame.height * row_values)
            return raw.reshape((frame.height, row_values))[:, : frame.width]
        raise ValueError(f"Unsupported depth encoding: {frame.encoding}")

    def _stamp_from_depth_msg(self, depth_msg) -> Time:
        try:
            stamp = depth_msg.header().stamp()
            return Time(sec=int(stamp.sec()), nanosec=int(stamp.nanosec()))
        except Exception:
            return self.get_clock().now().to_msg()

    def _compute_roi_distance(self, depth: np.ndarray, scale: float) -> float:
        """基于配置 ROI 内的有效深度样本计算障碍物距离。"""
        h, w = depth.shape[:2]
        x0 = int(float(self._roi.get("x_min", 0.35)) * w)
        x1 = int(float(self._roi.get("x_max", 0.65)) * w)
        y0 = int(float(self._roi.get("y_min", 0.35)) * h)
        y1 = int(float(self._roi.get("y_max", 0.70)) * h)
        roi = depth[max(0, y0): min(h, y1): self._distance_sample_step, max(0, x0): min(w, x1): self._distance_sample_step]
        min_native = self._min_depth / scale
        max_native = self._max_depth / scale
        if np.issubdtype(roi.dtype, np.floating):
            valid = roi[np.isfinite(roi) & (roi >= min_native) & (roi <= max_native)]
        else:
            valid = roi[(roi >= min_native) & (roi <= max_native)]
        if valid.size == 0:
            return float("nan")
        if self._method == "min":
            return float(np.min(valid) * scale)
        percentile = min(100.0, max(0.0, self._percentile))
        kth = int((percentile / 100.0) * (valid.size - 1))
        return float(np.partition(valid, kth)[kth] * scale)

    def _publish_depth_image(self, side: str, frame: DepthFrame):
        msg = Image()
        msg.header = Header(stamp=frame.stamp, frame_id=frame.frame_id)
        msg.height = frame.height
        msg.width = frame.width
        msg.encoding = frame.encoding
        msg.is_bigendian = frame.is_bigendian
        msg.step = frame.step
        msg.data = frame.raw_data
        if side == "front":
            self._front_image_pub.publish(msg)
        else:
            self._back_image_pub.publish(msg)
        self._last_image_sequence[side] = frame.sequence
        self._published_images[side] += 1

    def _point_cloud_worker(self):
        """固定频率后台生成点云，避免 RViz 渲染负载随 DDS 到帧抖动。"""
        period = self._pc_publish_period if self._pc_publish_period > 0.0 else 0.2
        while not self._pc_thread_stop.is_set():
            self._pc_thread_stop.wait(period)
            if self._pc_thread_stop.is_set():
                break
            try:
                self._publish_latest_point_cloud()
            except Exception as exc:
                self._processing_errors += 1
                self._publish_status(f"point cloud worker recovered from error: {exc}", error=True)

    def _publish_latest_point_cloud(self):
        """合并发布前后最新点云，减少 RViz 订阅和渲染负载。"""
        frames = self._snapshot_frames()
        now = time.time()
        valid_frames = [
            (side, frame)
            for side, frame in frames.items()
            if frame is not None and not self._is_frame_stale(frame, now)
        ]
        if not valid_frames:
            if now - self._last_empty_cloud_publish >= max(0.5, self._pc_publish_period):
                cloud = self._create_xyz32_cloud(
                    Header(stamp=self.get_clock().now().to_msg(), frame_id=self._marker_frame),
                    np.empty((0, 3), dtype=np.float32),
                )
                self._points_pub.publish(cloud)
                self._last_empty_cloud_publish = now
            return
        if all(frame.sequence == self._last_pc_sequences.get(side, -1) for side, frame in valid_frames):
            return
        try:
            point_sets = [self._depth_to_points(self._frame_to_depth(frame), frame.scale, side) for side, frame in valid_frames]
            points = np.concatenate(point_sets, axis=0) if len(point_sets) > 1 else point_sets[0]
            stamp = valid_frames[-1][1].stamp
            cloud = self._create_xyz32_cloud(Header(stamp=stamp, frame_id=self._marker_frame), points)
            self._points_pub.publish(cloud)
            for side, frame in valid_frames:
                self._last_pc_sequences[side] = frame.sequence
            self._published_pointclouds += 1
        except Exception as exc:
            status = String()
            status.data = f"point cloud publish failed: {exc}"
            self._status_pub.publish(status)
            self.get_logger().error(status.data)

    def _depth_to_points(self, depth: np.ndarray, scale: float, side: str) -> np.ndarray:
        """将采样深度像素投影到相机坐标系或演示基坐标系。"""
        h, w = depth.shape[:2]
        if self._pointcloud_roi_only:
            x0 = max(0, int(float(self._roi.get("x_min", 0.35)) * w))
            x1 = min(w, int(float(self._roi.get("x_max", 0.65)) * w))
            y0 = max(0, int(float(self._roi.get("y_min", 0.35)) * h))
            y1 = min(h, int(float(self._roi.get("y_max", 0.70)) * h))
        else:
            x0, x1, y0, y1 = 0, w, 0, h

        depth_roi = depth[y0:y1, x0:x1]
        sampled_depth = depth_roi[0:depth_roi.shape[0]:self._sample_step, 0:depth_roi.shape[1]:self._sample_step]
        min_native = self._min_depth / scale
        max_native = self._max_depth / scale
        if np.issubdtype(sampled_depth.dtype, np.floating):
            valid_mask = np.isfinite(sampled_depth) & (sampled_depth >= min_native) & (sampled_depth <= max_native)
        else:
            valid_mask = (sampled_depth >= min_native) & (sampled_depth <= max_native)
        if not np.any(valid_mask):
            return np.empty((0, 3), dtype=np.float32)

        v_idx, u_idx = np.nonzero(valid_mask)
        if v_idx.size > self._max_points:
            v_idx = v_idx[: self._max_points]
            u_idx = u_idx[: self._max_points]

        z = sampled_depth[v_idx, u_idx].astype(np.float32, copy=False) * scale
        u = (x0 + u_idx * self._sample_step).astype(np.float32, copy=False)
        v = (y0 + v_idx * self._sample_step).astype(np.float32, copy=False)
        x_cam = (u - self._intrinsics.cx) * z / self._intrinsics.fx
        y_cam = (v - self._intrinsics.cy) * z / self._intrinsics.fy

        if self._pointcloud_in_base:
            # 演示基坐标系：+X 为前方，-X 为后方，+Y 为左侧，+Z 为上方。
            x = z if side == "front" else -z
            y = -x_cam
            point_z = -y_cam
        else:
            x = x_cam
            y = y_cam
            point_z = z

        return np.ascontiguousarray(np.column_stack((x, y, point_z)), dtype=np.float32)

    def _create_xyz32_cloud(self, header: Header, points: np.ndarray) -> PointCloud2:
        """直接使用 NumPy 内存构造点云，避免高密度点云产生大量 Python 对象。"""
        points = np.ascontiguousarray(points, dtype=np.float32)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = points.tobytes()
        return cloud

    def _publish_marker(self, side: str, distance: float, stamp):
        """发布 RViz 距离球和文字标签。"""
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id=self._marker_frame)
        marker.ns = "safety_guard_distance"
        marker.id = 1 if side == "front" else 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        signed_distance = float(distance) if math.isfinite(distance) else 0.0
        marker.pose.position.x = signed_distance if side == "front" else -signed_distance
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.a = 0.9
        danger_distance = self._front_danger if side == "front" else self._back_danger
        in_danger = math.isfinite(distance) and distance <= danger_distance
        marker.color.r = 1.0 if in_danger else 0.0
        marker.color.g = 0.0 if in_danger else 1.0
        marker.color.b = 0.0

        text = Marker()
        text.header = Header(stamp=stamp, frame_id=self._marker_frame)
        text.ns = "safety_guard_label"
        text.id = 11 if side == "front" else 12
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = marker.pose.position.x
        text.pose.position.y = 0.0
        text.pose.position.z = 0.45
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color.a = 1.0
        text.color.r = marker.color.r
        text.color.g = marker.color.g
        text.color.b = marker.color.b
        label = "FRONT" if side == "front" else "BACK"
        dist_text = "nan" if not math.isfinite(distance) else f"{distance:.2f}m"
        text.text = f"{label} {dist_text}"

        arr = MarkerArray()
        arr.markers.append(marker)
        arr.markers.append(text)
        self._markers_pub.publish(arr)

    def destroy_node(self):
        self._worker_stop.set()
        self._pc_thread_stop.set()
        self._watchdog_stop.set()
        if self._main_thread is not None:
            self._main_thread.join(timeout=1.0)
        if self._pc_thread is not None:
            self._pc_thread.join(timeout=1.0)
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=1.0)
        with self._frame_lock:
            self._latest_frames["front"] = None
            self._latest_frames["back"] = None
        self._middleware = None
        super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="safety_guard_config.yaml 配置文件路径")
    args = parser.parse_args(argv)

    rclpy.init()
    node = DepthBridgeNode(args.config)

    def exit_now(sig, frame):
        # DDS 原生线程在退出时偶尔阻塞；launch 结束时直接退出该桥接进程。
        os._exit(0)

    signal.signal(signal.SIGTERM, exit_now)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        # DDS 中间件内部可能持有原生线程；确保 Ctrl+C 后进程彻底退出，避免下次启动直接卡顿。
        os._exit(0)


if __name__ == "__main__":
    main()
