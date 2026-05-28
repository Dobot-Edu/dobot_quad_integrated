#!/usr/bin/env python3

import argparse
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from dobot_integrated_demo.config import ensure_valid_cyclonedds_uri, load_yaml, resolve_path


@dataclass
class ImuSample:
    quaternion: list[float]
    gyroscope: list[float]
    accelerometer: list[float]
    rpy: list[float]
    stamp: float


class ImuReaderNode(Node):
    """Read Dobot lower-state IMU data through DDS and publish ROS 2 topics."""

    def __init__(self, config_file: str):
        super().__init__("dobot_balance_imu_reader_node")
        self._config_file = config_file
        self._config = load_yaml(config_file)

        dds_cfg = self._config.get("dds", {})
        log_cfg = self._config.get("logging", {})
        self._topic = str(dds_cfg.get("lower_state_topic", "rt/lower/state"))
        self._dds_config = resolve_path(dds_cfg.get("config_file"), config_file)
        self._dds_domain_id = int(dds_cfg.get("domain_id", 0))
        self._publish_period = max(0.0, float(dds_cfg.get("publish_period_seconds", 0.02)))
        self._log_period = float(log_cfg.get("imu_log_period_seconds", 1.0))
        self._cyclonedds_uri = ensure_valid_cyclonedds_uri(config_file)

        self._latest_sample: ImuSample | None = None
        self._received_count = 0
        self._published_count = 0
        self._last_publish_time = 0.0
        self._last_log_time = 0.0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._raw_pub = self.create_publisher(Float32MultiArray, "/balance_control/imu_raw", sensor_qos)
        self._rpy_pub = self.create_publisher(Float32MultiArray, "/balance_control/imu_rpy", sensor_qos)
        self._status_pub = self.create_publisher(String, "/balance_control/imu_status", 10)

        self._middleware = None
        self._init_dds()
        self.create_timer(max(0.005, self._publish_period), self._publish_latest)

        self.get_logger().info("IMU 读取节点已启动，DDS 话题: %s" % self._topic)
        self.get_logger().info("CYCLONEDDS_URI: %s" % (self._cyclonedds_uri or "<unset>"))
        self.get_logger().info("DDS 配置: %s" % (self._dds_config or f"domain_id={self._dds_domain_id}"))

    def _init_dds(self):
        import dds_middleware_python as dds

        if self._dds_config:
            self._middleware = dds.PyDDSMiddleware(self._dds_config)
        else:
            self._middleware = dds.PyDDSMiddleware(self._dds_domain_id)

        # The SDK's LowerState helper uses the reader QoS from dds_config.yaml.
        self._middleware.subscribeLowerState(self._topic, self._handle_lower_state)

    def _handle_lower_state(self, state):
        try:
            imu = state.imu_state()
            sample = ImuSample(
                quaternion=self._safe_float_list(imu.quaternion(), 4),
                gyroscope=self._safe_float_list(imu.gyroscope(), 3),
                accelerometer=self._safe_float_list(imu.accelerometer(), 3),
                rpy=self._safe_float_list(imu.rpy(), 3),
                stamp=time.time(),
            )
            if len(sample.rpy) != 3 or not all(math.isfinite(v) for v in sample.rpy[:2]):
                self._publish_status("drop imu sample: invalid rpy", error=True)
                return
            self._latest_sample = sample
            self._received_count += 1
        except Exception as exc:
            self._publish_status(f"lower-state imu parse failed: {exc}", error=True)

    def _publish_latest(self):
        sample = self._latest_sample
        if sample is None:
            self._publish_periodic_status("waiting for imu samples")
            return

        now = time.time()
        if self._publish_period > 0.0 and now - self._last_publish_time < self._publish_period:
            return
        self._last_publish_time = now

        raw = Float32MultiArray()
        raw.data = sample.quaternion + sample.gyroscope + sample.accelerometer + sample.rpy
        self._raw_pub.publish(raw)

        rpy = Float32MultiArray()
        rpy.data = sample.rpy
        self._rpy_pub.publish(rpy)
        self._published_count += 1

        if self._log_period > 0.0 and now - self._last_log_time >= self._log_period:
            self._last_log_time = now
            roll, pitch, yaw = [math.degrees(v) for v in sample.rpy]
            self.get_logger().info(
                "[IMU]  rpy(deg) roll=%+7.2f | pitch=%+7.2f | yaw=%+7.2f | rx=%06d | pub=%06d"
                % (roll, pitch, yaw, self._received_count, self._published_count)
            )

    def _publish_periodic_status(self, text: str):
        now = time.time()
        if self._log_period <= 0.0 or now - self._last_log_time < self._log_period:
            return
        self._last_log_time = now
        self._publish_status(text, error=False)

    def _publish_status(self, text: str, error: bool = False):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        if error:
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    @staticmethod
    def _safe_float_list(values, expected_len: int) -> list[float]:
        result = [float(v) for v in list(values)[:expected_len]]
        if len(result) < expected_len:
            result.extend([0.0] * (expected_len - len(result)))
        return result

    def destroy_node(self):
        self._middleware = None
        super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="balance_control_config.yaml 配置文件路径")
    args = parser.parse_args(argv)

    rclpy.init()
    node = ImuReaderNode(args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
