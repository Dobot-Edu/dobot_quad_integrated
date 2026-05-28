#!/usr/bin/env python3

import argparse
import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from dobot_integrated_demo.config import load_yaml
from dobot_integrated_interfaces.msg import SafetyState
from dobot_integrated_interfaces.srv import RobotCommand


@dataclass
class DistanceSample:
    value: float = float("nan")
    stamp: float = 0.0


class SafetyArbitratorNode(Node):
    """Turn depth distances into integrated safety state and optional stop events."""

    SAFE = "SAFE"
    FRONT_DANGER = "FRONT_DANGER"
    BACK_DANGER = "BACK_DANGER"
    BOTH_DANGER = "BOTH_DANGER"
    RECOVERING = "RECOVERING"
    SENSOR_STALE = "SENSOR_STALE"

    def __init__(self, config_file: str):
        super().__init__("dobot_integrated_safety_arbitrator_node")
        self._config = load_yaml(config_file)
        safety_cfg = self._config.get("safety", {})
        integrated_cfg = self._config.get("integrated", {})

        self._front_danger = float(safety_cfg.get("front_danger_distance_m", 0.5))
        self._front_recover = float(safety_cfg.get("front_recover_distance_m", 0.7))
        self._back_danger = float(safety_cfg.get("back_danger_distance_m", 0.5))
        self._back_recover = float(safety_cfg.get("back_recover_distance_m", 0.7))
        self._recover_stable_seconds = float(safety_cfg.get("recover_stable_seconds", 2.0))
        self._stale_timeout = float(safety_cfg.get("stale_timeout_seconds", 3.0))
        self._tick_period = float(safety_cfg.get("safety_tick_period_seconds", 0.1))
        self._auto_emergency_stop = bool(integrated_cfg.get("auto_emergency_stop", False))

        self._front = DistanceSample()
        self._back = DistanceSample()
        self._state = self.SENSOR_STALE
        self._safe_since: float | None = None
        self._last_event = "waiting for depth samples"
        self._last_stop_state = ""

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_pub = self.create_publisher(SafetyState, "/integrated/safety_state", 10)
        self._legacy_state_pub = self.create_publisher(String, "/safety_guard/state", 10)
        self._event_pub = self.create_publisher(String, "/safety_guard/events", 10)
        self._client = self.create_client(RobotCommand, "/integrated/robot_command")
        self.create_subscription(Float32, "/safety_guard/front/distance", self._front_cb, sensor_qos)
        self.create_subscription(Float32, "/safety_guard/back/distance", self._back_cb, sensor_qos)
        self.create_timer(max(0.02, self._tick_period), self._tick)

        self.get_logger().info(
            "综合安全仲裁已启动，auto_emergency_stop=%s；默认策略为发布状态并由动作网关拦截危险动作"
            % self._auto_emergency_stop
        )

    def _front_cb(self, msg: Float32):
        self._front = DistanceSample(float(msg.data), time.time())

    def _back_cb(self, msg: Float32):
        self._back = DistanceSample(float(msg.data), time.time())

    def _tick(self):
        now = time.time()
        next_state = self._evaluate_state(now)
        if next_state != self._state:
            self._transition(next_state)
        self._publish_state()

    def _evaluate_state(self, now: float) -> str:
        front_danger = self._is_distance_le(self._front.value, self._front_danger)
        back_danger = self._is_distance_le(self._back.value, self._back_danger)
        if front_danger and back_danger:
            self._safe_since = None
            return self.BOTH_DANGER
        if front_danger:
            self._safe_since = None
            return self.FRONT_DANGER
        if back_danger:
            self._safe_since = None
            return self.BACK_DANGER

        front_stale = self._is_stale(self._front, now)
        back_stale = self._is_stale(self._back, now)
        if front_stale or back_stale:
            self._safe_since = None
            return self.SENSOR_STALE

        recovered = self._is_distance_gt(self._front.value, self._front_recover) and self._is_distance_gt(
            self._back.value, self._back_recover
        )
        if not recovered:
            self._safe_since = None
            return self.RECOVERING

        if self._state in (self.FRONT_DANGER, self.BACK_DANGER, self.BOTH_DANGER, self.RECOVERING):
            if self._safe_since is None:
                self._safe_since = now
                return self.RECOVERING
            if now - self._safe_since < self._recover_stable_seconds:
                return self.RECOVERING

        return self.SAFE

    def _transition(self, next_state: str):
        old = self._state
        self._state = next_state
        self._last_event = (
            "%s -> %s; front=%.3fm, back=%.3fm"
            % (old, next_state, self._front.value, self._back.value)
        )
        msg = String()
        msg.data = self._last_event
        self._event_pub.publish(msg)
        if next_state in (self.FRONT_DANGER, self.BACK_DANGER, self.BOTH_DANGER):
            self.get_logger().warning(self._last_event)
            self._request_emergency_stop(next_state)
        else:
            self.get_logger().info(self._last_event)
            if next_state == self.SAFE:
                self._last_stop_state = ""

    def _publish_state(self):
        msg = SafetyState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.front_distance_m = float(self._front.value) if math.isfinite(self._front.value) else float("nan")
        msg.back_distance_m = float(self._back.value) if math.isfinite(self._back.value) else float("nan")
        msg.front_danger = self._state in (self.FRONT_DANGER, self.BOTH_DANGER)
        msg.back_danger = self._state in (self.BACK_DANGER, self.BOTH_DANGER)
        msg.sensor_stale = self._state == self.SENSOR_STALE
        msg.event = self._last_event
        self._state_pub.publish(msg)

        legacy = String()
        legacy.data = self._state
        self._legacy_state_pub.publish(legacy)

    def _request_emergency_stop(self, state: str):
        if not self._auto_emergency_stop:
            return
        if self._last_stop_state == state:
            return
        self._last_stop_state = state
        if not self._client.service_is_ready():
            self._client.wait_for_service(timeout_sec=0.1)
        if not self._client.service_is_ready():
            self.get_logger().warning("综合动作服务不可用，无法发送安全急停")
            return

        req = RobotCommand.Request()
        req.action_name = "emergency_stop"
        req.params_json = json.dumps({"reason": state}, ensure_ascii=False)
        req.source = "safety"
        req.priority = 100
        req.require_safety_check = False
        self._client.call_async(req)
        self.get_logger().warning("已发送安全急停请求: %s" % state)

    def _is_stale(self, sample: DistanceSample, now: float) -> bool:
        return sample.stamp <= 0.0 or now - sample.stamp > self._stale_timeout

    @staticmethod
    def _is_distance_le(value: float, threshold: float) -> bool:
        return math.isfinite(value) and value <= threshold

    @staticmethod
    def _is_distance_gt(value: float, threshold: float) -> bool:
        return math.isfinite(value) and value > threshold


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="integrated_demo.yaml 配置文件路径")
    args = parser.parse_args(argv)

    rclpy.init()
    node = SafetyArbitratorNode(args.config)
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


if __name__ == "__main__":
    main()
