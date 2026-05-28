#!/usr/bin/env python3

import argparse
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from dobot_integrated_demo.config import load_yaml
from dobot_integrated_interfaces.msg import BalanceStatus
from dobot_integrated_interfaces.srv import RobotCommand


@dataclass
class AxisPid:
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    previous_error: float | None = None

    def update(self, error: float, dt: float, integral_limit: float) -> float:
        dt = max(1e-3, dt)
        self.integral += error * dt
        self.integral = clamp(self.integral, -integral_limit, integral_limit)
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


@dataclass
class AttitudeState:
    raw_roll_deg: float = 0.0
    raw_pitch_deg: float = 0.0
    raw_yaw_deg: float = 0.0
    filtered_roll_deg: float = 0.0
    filtered_pitch_deg: float = 0.0
    command_roll_deg: float = 0.0
    command_pitch_deg: float = 0.0
    stamp: float = 0.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class BalanceCompensatorNode(Node):
    """Compute attitude compensation and request execution through motion gateway."""

    def __init__(self, config_file: str):
        super().__init__("dobot_integrated_balance_compensator_node")
        self._config = load_yaml(config_file)
        filter_cfg = self._config.get("filter", {})
        control_cfg = self._config.get("control", {})
        log_cfg = self._config.get("logging", {})

        self._enable_execute = bool(control_cfg.get("enable_balance_execute", True))
        self._alpha = clamp(float(filter_cfg.get("low_pass_alpha", 0.25)), 0.01, 1.0)
        self._ma_window = max(1, int(filter_cfg.get("moving_average_window", 5)))
        self._roll_window: deque[float] = deque(maxlen=self._ma_window)
        self._pitch_window: deque[float] = deque(maxlen=self._ma_window)

        self._target_roll = float(control_cfg.get("target_roll_deg", 0.0))
        self._target_pitch = float(control_cfg.get("target_pitch_deg", 0.0))
        self._trigger_threshold = abs(float(control_cfg.get("trigger_threshold_deg", 3.0)))
        self._settle_threshold = abs(float(control_cfg.get("settle_threshold_deg", 1.5)))
        self._max_compensation = abs(float(control_cfg.get("max_compensation_deg", 10.0)))
        self._action_duration = clamp(float(control_cfg.get("action_duration_seconds", 0.8)), 0.5, 5.0)
        self._action_cooldown = max(0.0, float(control_cfg.get("action_cooldown_seconds", 1.2)))
        self._roll_sign = float(control_cfg.get("roll_output_sign", -1.0))
        self._pitch_sign = float(control_cfg.get("pitch_output_sign", -1.0))
        self._integral_limit = abs(float(control_cfg.get("integral_limit_deg_s", 8.0)))
        self._mode = str(control_cfg.get("compensation_mode", "combined")).lower()
        if self._mode not in ("combined", "axis"):
            self._mode = "combined"

        self._roll_pid = AxisPid(
            kp=float(control_cfg.get("kp_roll", 0.8)),
            ki=float(control_cfg.get("ki_roll", 0.0)),
            kd=float(control_cfg.get("kd_roll", 0.08)),
        )
        self._pitch_pid = AxisPid(
            kp=float(control_cfg.get("kp_pitch", 0.8)),
            ki=float(control_cfg.get("ki_pitch", 0.0)),
            kd=float(control_cfg.get("kd_pitch", 0.08)),
        )
        self._log_period = float(log_cfg.get("control_log_period_seconds", 0.5))

        self._last_sample_time = 0.0
        self._last_action_time = 0.0
        self._last_log_time = 0.0
        self._state = AttitudeState()
        self._filter_initialized = False
        self._action_lock = threading.Lock()
        self._last_event = "waiting for imu samples"

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Float32MultiArray, "/balance_control/imu_rpy", self._rpy_cb, sensor_qos)
        self._attitude_pub = self.create_publisher(Float32MultiArray, "/balance_control/attitude", sensor_qos)
        self._attitude_report_pub = self.create_publisher(String, "/balance_control/attitude_report", 10)
        self._state_pub = self.create_publisher(String, "/balance_control/state", 10)
        self._status_pub = self.create_publisher(BalanceStatus, "/integrated/balance_status", 10)
        self._client = self.create_client(RobotCommand, "/integrated/robot_command")

        self.get_logger().info(
            "综合姿态补偿节点已启动: execute=%s mode=%s trigger=%.2fdeg"
            % (self._enable_execute, self._mode, self._trigger_threshold)
        )

    def _rpy_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        now = time.time()
        dt = now - self._last_sample_time if self._last_sample_time > 0.0 else 0.02
        self._last_sample_time = now

        raw_roll = math.degrees(float(msg.data[0]))
        raw_pitch = math.degrees(float(msg.data[1]))
        raw_yaw = math.degrees(float(msg.data[2])) if len(msg.data) > 2 else 0.0
        if not (math.isfinite(raw_roll) and math.isfinite(raw_pitch)):
            return

        filtered_roll, filtered_pitch = self._filter(raw_roll, raw_pitch)
        roll_error = filtered_roll - self._target_roll
        pitch_error = filtered_pitch - self._target_pitch
        roll_cmd = self._roll_sign * self._roll_pid.update(roll_error, dt, self._integral_limit)
        pitch_cmd = self._pitch_sign * self._pitch_pid.update(pitch_error, dt, self._integral_limit)
        roll_cmd = clamp(roll_cmd, -self._max_compensation, self._max_compensation)
        pitch_cmd = clamp(pitch_cmd, -self._max_compensation, self._max_compensation)
        if abs(roll_error) < self._settle_threshold:
            roll_cmd = 0.0
        if abs(pitch_error) < self._settle_threshold:
            pitch_cmd = 0.0

        self._state = AttitudeState(
            raw_roll_deg=raw_roll,
            raw_pitch_deg=raw_pitch,
            raw_yaw_deg=raw_yaw,
            filtered_roll_deg=filtered_roll,
            filtered_pitch_deg=filtered_pitch,
            command_roll_deg=roll_cmd,
            command_pitch_deg=pitch_cmd,
            stamp=now,
        )
        self._publish_attitude()
        self._publish_attitude_report(roll_error, pitch_error)
        self._publish_balance_status(compensating=self._action_lock.locked())
        self._log_control(roll_error, pitch_error, now)

        if self._should_trigger(roll_error, pitch_error, now):
            self._start_compensation(roll_cmd, pitch_cmd)

    def _filter(self, raw_roll: float, raw_pitch: float) -> tuple[float, float]:
        self._roll_window.append(raw_roll)
        self._pitch_window.append(raw_pitch)
        avg_roll = sum(self._roll_window) / len(self._roll_window)
        avg_pitch = sum(self._pitch_window) / len(self._pitch_window)
        if not self._filter_initialized:
            self._state.filtered_roll_deg = avg_roll
            self._state.filtered_pitch_deg = avg_pitch
            self._filter_initialized = True
        else:
            self._state.filtered_roll_deg = self._alpha * avg_roll + (1.0 - self._alpha) * self._state.filtered_roll_deg
            self._state.filtered_pitch_deg = self._alpha * avg_pitch + (1.0 - self._alpha) * self._state.filtered_pitch_deg
        return self._state.filtered_roll_deg, self._state.filtered_pitch_deg

    def _should_trigger(self, roll_error: float, pitch_error: float, now: float) -> bool:
        if not self._enable_execute:
            return False
        if abs(roll_error) < self._trigger_threshold and abs(pitch_error) < self._trigger_threshold:
            return False
        if now - self._last_action_time < self._action_cooldown:
            return False
        if self._action_lock.locked():
            return False
        return True

    def _start_compensation(self, roll_cmd: float, pitch_cmd: float):
        self._last_action_time = time.time()
        thread = threading.Thread(
            target=self._execute_compensation,
            args=(roll_cmd, pitch_cmd),
            daemon=True,
        )
        thread.start()

    def _execute_compensation(self, roll_cmd: float, pitch_cmd: float):
        with self._action_lock:
            self._last_event = (
                "[TRIGGER] compensation roll_cmd=%+7.2fdeg pitch_cmd=%+7.2fdeg duration=%.2fs mode=%s"
                % (roll_cmd, pitch_cmd, self._action_duration, self._mode)
            )
            self._publish_state(self._last_event)
            self._publish_balance_status(compensating=True)
            if not self._client.service_is_ready():
                self._client.wait_for_service(timeout_sec=0.2)
            if not self._client.service_is_ready():
                self._last_event = "motion gateway unavailable for balance compensation"
                self._publish_state(self._last_event)
                return
            req = RobotCommand.Request()
            req.action_name = "balance_compensate"
            req.params_json = json.dumps(
                {
                    "roll_deg": roll_cmd,
                    "pitch_deg": pitch_cmd,
                    "duration": self._action_duration,
                    "mode": self._mode,
                },
                ensure_ascii=False,
            )
            req.source = "balance"
            req.priority = 60
            req.require_safety_check = False
            self._client.call_async(req)
            self._publish_balance_status(compensating=False)

    def _publish_attitude(self):
        msg = Float32MultiArray()
        msg.data = [
            self._state.raw_roll_deg,
            self._state.raw_pitch_deg,
            self._state.filtered_roll_deg,
            self._state.filtered_pitch_deg,
            self._state.command_roll_deg,
            self._state.command_pitch_deg,
        ]
        self._attitude_pub.publish(msg)

    def _publish_attitude_report(self, roll_error: float, pitch_error: float):
        status = "stable"
        if abs(self._state.command_roll_deg) > 0.0 or abs(self._state.command_pitch_deg) > 0.0:
            status = "compensating"
        report = String()
        report.data = (
            "[Balance Attitude]\n"
            "raw      roll=%+7.2f deg | pitch=%+7.2f deg\n"
            "filtered roll=%+7.2f deg | pitch=%+7.2f deg\n"
            "error    roll=%+7.2f deg | pitch=%+7.2f deg\n"
            "command  roll=%+7.2f deg | pitch=%+7.2f deg\n"
            "status   %s"
            % (
                self._state.raw_roll_deg,
                self._state.raw_pitch_deg,
                self._state.filtered_roll_deg,
                self._state.filtered_pitch_deg,
                roll_error,
                pitch_error,
                self._state.command_roll_deg,
                self._state.command_pitch_deg,
                status,
            )
        )
        self._attitude_report_pub.publish(report)

    def _publish_balance_status(self, compensating: bool):
        msg = BalanceStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = "COMPENSATING" if compensating else "STABLE"
        msg.raw_roll_deg = float(self._state.raw_roll_deg)
        msg.raw_pitch_deg = float(self._state.raw_pitch_deg)
        msg.filtered_roll_deg = float(self._state.filtered_roll_deg)
        msg.filtered_pitch_deg = float(self._state.filtered_pitch_deg)
        msg.command_roll_deg = float(self._state.command_roll_deg)
        msg.command_pitch_deg = float(self._state.command_pitch_deg)
        msg.compensating = bool(compensating)
        msg.event = self._last_event
        self._status_pub.publish(msg)

    def _publish_state(self, text: str):
        msg = String()
        msg.data = text
        self._state_pub.publish(msg)
        self.get_logger().info(text)

    def _log_control(self, roll_error: float, pitch_error: float, now: float):
        if self._log_period <= 0.0 or now - self._last_log_time < self._log_period:
            return
        self._last_log_time = now
        self.get_logger().info(
            "[CTRL] raw R/P=(%+7.2f,%+7.2f)deg | filtered R/P=(%+7.2f,%+7.2f)deg | "
            "error R/P=(%+7.2f,%+7.2f)deg | cmd R/P=(%+7.2f,%+7.2f)deg"
            % (
                self._state.raw_roll_deg,
                self._state.raw_pitch_deg,
                self._state.filtered_roll_deg,
                self._state.filtered_pitch_deg,
                roll_error,
                pitch_error,
                self._state.command_roll_deg,
                self._state.command_pitch_deg,
            )
        )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="integrated_demo.yaml 配置文件路径")
    args = parser.parse_args(argv)

    rclpy.init()
    node = BalanceCompensatorNode(args.config)
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
