#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from dobot_integrated_demo.config import load_yaml
from dobot_integrated_interfaces.msg import BalanceStatus, SafetyState, SystemState


class DemoMonitorNode(Node):
    """Classroom-friendly terminal monitor for the integrated demo."""

    def __init__(self, config_file: str):
        super().__init__("dobot_integrated_demo_monitor_node")
        config = load_yaml(config_file)
        display_cfg = config.get("display", {})

        self._period = max(0.1, float(display_cfg.get("monitor_period_seconds", 0.5)))
        self._stale_timeout = max(0.5, float(display_cfg.get("stale_timeout_seconds", 2.0)))

        self._latest_imu: list[float] | None = None
        self._latest_attitude: list[float] | None = None
        self._latest_system: SystemState | None = None
        self._latest_safety: SafetyState | None = None
        self._latest_balance: BalanceStatus | None = None
        self._latest_command = "等待动作事件"
        self._latest_depth_status = "等待深度状态"
        self._latest_event = "等待综合数据"
        self._last_imu_time = 0.0
        self._last_attitude_time = 0.0
        self._last_system_time = 0.0
        self._last_safety_time = 0.0
        self._last_balance_time = 0.0
        self._last_command_time = 0.0
        self._trigger_count = 0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Float32MultiArray, "/balance_control/imu_rpy", self._imu_cb, sensor_qos)
        self.create_subscription(Float32MultiArray, "/balance_control/attitude", self._attitude_cb, sensor_qos)
        self.create_subscription(String, "/balance_control/state", self._state_cb, 10)
        self.create_subscription(String, "/balance_control/imu_status", self._imu_status_cb, 10)
        self.create_subscription(SystemState, "/integrated/system_state", self._system_cb, 10)
        self.create_subscription(SafetyState, "/integrated/safety_state", self._safety_cb, 10)
        self.create_subscription(BalanceStatus, "/integrated/balance_status", self._balance_cb, 10)
        self.create_subscription(String, "/integrated/command_events", self._command_event_cb, 10)
        self.create_subscription(String, "/safety_guard/depth_status", self._depth_status_cb, 10)
        self.create_timer(self._period, self._render)

        self.get_logger().info("综合教学面板已启动：主控台只展示语音、动作、安全、平衡摘要")

    def _imu_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._latest_imu = [math.degrees(float(v)) for v in msg.data[:3]]
            self._last_imu_time = time.time()

    def _attitude_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self._latest_attitude = [float(v) for v in msg.data[:6]]
            self._last_attitude_time = time.time()

    def _state_cb(self, msg: String):
        self._latest_event = msg.data
        if "[TRIGGER]" in msg.data or "compensation" in msg.data:
            self._trigger_count += 1

    def _imu_status_cb(self, msg: String):
        self._latest_event = msg.data

    def _system_cb(self, msg: SystemState):
        self._latest_system = msg
        self._last_system_time = time.time()

    def _safety_cb(self, msg: SafetyState):
        self._latest_safety = msg
        self._last_safety_time = time.time()

    def _balance_cb(self, msg: BalanceStatus):
        was_compensating = self._latest_balance.compensating if self._latest_balance else False
        self._latest_balance = msg
        self._last_balance_time = time.time()
        if msg.compensating and not was_compensating:
            self._trigger_count += 1

    def _command_event_cb(self, msg: String):
        self._latest_command = msg.data
        self._last_command_time = time.time()

    def _depth_status_cb(self, msg: String):
        self._latest_depth_status = msg.data

    def _render(self):
        now = time.time()
        imu_ok = self._latest_imu is not None and now - self._last_imu_time <= self._stale_timeout
        ctrl_ok = (
            self._latest_attitude is not None
            and now - self._last_attitude_time <= self._stale_timeout
        )
        system_ok = self._latest_system is not None and now - self._last_system_time <= self._stale_timeout
        safety_ok = self._latest_safety is not None and now - self._last_safety_time <= self._stale_timeout
        balance_ok = self._latest_balance is not None and now - self._last_balance_time <= self._stale_timeout
        command_ok = self._last_command_time > 0.0 and now - self._last_command_time <= max(10.0, self._stale_timeout)

        imu = self._latest_imu or [float("nan"), float("nan"), float("nan")]
        att = self._latest_attitude or [float("nan")] * 6
        raw_roll, raw_pitch, filt_roll, filt_pitch, cmd_roll, cmd_pitch = att
        system = self._latest_system
        safety = self._latest_safety
        balance = self._latest_balance
        state = system.state if system_ok and system else "WAIT"
        mode = system.mode if system_ok and system else "-"
        action = system.active_action if system_ok and system and system.active_action else "-"
        safety_state = safety.state if safety_ok and safety else "WAIT"
        front = safety.front_distance_m if safety_ok and safety else float("nan")
        back = safety.back_distance_m if safety_ok and safety else float("nan")
        balance_state = balance.state if balance_ok and balance else "WAIT"
        filt_roll = balance.filtered_roll_deg if balance_ok and balance else filt_roll
        filt_pitch = balance.filtered_pitch_deg if balance_ok and balance else filt_pitch
        cmd_roll = balance.command_roll_deg if balance_ok and balance else cmd_roll
        cmd_pitch = balance.command_pitch_deg if balance_ok and balance else cmd_pitch
        command = self._latest_command if command_ok else "等待动作事件"

        lines = [
            "",
            "================ Dobot Integrated Teaching Console ================",
            "[SYSTEM]  state=%s | mode=%s | action=%s" % (state, mode, action),
            "[VOICE]   last_command=%s" % command,
            "[SAFETY]  state=%s | front=%s | back=%s | %s"
            % (safety_state, self._fmt_m(front), self._fmt_m(back), "OK" if safety_ok else "WAIT"),
            "[BALANCE] state=%s | roll=%s | pitch=%s | cmd=(%s,%s) | %s"
            % (
                balance_state,
                self._fmt_deg(filt_roll),
                self._fmt_deg(filt_pitch),
                self._fmt_deg(cmd_roll),
                self._fmt_deg(cmd_pitch),
                "OK" if balance_ok or ctrl_ok or imu_ok else "WAIT",
            ),
            "[IMU]     raw_roll=%s | raw_pitch=%s | yaw=%s | %s"
            % (
                self._fmt_deg(imu[0] if imu_ok else raw_roll),
                self._fmt_deg(imu[1] if imu_ok else raw_pitch),
                self._fmt_deg(imu[2]),
                "OK" if imu_ok else "WAIT",
            ),
            "[EVENT]   balance_triggers=%03d | %s"
            % (self._trigger_count, self._short_event(system.last_event if system_ok and system else self._latest_event)),
            "[DEPTH]   %s" % self._short_event(self._latest_depth_status),
            "===================================================================",
        ]
        self.get_logger().info("\n".join(lines))

    @staticmethod
    def _fmt_m(value: float) -> str:
        return "nan" if not math.isfinite(value) else "%.2fm" % value

    @staticmethod
    def _fmt_deg(value: float) -> str:
        return "nan" if not math.isfinite(value) else "%+.1fdeg" % value

    @staticmethod
    def _short_event(text: str, limit: int = 100) -> str:
        text = " ".join(str(text or "-").split())
        return text if len(text) <= limit else text[: limit - 3] + "..."


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="integrated_demo.yaml 配置文件路径")
    args = parser.parse_args(argv)

    rclpy.init()
    node = DemoMonitorNode(args.config)
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
