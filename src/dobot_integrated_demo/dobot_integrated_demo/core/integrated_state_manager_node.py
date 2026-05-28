#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from dobot_integrated_demo.config import load_yaml
from dobot_integrated_interfaces.msg import BalanceStatus, SafetyState, SystemState
from dobot_integrated_interfaces.srv import SetDemoMode


class IntegratedStateManagerNode(Node):
    """Classroom-facing state machine for the integrated demo."""

    def __init__(
        self,
        config_file: str,
        enable_voice: bool | None = None,
        enable_safety: bool | None = None,
        enable_balance: bool | None = None,
    ):
        super().__init__("dobot_integrated_state_manager_node")
        self._config = load_yaml(config_file)
        integrated_cfg = self._config.get("integrated", {})
        self._mode_enabled = {
            "voice": bool(integrated_cfg.get("enable_voice", True)) if enable_voice is None else enable_voice,
            "safety": bool(integrated_cfg.get("enable_safety", True)) if enable_safety is None else enable_safety,
            "balance": bool(integrated_cfg.get("enable_balance", True)) if enable_balance is None else enable_balance,
        }
        self._state = "LISTENING" if self._mode_enabled["voice"] else "IDLE"
        self._active_action = ""
        self._safety_state = "UNKNOWN"
        self._balance_state = "UNKNOWN"
        self._last_event = "integrated demo starting"
        self._last_command_time = 0.0
        self._command_hold_seconds = float(integrated_cfg.get("command_state_hold_seconds", 1.0))

        self._pub = self.create_publisher(SystemState, "/integrated/system_state", 10)
        self._text_pub = self.create_publisher(String, "/integrated/system_state_text", 10)
        self.create_subscription(SafetyState, "/integrated/safety_state", self._safety_cb, 10)
        self.create_subscription(BalanceStatus, "/integrated/balance_status", self._balance_cb, 10)
        self.create_subscription(String, "/integrated/command_events", self._command_event_cb, 10)
        self._mode_srv = self.create_service(SetDemoMode, "/integrated/set_demo_mode", self._set_mode)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("综合状态机已启动: %s" % self._mode_enabled)

    def _safety_cb(self, msg: SafetyState):
        self._safety_state = msg.state
        if msg.state in ("FRONT_DANGER", "BACK_DANGER", "BOTH_DANGER"):
            self._state = "AVOIDING"
            self._last_event = msg.event
        elif msg.state == "SENSOR_STALE" and self._mode_enabled["safety"]:
            if self._state not in ("EXECUTING", "BALANCE_COMPENSATING"):
                self._state = "IDLE"
            self._last_event = msg.event
        elif self._state == "AVOIDING" and msg.state == "SAFE":
            self._state = "LISTENING" if self._mode_enabled["voice"] else "IDLE"
            self._last_event = msg.event

    def _balance_cb(self, msg: BalanceStatus):
        self._balance_state = msg.state
        if msg.compensating:
            self._state = "BALANCE_COMPENSATING"
            self._active_action = "balance_compensate"
            self._last_event = msg.event
            self._last_command_time = time.time()

    def _command_event_cb(self, msg: String):
        text = msg.data
        self._last_event = text
        now = time.time()
        if text.startswith("accepted command:"):
            self._active_action = self._extract_action(text)
            if "source=balance" in text or self._active_action.startswith("balance_"):
                self._state = "BALANCE_COMPENSATING"
            elif self._active_action in ("emergency_stop", "stop"):
                self._state = "AVOIDING"
            else:
                self._state = "EXECUTING"
            self._last_command_time = now
        elif text.startswith("completed command:"):
            self._active_action = ""
            if self._safety_state in ("FRONT_DANGER", "BACK_DANGER", "BOTH_DANGER"):
                self._state = "AVOIDING"
            else:
                self._state = "LISTENING" if self._mode_enabled["voice"] else "IDLE"
            self._last_command_time = now

    def _set_mode(self, request: SetDemoMode.Request, response: SetDemoMode.Response):
        mode = request.mode.strip().lower()
        if mode not in self._mode_enabled:
            response.success = False
            response.message = "未知模式: %s" % mode
            response.active_mode = self._active_mode_text()
            return response
        self._mode_enabled[mode] = bool(request.enabled)
        response.success = True
        response.message = "%s set to %s" % (mode, request.enabled)
        response.active_mode = self._active_mode_text()
        self._last_event = response.message
        return response

    def _tick(self):
        now = time.time()
        if self._state in ("EXECUTING", "BALANCE_COMPENSATING"):
            if now - self._last_command_time > self._command_hold_seconds and not self._active_action:
                self._state = "LISTENING" if self._mode_enabled["voice"] else "IDLE"
        self._publish()

    def _publish(self):
        msg = SystemState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.mode = self._active_mode_text()
        msg.active_action = self._active_action
        msg.safety_state = self._safety_state
        msg.balance_state = self._balance_state
        msg.last_event = self._last_event
        self._pub.publish(msg)

        text = String()
        text.data = (
            "state=%s | mode=%s | action=%s | safety=%s | balance=%s | event=%s"
            % (
                msg.state,
                msg.mode,
                msg.active_action or "-",
                msg.safety_state,
                msg.balance_state,
                msg.last_event,
            )
        )
        self._text_pub.publish(text)

    def _active_mode_text(self) -> str:
        return ",".join(k for k, enabled in self._mode_enabled.items() if enabled) or "none"

    @staticmethod
    def _extract_action(text: str) -> str:
        marker = "action="
        if marker not in text:
            return ""
        tail = text.split(marker, 1)[1]
        return tail.split(" ", 1)[0]


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="integrated_demo.yaml 配置文件路径")
    parser.add_argument("--enable-voice", default=None, help="是否启用语音模块 true/false")
    parser.add_argument("--enable-safety", default=None, help="是否启用安全模块 true/false")
    parser.add_argument("--enable-balance", default=None, help="是否启用平衡模块 true/false")
    args = parser.parse_args(argv)

    def parse_bool(value):
        if value is None:
            return None
        return str(value).lower() in ("1", "true", "yes", "on")

    rclpy.init()
    node = IntegratedStateManagerNode(
        args.config,
        enable_voice=parse_bool(args.enable_voice),
        enable_safety=parse_bool(args.enable_safety),
        enable_balance=parse_bool(args.enable_balance),
    )
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
