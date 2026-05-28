#!/usr/bin/env python3

import argparse
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from dobot_integrated_demo.config import load_yaml
from dobot_integrated_demo.voice.audio_player import AudioPlayer
from dobot_integrated_interfaces.msg import SafetyState
from dobot_integrated_interfaces.srv import RobotCommand


class MotionGatewayNode(Node):
    """The only node that owns RobotClient and executes robot commands."""

    SAFE = "SAFE"
    SENSOR_STALE = "SENSOR_STALE"
    FRONT_DANGER = "FRONT_DANGER"
    BACK_DANGER = "BACK_DANGER"
    BOTH_DANGER = "BOTH_DANGER"

    LOCOMOTION_ACTIONS = {
        "walk_forward",
        "walk_backward",
        "move_left",
        "move_right",
        "rotate_left",
        "rotate_right",
    }

    def __init__(
        self,
        config_file: str,
        grpc_addr: str | None = None,
        safety_enabled: bool | None = None,
        feedback_audio_test_key: str = "",
    ):
        super().__init__("dobot_integrated_motion_gateway_node")
        self._config_file = config_file
        self._config = load_yaml(config_file)
        robot_cfg = self._config.get("robot", {})
        gateway_cfg = self._config.get("motion_gateway", {})
        integrated_cfg = self._config.get("integrated", {})

        self._grpc_addr = grpc_addr or robot_cfg.get("grpc_addr", "192.168.5.2:50051")
        self._enable_safety = bool(integrated_cfg.get("enable_safety", True)) if safety_enabled is None else safety_enabled
        self._enable_execute = bool(robot_cfg.get("enable_execute", True))
        self._enable_builtin_oa = bool(robot_cfg.get("enable_builtin_obstacle_avoidance", True))
        self._enter_balance_stand = bool(robot_cfg.get("enter_balance_stand_on_start", False))
        self._reject_on_sensor_stale = bool(gateway_cfg.get("reject_locomotion_on_sensor_stale", True))

        self._robot = None
        self._connected = False
        self._command_lock = threading.Lock()
        self._current_action = ""
        self._system_state = "STARTING"
        self._last_safety = SafetyState()
        self._last_safety.state = self.SENSOR_STALE

        self._audio_player = None
        self._feedback_files: dict[str, str] = {}
        self._action_feedback_map: dict[str, str] = {}
        self._feedback_play_async = False

        self._event_pub = self.create_publisher(String, "/integrated/command_events", 10)
        self._connect_robot()
        self._init_feedback_audio()
        if feedback_audio_test_key:
            self._play_feedback_by_key("startup_test", feedback_audio_test_key)

        self._dispatch = {
            "walk_forward": self._walk_forward,
            "walk_backward": self._walk_backward,
            "move_left": self._move_left,
            "move_right": self._move_right,
            "rotate_left": self._rotate_left,
            "rotate_right": self._rotate_right,
            "wave": self._wave,
            "introduce": self._introduce,
            "play_feedback": self._play_feedback_action,
            "audio_test": self._play_feedback_action,
            "balance_stand": self._balance_stand,
            "balance_compensate": self._balance_compensate,
            "balance_neutral": self._balance_neutral,
            "enable_obstacle_avoidance": self._enable_obstacle_avoidance,
            "emergency_stop": self._emergency_stop,
            "stop": self._emergency_stop,
        }

        self.create_subscription(SafetyState, "/integrated/safety_state", self._safety_cb, 10)
        self._srv = self.create_service(
            RobotCommand,
            "/integrated/robot_command",
            self._handle_command,
        )

        self._system_state = "LISTENING" if self._connected or not self._enable_execute else "DISCONNECTED"
        self._publish_event(
            "motion_gateway ready: grpc=%s, execute=%s, safety=%s, builtin_oa=%s"
            % (self._grpc_addr, self._enable_execute, self._enable_safety, self._enable_builtin_oa)
        )

    def _connect_robot(self):
        if not self._enable_execute:
            self._connected = False
            self._publish_event("dry-run mode: robot execution disabled")
            return
        try:
            from dobot_quad import RobotClient

            self.get_logger().info("正在连接机器人: %s" % self._grpc_addr)
            self._robot = RobotClient(self._grpc_addr)
            self._robot.enable_safety_ready()
            if self._enable_builtin_oa:
                try:
                    self._robot.set_obstacle_avoidance(True)
                except Exception as exc:
                    self.get_logger().warning("set_obstacle_avoidance failed: %s" % exc)
            if self._enter_balance_stand:
                self._robot.balance_stand(show_progress=False)
            self._connected = True
            self._publish_event("robot connected: state=%s" % self._get_robot_state())
        except Exception as exc:
            self._robot = None
            self._connected = False
            self._publish_event("robot connection failed: %s" % exc, warn=True)

    def _init_feedback_audio(self):
        feedback_cfg = self._config.get("feedback_audio", {})
        if not feedback_cfg.get("enabled", True):
            self.get_logger().info("本地反馈音频已禁用")
            return

        robot_cfg = self._config.get("robot", {})
        self._audio_player = AudioPlayer(
            dds_domain_id=robot_cfg.get("dds_domain_id", 0),
            dds_config=self._resolve_dds_config(robot_cfg.get("dds_config")),
            topic_name=str(feedback_cfg.get("voice_cmd_topic", "rt/voice/cmd_tmp")),
            action_topic_name=str(feedback_cfg.get("voice_action_topic", "rt/action/state")),
            protocol=str(feedback_cfg.get("voice_cmd_protocol", "task")),
        )
        try:
            self._audio_player.init()
        except Exception as exc:
            self.get_logger().warning("反馈音频初始化失败，将仅输出日志: %s" % exc)
            self._audio_player = None
            return

        base_dir = self._resolve_feedback_base_dir(feedback_cfg)
        files_cfg = feedback_cfg.get("files", {})
        self._feedback_files = {
            key: str((base_dir / file_name).resolve())
            for key, file_name in files_cfg.items()
        }
        self._action_feedback_map = dict(feedback_cfg.get("action_map", {}))
        self.get_logger().info(
            "feedback audio ready: async=%s files=%d base=%s"
            % (self._feedback_play_async, len(self._feedback_files), base_dir)
        )
        for key, path in self._feedback_files.items():
            status = "OK" if Path(path).exists() else "MISSING"
            self.get_logger().info("反馈音频[%s]: %s (%s)" % (key, path, status))

    def _resolve_dds_config(self, configured_path: str | None) -> str | None:
        if not configured_path:
            return None

        path = Path(configured_path)
        if path.is_absolute() and path.exists():
            return str(path)

        candidates = []
        config_path = Path(self._config_file) if self._config_file else None
        if config_path and config_path.exists():
            candidates.append((config_path.parent / path).resolve())

        package_root = Path(__file__).resolve().parent.parent
        candidates.append((package_root / path).resolve())
        candidates.append((Path.cwd() / path).resolve())
        candidates.append(
            (
                Path.cwd()
                / "dobot_quad_sdk-main"
                / "low_level"
                / "python"
                / "config"
                / "dds_config.yaml"
            ).resolve()
        )
        candidates.append(
            (
                Path.cwd()
                / "src"
                / "dobot_quad_sdk-main"
                / "low_level"
                / "python"
                / "config"
                / "dds_config.yaml"
            ).resolve()
        )
        candidates.append(
            (
                package_root.parent
                / "dobot_quad_sdk-main"
                / "low_level"
                / "python"
                / "config"
                / "dds_config.yaml"
            ).resolve()
        )

        for candidate in candidates:
            if candidate.exists():
                self.get_logger().info("DDS 配置文件: %s" % str(candidate))
                return str(candidate)

        self.get_logger().warning(
            "未找到 DDS 配置文件，回退为 domain_id 模式: %s" % configured_path
        )
        return None

    def _resolve_feedback_base_dir(self, feedback_cfg: dict) -> Path:
        configured = feedback_cfg.get("base_dir", "")
        if configured:
            base_dir = Path(configured).expanduser()
            if base_dir.is_absolute():
                return base_dir.resolve()
            cwd_candidate = (Path.cwd() / base_dir).resolve()
            if cwd_candidate.exists():
                return cwd_candidate
            cfg = Path(self._config_file).expanduser()
            if cfg.exists():
                candidate = (cfg.parent / base_dir).resolve()
                if candidate.exists():
                    return candidate
            try:
                candidate = Path(get_package_share_directory("dobot_integrated_demo")) / str(configured)
                if candidate.exists():
                    return candidate.resolve()
            except Exception:
                pass
            return (Path.cwd() / base_dir).resolve()

        candidates = [
            Path.cwd() / "wavs",
            Path.cwd() / ".." / "wavs",
        ]
        try:
            candidates.append(Path(get_package_share_directory("dobot_integrated_demo")) / "wavs")
        except Exception:
            pass
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        return (Path.cwd() / "wavs").resolve()

    def _safety_cb(self, msg: SafetyState):
        self._last_safety = msg

    def _handle_command(self, request: RobotCommand.Request, response: RobotCommand.Response):
        action = request.action_name.strip()
        source = request.source.strip() or "unknown"
        params = self._parse_params(request.params_json)

        response.robot_state = self._get_robot_state()
        response.system_state = self._system_state

        handler = self._dispatch.get(action)
        if handler is None:
            response.accepted = False
            response.success = False
            response.message = "未知动作: %s" % action
            self._publish_event(response.message, warn=True)
            return response

        allowed, reason = self._safety_allows(action, request.require_safety_check)
        if not allowed:
            response.accepted = False
            response.success = False
            response.message = reason
            self._publish_event(
                "blocked command: action=%s source=%s reason=%s" % (action, source, reason),
                warn=True,
            )
            return response

        if self._enable_execute and (not self._connected or self._robot is None):
            response.accepted = False
            response.success = False
            response.message = "机器人未连接"
            response.robot_state = "disconnected"
            self._publish_event("command rejected because robot is disconnected: %s" % action, warn=True)
            return response

        with self._command_lock:
            self._current_action = action
            self._system_state = self._state_for_action(action, source)
            self._publish_event(
                "accepted command: action=%s source=%s priority=%d params=%s"
                % (action, source, int(request.priority), params)
            )
            self._play_feedback_for_action(action)
            started = time.time()
            try:
                ok, message = handler(params)
            except Exception as exc:
                ok = False
                message = "执行异常: %s" % exc
                self.get_logger().error(message)

            elapsed = time.time() - started
            self._current_action = ""
            self._system_state = "LISTENING" if self._safety_allows_listening() else "AVOIDING"

        response.accepted = True
        response.success = bool(ok)
        response.message = "%s (%.2fs)" % (message, elapsed)
        response.robot_state = self._get_robot_state()
        response.system_state = self._system_state
        level_warn = not ok
        self._publish_event(
            "completed command: action=%s success=%s message=%s" % (action, ok, response.message),
            warn=level_warn,
        )
        return response

    def _parse_params(self, params_json: str) -> dict:
        if not params_json:
            return {}
        try:
            data = json.loads(params_json)
            return data if isinstance(data, dict) else {}
        except json.JSONDecodeError:
            return {}

    def _safety_allows(self, action: str, require_check: bool) -> tuple[bool, str]:
        if not require_check:
            return True, "safety check skipped"
        if not self._enable_safety:
            return True, "safety module disabled"
        state = self._last_safety.state or self.SENSOR_STALE
        if action not in self.LOCOMOTION_ACTIONS:
            return True, "non-locomotion action"
        if state == self.SAFE:
            return True, "safe"
        if state == self.SENSOR_STALE and self._reject_on_sensor_stale:
            return False, "安全传感器超时，拒绝移动动作"
        if state == self.FRONT_DANGER and action == "walk_forward":
            return False, "前方危险，拒绝前进"
        if state == self.BACK_DANGER and action == "walk_backward":
            return False, "后方危险，拒绝后退"
        if state == self.BOTH_DANGER and action in {"walk_forward", "walk_backward", "move_left", "move_right"}:
            return False, "前后均危险，拒绝平移动作"
        if state in (self.FRONT_DANGER, self.BACK_DANGER) and action in {"move_left", "move_right"}:
            return False, "处于避障状态，拒绝横移"
        return True, "allowed"

    def _safety_allows_listening(self) -> bool:
        return self._last_safety.state in ("", self.SAFE)

    def _state_for_action(self, action: str, source: str) -> str:
        if action in ("emergency_stop", "stop"):
            return "AVOIDING"
        if action.startswith("balance_") or source == "balance":
            return "BALANCE_COMPENSATING"
        return "EXECUTING"

    def _play_feedback_for_action(self, action_name: str):
        feedback_key = self._action_feedback_map.get(action_name)
        if not feedback_key:
            return
        self._play_feedback_by_key(feedback_key)

    def _play_feedback_by_key(self, feedback_key: str):
        if not self._audio_player:
            return

        file_path = self._feedback_files.get(feedback_key)
        if not file_path:
            self.get_logger().warning("未配置反馈音频 key: %s" % feedback_key)
            return

        if not Path(file_path).exists():
            self.get_logger().warning("反馈音频文件不存在: %s" % file_path)
            return

        if not self._feedback_play_async:
            self._play_feedback_file(feedback_key, file_path)
            return
        thread = threading.Thread(
            target=self._play_feedback_file,
            args=(feedback_key, file_path),
            daemon=True,
        )
        thread.start()

    def _play_feedback_file(self, feedback_key: str, file_path: str):
        try:
            if not self._audio_player:
                return
            self.get_logger().info("播放动作反馈音频: key=%s file=%s" % (feedback_key, file_path))
            self._audio_player.play_local_wav(file_path)
        except Exception as exc:
            self.get_logger().warning("反馈音频播放失败: %s" % exc)

    def _walk_forward(self, params: dict) -> tuple[bool, str]:
        distance = float(params.get("distance", 0.5))
        res = self._call_robot("walk_forward", distance=distance, show_progress=False)
        return self._is_success(res), "前进 %.2fm %s" % (distance, self._ok_text(res))

    def _walk_backward(self, params: dict) -> tuple[bool, str]:
        distance = float(params.get("distance", 0.3))
        res = self._call_robot("walk_backward", distance=distance, show_progress=False)
        return self._is_success(res), "后退 %.2fm %s" % (distance, self._ok_text(res))

    def _move_left(self, params: dict) -> tuple[bool, str]:
        distance = float(params.get("distance", 0.2))
        res = self._call_robot("move_left", distance=distance, show_progress=False)
        return self._is_success(res), "左移 %.2fm %s" % (distance, self._ok_text(res))

    def _move_right(self, params: dict) -> tuple[bool, str]:
        distance = float(params.get("distance", 0.2))
        res = self._call_robot("move_right", distance=distance, show_progress=False)
        return self._is_success(res), "右移 %.2fm %s" % (distance, self._ok_text(res))

    def _rotate_left(self, params: dict) -> tuple[bool, str]:
        angle = float(params.get("angle", 45.0))
        res = self._call_robot("rotate_left", angle=angle, show_progress=False)
        return self._is_success(res), "左转 %.1fdeg %s" % (angle, self._ok_text(res))

    def _rotate_right(self, params: dict) -> tuple[bool, str]:
        angle = float(params.get("angle", 45.0))
        res = self._call_robot("rotate_right", angle=angle, show_progress=False)
        return self._is_success(res), "右转 %.1fdeg %s" % (angle, self._ok_text(res))

    def _wave(self, params: dict) -> tuple[bool, str]:
        res = self._call_robot("wave", show_progress=False)
        return self._is_success(res), "摆手 %s" % self._ok_text(res)

    def _introduce(self, params: dict) -> tuple[bool, str]:
        return True, "自我介绍播报完成"

    def _play_feedback_action(self, params: dict) -> tuple[bool, str]:
        key = str(params.get("key", "greeting")).strip() or "greeting"
        action_name = str(params.get("action", "audio_test")).strip() or "audio_test"
        file_path = self._feedback_files.get(key)
        if not file_path:
            return False, "feedback audio key is not configured: %s" % key
        if not Path(file_path).exists():
            return False, "feedback audio file does not exist: %s" % file_path
        self._play_feedback_file(key, file_path)
        return True, "requested feedback audio key=%s file=%s" % (key, file_path)

    def _balance_stand(self, params: dict) -> tuple[bool, str]:
        res = self._call_robot("balance_stand", show_progress=False)
        return self._is_success(res), "切换平衡站立 %s" % self._ok_text(res)

    def _balance_neutral(self, params: dict) -> tuple[bool, str]:
        duration = float(params.get("duration", 0.5))
        res = self._call_robot("balance_neutral", duration=duration, show_progress=False)
        return self._is_success(res), "姿态回中 %s" % self._ok_text(res)

    def _balance_compensate(self, params: dict) -> tuple[bool, str]:
        roll = float(params.get("roll_deg", 0.0))
        pitch = float(params.get("pitch_deg", 0.0))
        duration = max(1.0, float(params.get("duration", 1.0)))
        mode = str(params.get("mode", "combined"))
        if not math.isfinite(roll) or not math.isfinite(pitch):
            return False, "补偿角度无效"
        if mode == "axis":
            ok = True
            if abs(roll) > 0.01:
                ok = ok and self._is_success(self._call_robot("balance_roll", roll, duration, "dynamic", show_progress=False))
            if abs(pitch) > 0.01:
                ok = ok and self._is_success(self._call_robot("balance_pitch", pitch, duration, "dynamic", show_progress=False))
            return ok, "分轴姿态补偿 roll=%+.2f pitch=%+.2f %s" % (roll, pitch, "成功" if ok else "失败")
        res = self._call_robot(
            "dynamic_pose",
            duration=duration,
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=0.0,
            height_m=0.0,
            show_progress=False,
        )
        return self._is_success(res), "组合姿态补偿 roll=%+.2f pitch=%+.2f %s" % (roll, pitch, self._ok_text(res))

    def _enable_obstacle_avoidance(self, params: dict) -> tuple[bool, str]:
        enable = bool(params.get("enable", True))
        res = self._call_robot("set_obstacle_avoidance", enable)
        return self._is_success(res, default=True), "内置避障设置为 %s" % enable

    def _emergency_stop(self, params: dict) -> tuple[bool, str]:
        res = self._call_robot("emergency", show_progress=False)
        return self._is_success(res), "急停 %s" % self._ok_text(res)

    def _call_robot(self, method: str, *args, **kwargs):
        if not self._enable_execute:
            self.get_logger().info("[dry-run] %s args=%s kwargs=%s" % (method, args, kwargs))
            return _DryRunResult()
        func = getattr(self._robot, method)
        return func(*args, **kwargs)

    @staticmethod
    def _is_success(res, default: bool = False) -> bool:
        if isinstance(res, _DryRunResult):
            return True
        if res is None:
            return default
        return bool(getattr(res, "success", default))

    def _ok_text(self, res) -> str:
        return "成功" if self._is_success(res) else "失败"

    def _get_robot_state(self) -> str:
        if self._enable_execute and self._connected and self._robot:
            try:
                return str(self._robot.get_current_state_name())
            except Exception:
                return "unknown"
        return "dry-run" if not self._enable_execute else "disconnected"

    def _publish_event(self, text: str, warn: bool = False):
        msg = String()
        msg.data = text
        self._event_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def destroy_node(self):
        if self._robot is not None:
            try:
                self._robot.close()
            except Exception:
                pass
        self._audio_player = None
        super().destroy_node()


class _DryRunResult:
    success = True


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="integrated_demo.yaml 配置文件路径")
    parser.add_argument("--grpc-addr", default=None, help="覆盖配置文件中的机器人 gRPC 地址")
    parser.add_argument("--safety-enabled", default=None, help="是否启用安全仲裁 true/false")
    parser.add_argument("--feedback-audio-test-key", default="", help="play one feedback wav after startup")
    args = parser.parse_args(argv)

    safety_enabled = None
    if args.safety_enabled is not None:
        safety_enabled = str(args.safety_enabled).lower() in ("1", "true", "yes", "on")

    rclpy.init()
    node = MotionGatewayNode(
        args.config,
        args.grpc_addr,
        safety_enabled=safety_enabled,
        feedback_audio_test_key=args.feedback_audio_test_key,
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
