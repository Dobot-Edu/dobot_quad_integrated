#!/usr/bin/env python3
"""
语音控制主节点 - 串联完整的语音交互流程 (ROS2 Node 版)

功能:
  - 作为 ROS2 节点运行，使用 rclpy
  - 启动麦克风采集，持续监听语音输入
  - 检测到语音段后调用 ASR 识别文本
  - 将文本通过意图解析器匹配为动作
  - 动作触发后由动作服务端播放本地反馈音频
  - 通过 /integrated/robot_command ROS2 Service 执行机器人动作
  - 提供完整的运行日志

流程:
  [麦克风采集] → [VAD 切分] → [ASR 识别] → [意图解析]
      → [Service 调用动作 + 本地反馈音频] → [返回监听]

使用方式:
  # 通过 launch 启动（推荐，会同时启动 action server）
  ros2 launch dobot_integrated_demo integrated_demo.launch.py

  # 单独运行（需先启动 motion_gateway_node）
  ros2 run dobot_integrated_demo voice_control_node
"""

import os
import sys
import time
import signal
import logging
import argparse
import threading
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from dobot_integrated_demo.voice.audio_capture import AudioCapture
from dobot_integrated_demo.voice.asr_engine import BaiduASREngine, VoskGrammarASREngine
from dobot_integrated_demo.voice.intent_parser import IntentParser
from dobot_integrated_demo.voice.action_executor import ActionExecutor

logger = logging.getLogger("voice_control")


class VoiceControlNode(Node):
    """
    语音控制主节点 (ROS2 Node)

    串联音频采集、ASR、意图解析和动作执行的完整流程。
    动作执行通过 ROS2 Service Client 调用 /integrated/robot_command 服务。

    Parameters
    ----------
    config : dict
        配置字典（从 YAML 文件加载）
    """

    def __init__(self, config: dict):
        super().__init__("voice_control_node")
        self._config = config
        self._running = False

        # 处理锁 - 防止并发处理多个语音段
        self._processing_lock = threading.Lock()
        self._is_processing = False
        self._pending_pcm_data: bytes | None = None

        # ---- 初始化各模块 ----

        # 音频采集
        audio_cfg = config.get("audio", {})
        vad_cfg = audio_cfg.get("vad", {})
        robot_cfg = config.get("robot", {})
        dds_config = self._resolve_dds_config(robot_cfg.get("dds_config"))

        self._audio_capture = AudioCapture(
            dds_domain_id=robot_cfg.get("dds_domain_id", 0),
            dds_config=dds_config,
            sample_rate=audio_cfg.get("sample_rate", 24000),
            channels=audio_cfg.get("channels", 1),
            sample_width=audio_cfg.get("sample_width", 2),
            energy_threshold=vad_cfg.get("energy_threshold", 500),
            silence_duration=vad_cfg.get("silence_duration", 1.5),
            max_record_duration=vad_cfg.get("max_record_duration", 8.0),
            min_speech_duration=vad_cfg.get("min_speech_duration", 0.3),
            adaptive_noise=vad_cfg.get("adaptive_noise", True),
            start_threshold_ratio=vad_cfg.get("start_threshold_ratio", 3.0),
            stop_threshold_ratio=vad_cfg.get("stop_threshold_ratio", 1.8),
            noise_alpha=vad_cfg.get("noise_alpha", 0.03),
            pre_speech_padding_seconds=vad_cfg.get("pre_speech_padding_seconds", 0.25),
        )

        # ASR 引擎
        asr_cfg = config.get("asr", {})
        provider = str(asr_cfg.get("provider", "vosk")).lower()
        baidu_asr_cfg = asr_cfg.get("baidu", {})
        baidu_engine = BaiduASREngine(
            app_id=baidu_asr_cfg.get("app_id", ""),
            api_key=baidu_asr_cfg.get("api_key", ""),
            secret_key=baidu_asr_cfg.get("secret_key", ""),
            dev_pid=baidu_asr_cfg.get("dev_pid", 1537),
            connect_timeout=baidu_asr_cfg.get("connect_timeout", 3.0),
            read_timeout=baidu_asr_cfg.get("read_timeout", 8.0),
            retry_count=baidu_asr_cfg.get("retry_count", 2),
            fallback_dev_pids=baidu_asr_cfg.get("fallback_dev_pids", [1936]),
        )
        if provider == "vosk":
            vosk_cfg = asr_cfg.get("vosk", {})
            self._asr_engine = VoskGrammarASREngine(
                model_path=self._resolve_model_path(vosk_cfg.get("model_path", "models/vosk-model-small-cn-0.22")),
                grammar_phrases=self._build_command_grammar(config.get("commands", [])),
                sample_rate=int(vosk_cfg.get("sample_rate", 16000)),
                fallback_engine=baidu_engine if bool(vosk_cfg.get("fallback_to_baidu", False)) else None,
                use_grammar=bool(vosk_cfg.get("use_grammar", False)),
            )
        else:
            self._asr_engine = baidu_engine

        # 意图解析
        commands = config.get("commands", [])
        unknown_feedback = config.get("unknown_feedback", "没有听清，请再说一次")
        self._intent_parser = IntentParser(
            commands=commands,
            unknown_feedback=unknown_feedback,
        )

        # 动作执行 (ROS2 Service Client)
        service_cfg = config.get("service", {})
        self._action_executor = ActionExecutor(
            ros_node=self,
            service_name=service_cfg.get("action_service", "/integrated/robot_command"),
            timeout_sec=service_cfg.get("action_timeout", 30.0),
            source=service_cfg.get("source", "voice"),
            priority=int(service_cfg.get("priority", 20)),
        )

        logger.info("=" * 60)
        logger.info("语音控制节点初始化完成 (ROS2 Service 模式)")
        logger.info("ASR provider: %s", provider)
        logger.info("=" * 60)

    def _build_command_grammar(self, commands: list[dict]) -> list[str]:
        phrases = []
        for cmd in commands:
            keyword = str(cmd.get("keyword", "")).strip()
            if keyword:
                phrases.append(keyword)
            for synonym in cmd.get("synonyms", []):
                synonym = str(synonym).strip()
                if synonym:
                    phrases.append(synonym)
        return phrases

    def _resolve_model_path(self, configured_path: str) -> str:
        path = Path(configured_path).expanduser()
        if path.is_absolute() and path.exists():
            return str(path)
        candidates = [
            Path.cwd() / path,
            Path.cwd() / "models" / path,
            Path.cwd() / "src" / "dobot_integrated_demo" / str(path),
        ]
        try:
            share_dir = Path(get_package_share_directory("dobot_integrated_demo"))
            candidates.append(share_dir / path)
        except Exception:
            pass
        for candidate in candidates:
            if candidate.exists():
                return str(candidate.resolve())
        return str((Path.cwd() / path).resolve())

    def _resolve_dds_config(self, configured_path: str | None) -> str | None:
        if not configured_path:
            return None

        path = Path(configured_path)
        if path.is_absolute() and path.exists():
            return str(path)

        candidates = []
        config_path = (
            Path(__file__).resolve().parent.parent
            / "config"
            / "integrated_demo.yaml"
        )
        if config_path.exists():
            candidates.append((config_path.parent / path).resolve())

        package_root = Path(__file__).resolve().parent.parent
        candidates.append((package_root / path).resolve())
        candidates.append((Path.cwd() / path).resolve())

        try:
            share_dir = Path(get_package_share_directory("dobot_integrated_demo"))
            candidates.append((share_dir / path).resolve())
        except Exception:
            pass

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

        for candidate in candidates:
            if candidate.exists():
                logger.info("DDS 配置文件: %s", candidate)
                return str(candidate)

        logger.warning(
            "未找到 DDS 配置文件，回退为 domain_id 模式: %s", configured_path
        )
        return None

    def start(self):
        """启动语音控制流程"""
        self._running = True

        logger.info("正在启动各模块...")

        # 1. 等待动作服务上线
        service_timeout = self._config.get("service", {}).get("wait_timeout", 10.0)
        if self._action_executor.wait_for_service(timeout_sec=service_timeout):
            logger.info("[OK] 综合动作服务 /integrated/robot_command 已就绪")
        else:
            logger.warning(
                "[WARN] 综合动作服务 /integrated/robot_command 未就绪 - "
                "将在无动作模式下运行（仅测试语音识别流程）"
            )

        # 2. 预热 ASR，减少首轮请求延迟
        if self._asr_engine.warmup():
            logger.info("[OK] ASR 预热成功")

        # 3. 启动音频采集（注册语音段完成回调）
        try:
            self._audio_capture.start(on_speech_complete=self._on_speech_complete)
            logger.info("[OK] 音频采集已启动")
        except Exception as e:
            logger.error("[FAIL] 音频采集启动失败: %s", e)
            return

        logger.info("")
        logger.info("=" * 60)
        logger.info("  语音控制已就绪 - 请说出指令")
        logger.info("  支持指令: 向前走, 向后退, 向左移动, 向右移动,")
        logger.info("           向左转, 向右转")
        logger.info("  按 Ctrl+C 退出")
        logger.info("=" * 60)
        logger.info("")

        # 主循环 - 保持运行
        try:
            while self._running and rclpy.ok():
                # 使 ROS2 回调能够执行（如服务响应）
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            logger.info("收到退出信号")
        finally:
            self.stop()

    def stop(self):
        """停止语音控制"""
        self._running = False
        logger.info("正在停止语音控制...")

        self._audio_capture.stop()
        self._action_executor.disconnect()

        logger.info("语音控制已停止")

    def _on_speech_complete(self, pcm_data: bytes):
        """
        语音段完成回调 - 在音频采集线程中调用

        启动新线程处理语音段，避免阻塞音频采集。

        Parameters
        ----------
        pcm_data : bytes
            完整的 PCM 音频数据
        """
        # 如果正在处理上一个语音段，跳过
        if self._is_processing:
            self._pending_pcm_data = pcm_data
            logger.warning("上一个语音指令正在处理中，暂存最新语音段")
            return

        # 在新线程中处理
        thread = threading.Thread(
            target=self._process_speech,
            args=(pcm_data,),
            daemon=True,
        )
        thread.start()

    def _process_speech(self, pcm_data: bytes):
        """
        处理完整的语音段

        流程: ASR识别 → 意图解析 → 动作执行(Service)

        Parameters
        ----------
        pcm_data : bytes
            PCM 音频数据
        """
        with self._processing_lock:
            self._is_processing = True

        try:
            start_time = time.time()
            logger.info("-" * 40)
            logger.info(">> 开始处理语音段 (%d bytes)", len(pcm_data))

            # Step 1: ASR 语音识别
            logger.info("[Step 1] ASR 语音识别...")
            text = self._asr_engine.recognize(
                pcm_data,
                sample_rate=self._config.get("audio", {}).get("sample_rate", 24000),
            )

            if text is None:
                logger.warning("[Step 1] ASR 识别失败")
                return

            logger.info("[Step 1] ASR 识别结果: '%s'", text)

            # Step 2: 意图解析
            logger.info("[Step 2] 意图解析...")
            intent = self._intent_parser.parse(text)

            if not intent.matched:
                logger.info("[Step 2] 未匹配到指令")
                return

            logger.info(
                "[Step 2] 匹配成功 - 动作: %s, 参数: %s, 置信度: %.2f",
                intent.action,
                intent.params,
                intent.confidence,
            )

            # Step 3: 通过 ROS2 Service 执行动作
            logger.info("[Step 3] 请求动作服务: %s", intent.action)
            if self._action_executor.is_connected:
                success = self._action_executor.execute(intent.action, intent.params)
                if success:
                    logger.info("[Step 3] 动作执行成功")
                else:
                    logger.warning("[Step 3] 动作执行失败")
            else:
                logger.warning("[Step 3] 动作服务不可用，跳过动作执行")

            elapsed = time.time() - start_time
            logger.info(">> 语音处理完成 (总耗时 %.2fs)", elapsed)
            logger.info("-" * 40)

        except Exception as e:
            logger.error("语音处理异常: %s", e, exc_info=True)

        finally:
            with self._processing_lock:
                self._is_processing = False

            pending_pcm_data = self._pending_pcm_data
            self._pending_pcm_data = None
            if pending_pcm_data:
                logger.info("处理暂存语音段 (%d bytes)", len(pending_pcm_data))
                self._on_speech_complete(pending_pcm_data)


def setup_logging(config: dict):
    """配置日志系统"""
    log_cfg = config.get("logging", {})
    level_name = log_cfg.get("level", "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)

    # 格式
    fmt = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
    datefmt = "%Y-%m-%d %H:%M:%S"

    handlers = [logging.StreamHandler(sys.stdout)]

    # 文件输出
    if log_cfg.get("file_output", False):
        file_path = log_cfg.get("file_path", "voice_control.log")
        handlers.append(logging.FileHandler(file_path, encoding="utf-8"))

    logging.basicConfig(level=level, format=fmt, datefmt=datefmt, handlers=handlers)


def load_config(config_path: str) -> dict:
    """加载 YAML 配置文件"""
    path = Path(config_path)
    if not path.exists():
        logger.error("配置文件不存在: %s", config_path)
        sys.exit(1)

    with open(path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    return config


def main():
    """入口函数"""
    parser = argparse.ArgumentParser(description="Demo 1: 语音交互与智能感知")
    parser.add_argument(
        "--config",
        type=str,
        default="",
        help="配置文件路径 (YAML)",
    )
    args, unknown = parser.parse_known_args()

    # 确定配置文件路径
    config_path = args.config
    if not config_path:
        # 尝试默认路径
        candidates = [
            "config/integrated_demo.yaml",
            os.path.join(
                os.path.dirname(__file__), "..", "..", "config", "integrated_demo.yaml"
            ),
        ]
        # ROS2 安装路径
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = get_package_share_directory("dobot_integrated_demo")
            candidates.insert(
                0, os.path.join(share_dir, "config", "integrated_demo.yaml")
            )
        except Exception:
            pass

        for candidate in candidates:
            if os.path.exists(candidate):
                config_path = candidate
                break

    if not config_path:
        print("错误: 未找到配置文件，请通过 --config 参数指定")
        print(
            "示例: ros2 run dobot_integrated_demo voice_control_node --config config/integrated_demo.yaml"
        )
        sys.exit(1)

    # 加载配置
    config = load_config(config_path)

    # 配置日志
    setup_logging(config)

    logger.info("配置文件: %s", config_path)

    # 初始化 ROS2
    rclpy.init(args=unknown)

    # 创建并启动节点
    node = VoiceControlNode(config)

    # 注册轻量信号处理：只通知主循环退出，销毁和 shutdown 统一在 finally 中完成。
    def signal_handler(sig, frame):
        logger.info("收到信号 %s，正在退出...", sig)
        node.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 启动
    try:
        node.start()
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
