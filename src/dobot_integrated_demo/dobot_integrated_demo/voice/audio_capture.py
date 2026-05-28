#!/usr/bin/env python3
"""
音频采集模块 - 通过 DDS 从机器人麦克风采集音频

功能:
  - 订阅 DDS rt/voice/state 话题获取实时音频流
  - 内置 VAD (Voice Activity Detection) 语音活动检测
  - 基于能量阈值的静音检测，自动切分语音段
  - 积累完整语音段后通过回调输出 PCM 数据
"""

import time
import struct
import logging
import threading
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class AudioCapture:
    """
    DDS 音频采集器

    通过订阅机器人的 rt/voice/state 话题持续采集音频，
    使用 VAD 检测语音段的起止，将完整语音段通过回调返回。

    Parameters
    ----------
    dds_domain_id : int
        DDS domain ID, 默认 0
    dds_config : str, optional
        DDS 配置文件路径, 为 None 时使用 domain_id 初始化
    sample_rate : int
        采样率，默认 24000
    channels : int
        声道数，默认 1
    sample_width : int
        采样宽度（字节），默认 2 (16bit)
    energy_threshold : int
        VAD 能量阈值，默认 500
    silence_duration : float
        静音判定时间（秒），默认 1.5
    max_record_duration : float
        最长录音时间（秒），默认 8.0
    min_speech_duration : float
        最短有效语音时间（秒），默认 0.3
    adaptive_noise : bool
        是否根据环境噪声动态调整 VAD 阈值
    """

    def __init__(
        self,
        dds_domain_id: int = 0,
        dds_config: Optional[str] = None,
        sample_rate: int = 24000,
        channels: int = 1,
        sample_width: int = 2,
        energy_threshold: int = 500,
        silence_duration: float = 1.5,
        max_record_duration: float = 8.0,
        min_speech_duration: float = 0.3,
        adaptive_noise: bool = True,
        start_threshold_ratio: float = 3.0,
        stop_threshold_ratio: float = 1.8,
        noise_alpha: float = 0.03,
        pre_speech_padding_seconds: float = 0.25,
    ):
        self.sample_rate = sample_rate
        self.channels = channels
        self.sample_width = sample_width
        self.energy_threshold = energy_threshold
        self.silence_duration = silence_duration
        self.max_record_duration = max_record_duration
        self.min_speech_duration = min_speech_duration
        self.adaptive_noise = adaptive_noise
        self.start_threshold_ratio = max(1.2, float(start_threshold_ratio))
        self.stop_threshold_ratio = max(1.0, float(stop_threshold_ratio))
        self.noise_alpha = min(0.5, max(0.001, float(noise_alpha)))
        self.pre_speech_padding_seconds = max(0.0, float(pre_speech_padding_seconds))

        # VAD 状态
        self._is_speaking = False
        self._speech_buffer = bytearray()
        self._silence_start_time: Optional[float] = None
        self._speech_start_time: Optional[float] = None
        self._noise_floor = float(energy_threshold)
        self._recent_silence = bytearray()
        self._recent_silence_max_bytes = max(
            0,
            int(self.sample_rate * self.sample_width * self.channels * self.pre_speech_padding_seconds),
        )

        # 线程安全
        self._lock = threading.Lock()
        self._running = False

        # 语音段完成回调
        self._on_speech_complete: Optional[Callable[[bytes], None]] = None

        # DDS 中间件
        self._middleware = None
        self._dds_config = dds_config
        self._dds_domain_id = dds_domain_id

    def start(self, on_speech_complete: Callable[[bytes], None]):
        """
        启动音频采集

        Parameters
        ----------
        on_speech_complete : Callable[[bytes], None]
            语音段完成时的回调函数，参数为完整的 PCM 音频数据
        """
        import dds_middleware_python as dds

        self._on_speech_complete = on_speech_complete
        self._running = True

        # 初始化 DDS
        if self._dds_config:
            self._middleware = dds.PyDDSMiddleware(self._dds_config)
        else:
            self._middleware = dds.PyDDSMiddleware(self._dds_domain_id)

        # QoS 配置 - 与 SDK e8_voice_sub.py 一致
        qos_config = {
            "reliability": "best_effort",
            "history_kind": "keep_last",
            "history_depth": 1,
            "durability": "volatile",
        }

        # 订阅语音状态话题
        self._middleware.subscribeVoiceState(
            "rt/voice/state", self._voice_state_callback, qos_config
        )

        logger.info(
            "音频采集已启动 - 采样率: %dHz, 基础能量阈值: %d, 静音判定: %.1fs, 自适应噪声: %s",
            self.sample_rate,
            self.energy_threshold,
            self.silence_duration,
            self.adaptive_noise,
        )

    def stop(self):
        """停止音频采集"""
        self._running = False
        logger.info("音频采集已停止")

    def _voice_state_callback(self, voice_state_msg):
        """
        DDS VoiceState 消息回调

        每次收到音频数据块后，计算能量值并进行 VAD 判断。
        """
        if not self._running:
            return

        try:
            # 获取 PCM 音频数据
            raw_data = bytes(voice_state_msg.data_())
            if not raw_data:
                return

            # 计算当前数据块的能量值 (RMS)
            energy = self._compute_energy(raw_data)

            with self._lock:
                self._process_vad(raw_data, energy)

        except Exception as e:
            logger.error("音频处理异常: %s", e)

    def _compute_energy(self, pcm_data: bytes) -> float:
        """
        计算 PCM 音频数据的 RMS 能量值

        Parameters
        ----------
        pcm_data : bytes
            16bit PCM 音频数据

        Returns
        -------
        float
            RMS 能量值
        """
        if len(pcm_data) < 2:
            return 0.0

        # 将 PCM bytes 解析为 16bit signed int
        num_samples = len(pcm_data) // 2
        try:
            samples = struct.unpack(f"<{num_samples}h", pcm_data[:num_samples * 2])
        except struct.error:
            return 0.0

        if not samples:
            return 0.0

        # 计算 RMS
        sum_sq = sum(s * s for s in samples)
        rms = (sum_sq / len(samples)) ** 0.5
        return rms

    def _process_vad(self, pcm_data: bytes, energy: float):
        """
        VAD 处理逻辑

        状态机:
          - 未说话 + 能量高 → 开始录音
          - 说话中 + 能量高 → 继续录音，重置静音计时
          - 说话中 + 能量低 → 开始静音计时
          - 说话中 + 静音超时 → 结束录音，输出语音段
          - 说话中 + 超过最大时长 → 强制结束
        """
        now = time.time()
        start_threshold = self._start_threshold()
        stop_threshold = self._stop_threshold()

        if not self._is_speaking:
            self._update_noise_floor(energy)
            self._append_recent_silence(pcm_data)
            # 当前未说话，检测是否开始说话
            if energy > start_threshold:
                self._is_speaking = True
                self._speech_buffer = bytearray(self._recent_silence)
                self._speech_buffer.extend(pcm_data)
                self._speech_start_time = now
                self._silence_start_time = None
                logger.debug(
                    "检测到语音开始 (能量: %.1f, start阈值: %.1f, noise: %.1f)",
                    energy,
                    start_threshold,
                    self._noise_floor,
                )
        else:
            # 当前正在说话，追加数据
            self._speech_buffer.extend(pcm_data)

            if energy > stop_threshold:
                # 仍有声音，重置静音计时
                self._silence_start_time = None
            else:
                # 静音，开始或继续计时
                if self._silence_start_time is None:
                    self._silence_start_time = now

                # 检查静音是否超过阈值
                silence_elapsed = now - self._silence_start_time
                if silence_elapsed >= self.silence_duration:
                    self._finish_speech("静音超时")
                    return

            # 检查是否超过最大录音时长
            if self._speech_start_time:
                speech_elapsed = now - self._speech_start_time
                if speech_elapsed >= self.max_record_duration:
                    self._finish_speech("达到最大录音时长")
                    return

    def _start_threshold(self) -> float:
        if not self.adaptive_noise:
            return float(self.energy_threshold)
        return max(float(self.energy_threshold), self._noise_floor * self.start_threshold_ratio)

    def _stop_threshold(self) -> float:
        if not self.adaptive_noise:
            return float(self.energy_threshold)
        return max(float(self.energy_threshold) * 0.75, self._noise_floor * self.stop_threshold_ratio)

    def _update_noise_floor(self, energy: float):
        if not self.adaptive_noise:
            return
        # 只在未说话状态估计环境噪声，并限制突发噪声对噪声底的拉升速度。
        capped_energy = min(float(energy), self._noise_floor * 3.0)
        self._noise_floor = (1.0 - self.noise_alpha) * self._noise_floor + self.noise_alpha * capped_energy

    def _append_recent_silence(self, pcm_data: bytes):
        if self._recent_silence_max_bytes <= 0:
            return
        self._recent_silence.extend(pcm_data)
        if len(self._recent_silence) > self._recent_silence_max_bytes:
            del self._recent_silence[: len(self._recent_silence) - self._recent_silence_max_bytes]

    def _finish_speech(self, reason: str):
        """
        结束语音段录制，触发回调

        Parameters
        ----------
        reason : str
            结束原因（用于日志）
        """
        speech_data = bytes(self._speech_buffer)

        # 计算语音时长
        duration = len(speech_data) / (self.sample_rate * self.sample_width * self.channels)

        # 重置状态
        self._is_speaking = False
        self._speech_buffer = bytearray()
        self._silence_start_time = None
        self._speech_start_time = None

        # 检查最短时长
        if duration < self.min_speech_duration:
            logger.debug("语音段过短 (%.2fs), 已忽略", duration)
            return

        logger.info(
            "语音段完成 - 时长: %.2fs, 大小: %d bytes, 原因: %s",
            duration,
            len(speech_data),
            reason,
        )

        # 触发回调
        if self._on_speech_complete:
            try:
                self._on_speech_complete(speech_data)
            except Exception as e:
                logger.error("语音段回调异常: %s", e)

    @property
    def is_listening(self) -> bool:
        """是否正在采集"""
        return self._running

    @property
    def is_speaking(self) -> bool:
        """是否检测到正在说话"""
        with self._lock:
            return self._is_speaking
