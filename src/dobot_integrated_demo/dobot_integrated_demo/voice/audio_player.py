#!/usr/bin/env python3
"""
音频播放模块 - 通过 DDS 向机器人扬声器发送音频

功能:
  - 通过 DDS rt/voice/cmd 话题向机器人发送音频数据
  - 支持 streaming 模式（实时推送 PCM 数据块）
  - 支持 file 模式（播放机器人主机上的音频文件）
"""

import io
import time
import wave
import struct
import logging
import uuid
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# 每次推送的 PCM 数据块大小 (100ms of 24kHz/16bit/mono = 4800 bytes)
CHUNK_SIZE = 4800


class AudioPlayer:
    """
    DDS 音频播放器

    通过 DDS VoiceCmd 话题向机器人扬声器发送音频数据。

    Parameters
    ----------
    dds_domain_id : int
        DDS domain ID, 默认 0
    dds_config : str, optional
        DDS 配置文件路径
    """

    def __init__(
        self,
        dds_domain_id: int = 0,
        dds_config: Optional[str] = None,
        topic_name: str = "rt/voice/cmd",
        action_topic_name: Optional[str] = None,
    ):
        self._dds_domain_id = dds_domain_id
        self._dds_config = dds_config
        self._topic_name = topic_name
        self._action_topic_name = action_topic_name
        self._middleware = None
        self._dds = None
        self._initialized = False

    def init(self):
        """初始化 DDS 中间件和 VoiceCmd 发布者"""
        import dds_middleware_python as dds

        self._dds = dds
        self._ensure_voice_cmd_supported()

        if self._dds_config:
            self._middleware = dds.PyDDSMiddleware(self._dds_config)
        else:
            self._middleware = dds.PyDDSMiddleware(self._dds_domain_id)

        # QoS 配置 - 与 SDK e7_voice_pub.py 一致
        qos_config = {
            "reliability": "reliable",
            "history_kind": "keep_last",
            "history_depth": 5,
            "durability": "volatile",
        }

        self._middleware.createVoiceCmdWriter(self._topic_name, qos_config)

        # DDS 发现延迟 - Writer 创建后需要等待 Reader 发现
        time.sleep(1)
        self._initialized = True
        logger.info("音频播放器初始化完成: voice_cmd_topic=%s", self._topic_name)

    def play_streaming(self, pcm_data: bytes, chunk_size: int = CHUNK_SIZE):
        """
        以 streaming 模式播放 PCM 音频

        将 PCM 数据分块发送到机器人扬声器。

        Parameters
        ----------
        pcm_data : bytes
            24kHz, 16bit, 单声道 PCM 音频数据
        chunk_size : int
            每次发送的字节数，默认 4800 (100ms)
        """
        if not self._initialized:
            logger.error("播放器未初始化，请先调用 init()")
            return

        if not pcm_data:
            logger.warning("播放数据为空")
            return

        total_size = len(pcm_data)
        task_id = uuid.uuid4().hex
        offset = 0
        chunk_count = 0

        logger.info("开始 VoiceCmd streaming 播放 - task_id=%s 总大小: %d bytes", task_id, total_size)

        while offset < total_size:
            chunk = pcm_data[offset : offset + chunk_size]

            voice_cmd = self._dds.VoiceCmd()
            voice_cmd.header(self._make_header())
            voice_cmd.priority(self._dds.VoicePriority.kNormal)
            voice_cmd.task_id(task_id)
            voice_cmd.type("streaming")
            voice_cmd.path("")
            voice_cmd.flag(False)
            voice_cmd.data(list(chunk))

            self._middleware.publishVoiceCmd(voice_cmd)

            offset += chunk_size
            chunk_count += 1

            # 按照实际时间间隔发送，避免数据堆积
            # 4800 bytes = 100ms at 24kHz/16bit/mono
            time.sleep(chunk_size / (24000 * 2) * 0.9)  # 略快于实时

        voice_cmd = self._dds.VoiceCmd()
        voice_cmd.header(self._make_header())
        voice_cmd.priority(self._dds.VoicePriority.kNormal)
        voice_cmd.task_id(task_id)
        voice_cmd.type("streaming")
        voice_cmd.path("")
        voice_cmd.flag(True)
        voice_cmd.data([])
        self._middleware.publishVoiceCmd(voice_cmd)
        logger.info("VoiceCmd streaming 播放完成 - task_id=%s 共发送 %d 个数据块", task_id, chunk_count)

    def play_local_wav(self, file_path: str, chunk_size: int = CHUNK_SIZE):
        """
        播放开发机本地 WAV 文件。

        会将 WAV 转为机器人扬声器要求的 24kHz/16bit/单声道 PCM，
        然后通过 streaming 模式发送。
        """
        path = Path(file_path)
        if not path.exists():
            logger.error("本地音频文件不存在: %s", file_path)
            return

        try:
            pcm_data = self._wav_file_to_pcm_24k(path)
        except Exception as e:
            logger.error("本地音频文件解析失败: %s - %s", file_path, e)
            return

        if not pcm_data:
            logger.warning("本地音频文件无可播放数据: %s", file_path)
            return

        logger.info("开始播放本地反馈音频: %s", file_path)
        self.play_streaming(pcm_data, chunk_size=chunk_size)

    def play_file(self, file_path: str):
        """
        播放机器人主机上的音频文件

        Parameters
        ----------
        file_path : str
            机器人主机上的音频文件绝对路径
        """
        if not self._initialized:
            logger.error("播放器未初始化，请先调用 init()")
            return

        voice_cmd = self._dds.VoiceCmd()
        voice_cmd.header(self._make_header())
        voice_cmd.priority(self._dds.VoicePriority.kNormal)
        voice_cmd.task_id(uuid.uuid4().hex)
        voice_cmd.type("file")
        voice_cmd.path(file_path)
        voice_cmd.data([])
        voice_cmd.flag(False)

        self._middleware.publishVoiceCmd(voice_cmd)
        logger.info("已发送文件播放命令: %s", file_path)

    def _make_header(self):
        header = self._dds.Header()
        stamp = self._dds.Time()
        now = time.time()
        stamp.sec(int(now))
        stamp.nanosec(int((now - int(now)) * 1e9))
        header.stamp(stamp)
        header.frame_id("voice_cmd")
        return header

    def _ensure_voice_cmd_supported(self):
        voice_cmd = self._dds.VoiceCmd()
        missing = [
            name
            for name in ("header", "priority", "task_id", "type", "path", "data", "flag")
            if not hasattr(voice_cmd, name)
        ]
        if missing:
            raise RuntimeError(
                "dds_middleware_python.VoiceCmd 缺少字段: %s。请安装 v1.2 SDK 的 0.23.x wheel。"
                % ", ".join(missing)
            )

    def _wav_file_to_pcm_24k(self, file_path: Path) -> bytes:
        with file_path.open("rb") as f:
            wav_bytes = f.read()
        return self._wav_bytes_to_pcm_24k(wav_bytes)

    def _wav_bytes_to_pcm_24k(self, wav_data: bytes) -> bytes:
        with wave.open(io.BytesIO(wav_data), "rb") as wf:
            channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            src_rate = wf.getframerate()
            pcm_data = wf.readframes(wf.getnframes())

        if sample_width != 2:
            raise ValueError(f"仅支持 16bit WAV，当前为 {sample_width * 8}bit")

        pcm_mono = self._to_mono_pcm_16bit(pcm_data, channels)
        if src_rate == 24000:
            return pcm_mono
        return self._resample_pcm_16bit(pcm_mono, src_rate, 24000)

    def _to_mono_pcm_16bit(self, pcm_data: bytes, channels: int) -> bytes:
        if channels == 1:
            return pcm_data

        if channels <= 0:
            raise ValueError(f"非法声道数: {channels}")

        num_samples = len(pcm_data) // 2
        samples = struct.unpack(f"<{num_samples}h", pcm_data[: num_samples * 2])

        mono_samples = []
        for index in range(0, len(samples), channels):
            frame = samples[index : index + channels]
            if not frame:
                continue
            mono_samples.append(int(sum(frame) / len(frame)))

        return struct.pack(f"<{len(mono_samples)}h", *mono_samples)

    def _resample_pcm_16bit(
        self, pcm_data: bytes, src_rate: int, dst_rate: int
    ) -> bytes:
        if src_rate <= 0 or dst_rate <= 0:
            raise ValueError(f"非法采样率: {src_rate} -> {dst_rate}")

        num_src = len(pcm_data) // 2
        if num_src == 0 or src_rate == dst_rate:
            return pcm_data

        samples_src = struct.unpack(f"<{num_src}h", pcm_data[: num_src * 2])
        num_dst = int(num_src * dst_rate / src_rate)
        if num_dst <= 0:
            return b""

        samples_dst = []
        for i in range(num_dst):
            pos = i * (num_src - 1) / max(num_dst - 1, 1)
            idx = int(pos)
            frac = pos - idx

            if idx + 1 < num_src:
                sample = int(
                    samples_src[idx] * (1 - frac) + samples_src[idx + 1] * frac
                )
            else:
                sample = samples_src[idx]

            samples_dst.append(max(-32768, min(32767, sample)))

        return struct.pack(f"<{len(samples_dst)}h", *samples_dst)
