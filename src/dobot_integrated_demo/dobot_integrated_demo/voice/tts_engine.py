#!/usr/bin/env python3
"""
TTS (Text-to-Speech) 语音合成引擎

功能:
  - 提供 TTS 抽象基类，方便后续扩展不同的语音合成服务
  - 实现百度语音合成适配层 (BaiduTTSEngine)
  - 将文本发送到百度 REST API，返回音频数据

百度语音合成 API 文档:
  https://ai.baidu.com/ai-doc/SPEECH/Qk38y8lrl
"""

import io
import json
import time
import wave
import struct
import logging
from abc import ABC, abstractmethod
from typing import Optional

import requests

logger = logging.getLogger(__name__)


class TTSEngine(ABC):
    """TTS 引擎抽象基类"""

    @abstractmethod
    def synthesize(self, text: str) -> Optional[bytes]:
        """
        文本合成语音

        Parameters
        ----------
        text : str
            要合成的文本

        Returns
        -------
        Optional[bytes]
            PCM 音频数据 (24kHz, 16bit, 单声道)，失败返回 None
        """
        pass


class BaiduTTSEngine(TTSEngine):
    """
    百度语音合成引擎

    使用百度 AI 开放平台的在线语音合成 REST API。

    Parameters
    ----------
    app_id : str
        百度 AI 应用的 App ID
    api_key : str
        百度 AI 应用的 API Key
    secret_key : str
        百度 AI 应用的 Secret Key
    per : int
        发音人, 0=女声, 1=男声, 3=度逍遥, 4=度丫丫
    spd : int
        语速, 0-15, 默认 5
    pit : int
        音调, 0-15, 默认 5
    vol : int
        音量, 0-15, 默认 10
    aue : int
        音频格式, 3=mp3, 4=pcm-16k, 5=pcm-8k, 6=wav
    """

    # 百度 token 获取地址
    TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token"
    # 百度语音合成地址
    TTS_URL = "https://tsn.baidu.com/text2audio"

    def __init__(
        self,
        app_id: str,
        api_key: str,
        secret_key: str,
        per: int = 0,
        spd: int = 5,
        pit: int = 5,
        vol: int = 10,
        aue: int = 6,
    ):
        self._app_id = app_id
        self._api_key = api_key
        self._secret_key = secret_key
        self._per = per
        self._spd = spd
        self._pit = pit
        self._vol = vol
        self._aue = aue

        # Token 缓存
        self._access_token: Optional[str] = None
        self._token_expire_time: float = 0

        # 音频缓存 - 相同文本不重复合成
        self._audio_cache: dict[str, bytes] = {}

        logger.info("百度 TTS 引擎初始化完成 (per=%d, spd=%d, vol=%d)", per, spd, vol)

    def _get_access_token(self) -> str:
        """
        获取百度 API 的 access_token (带缓存)

        Returns
        -------
        str
            有效的 access_token

        Raises
        ------
        RuntimeError
            获取 token 失败
        """
        if self._access_token and time.time() < self._token_expire_time - 60:
            return self._access_token

        logger.info("正在获取百度 TTS access_token...")

        try:
            resp = requests.post(
                self.TOKEN_URL,
                data={
                    "grant_type": "client_credentials",
                    "client_id": self._api_key,
                    "client_secret": self._secret_key,
                },
                timeout=10,
            )
            resp.raise_for_status()
            result = resp.json()

            if "access_token" not in result:
                raise RuntimeError(f"获取 token 失败: {result}")

            self._access_token = result["access_token"]
            self._token_expire_time = time.time() + result.get("expires_in", 2592000)

            logger.info("百度 TTS access_token 获取成功")
            return self._access_token

        except requests.RequestException as e:
            raise RuntimeError(f"获取百度 TTS access_token 网络异常: {e}")

    def _resample_16k_to_24k(self, pcm_data: bytes) -> bytes:
        """
        将 16kHz PCM 数据升采样到 24kHz

        机器人扬声器要求 24kHz，百度 TTS 输出为 16kHz。

        Parameters
        ----------
        pcm_data : bytes
            16kHz 16bit 单声道 PCM 数据

        Returns
        -------
        bytes
            24kHz 16bit 单声道 PCM 数据
        """
        num_samples_16k = len(pcm_data) // 2
        if num_samples_16k == 0:
            return b""

        samples_16k = struct.unpack(f"<{num_samples_16k}h", pcm_data[:num_samples_16k * 2])
        num_samples_24k = int(num_samples_16k * 24000 / 16000)

        samples_24k = []
        for i in range(num_samples_24k):
            pos = i * (num_samples_16k - 1) / max(num_samples_24k - 1, 1)
            idx = int(pos)
            frac = pos - idx

            if idx + 1 < num_samples_16k:
                sample = int(samples_16k[idx] * (1 - frac) + samples_16k[idx + 1] * frac)
            else:
                sample = samples_16k[idx]

            sample = max(-32768, min(32767, sample))
            samples_24k.append(sample)

        return struct.pack(f"<{len(samples_24k)}h", *samples_24k)

    def _wav_to_pcm_24k(self, wav_data: bytes) -> bytes:
        """
        将 WAV 格式音频转换为 24kHz PCM

        Parameters
        ----------
        wav_data : bytes
            WAV 格式音频数据

        Returns
        -------
        bytes
            24kHz 16bit 单声道 PCM 数据
        """
        try:
            with wave.open(io.BytesIO(wav_data), "rb") as wf:
                pcm_data = wf.readframes(wf.getnframes())
                src_rate = wf.getframerate()

            if src_rate == 24000:
                return pcm_data
            elif src_rate == 16000:
                return self._resample_16k_to_24k(pcm_data)
            else:
                # 通用重采样
                num_src = len(pcm_data) // 2
                samples_src = struct.unpack(f"<{num_src}h", pcm_data[:num_src * 2])
                num_dst = int(num_src * 24000 / src_rate)

                samples_dst = []
                for i in range(num_dst):
                    pos = i * (num_src - 1) / max(num_dst - 1, 1)
                    idx = int(pos)
                    frac = pos - idx
                    if idx + 1 < num_src:
                        s = int(samples_src[idx] * (1 - frac) + samples_src[idx + 1] * frac)
                    else:
                        s = samples_src[idx]
                    samples_dst.append(max(-32768, min(32767, s)))

                return struct.pack(f"<{len(samples_dst)}h", *samples_dst)

        except Exception as e:
            logger.error("WAV 转 PCM 失败: %s", e)
            return b""

    def synthesize(self, text: str) -> Optional[bytes]:
        """
        调用百度语音合成

        Parameters
        ----------
        text : str
            要合成的文本 (最长 1024 字节 UTF-8)

        Returns
        -------
        Optional[bytes]
            24kHz 16bit 单声道 PCM 音频数据，失败返回 None
        """
        if not text:
            return None

        # 检查缓存
        if text in self._audio_cache:
            logger.debug("TTS 缓存命中: '%s'", text)
            return self._audio_cache[text]

        try:
            token = self._get_access_token()
        except RuntimeError as e:
            logger.error("获取 TTS access_token 失败: %s", e)
            return None

        # 构造请求参数
        params = {
            "tex": text,
            "tok": token,
            "cuid": f"dobot_quad_{self._app_id}",
            "ctp": 1,
            "lan": "zh",
            "per": self._per,
            "spd": self._spd,
            "pit": self._pit,
            "vol": self._vol,
            "aue": self._aue,
        }

        logger.debug("发送 TTS 请求: '%s'", text)

        try:
            resp = requests.post(self.TTS_URL, data=params, timeout=15)

            # 检查是否返回的是音频数据 (Content-Type 不是 json)
            content_type = resp.headers.get("Content-Type", "")
            if "json" in content_type or "text" in content_type:
                # 返回的是错误信息
                try:
                    error = resp.json()
                    logger.error(
                        "TTS 合成失败 - err_no: %s, err_msg: %s",
                        error.get("err_no"),
                        error.get("err_msg"),
                    )
                except json.JSONDecodeError:
                    logger.error("TTS 合成失败: %s", resp.text[:200])
                return None

            # 返回的是音频数据
            audio_raw = resp.content
            logger.info("TTS 合成成功: '%s' (%d bytes)", text, len(audio_raw))

            # 根据请求的格式转换为 24kHz PCM
            if self._aue == 6:
                # WAV 格式 -> 提取 PCM 并重采样到 24kHz
                pcm_24k = self._wav_to_pcm_24k(audio_raw)
            elif self._aue == 4:
                # PCM 16kHz -> 重采样到 24kHz
                pcm_24k = self._resample_16k_to_24k(audio_raw)
            else:
                # MP3 或其他格式，无法直接转换为 PCM streaming
                # 对于 DDS streaming 播放，建议使用 WAV 或 PCM 格式
                logger.warning("不支持的 TTS 音频格式 (aue=%d)，建议使用 aue=6(wav) 或 aue=4(pcm)", self._aue)
                return None

            if pcm_24k:
                # 缓存结果
                self._audio_cache[text] = pcm_24k

            return pcm_24k

        except requests.RequestException as e:
            logger.error("TTS 网络请求异常: %s", e)
            return None
