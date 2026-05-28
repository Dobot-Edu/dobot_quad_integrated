#!/usr/bin/env python3
"""
ASR (Automatic Speech Recognition) 语音识别引擎

功能:
  - 提供 ASR 抽象基类，方便后续扩展不同的语音识别服务
  - 实现百度语音识别适配层 (BaiduASREngine)
  - 将 PCM 音频数据发送到百度 REST API，返回识别文本

百度语音识别 API 文档:
  https://ai.baidu.com/ai-doc/SPEECH/Vk38lxily
"""

import json
import time
import base64
import logging
import struct
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import requests

logger = logging.getLogger(__name__)


class ASREngine(ABC):
    """ASR 引擎抽象基类"""

    @abstractmethod
    def recognize(self, audio_data: bytes, sample_rate: int = 16000) -> Optional[str]:
        """
        语音识别

        Parameters
        ----------
        audio_data : bytes
            PCM 音频数据（16bit, 单声道）
        sample_rate : int
            采样率

        Returns
        -------
        Optional[str]
            识别出的文本，识别失败返回 None
        """
        pass


class BaiduASREngine(ASREngine):
    """
    百度语音识别引擎

    使用百度 AI 开放平台的短语音识别 REST API。

    Parameters
    ----------
    app_id : str
        百度 AI 应用的 App ID
    api_key : str
        百度 AI 应用的 API Key
    secret_key : str
        百度 AI 应用的 Secret Key
    dev_pid : int
        语言模型, 1537=普通话(输入法模型), 1936=普通话(远场)
    """

    # 百度 token 获取地址
    TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token"
    # 百度短语音识别地址
    ASR_URL = "https://vop.baidu.com/server_api"
    FALLBACK_DEV_PID = 1537

    def __init__(
        self,
        app_id: str,
        api_key: str,
        secret_key: str,
        dev_pid: int = 1537,
        connect_timeout: float = 3.0,
        read_timeout: float = 8.0,
        retry_count: int = 2,
        fallback_dev_pids: Optional[list[int]] = None,
    ):
        self._app_id = app_id
        self._api_key = api_key
        self._secret_key = secret_key
        self._dev_pid = dev_pid
        self._connect_timeout = connect_timeout
        self._read_timeout = read_timeout
        self._retry_count = max(1, retry_count)
        self._fallback_dev_pids = [1936] if fallback_dev_pids is None else list(fallback_dev_pids)

        # Token 缓存
        self._access_token: Optional[str] = None
        self._token_expire_time: float = 0

        logger.info("百度 ASR 引擎初始化完成 (dev_pid=%d)", dev_pid)

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
        # 检查缓存是否有效 (提前 60s 刷新)
        if self._access_token and time.time() < self._token_expire_time - 60:
            return self._access_token

        logger.info("正在获取百度 API access_token...")

        try:
            resp = requests.post(
                self.TOKEN_URL,
                data={
                    "grant_type": "client_credentials",
                    "client_id": self._api_key,
                    "client_secret": self._secret_key,
                },
                timeout=(self._connect_timeout, self._read_timeout),
            )
            resp.raise_for_status()
            result = resp.json()

            if "access_token" not in result:
                raise RuntimeError(f"获取 token 失败: {result}")

            self._access_token = result["access_token"]
            # expires_in 通常为 2592000 (30天)
            self._token_expire_time = time.time() + result.get("expires_in", 2592000)

            logger.info("百度 API access_token 获取成功")
            return self._access_token

        except requests.RequestException as e:
            raise RuntimeError(f"获取百度 access_token 网络异常: {e}")

    def warmup(self) -> bool:
        """预热 access_token，减少首轮识别延迟。"""
        try:
            self._get_access_token()
            return True
        except RuntimeError as e:
            logger.warning("ASR 预热失败: %s", e)
            return False

    def _resample_24k_to_16k(self, pcm_data: bytes) -> bytes:
        """
        将 24kHz PCM 数据降采样到 16kHz

        使用简单的线性插值重采样。对于语音识别场景精度足够。

        Parameters
        ----------
        pcm_data : bytes
            24kHz 16bit 单声道 PCM 数据

        Returns
        -------
        bytes
            16kHz 16bit 单声道 PCM 数据
        """
        # 解析 24kHz 样本
        num_samples_24k = len(pcm_data) // 2
        if num_samples_24k == 0:
            return b""

        samples_24k = struct.unpack(
            f"<{num_samples_24k}h", pcm_data[: num_samples_24k * 2]
        )

        # 计算 16kHz 样本数
        num_samples_16k = int(num_samples_24k * 16000 / 24000)

        # 线性插值重采样
        samples_16k = []
        for i in range(num_samples_16k):
            # 在 24kHz 样本中的浮点位置
            pos = i * (num_samples_24k - 1) / max(num_samples_16k - 1, 1)
            idx = int(pos)
            frac = pos - idx

            if idx + 1 < num_samples_24k:
                sample = int(
                    samples_24k[idx] * (1 - frac) + samples_24k[idx + 1] * frac
                )
            else:
                sample = samples_24k[idx]

            # 限幅
            sample = max(-32768, min(32767, sample))
            samples_16k.append(sample)

        return struct.pack(f"<{len(samples_16k)}h", *samples_16k)

    def recognize(self, audio_data: bytes, sample_rate: int = 24000) -> Optional[str]:
        """
        调用百度语音识别

        Parameters
        ----------
        audio_data : bytes
            PCM 音频数据 (16bit, 单声道)
        sample_rate : int
            输入音频采样率, 默认 24000 (机器人麦克风采样率)

        Returns
        -------
        Optional[str]
            识别出的文本，失败返回 None
        """
        if not audio_data:
            logger.warning("音频数据为空，跳过识别")
            return None

        try:
            token = self._get_access_token()
        except RuntimeError as e:
            logger.error("获取 access_token 失败: %s", e)
            return None

        # 百度 ASR 支持 16kHz，需要从 24kHz 降采样
        if sample_rate == 24000:
            audio_data = self._resample_24k_to_16k(audio_data)
            sample_rate = 16000

        # Base64 编码音频数据
        audio_base64 = base64.b64encode(audio_data).decode("utf-8")

        dev_pids = [self._dev_pid]
        for fallback_dev_pid in self._fallback_dev_pids:
            if fallback_dev_pid not in dev_pids:
                dev_pids.append(fallback_dev_pid)
        if self.FALLBACK_DEV_PID not in dev_pids:
            dev_pids.append(self.FALLBACK_DEV_PID)

        for dev_pid in dev_pids:
            payload = {
                "format": "pcm",
                "rate": sample_rate,
                "channel": 1,
                "cuid": f"dobot_quad_{self._app_id}",
                "token": token,
                "dev_pid": dev_pid,
                "speech": audio_base64,
                "len": len(audio_data),
            }

            for attempt_index in range(1, self._retry_count + 1):
                logger.info(
                    "发送 ASR 请求 - attempt=%d/%d, dev_pid=%d, rate=%d, len=%d",
                    attempt_index,
                    self._retry_count,
                    dev_pid,
                    sample_rate,
                    len(audio_data),
                )

                try:
                    resp = requests.post(
                        self.ASR_URL,
                        json=payload,
                        headers={"Content-Type": "application/json"},
                        timeout=(self._connect_timeout, self._read_timeout),
                    )
                    resp.raise_for_status()
                    result = resp.json()

                    if result.get("err_no") == 0:
                        text = result["result"][0] if result.get("result") else ""
                        text = text.rstrip("，。！？,!?.")
                        if not text.strip():
                            logger.warning("ASR 返回空文本")
                            return None
                        logger.info("ASR 识别成功: '%s'", text)
                        return text

                    err_no = result.get("err_no")
                    err_msg = result.get("err_msg")
                    logger.warning(
                        "ASR 识别失败 - attempt=%d/%d, dev_pid=%d, err_no=%s, err_msg=%s",
                        attempt_index,
                        self._retry_count,
                        dev_pid,
                        err_no,
                        err_msg,
                    )

                    if err_no == 3302:
                        logger.warning("ASR 参数可能不兼容，切换模型继续重试")
                        break

                    if attempt_index < self._retry_count:
                        time.sleep(0.3)
                        continue

                except requests.RequestException as e:
                    logger.error("ASR 网络请求异常: %s", e)
                    if attempt_index < self._retry_count:
                        time.sleep(0.5)
                        continue
                except (json.JSONDecodeError, KeyError, IndexError) as e:
                    logger.error("ASR 响应解析异常: %s", e)
                    break

        return None


class VoskGrammarASREngine(ASREngine):
    """Offline grammar-constrained ASR for fixed classroom commands."""

    def __init__(
        self,
        model_path: str,
        grammar_phrases: list[str],
        sample_rate: int = 16000,
        fallback_engine: Optional[ASREngine] = None,
        use_grammar: bool = False,
    ):
        self._model_path = Path(model_path).expanduser()
        self._sample_rate = int(sample_rate)
        self._fallback_engine = fallback_engine
        self._grammar_phrases = self._normalize_grammar(grammar_phrases)
        self._use_grammar = bool(use_grammar)
        self._model = None
        self._available = False
        self._init_model()

    def _init_model(self):
        if not self._model_path.exists():
            logger.warning("Vosk 模型目录不存在: %s", self._model_path)
            return
        try:
            from vosk import Model, SetLogLevel

            SetLogLevel(-1)
            self._model = Model(str(self._model_path))
            self._available = True
            logger.info(
                "Vosk 离线命令识别初始化完成: model=%s, grammar=%s, phrases=%d",
                self._model_path,
                self._use_grammar,
                len(self._grammar_phrases),
            )
        except Exception as exc:
            self._available = False
            logger.warning("Vosk 初始化失败，将使用备用 ASR: %s", exc)

    @staticmethod
    def _normalize_grammar(phrases: list[str]) -> list[str]:
        result = []
        seen = set()
        for phrase in phrases:
            text = str(phrase).strip()
            if not text or text in seen:
                continue
            seen.add(text)
            result.append(text)
        return result

    def warmup(self) -> bool:
        return self._available or (self._fallback_engine.warmup() if hasattr(self._fallback_engine, "warmup") else False)

    def recognize(self, audio_data: bytes, sample_rate: int = 24000) -> Optional[str]:
        if self._available:
            text = self._recognize_vosk(audio_data, sample_rate)
            if text:
                return text
            logger.warning("Vosk 未识别到命令")
            return None

        if self._fallback_engine is not None:
            return self._fallback_engine.recognize(audio_data, sample_rate)
        return None

    def _recognize_vosk(self, audio_data: bytes, sample_rate: int) -> Optional[str]:
        from vosk import KaldiRecognizer

        if sample_rate != self._sample_rate:
            audio_data = resample_pcm_16bit(audio_data, sample_rate, self._sample_rate)

        if self._use_grammar and self._grammar_phrases:
            grammar_json = json.dumps(self._grammar_phrases, ensure_ascii=False)
            recognizer = KaldiRecognizer(self._model, self._sample_rate, grammar_json)
        else:
            recognizer = KaldiRecognizer(self._model, self._sample_rate)
        recognizer.SetWords(False)
        recognizer.AcceptWaveform(audio_data)
        result = json.loads(recognizer.FinalResult() or "{}")
        text = str(result.get("text", "")).strip()
        text = normalize_asr_text(text)
        if text:
            logger.info("Vosk 命令识别成功: '%s'", text)
            return text
        return None


def normalize_asr_text(text: str) -> str:
    return "".join(ch for ch in text.strip() if not ch.isspace()).rstrip("，。！？,!?.")


def resample_pcm_16bit(pcm_data: bytes, src_rate: int, dst_rate: int) -> bytes:
    if not pcm_data or src_rate == dst_rate:
        return pcm_data
    num_src = len(pcm_data) // 2
    if num_src <= 0:
        return b""
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
            sample = int(samples_src[idx] * (1 - frac) + samples_src[idx + 1] * frac)
        else:
            sample = samples_src[idx]
        samples_dst.append(max(-32768, min(32767, sample)))
    return struct.pack(f"<{len(samples_dst)}h", *samples_dst)
