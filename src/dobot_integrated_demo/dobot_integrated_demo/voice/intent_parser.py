#!/usr/bin/env python3
"""
意图解析模块 - 将 ASR 识别文本映射为动作意图

功能:
  - 基于配置化的指令词表进行匹配
  - 支持同义词匹配
  - 支持模糊匹配（容忍 ASR 小幅误差）
  - 返回匹配到的动作名称、参数和播报文本
"""

import logging
import re
from dataclasses import dataclass, field
from difflib import SequenceMatcher
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class CommandDef:
    """语音指令定义"""

    keyword: str
    synonyms: list[str] = field(default_factory=list)
    action: str = ""
    params: dict = field(default_factory=dict)
    feedback: str = ""


@dataclass
class IntentResult:
    """意图解析结果"""

    matched: bool = False
    action: str = ""
    params: dict = field(default_factory=dict)
    feedback: str = ""
    keyword: str = ""
    confidence: float = 0.0


class IntentParser:
    """
    意图解析器

    将 ASR 识别出的文本与配置的指令词表进行匹配，
    支持精确匹配、同义词匹配和模糊匹配三级策略。

    Parameters
    ----------
    commands : list[dict]
        指令词表配置，每项包含 keyword, synonyms, action, params, feedback
    unknown_feedback : str
        未识别时的反馈文本
    fuzzy_threshold : float
        模糊匹配阈值 (0-1)，默认 0.6
    """

    def __init__(
        self,
        commands: list[dict],
        unknown_feedback: str = "没有听清，请再说一次",
        fuzzy_threshold: float = 0.6,
    ):
        self._commands: list[CommandDef] = []
        self._unknown_feedback = unknown_feedback
        self._fuzzy_threshold = fuzzy_threshold

        # 解析指令配置
        for cmd_dict in commands:
            cmd = CommandDef(
                keyword=cmd_dict.get("keyword", ""),
                synonyms=cmd_dict.get("synonyms", []),
                action=cmd_dict.get("action", ""),
                params=cmd_dict.get("params", {}),
                feedback=cmd_dict.get("feedback", ""),
            )
            self._commands.append(cmd)

        logger.info("意图解析器初始化 - 共 %d 条指令", len(self._commands))
        for cmd in self._commands:
            logger.debug(
                "  指令: '%s' -> %s (同义词: %s)",
                cmd.keyword,
                cmd.action,
                cmd.synonyms,
            )

        # 固定口令场景下，对百度 ASR 的常见误识别做轻量归一化
        self._normalization_rules = {
            "向左传": "向左转",
            "向左站": "向左转",
            "向左钻": "向左转",
            "向右传": "向右转",
            "向右站": "向右转",
            "向右赚": "向右转",
            "向左一动": "向左移动",
            "向右一动": "向右移动",
            "向左移洞": "向左移动",
            "向右移洞": "向右移动",
            "向前周": "向前走",
            "向后推": "向后退",
        }
        self._normalization_rules.update(
            {
                "下午着走": "向前走",
                "下午走": "向前走",
                "乡下走": "向前走",
                "向钱走": "向前走",
                "像前走": "向前走",
                "向后左转": "",
            }
        )

    def _normalize_text(self, text: str) -> str:
        text_normalized = text.strip().lower()
        text_normalized = re.sub(r"[\s，。！？,!?.、；;：:]", "", text_normalized)
        for src, dst in self._normalization_rules.items():
            if src in text_normalized:
                text_normalized = text_normalized.replace(src, dst)
        return text_normalized

    @staticmethod
    def _phrase_matches(text_normalized: str, phrase: str) -> bool:
        phrase = str(phrase).strip().lower()
        if not phrase:
            return False
        if text_normalized == phrase:
            return True
        # Very short Chinese synonyms such as "向后" are useful as exact commands,
        # but substring matching them inside mixed ASR text can trigger the wrong action.
        if len(phrase) <= 2:
            return False
        return phrase in text_normalized

    def parse(self, text: str) -> IntentResult:
        """
        解析文本意图

        匹配策略（优先级从高到低）:
        1. 精确匹配 - 文本包含关键词
        2. 同义词匹配 - 文本包含任一同义词
        3. 模糊匹配 - 文本与关键词/同义词的相似度超过阈值

        Parameters
        ----------
        text : str
            ASR 识别出的文本

        Returns
        -------
        IntentResult
            解析结果
        """
        if not text:
            return IntentResult(
                matched=False,
                feedback=self._unknown_feedback,
            )

        # 归一化文本
        text_normalized = self._normalize_text(text)

        logger.info("开始解析意图: '%s' (归一化: '%s')", text, text_normalized)

        # 第一轮：精确匹配（包含关键词）
        for cmd in self._commands:
            if self._phrase_matches(text_normalized, cmd.keyword):
                logger.info(
                    "精确匹配成功: '%s' -> %s (关键词: '%s')",
                    text,
                    cmd.action,
                    cmd.keyword,
                )
                return IntentResult(
                    matched=True,
                    action=cmd.action,
                    params=dict(cmd.params),
                    feedback=cmd.feedback,
                    keyword=cmd.keyword,
                    confidence=1.0,
                )

        # 第二轮：同义词匹配
        for cmd in self._commands:
            for synonym in cmd.synonyms:
                if self._phrase_matches(text_normalized, synonym):
                    logger.info(
                        "同义词匹配成功: '%s' -> %s (同义词: '%s', 关键词: '%s')",
                        text,
                        cmd.action,
                        synonym,
                        cmd.keyword,
                    )
                    return IntentResult(
                        matched=True,
                        action=cmd.action,
                        params=dict(cmd.params),
                        feedback=cmd.feedback,
                        keyword=cmd.keyword,
                        confidence=0.9,
                    )

        # 第三轮：模糊匹配
        best_match: Optional[CommandDef] = None
        best_score = 0.0
        best_word = ""

        for cmd in self._commands:
            # 与关键词比较
            score = SequenceMatcher(None, text_normalized, cmd.keyword).ratio()
            if score > best_score:
                best_score = score
                best_match = cmd
                best_word = cmd.keyword

            # 与同义词比较
            for synonym in cmd.synonyms:
                score = SequenceMatcher(None, text_normalized, synonym).ratio()
                if score > best_score:
                    best_score = score
                    best_match = cmd
                    best_word = synonym

        length_ok = bool(best_word) and len(text_normalized) <= len(best_word)
        if best_match and best_score >= self._fuzzy_threshold and length_ok:
            logger.info(
                "模糊匹配成功: '%s' -> %s (匹配词: '%s', 相似度: %.2f)",
                text,
                best_match.action,
                best_word,
                best_score,
            )
            return IntentResult(
                matched=True,
                action=best_match.action,
                params=dict(best_match.params),
                feedback=best_match.feedback,
                keyword=best_match.keyword,
                confidence=best_score,
            )

        # 未匹配
        logger.info(
            "未匹配到指令: '%s' (最佳候选: '%s', 相似度: %.2f)",
            text,
            best_word,
            best_score,
        )
        return IntentResult(
            matched=False,
            feedback=self._unknown_feedback,
        )
