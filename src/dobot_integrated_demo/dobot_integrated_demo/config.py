#!/usr/bin/env python3

from pathlib import Path
from typing import Any
import os

import yaml


def load_yaml(path: str) -> dict[str, Any]:
    config_path = Path(path).expanduser()
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with config_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def resolve_path(path_value: str | None, config_file: str | None = None) -> str | None:
    if not path_value:
        return None

    path = Path(path_value).expanduser()
    if path.is_absolute() and path.exists():
        return str(path)

    candidates: list[Path] = []
    if config_file:
        cfg_path = Path(config_file).expanduser()
        if cfg_path.exists():
            candidates.append((cfg_path.parent / path).resolve())

    cwd = Path.cwd()
    candidates.extend(
        [
            (cwd / path).resolve(),
            (cwd / "src" / path).resolve(),
            (cwd / ".." / path).resolve(),
            (cwd / "dobot_quad_sdk-main" / "low_level" / "python" / "config" / "dds_config.yaml").resolve(),
            (cwd / ".." / "dobot_quad_sdk-main" / "low_level" / "python" / "config" / "dds_config.yaml").resolve(),
        ]
    )

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return None


def normalize_file_uri(uri: str) -> Path | None:
    if not uri.startswith("file://"):
        return None
    return Path(uri[len("file://") :]).expanduser()


def ensure_valid_cyclonedds_uri(config_file: str | None = None) -> str | None:
    current = os.environ.get("CYCLONEDDS_URI", "").strip()
    current_path = normalize_file_uri(current) if current else None
    if current_path and current_path.exists():
        return current

    cwd = Path.cwd()
    candidates: list[Path] = [
        (cwd / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
        (cwd / ".." / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
        (cwd / "src" / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
    ]
    if config_file:
        cfg = Path(config_file).expanduser()
        if cfg.exists():
            candidates.extend(
                [
                    (cfg.parent / ".." / ".." / ".." / ".." / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
                    (cfg.parent / ".." / ".." / ".." / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
                ]
            )

    for candidate in candidates:
        if candidate.exists():
            uri = "file://" + str(candidate)
            os.environ["CYCLONEDDS_URI"] = uri
            return uri

    if current and current_path and not current_path.exists():
        os.environ.pop("CYCLONEDDS_URI", None)
        return None
    return current or None
