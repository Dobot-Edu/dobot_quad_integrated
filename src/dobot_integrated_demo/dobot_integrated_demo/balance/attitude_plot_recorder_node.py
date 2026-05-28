#!/usr/bin/env python3

import argparse
import csv
import html
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from dobot_integrated_demo.config import load_yaml


class AttitudePlotRecorderNode(Node):
    """Record attitude samples and export CSV + self-contained HTML plot on shutdown."""

    COLUMNS = [
        "time_s",
        "raw_roll_deg",
        "raw_pitch_deg",
        "filtered_roll_deg",
        "filtered_pitch_deg",
        "command_roll_deg",
        "command_pitch_deg",
    ]

    def __init__(self, config_file: str):
        super().__init__("dobot_attitude_plot_recorder_node")
        config = load_yaml(config_file)
        plot_cfg = config.get("plot", {})
        configured_output = plot_cfg.get("output_dir", "balance_outputs")
        self._output_dir = self._resolve_output_dir(configured_output)
        self._max_samples = max(100, int(plot_cfg.get("max_samples", 5000)))
        self._samples: deque[list[float]] = deque(maxlen=self._max_samples)
        self._start_time: float | None = None
        self._written = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Float32MultiArray,
            "/balance_control/attitude",
            self._attitude_cb,
            sensor_qos,
        )
        self.get_logger().info(
            "曲线记录已启动：退出 launch 时生成 CSV 和 HTML，输出目录: %s" % self._output_dir
        )

    def _attitude_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            return
        now = time.time()
        if self._start_time is None:
            self._start_time = now
        values = [float(v) for v in msg.data[:6]]
        self._samples.append([now - self._start_time] + values)

    def destroy_node(self):
        self._write_outputs()
        super().destroy_node()

    def _write_outputs(self):
        if self._written:
            return
        self._written = True
        if not self._samples:
            self.get_logger().warning("未收到姿态数据，未生成曲线文件。")
            return

        if self._output_dir.exists() and not self._output_dir.is_dir():
            fallback = Path.cwd() / "balance_outputs"
            self.get_logger().warning(
                "配置的输出路径不是目录: %s，回退到: %s" % (self._output_dir, fallback)
            )
            self._output_dir = fallback.resolve()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = self._output_dir / f"balance_attitude_{stamp}.csv"
        html_path = self._output_dir / f"balance_attitude_{stamp}.html"

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(self.COLUMNS)
            writer.writerows(self._samples)

        html_path.write_text(self._build_html(list(self._samples), csv_path.name), encoding="utf-8")
        self.get_logger().info("姿态数据 CSV: %s" % csv_path)
        self.get_logger().info("姿态曲线 HTML: %s" % html_path)

    @staticmethod
    def _resolve_output_dir(path_value: str) -> Path:
        path = Path(str(path_value)).expanduser()
        if path.is_absolute():
            return path.resolve()
        # Use the directory where ros2 launch was invoked, normally the demo workspace root.
        return (Path.cwd() / path).resolve()

    def _build_html(self, rows: list[list[float]], csv_name: str) -> str:
        series = {
            "raw_roll": (1, "#1f77b4"),
            "filtered_roll": (3, "#0f8f5f"),
            "cmd_roll": (5, "#d62728"),
            "raw_pitch": (2, "#9467bd"),
            "filtered_pitch": (4, "#ff7f0e"),
            "cmd_pitch": (6, "#8c564b"),
        }
        width = 1100
        height = 640
        margin_left = 70
        margin_right = 30
        margin_top = 40
        margin_bottom = 70
        plot_w = width - margin_left - margin_right
        plot_h = height - margin_top - margin_bottom

        times = [r[0] for r in rows]
        values = [value for r in rows for value in r[1:]]
        t_min, t_max = min(times), max(times)
        if t_max <= t_min:
            t_max = t_min + 1.0
        v_min, v_max = min(values), max(values)
        pad = max(1.0, (v_max - v_min) * 0.15)
        v_min -= pad
        v_max += pad
        if v_max <= v_min:
            v_max = v_min + 1.0

        def x_of(t: float) -> float:
            return margin_left + (t - t_min) / (t_max - t_min) * plot_w

        def y_of(v: float) -> float:
            return margin_top + (v_max - v) / (v_max - v_min) * plot_h

        grid = []
        for i in range(6):
            ratio = i / 5
            y = margin_top + ratio * plot_h
            val = v_max - ratio * (v_max - v_min)
            grid.append(
                f'<line x1="{margin_left}" y1="{y:.1f}" x2="{width-margin_right}" y2="{y:.1f}" stroke="#e6e6e6"/>'
            )
            grid.append(
                f'<text x="{margin_left-10}" y="{y+4:.1f}" text-anchor="end" font-size="12">{val:.1f}</text>'
            )
        for i in range(6):
            ratio = i / 5
            x = margin_left + ratio * plot_w
            t = t_min + ratio * (t_max - t_min)
            grid.append(
                f'<line x1="{x:.1f}" y1="{margin_top}" x2="{x:.1f}" y2="{height-margin_bottom}" stroke="#f0f0f0"/>'
            )
            grid.append(
                f'<text x="{x:.1f}" y="{height-margin_bottom+24}" text-anchor="middle" font-size="12">{t:.1f}s</text>'
            )

        paths = []
        legend = []
        for idx, (name, (col, color)) in enumerate(series.items()):
            points = " ".join(f"{x_of(r[0]):.1f},{y_of(r[col]):.1f}" for r in rows)
            paths.append(
                f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>'
            )
            lx = margin_left + (idx % 3) * 220
            ly = height - 32 + (idx // 3) * 18
            legend.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+28}" y2="{ly}" stroke="{color}" stroke-width="3"/>')
            legend.append(f'<text x="{lx+36}" y="{ly+4}" font-size="13">{html.escape(name)}</text>')

        return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>Dobot Balance Attitude Plot</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 24px; color: #222; }}
    .meta {{ color: #666; margin-bottom: 12px; }}
    svg {{ max-width: 100%; border: 1px solid #ddd; background: #fff; }}
  </style>
</head>
<body>
  <h1>Dobot Balance Attitude Plot</h1>
  <div class="meta">Samples: {len(rows)} | CSV: {html.escape(csv_name)}</div>
  <svg width="{width}" height="{height}" viewBox="0 0 {width} {height}">
    <text x="{width / 2:.1f}" y="24" text-anchor="middle" font-size="18">Raw / Filtered / Command Attitude</text>
    {''.join(grid)}
    <line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{height-margin_bottom}" stroke="#333"/>
    <line x1="{margin_left}" y1="{height-margin_bottom}" x2="{width-margin_right}" y2="{height-margin_bottom}" stroke="#333"/>
    <text x="18" y="{margin_top + plot_h / 2:.1f}" transform="rotate(-90 18,{margin_top + plot_h / 2:.1f})" text-anchor="middle" font-size="13">Angle (deg)</text>
    <text x="{margin_left + plot_w / 2:.1f}" y="{height-18}" text-anchor="middle" font-size="13">Time (s)</text>
    {''.join(paths)}
    {''.join(legend)}
  </svg>
</body>
</html>
"""


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="balance_control_config.yaml 配置文件路径")
    args = parser.parse_args(argv)

    rclpy.init()
    node = AttitudePlotRecorderNode(args.config)
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
