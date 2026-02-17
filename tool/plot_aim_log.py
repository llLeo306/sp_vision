#!/usr/bin/env python3
"""
Plot aim debug curves from logs/analysis/aim_trace_*.csv.
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys


def _ensure_matplotlib() -> None:
    try:
        import matplotlib  # noqa: F401
        return
    except ModuleNotFoundError:
        pass

    venv_python = (Path.cwd() / ".venv/bin/python")
    if os.environ.get("SPV_PLOTTER_BOOTSTRAPPED") != "1" and venv_python.exists():
        env = os.environ.copy()
        env["SPV_PLOTTER_BOOTSTRAPPED"] = "1"
        os.execve(str(venv_python), [str(venv_python), *sys.argv], env)

    raise SystemExit(
        "未找到 matplotlib。可执行：\n"
        "1) source .venv/bin/activate && python tool/plot_aim_log.py\n"
        "2) 或安装: python3 -m pip install matplotlib"
    )


def _parse_float(raw: str) -> float:
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float("nan")


def _parse_int(raw: str) -> int:
    try:
        return int(float(raw))
    except (TypeError, ValueError):
        return 0


def _find_latest_csv(log_dir: Path) -> Path:
    candidates = sorted(log_dir.glob("aim_trace_*.csv"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"未找到日志文件: {log_dir}/aim_trace_*.csv")
    return candidates[-1]


def _load_csv(csv_path: Path) -> dict[str, list[float]]:
    keys_float = {
        "time_s",
        "bullet_speed",
        "target_count",
        "target_distance",
        "target_vyaw",
        "gimbal_yaw",
        "gimbal_pitch",
        "cmd_yaw",
        "cmd_pitch",
        "yaw_err",
        "pitch_err",
        "control",
        "shoot",
        "aim_valid",
        "aim_x",
        "aim_y",
        "aim_z",
        "aim_a",
    }
    keys_int = {"seq", "mode", "aim_armor_name", "aim_armor_type"}

    data: dict[str, list[float]] = {k: [] for k in keys_float | keys_int}
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in keys_float:
                data[key].append(_parse_float(row.get(key, "")))
            for key in keys_int:
                data[key].append(_parse_int(row.get(key, "")))
    return data


def _plot(csv_path: Path, output_path: Path, show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = _load_csv(csv_path)
    t = data["time_s"]
    if not t:
        raise RuntimeError(f"日志为空: {csv_path}")

    fire_t = [ti for ti, shoot in zip(t, data["shoot"]) if shoot > 0.5]

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    ax0, ax1, ax2, ax3 = axes

    ax0.plot(t, data["cmd_yaw"], label="cmd_yaw(rad)", linewidth=1.3)
    ax0.plot(t, data["gimbal_yaw"], label="gimbal_yaw(rad)", linewidth=1.0, alpha=0.9)
    ax0.set_ylabel("Yaw(rad)")
    ax0.grid(alpha=0.35)
    ax0.legend(loc="upper right")

    ax1.plot(t, data["cmd_pitch"], label="cmd_pitch(rad)", linewidth=1.3)
    ax1.plot(t, data["gimbal_pitch"], label="gimbal_pitch(rad)", linewidth=1.0, alpha=0.9)
    ax1.set_ylabel("Pitch(rad)")
    ax1.grid(alpha=0.35)
    ax1.legend(loc="upper right")

    ax2.plot(t, data["yaw_err"], label="yaw_err(rad)", linewidth=1.2)
    ax2.plot(t, data["pitch_err"], label="pitch_err(rad)", linewidth=1.2)
    ax2.axhline(0.0, color="black", linewidth=0.8, alpha=0.7)
    ax2.set_ylabel("Error(rad)")
    ax2.grid(alpha=0.35)
    ax2.legend(loc="upper right")

    ax3.plot(t, data["control"], label="control", drawstyle="steps-post", linewidth=1.5)
    ax3.plot(t, data["shoot"], label="shoot", drawstyle="steps-post", linewidth=1.5)
    ax3.plot(t, data["aim_valid"], label="aim_valid", drawstyle="steps-post", linewidth=1.2)
    ax3.set_ylim(-0.1, 1.2)
    ax3.set_ylabel("State")
    ax3.set_xlabel("time(s)")
    ax3.grid(alpha=0.35)
    ax3.legend(loc="upper right")

    for ax in (ax0, ax1, ax2):
        for x in fire_t:
            ax.axvline(x, color="red", linewidth=0.5, alpha=0.18)

    fig.suptitle(f"aim log: {csv_path.name}  frames={len(t)}", fontsize=11)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    print(f"[OK] 曲线图已保存: {output_path}")

    if show:
        plt.show()
    plt.close(fig)


def main() -> None:
    _ensure_matplotlib()

    parser = argparse.ArgumentParser(description="Plot SpVisionAimer CSV logs.")
    parser.add_argument("--input", "-i", type=str, default="", help="CSV 文件路径，不填则使用最新日志")
    parser.add_argument("--log-dir", type=str, default="logs/analysis", help="日志目录（用于找最新 CSV）")
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="",
        help="输出 PNG 路径，不填则输出到与 CSV 同目录同名 .png",
    )
    parser.add_argument("--show", action="store_true", help="显示窗口（默认仅保存图片）")
    args = parser.parse_args()

    if args.input:
        csv_path = Path(args.input).resolve()
    else:
        csv_path = _find_latest_csv(Path(args.log_dir).resolve())

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV 不存在: {csv_path}")

    output_path = Path(args.output).resolve() if args.output else csv_path.with_suffix(".png")
    _plot(csv_path, output_path, args.show)


if __name__ == "__main__":
    main()
