#!/usr/bin/env python3
"""
Plot outpost modeling debug curves from logs/analysis/outpost_debug_*.csv.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import sys


def _ensure_matplotlib() -> None:
    try:
        import matplotlib  # noqa: F401
        return
    except ModuleNotFoundError:
        pass

    venv_python = Path.cwd() / ".venv/bin/python"
    if os.environ.get("SPV_PLOTTER_BOOTSTRAPPED") != "1" and venv_python.exists():
        env = os.environ.copy()
        env["SPV_PLOTTER_BOOTSTRAPPED"] = "1"
        os.execve(str(venv_python), [str(venv_python), *sys.argv], env)

    raise SystemExit(
        "未找到 matplotlib。可执行：\n"
        "1) source .venv/bin/activate && python tool/plot_outpost_debug.py\n"
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
    candidates = sorted(log_dir.glob("outpost_debug_*.csv"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"未找到日志文件: {log_dir}/outpost_debug_*.csv")
    return candidates[-1]


def _load_csv(csv_path: Path) -> dict[str, list[float]]:
    float_keys = {
        "time_s",
        "target_count",
        "armors_count",
        "match_pos_diff",
        "match_yaw_diff",
        "gate_distance",
        "gate_yaw",
        "used_relaxed_jump",
        "min_reproj_err_px",
        "outpost_idx",
        "outpost_dz",
        "outpost_z_diff",
        "outpost_switch",
        "decision_match",
        "aim_valid",
        "fire",
        "is_outpost",
        "match_ok",
    }
    int_keys = {"seq", "target_name"}

    data: dict[str, list[float]] = {k: [] for k in float_keys | int_keys}
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in float_keys:
                data[key].append(_parse_float(row.get(key, "")))
            for key in int_keys:
                data[key].append(_parse_int(row.get(key, "")))
    return data


def _nan_mean(xs: list[float]) -> float:
    vals = [x for x in xs if math.isfinite(x)]
    if not vals:
        return float("nan")
    return sum(vals) / float(len(vals))


def _nan_p95(xs: list[float]) -> float:
    vals = sorted(x for x in xs if math.isfinite(x))
    if not vals:
        return float("nan")
    idx = int(0.95 * (len(vals) - 1))
    return vals[idx]


def _plot(csv_path: Path, output_path: Path, show: bool) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = _load_csv(csv_path)
    t = data["time_s"]
    if not t:
        raise RuntimeError(f"日志为空: {csv_path}")

    fire_t = [ti for ti, fire in zip(t, data["fire"]) if fire > 0.5]
    switch_t = [ti for ti, sw in zip(t, data["outpost_switch"]) if sw > 0.5]

    fig, axes = plt.subplots(4, 1, figsize=(15, 12), sharex=True)
    ax0, ax1, ax2, ax3 = axes

    ax0.plot(t, data["min_reproj_err_px"], label="min_reproj_err_px", linewidth=1.2)
    ax0.plot(t, data["match_pos_diff"], label="match_pos_diff(m)", linewidth=1.1)
    ax0.plot(t, data["gate_distance"], label="gate_distance(m)", linewidth=1.0, linestyle="--")
    ax0.set_ylabel("Reproj/Pos")
    ax0.grid(alpha=0.35)
    ax0.legend(loc="upper right")

    ax1.plot(t, data["match_yaw_diff"], label="match_yaw_diff(rad)", linewidth=1.1)
    ax1.plot(t, data["gate_yaw"], label="gate_yaw(rad)", linewidth=1.0, linestyle="--")
    ax1.set_ylabel("Yaw(rad)")
    ax1.grid(alpha=0.35)
    ax1.legend(loc="upper right")

    ax2.plot(t, data["outpost_idx"], label="outpost_idx", drawstyle="steps-post", linewidth=1.4)
    ax2.plot(t, data["outpost_dz"], label="outpost_dz(m)", linewidth=1.1)
    ax2.plot(t, data["outpost_z_diff"], label="outpost_z_diff(m)", linewidth=1.0, alpha=0.9)
    ax2.plot(t, data["outpost_switch"], label="outpost_switch", drawstyle="steps-post", linewidth=1.0)
    ax2.set_ylabel("Outpost State")
    ax2.grid(alpha=0.35)
    ax2.legend(loc="upper right")

    ax3.plot(t, data["match_ok"], label="match_ok", drawstyle="steps-post", linewidth=1.3)
    ax3.plot(t, data["decision_match"], label="decision_match", drawstyle="steps-post", linewidth=1.3)
    ax3.plot(t, data["aim_valid"], label="aim_valid", drawstyle="steps-post", linewidth=1.3)
    ax3.plot(t, data["fire"], label="fire", drawstyle="steps-post", linewidth=1.3)
    ax3.plot(t, data["used_relaxed_jump"], label="used_relaxed_jump", drawstyle="steps-post", linewidth=1.0)
    ax3.set_ylabel("Flags")
    ax3.set_xlabel("time(s)")
    ax3.set_ylim(-0.1, 1.2)
    ax3.grid(alpha=0.35)
    ax3.legend(loc="upper right")

    for ax in (ax0, ax1, ax2):
        for x in fire_t:
            ax.axvline(x, color="red", linewidth=0.5, alpha=0.14)
        for x in switch_t:
            ax.axvline(x, color="purple", linewidth=0.6, alpha=0.18)

    mean_reproj = _nan_mean(data["min_reproj_err_px"])
    p95_reproj = _nan_p95(data["min_reproj_err_px"])
    mean_pos = _nan_mean(data["match_pos_diff"])
    mean_yaw = _nan_mean(data["match_yaw_diff"])
    fire_count = sum(1 for x in data["fire"] if x > 0.5)
    match_ratio = _nan_mean(data["match_ok"])
    title = (
        f"outpost debug: {csv_path.name}  frames={len(t)}  fire={fire_count}  "
        f"match_ratio={match_ratio:.3f}  reproj_mean={mean_reproj:.3f}px  "
        f"reproj_p95={p95_reproj:.3f}px  pos_mean={mean_pos:.4f}m  yaw_mean={mean_yaw:.4f}rad"
    )
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    print(f"[OK] 曲线图已保存: {output_path}")
    print(
        "[Summary] "
        f"fire={fire_count}, match_ratio={match_ratio:.3f}, "
        f"reproj_mean={mean_reproj:.3f}px, reproj_p95={p95_reproj:.3f}px"
    )

    if show:
        plt.show()
    plt.close(fig)


def main() -> None:
    _ensure_matplotlib()

    parser = argparse.ArgumentParser(description="Plot outpost model debug CSV.")
    parser.add_argument("--input", "-i", type=str, default="", help="CSV 文件路径，不填则使用最新日志")
    parser.add_argument("--log-dir", type=str, default="logs/analysis", help="日志目录（用于找最新 CSV）")
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="",
        help="输出 PNG 路径，不填则输出到与 CSV 同目录同名 .png",
    )
    parser.add_argument("--show", action="store_true", help="显示窗口（默认只保存）")
    args = parser.parse_args()

    csv_path = Path(args.input).resolve() if args.input else _find_latest_csv(Path(args.log_dir).resolve())
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV 不存在: {csv_path}")

    output_path = Path(args.output).resolve() if args.output else csv_path.with_suffix(".png")
    _plot(csv_path, output_path, args.show)


if __name__ == "__main__":
    main()
