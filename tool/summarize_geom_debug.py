#!/usr/bin/env python3
"""Summarize outpost debug CSV with geometry quality metrics."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def to_f(raw: str) -> float:
    try:
        return float(raw)
    except Exception:
        return float("nan")


def to_i(raw: str) -> int:
    try:
        return int(float(raw))
    except Exception:
        return 0


def quantile(vals: list[float], q: float) -> float:
    arr = sorted(v for v in vals if math.isfinite(v))
    if not arr:
        return float("nan")
    idx = int(q * (len(arr) - 1))
    return arr[idx]


def find_bad_segments(rows: list[dict], min_len: int = 8) -> list[tuple[int, int, int]]:
    segs: list[tuple[int, int, int]] = []
    i = 0
    n = len(rows)
    while i < n:
        r = rows[i]
        bad = to_i(r.get("match_ok", "0")) == 1 and to_i(r.get("geom_match_ok", "0")) == 0
        if not bad:
            i += 1
            continue
        j = i
        while j < n:
            rr = rows[j]
            bad2 = to_i(rr.get("match_ok", "0")) == 1 and to_i(rr.get("geom_match_ok", "0")) == 0
            if not bad2:
                break
            j += 1
        if j - i >= min_len:
            segs.append((i + 1, j, j - i))
        i = j
    segs.sort(key=lambda x: x[2], reverse=True)
    return segs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()

    rows = list(csv.DictReader(args.csv.open("r", encoding="utf-8")))
    if not rows:
        print("empty csv")
        return

    n = len(rows)
    match_ratio = sum(to_i(r.get("match_ok", "0")) for r in rows) / n
    geom_ratio = sum(to_i(r.get("geom_match_ok", "0")) for r in rows) / n

    match_only = [r for r in rows if to_i(r.get("match_ok", "0")) == 1]
    match_bad = [r for r in match_only if to_i(r.get("geom_match_ok", "0")) == 0]

    reproj = [to_f(r.get("min_reproj_err_px", "nan")) for r in match_only]
    center = [to_f(r.get("min_center_err_px", "nan")) for r in match_only]
    iou = [to_f(r.get("min_bbox_iou", "nan")) for r in match_only]
    set_iou_visible = [to_f(r.get("set_iou_visible", "nan")) for r in match_only]
    set_iou_norm = [to_f(r.get("set_iou_norm", "nan")) for r in match_only]
    set_cov = [to_f(r.get("set_coverage", "nan")) for r in match_only]
    set_recall = [to_f(r.get("set_visible_recall", "nan")) for r in match_only]

    print(f"csv: {args.csv}")
    print(f"rows: {n}")
    print(f"match_ratio: {match_ratio:.4f}")
    print(f"geom_ratio: {geom_ratio:.4f}")
    print(f"match_but_geom_bad: {len(match_bad)} ({(len(match_bad)/max(1,len(match_only))):.3f} of matched)")
    print(
        "matched quantiles: "
        f"reproj_p50={quantile(reproj, 0.5):.2f} reproj_p90={quantile(reproj, 0.9):.2f} reproj_p95={quantile(reproj, 0.95):.2f}; "
        f"center_p50={quantile(center, 0.5):.2f} center_p90={quantile(center, 0.9):.2f} center_p95={quantile(center, 0.95):.2f}; "
        f"iou_p10={quantile(iou, 0.1):.3f}"
    )
    print(
        "set-overlap: "
        f"visible_iou_p50={quantile(set_iou_visible, 0.5):.3f} visible_iou_p90={quantile(set_iou_visible, 0.9):.3f}; "
        f"norm_iou_p50={quantile(set_iou_norm, 0.5):.3f}; "
        f"coverage_p50={quantile(set_cov, 0.5):.3f}; recall_p50={quantile(set_recall, 0.5):.3f}"
    )

    relaxed = sum(to_i(r.get("used_relaxed_jump", "0")) for r in rows)
    emergency = sum(to_i(r.get("used_emergency_relock", "0")) for r in rows)
    print(f"used_relaxed_jump: {relaxed}")
    print(f"used_emergency_relock: {emergency}")

    segs = find_bad_segments(rows)
    print("top_bad_segments(seq_start, seq_end, len):")
    for seg in segs[:8]:
        print(seg)


if __name__ == "__main__":
    main()
