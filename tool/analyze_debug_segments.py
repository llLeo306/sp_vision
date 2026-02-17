#!/usr/bin/env python3
"""
Analyze outpost_debug CSV and export frame-by-frame debug images for worst unmatched segments.
Requires cv2 in .venv.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List

import cv2


def to_i(v: str, default: int = 0) -> int:
    try:
        return int(float(v))
    except Exception:
        return default


def to_f(v: str, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


@dataclass
class Segment:
    start_seq: int
    end_seq: int
    length: int
    mean_reproj: float


def find_segments(rows: List[dict], min_len: int) -> List[Segment]:
    segments: List[Segment] = []
    i = 0
    n = len(rows)
    while i < n:
        r = rows[i]
        unmatched = to_i(r.get("match_ok", "0")) == 0 and to_i(r.get("target_count", "0")) > 0
        if not unmatched:
            i += 1
            continue
        j = i
        vals: List[float] = []
        while j < n:
            rr = rows[j]
            cond = to_i(rr.get("match_ok", "0")) == 0 and to_i(rr.get("target_count", "0")) > 0
            if not cond:
                break
            v = rr.get("min_reproj_err_px", "")
            if v not in ("", "nan"):
                vals.append(to_f(v))
            j += 1
        length = j - i
        if length >= min_len:
            mean_reproj = sum(vals) / len(vals) if vals else 0.0
            segments.append(
                Segment(
                    start_seq=to_i(rows[i].get("seq", "0")),
                    end_seq=to_i(rows[j - 1].get("seq", "0")),
                    length=length,
                    mean_reproj=mean_reproj,
                )
            )
        i = j
    segments.sort(key=lambda s: (s.length, s.mean_reproj), reverse=True)
    return segments


def export_segment_frames(
    video_path: Path, segment: Segment, out_dir: Path, max_frames: int = 180
) -> int:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return 0
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    exported = 0
    start = max(1, segment.start_seq)
    end = segment.end_seq
    for seq in range(start, end + 1):
        if exported >= max_frames:
            break
        frame_idx = (seq - 1) % max(1, frame_count)
        cap.set(cv2.CAP_PROP_POS_FRAMES, float(frame_idx))
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        cv2.putText(
            frame,
            f"seq={seq} seg=[{segment.start_seq},{segment.end_seq}]",
            (20, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        out_path = out_dir / f"seq_{seq:06d}.jpg"
        cv2.imwrite(str(out_path), frame)
        exported += 1
    cap.release()
    return exported


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--video", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--min-len", type=int, default=8)
    parser.add_argument("--topk", type=int, default=3)
    args = parser.parse_args()

    csv_path = Path(args.csv)
    rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8")))
    segments = find_segments(rows, args.min_len)
    print(f"[INFO] segments_found={len(segments)}")
    out_root = Path(args.out_dir)
    out_root.mkdir(parents=True, exist_ok=True)

    for idx, seg in enumerate(segments[: args.topk], start=1):
        seg_dir = out_root / f"seg_{idx:02d}_{seg.start_seq}_{seg.end_seq}"
        seg_dir.mkdir(parents=True, exist_ok=True)
        cnt = export_segment_frames(Path(args.video), seg, seg_dir)
        print(
            f"[SEG] #{idx} seq=[{seg.start_seq},{seg.end_seq}] len={seg.length} "
            f"mean_reproj={seg.mean_reproj:.2f} exported={cnt}"
        )


if __name__ == "__main__":
    main()

