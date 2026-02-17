#!/usr/bin/env python3
"""
Extract ambiguous frames from a source video based on outpost_debug CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import subprocess
from typing import Optional

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None



def parse_float(v: str) -> float:
    try:
        return float(v)
    except Exception:
        return float("nan")


def parse_int(v: str) -> int:
    try:
        return int(float(v))
    except Exception:
        return 0


def _frame_count_by_cv2(video_path: Path) -> Optional[int]:
    if cv2 is None:
        return None
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return None
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    return count if count > 0 else None


def _extract_by_cv2(video_path: Path, frame_idx: int, out_path: Path) -> bool:
    if cv2 is None:
        return False
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return False
    cap.set(cv2.CAP_PROP_POS_FRAMES, float(frame_idx))
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        return False
    return bool(cv2.imwrite(str(out_path), frame))


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract ambiguous frames by reprojection error.")
    parser.add_argument("--video", required=True, help="Video path.")
    parser.add_argument("--csv", required=True, help="outpost_debug CSV path.")
    parser.add_argument("--out-dir", default="logs/analysis/ambiguous_frames", help="Output dir.")
    parser.add_argument("--topk", type=int, default=40, help="Max frames to export.")
    parser.add_argument(
        "--min-err", type=float, default=20.0, help="Minimum min_reproj_err_px to keep."
    )
    parser.add_argument(
        "--include-unmatched",
        action="store_true",
        help="Include rows with match_ok=0 (default: only matched rows).",
    )
    args = parser.parse_args()

    video_path = Path(args.video)
    csv_path = Path(args.csv)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            seq = parse_int(row.get("seq", "0"))
            match_ok = parse_int(row.get("match_ok", "0"))
            match_reproj = parse_float(row.get("match_reproj_err", "nan"))
            min_reproj = parse_float(row.get("min_reproj_err_px", "nan"))
            err = match_reproj if math.isfinite(match_reproj) else min_reproj
            if not math.isfinite(err) or err < args.min_err:
                continue
            if (not args.include_unmatched) and match_ok == 0:
                continue
            rows.append(
                {
                    "seq": seq,
                    "err": err,
                    "match_ok": match_ok,
                    "match_id": parse_int(row.get("match_id", "-1")),
                    "target_name": parse_int(row.get("target_name", "0")),
                }
            )

    rows.sort(key=lambda r: r["err"], reverse=True)

    frame_count = _frame_count_by_cv2(video_path) or 0
    if frame_count <= 0:
        ffprobe_cmd = [
            "ffprobe",
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=nb_read_frames,nb_frames",
            "-of",
            "default=nokey=1:noprint_wrappers=1",
            str(video_path),
        ]
        probe = subprocess.run(ffprobe_cmd, capture_output=True, text=True, check=False)
        if probe.returncode != 0:
            raise RuntimeError(f"ffprobe failed and cv2 unavailable: {probe.stderr.strip()}")
        for line in probe.stdout.splitlines():
            line = line.strip()
            if line.isdigit():
                frame_count = max(frame_count, int(line))
    if frame_count <= 0:
        raise RuntimeError("video frame count invalid")

    exported = 0
    seen_seq: set[int] = set()
    for row in rows:
        if exported >= args.topk:
            break
        seq = int(row["seq"])
        if seq in seen_seq:
            continue
        seen_seq.add(seq)

        idx = max(0, seq - 1) % frame_count
        out_path = (
            out_dir
            / f"amb_seq{seq:06d}_f{idx:06d}_err{row['err']:.1f}_mid{row['match_id']}_ok{row['match_ok']}.jpg"
        )
        ok = _extract_by_cv2(video_path, idx, out_path)
        if not ok:
            ffmpeg_cmd = [
                "ffmpeg",
                "-v",
                "error",
                "-y",
                "-i",
                str(video_path),
                "-vf",
                f"select=eq(n\\,{idx})",
                "-frames:v",
                "1",
                str(out_path),
            ]
            result = subprocess.run(ffmpeg_cmd, capture_output=True, text=True, check=False)
            if result.returncode != 0:
                continue
        exported += 1
    print(f"[OK] exported {exported} frames to: {out_dir}")


if __name__ == "__main__":
    main()
