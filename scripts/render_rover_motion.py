#!/usr/bin/env python3
"""Render benchmark rover frames into a video using ffmpeg.

The benchmark emits PPM frames when visualization is enabled. This helper turns
those frames into a single video so rover motion can be reviewed easily.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Directory containing frame_*.ppm files")
    parser.add_argument("--output", required=True, help="Output video path, e.g. rover.mp4 or rover.gif")
    parser.add_argument("--fps", type=int, default=20, help="Frame rate for the rendered video")
    parser.add_argument(
        "--pattern",
        default="frame_%06d.ppm",
        help="ffmpeg input pattern relative to --input (default: frame_%%06d.ppm)",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    input_dir = Path(args.input)
    output_path = Path(args.output)

    if not input_dir.is_dir():
      print(f"input directory not found: {input_dir}", file=sys.stderr)
      return 1

    if shutil.which("ffmpeg") is None:
        print("ffmpeg is required for this helper but was not found on PATH.", file=sys.stderr)
        return 1

    frames = sorted(input_dir.glob("frame_*.ppm"))
    if not frames:
        print(f"no PPM frames found in {input_dir}", file=sys.stderr)
        return 1

    output_path.parent.mkdir(parents=True, exist_ok=True)

    input_pattern = str(input_dir / args.pattern)
    command = ["ffmpeg", "-y", "-framerate", str(args.fps), "-i", input_pattern]
    if output_path.suffix.lower() == ".gif":
        command.extend(["-vf", f"fps={args.fps},scale=trunc(iw/2)*2:trunc(ih/2)*2:flags=lanczos"])
    else:
        command.extend(["-pix_fmt", "yuv420p"])
    command.append(str(output_path))

    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        print("ffmpeg failed while rendering rover motion.", file=sys.stderr)
        return completed.returncode

    print(f"wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
