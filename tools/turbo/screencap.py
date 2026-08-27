#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
CAPTURES_DIR = SCRIPT_DIR / "captures"
DEFAULT_FPS = 20
DEFAULT_CRF = 23
DEFAULT_PRESET = "veryfast"


def dotenv_display() -> str | None:
  env_path = REPO_ROOT / ".env"
  if not env_path.exists():
    return None

  for line in env_path.read_text().splitlines():
    line = line.strip()
    if not line or line.startswith("#") or "=" not in line:
      continue
    key, value = line.split("=", 1)
    if key.strip() == "DISPLAY":
      return value.strip().strip("\"'")
  return None


def display_size(display: str) -> str:
  proc = subprocess.run(
    ["xdpyinfo", "-display", display],
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    check=False,
  )
  if proc.returncode != 0:
    print(proc.stderr.strip(), file=sys.stderr)
    raise SystemExit(f"could not read X11 display {display}")

  for line in proc.stdout.splitlines():
    fields = line.strip().split()
    if fields[:1] == ["dimensions:"] and len(fields) >= 2:
      return fields[1]
  raise SystemExit(f"could not find dimensions for X11 display {display}")


def ffmpeg_bin() -> str:
  if os.environ.get("FFMPEG"):
    return os.environ["FFMPEG"]
  if Path("/usr/bin/ffmpeg").exists():
    return "/usr/bin/ffmpeg"
  return "ffmpeg"


def capture_path() -> Path:
  CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
  timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
  for suffix in ["", *[f"_{i}" for i in range(1, 100)]]:
    path = CAPTURES_DIR / f"screencap_{timestamp}{suffix}.mp4"
    if not path.exists():
      return path
  raise SystemExit(f"could not choose a capture filename in {CAPTURES_DIR}")


def main() -> int:
  parser = argparse.ArgumentParser(description="Record the X11 display to tools/turbo/captures as H.264 MP4.")
  parser.add_argument("-d", "--duration", type=float, help="seconds to record; omit to record until Ctrl-C")
  parser.add_argument("--display", default=os.environ.get("DISPLAY") or dotenv_display() or ":0")
  parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
  parser.add_argument("--size", help="WIDTHxHEIGHT; defaults to the display dimensions")
  args = parser.parse_args()

  size = args.size or display_size(args.display)
  output_path = capture_path()
  ffmpeg = ffmpeg_bin()
  # Do not use +faststart here. Its in-place second pass can corrupt mdat if
  # finalization is interrupted, and local captures do not need it.
  cmd = [
    ffmpeg,
    "-hide_banner",
    "-loglevel", "warning",
    "-y",
    "-f", "x11grab",
    "-framerate", str(args.fps),
    "-video_size", size,
    "-i", f"{args.display}+0,0",
    "-an",
    "-c:v", "libx264",
    "-preset", DEFAULT_PRESET,
    "-crf", str(DEFAULT_CRF),
    "-pix_fmt", "yuv420p",
  ]
  if args.duration is not None:
    cmd += ["-t", str(args.duration)]
  cmd.append(str(output_path))

  print(f"recording {args.display} {size} @ {args.fps}fps -> {output_path}")
  # Keep ffmpeg out of the terminal's foreground process group. Otherwise one
  # Ctrl-C signals both processes and this handler sends ffmpeg a second SIGINT
  # while it is flushing the encoder or writing the MP4 index.
  proc = subprocess.Popen(cmd, start_new_session=True)
  interrupted = False
  try:
    proc.wait()
  except KeyboardInterrupt:
    interrupted = True
    print("\nstopping recording; waiting for ffmpeg to finalize the MP4...", file=sys.stderr)
    proc.send_signal(signal.SIGINT)
    while proc.poll() is None:
      try:
        proc.wait()
      except KeyboardInterrupt:
        print("\nalready finalizing; please wait...", file=sys.stderr)

  expected_exit = proc.returncode == 0 or (interrupted and proc.returncode == 255)
  if expected_exit:
    print(output_path)
    return 0

  return proc.returncode or 1


if __name__ == "__main__":
  raise SystemExit(main())
