#!/usr/bin/env python3
import argparse
import json
import os
import time
from typing import Any

from openpilot.tools.turbo.teleop_metrics import parse_serving_cell_extra, write_metrics_payload


def read_modem_stats(path: str) -> dict[str, Any] | None:
  try:
    with open(path) as f:
      modem = json.load(f)
  except (FileNotFoundError, json.JSONDecodeError):
    return None

  parsed_extra = parse_serving_cell_extra(modem.get("extra"))
  return {
    "monotonic_time": time.monotonic(),
    "modem": modem,
    "radio": parsed_extra,
  }


def run(args: argparse.Namespace) -> None:
  while True:
    payload = read_modem_stats(args.modem_file)
    if payload is not None:
      write_metrics_payload({"modem_stats": payload}, args.stats_file, args.latest_file)
    time.sleep(args.interval)


def main() -> None:
  parser = argparse.ArgumentParser(description="Log Turbo teleop modem/RF metrics")
  parser.add_argument("--modem-file", default=os.getenv("TURBO_MODEM_SOURCE_FILE", "/dev/shm/modem"), help="modem JSON source file")
  parser.add_argument("--interval", type=float, default=float(os.getenv("TURBO_MODEM_STATS_INTERVAL", "2.0")), help="sample interval in seconds")
  parser.add_argument("--stats-file", default=os.getenv("TURBO_MODEM_STATS_FILE", "/tmp/turbo_modem_stats.jsonl"), help="output JSONL file")
  parser.add_argument(
    "--latest-file",
    default=os.getenv("TURBO_MODEM_STATS_LATEST_FILE", "/tmp/turbo_modem_latest.json"),
    help="latest modem stats JSON file",
  )
  args = parser.parse_args()
  run(args)


if __name__ == "__main__":
  main()
