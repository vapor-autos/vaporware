import json
import os
import re
import time
from datetime import UTC, datetime
from typing import Any

from openpilot.common.params import Params
from openpilot.common.time_helpers import system_time_valid


DEFAULT_METRICS_DIR = "/tmp/turbo-metrics"
RUN_ID_MAX_LEN = 96


def _ensure_dir(path: str) -> str:
  os.makedirs(path, exist_ok=True)
  return path


def _safe_filename_token(value: str, max_len: int = RUN_ID_MAX_LEN) -> str:
  safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
  return safe[:max_len] or "unknown"


def _boot_id() -> str:
  try:
    with open("/proc/sys/kernel/random/boot_id") as f:
      return f.read().strip().split("-", 1)[0]
  except FileNotFoundError:
    return "no_boot_id"


def current_route() -> str | None:
  try:
    return Params().get("CurrentRoute")
  except Exception:
    return None


def metrics_run_id() -> str:
  return os.getenv("TURBO_METRICS_RUN_ID") or current_route() or f"boot_{_boot_id()}"


def wait_for_valid_time(timeout: float) -> bool:
  end = time.monotonic() + max(0.0, timeout)
  while not system_time_valid():
    if time.monotonic() >= end:
      return False
    time.sleep(0.25)
  return True


def _timestamp_token(wait_for_time: bool = False, timeout: float | None = None) -> str:
  if wait_for_time:
    wait_for_valid_time(timeout if timeout is not None else float(os.getenv("TURBO_METRICS_TIME_SYNC_TIMEOUT", "60.0")))

  if system_time_valid():
    return datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
  return f"unsynced_{time.monotonic_ns()}"


def metrics_metadata() -> dict[str, Any]:
  valid_time = system_time_valid()
  return {
    "monotonic_time": time.monotonic(),
    "time_valid": valid_time,
    "utc_time": datetime.now(UTC).isoformat() if valid_time else None,
    "run_id": metrics_run_id(),
    "route": current_route(),
  }


def default_metrics_jsonl_path(name: str, wait_for_time: bool = False, timeout: float | None = None) -> str:
  metrics_dir = _ensure_dir(os.getenv("TURBO_METRICS_DIR", DEFAULT_METRICS_DIR))
  timestamp = _timestamp_token(wait_for_time, timeout)
  run_id = _safe_filename_token(metrics_run_id())
  return os.path.join(metrics_dir, f"{timestamp}_{run_id}_{name}_{os.getpid()}.jsonl")


def default_latest_json_path(name: str) -> str:
  latest_dir = _ensure_dir(os.path.join(os.getenv("TURBO_METRICS_DIR", DEFAULT_METRICS_DIR), "latest"))
  return os.path.join(latest_dir, f"{name}.json")


def atomic_write_json(path: str, payload: dict[str, Any]) -> None:
  directory = os.path.dirname(path)
  if directory:
    os.makedirs(directory, exist_ok=True)
  tmp_path = f"{path}.tmp"
  with open(tmp_path, "w") as f:
    json.dump(payload, f, default=str, sort_keys=True)
    f.write("\n")
  os.replace(tmp_path, path)


def append_jsonl(path: str, payload: dict[str, Any]) -> None:
  directory = os.path.dirname(path)
  if directory:
    os.makedirs(directory, exist_ok=True)
  with open(path, "a") as f:
    f.write(json.dumps(payload, default=str, sort_keys=True) + "\n")


def write_metrics_payload(payload: dict[str, Any], jsonl_file: str | None = None, latest_file: str | None = None, print_line: bool = True) -> None:
  payload = {**payload, "metrics_meta": metrics_metadata()}
  line = json.dumps(payload, default=str, sort_keys=True)
  if print_line:
    print(line, flush=True)
  if jsonl_file:
    directory = os.path.dirname(jsonl_file)
    if directory:
      os.makedirs(directory, exist_ok=True)
    with open(jsonl_file, "a") as f:
      f.write(line + "\n")
  if latest_file:
    atomic_write_json(latest_file, payload)


def env_bool(name: str, default: bool = False) -> bool:
  value = os.getenv(name)
  if value is None:
    return default
  return value.strip().lower() in ("1", "true", "yes", "on")


def parse_serving_cell_extra(extra: str | None) -> dict[str, Any]:
  if not extra:
    return {}

  parts = extra.split(",")
  if len(parts) < 17 or parts[0] != "servingcell" or parts[2] != "LTE":
    return {}

  def get_int(index: int) -> int | None:
    try:
      return int(parts[index], 16) if index == 6 else int(parts[index])
    except (IndexError, ValueError):
      return None

  # Quectel LTE serving-cell format ends with tac, rsrp, rsrq, rssi, sinr, srxlev.
  rsrp = get_int(13)
  rsrq = get_int(14)
  rssi = get_int(15)
  sinr = get_int(16)
  return {
    "serving_cell_state": parts[1],
    "serving_cell_rat": parts[2],
    "duplex_mode": parts[3] if len(parts) > 3 else None,
    "mcc": get_int(4),
    "mnc": get_int(5),
    "cell_id": get_int(6),
    "pcid": get_int(7),
    "earfcn": get_int(8),
    "freq_band": get_int(9),
    "ul_bandwidth": get_int(10),
    "dl_bandwidth": get_int(11),
    "tac": get_int(12),
    "rsrp": rsrp,
    "rsrq": rsrq,
    "rssi": rssi,
    "sinr": sinr,
    "srxlev": get_int(17),
    "rsrp_ish": rsrp,
    "rsrq_ish": rsrq,
    "rssi_ish": rssi,
    "sinr_ish": sinr,
  }
