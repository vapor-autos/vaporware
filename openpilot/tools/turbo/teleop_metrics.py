import json
import os
from datetime import UTC, datetime
from typing import Any


DEFAULT_METRICS_DIR = "/tmp/turbo-metrics"


def _ensure_dir(path: str) -> str:
  os.makedirs(path, exist_ok=True)
  return path


def default_metrics_jsonl_path(name: str) -> str:
  metrics_dir = _ensure_dir(os.getenv("TURBO_METRICS_DIR", DEFAULT_METRICS_DIR))
  timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
  return os.path.join(metrics_dir, f"{timestamp}_{name}_{os.getpid()}.jsonl")


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
