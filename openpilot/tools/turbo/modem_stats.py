import json
import time
from typing import Any

from openpilot.tools.turbo.teleop_metrics import parse_serving_cell_extra


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
