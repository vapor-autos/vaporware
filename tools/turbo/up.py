#!/usr/bin/env python3
import sys

from openpilot.common.params import Params
from openpilot.common.hardware import PC, TICI


def main() -> None:
  params = Params()

  if "-down" in sys.argv:
    params.put_bool("GCS", False, block=True)
    params.put_bool("UGV", False, block=True)
    print("Turbo disabled")
  elif "-cal" in sys.argv:
    params.remove("CalibrationParams")
    params.remove("LiveTorqueParameters")
    print("Calibration parameters removed")
  elif PC:
    params.put_bool("GCS", True, block=True)
    params.put_bool("UGV", False, block=True)
    print("Turbo GCS enabled")
  elif TICI:
    params.put_bool("GCS", False, block=True)
    params.put_bool("UGV", True, block=True)
    print("Turbo UGV enabled")
  else:
    print("Turbo mode unchanged")


if __name__ == "__main__":
  main()
