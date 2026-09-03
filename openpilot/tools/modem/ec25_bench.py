#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path

from openpilot.common.hardware.modem.discovery import discover_ec25_modems, select_ec25_modem
from openpilot.common.hardware.modem.probe import ProbeError, ProbeResult, probe_modem


def _display_path(path: Path | None) -> str:
  return str(path) if path is not None else "not present"


def _print_human(result: ProbeResult) -> None:
  endpoints = result.endpoints
  print("EC25 probe (no data session or route changes)")
  print(f"USB device:       {endpoints.sysfs_name} ({endpoints.vendor_id}:{endpoints.product_id})")
  print(f"USB product:      {endpoints.product or 'unknown'}")
  print(f"USB speed:        {endpoints.usb_speed_mbps or 'unknown'} Mbps")
  print(f"Primary AT:       {_display_path(endpoints.primary_at_port)}")
  print(f"Secondary AT:     {_display_path(endpoints.secondary_at_port)}")
  print(f"Selected AT:      {result.selected_at_port}")
  print(f"QMI control:      {_display_path(endpoints.qmi_control)}")
  print(f"Network iface:    {endpoints.network_interface or 'not present'}")
  print(f"ModemManager:     {result.modem_manager}")
  print(f"Manufacturer:     {result.manufacturer or 'unknown'}")
  print(f"Model:            {result.model or 'unknown'}")
  print(f"Firmware:         {result.firmware or 'unknown'}")
  print(f"SIM:              {result.sim_state}")
  print(f"Radio:            {result.radio_state_name} ({result.radio_state if result.radio_state is not None else 'unknown'})")
  print(f"USB network mode: {result.usb_mode_name} ({result.usb_mode if result.usb_mode is not None else 'unknown'})")

  failed_attempts = [attempt for attempt in result.port_attempts if attempt.error]
  if failed_attempts:
    print("Port fallback:")
    for attempt in failed_attempts:
      print(f"  {attempt.port}: {attempt.error}")
  if result.radio_changed_to_off:
    print("Safety action:    changed radio to minimum functionality (CFUN=0)")
  if result.query_errors:
    print("Query status:")
    for command, error in result.query_errors.items():
      print(f"  {command}: {error}")


def _parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    description="Safely discover and probe a Quectel EC25 (2c7c:0125) without starting PPP/QMI or changing routes.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )
  parser.add_argument("--usb-device", help="Select a sysfs USB device name, for example 3-4.4")
  parser.add_argument("--timeout", type=float, default=3.0, help="Per-command AT timeout in seconds")
  parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
  parser.add_argument(
    "--keep-radio-off",
    action="store_true",
    help="If needed, issue AT+CFUN=0 and verify minimum functionality; this is the tool's only state-changing action",
  )
  return parser


def main() -> int:
  args = _parser().parse_args()
  try:
    endpoints = select_ec25_modem(discover_ec25_modems(), args.usb_device)
    result = probe_modem(endpoints, timeout=args.timeout, keep_radio_off=args.keep_radio_off)
  except (ValueError, ProbeError) as e:
    print(f"ec25 probe failed: {e}", file=sys.stderr)
    if isinstance(e, ProbeError):
      for attempt in e.attempts:
        print(f"  {attempt.port}: {attempt.error}", file=sys.stderr)
    return 1

  if args.json:
    print(json.dumps(result.as_dict(), indent=2, sort_keys=True))
  else:
    _print_human(result)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
