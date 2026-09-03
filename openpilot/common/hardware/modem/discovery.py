import subprocess
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any


QUECTEL_VENDOR_ID = "2c7c"
EC25_PRODUCT_ID = "0125"


def _read_text(path: Path) -> str | None:
  try:
    return path.read_text().strip()
  except (FileNotFoundError, OSError):
    return None


def _find_path(interface_path: Path | None, patterns: tuple[str, ...], root: Path) -> Path | None:
  if interface_path is None:
    return None

  for pattern in patterns:
    for candidate in sorted(interface_path.glob(pattern)):
      path = root / candidate.name
      if path.exists():
        return path
  return None


def _find_name(interface_path: Path | None, patterns: tuple[str, ...]) -> str | None:
  if interface_path is None:
    return None

  for pattern in patterns:
    candidates = sorted(interface_path.glob(pattern))
    if candidates:
      return candidates[0].name
  return None


@dataclass(frozen=True)
class ModemEndpoints:
  sysfs_name: str
  sysfs_path: Path
  vendor_id: str
  product_id: str
  manufacturer: str | None
  product: str | None
  usb_speed_mbps: int | None
  diagnostic_port: Path | None
  gnss_port: Path | None
  primary_at_port: Path | None
  secondary_at_port: Path | None
  qmi_control: Path | None
  network_interface: str | None

  @property
  def at_candidates(self) -> tuple[Path, ...]:
    return tuple(p for p in (self.primary_at_port, self.secondary_at_port) if p is not None)

  def as_dict(self) -> dict[str, Any]:
    return {
      "sysfs_name": self.sysfs_name,
      "sysfs_path": str(self.sysfs_path),
      "vendor_id": self.vendor_id,
      "product_id": self.product_id,
      "manufacturer": self.manufacturer,
      "product": self.product,
      "usb_speed_mbps": self.usb_speed_mbps,
      "diagnostic_port": str(self.diagnostic_port) if self.diagnostic_port else None,
      "gnss_port": str(self.gnss_port) if self.gnss_port else None,
      "primary_at_port": str(self.primary_at_port) if self.primary_at_port else None,
      "secondary_at_port": str(self.secondary_at_port) if self.secondary_at_port else None,
      "qmi_control": str(self.qmi_control) if self.qmi_control else None,
      "network_interface": self.network_interface,
    }


def _usb_interfaces(devices_path: Path, device_name: str) -> dict[int, Path]:
  interfaces: dict[int, Path] = {}
  for candidate in devices_path.glob(f"{device_name}:*"):
    number = _read_text(candidate / "bInterfaceNumber")
    if number is None:
      continue
    try:
      interfaces[int(number, 16)] = candidate
    except ValueError:
      continue
  return interfaces


def _parse_usb_speed(value: str | None) -> int | None:
  if value is None:
    return None
  try:
    return int(float(value))
  except ValueError:
    return None


def discover_ec25_modems(sysfs_root: Path = Path("/sys"), dev_root: Path = Path("/dev")) -> list[ModemEndpoints]:
  devices_path = sysfs_root / "bus/usb/devices"
  if not devices_path.is_dir():
    return []

  modems = []
  for device in sorted(devices_path.iterdir()):
    vendor_id = (_read_text(device / "idVendor") or "").lower()
    product_id = (_read_text(device / "idProduct") or "").lower()
    if vendor_id != QUECTEL_VENDOR_ID or product_id != EC25_PRODUCT_ID:
      continue

    interfaces = _usb_interfaces(devices_path, device.name)
    modems.append(
      ModemEndpoints(
        sysfs_name=device.name,
        sysfs_path=device,
        vendor_id=vendor_id,
        product_id=product_id,
        manufacturer=_read_text(device / "manufacturer"),
        product=_read_text(device / "product"),
        usb_speed_mbps=_parse_usb_speed(_read_text(device / "speed")),
        diagnostic_port=_find_path(interfaces.get(0), ("ttyUSB*", "tty/ttyUSB*"), dev_root),
        gnss_port=_find_path(interfaces.get(1), ("ttyUSB*", "tty/ttyUSB*"), dev_root),
        primary_at_port=_find_path(interfaces.get(2), ("ttyUSB*", "tty/ttyUSB*"), dev_root),
        secondary_at_port=_find_path(interfaces.get(3), ("ttyUSB*", "tty/ttyUSB*"), dev_root),
        qmi_control=_find_path(interfaces.get(4), ("usbmisc/cdc-wdm*", "cdc-wdm*"), dev_root),
        network_interface=_find_name(interfaces.get(4), ("net/*",)),
      )
    )
  return modems


def select_ec25_modem(modems: list[ModemEndpoints], sysfs_name: str | None = None) -> ModemEndpoints:
  if sysfs_name is not None:
    matches = [modem for modem in modems if modem.sysfs_name == sysfs_name]
    if not matches:
      raise ValueError(f"no EC25 found at USB device {sysfs_name!r}")
    return matches[0]

  if not modems:
    raise ValueError(f"no Quectel EC25 USB modem ({QUECTEL_VENDOR_ID}:{EC25_PRODUCT_ID}) found")
  if len(modems) > 1:
    names = ", ".join(modem.sysfs_name for modem in modems)
    raise ValueError(f"multiple EC25 modems found ({names}); select one with --usb-device")
  return modems[0]


RunCommand = Callable[..., subprocess.CompletedProcess[str]]


def modem_manager_state(run: RunCommand = subprocess.run) -> str:
  try:
    result = run(
      ["systemctl", "is-active", "ModemManager"],
      capture_output=True,
      text=True,
      timeout=2,
      check=False,
    )
  except (FileNotFoundError, subprocess.SubprocessError, OSError):
    return "unavailable"

  state = result.stdout.strip()
  if state:
    return state
  return "active" if result.returncode == 0 else "unknown"
