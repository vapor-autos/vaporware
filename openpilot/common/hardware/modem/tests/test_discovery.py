import subprocess
from pathlib import Path

import pytest

from openpilot.common.hardware.modem.discovery import discover_ec25_modems, modem_manager_state, select_ec25_modem


def _write(path: Path, value: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(value)


def _add_interface(devices: Path, device_name: str, number: int) -> Path:
  interface = devices / f"{device_name}:1.{number}"
  _write(interface / "bInterfaceNumber", f"{number:02x}\n")
  return interface


def test_discover_ec25_by_usb_interface(tmp_path: Path):
  sysfs_root = tmp_path / "sys"
  dev_root = tmp_path / "dev"
  devices = sysfs_root / "bus/usb/devices"
  device = devices / "3-4.4"
  _write(device / "idVendor", "2c7c\n")
  _write(device / "idProduct", "0125\n")
  _write(device / "manufacturer", "Quectel\n")
  _write(device / "product", "EC25-AFXD\n")
  _write(device / "speed", "480\n")

  for number, tty_name in enumerate(("ttyUSB8", "ttyUSB9", "ttyUSB10", "ttyUSB11")):
    interface = _add_interface(devices, device.name, number)
    (interface / tty_name).mkdir(parents=True)
    dev_root.mkdir(parents=True, exist_ok=True)
    (dev_root / tty_name).touch()

  qmi_interface = _add_interface(devices, device.name, 4)
  (qmi_interface / "usbmisc/cdc-wdm4").mkdir(parents=True)
  (qmi_interface / "net/wwan4").mkdir(parents=True)
  (dev_root / "cdc-wdm4").touch()

  modems = discover_ec25_modems(sysfs_root=sysfs_root, dev_root=dev_root)

  assert len(modems) == 1
  modem = modems[0]
  assert modem.sysfs_name == "3-4.4"
  assert modem.product == "EC25-AFXD"
  assert modem.usb_speed_mbps == 480
  assert modem.diagnostic_port == dev_root / "ttyUSB8"
  assert modem.gnss_port == dev_root / "ttyUSB9"
  assert modem.primary_at_port == dev_root / "ttyUSB10"
  assert modem.secondary_at_port == dev_root / "ttyUSB11"
  assert modem.qmi_control == dev_root / "cdc-wdm4"
  assert modem.network_interface == "wwan4"


def test_discovery_ignores_other_usb_products(tmp_path: Path):
  device = tmp_path / "sys/bus/usb/devices/1-1"
  _write(device / "idVendor", "2c7c\n")
  _write(device / "idProduct", "6007\n")

  assert discover_ec25_modems(sysfs_root=tmp_path / "sys", dev_root=tmp_path / "dev") == []


def test_select_reports_no_devices():
  with pytest.raises(ValueError, match="no Quectel EC25"):
    select_ec25_modem([])


def test_modem_manager_state():
  def run(command, **kwargs):
    assert command == ["systemctl", "is-active", "ModemManager"]
    assert kwargs["check"] is False
    return subprocess.CompletedProcess(command, 0, stdout="active\n", stderr="")

  assert modem_manager_state(run=run) == "active"
