import errno
from pathlib import Path

from openpilot.common.hardware.modem.at import AtCommandError, AtOpenError, AtResponse
from openpilot.common.hardware.modem.discovery import ModemEndpoints
from openpilot.common.hardware.modem.probe import probe_modem


def _endpoints(tmp_path: Path) -> ModemEndpoints:
  return ModemEndpoints(
    sysfs_name="3-4.4",
    sysfs_path=tmp_path / "sys/bus/usb/devices/3-4.4",
    vendor_id="2c7c",
    product_id="0125",
    manufacturer="Quectel",
    product="EC25-AFXD",
    usb_speed_mbps=480,
    diagnostic_port=tmp_path / "dev/ttyUSB0",
    gnss_port=tmp_path / "dev/ttyUSB1",
    primary_at_port=tmp_path / "dev/ttyUSB2",
    secondary_at_port=tmp_path / "dev/ttyUSB3",
    qmi_control=tmp_path / "dev/cdc-wdm0",
    network_interface="wwan0",
  )


class FakeChannel:
  commands: list[str] = []
  radio_state = 0

  def __init__(self, port: Path, timeout: float):
    self.port = port
    self.timeout = timeout

  def __enter__(self):
    if self.port.name == "ttyUSB2":
      raise AtOpenError(self.port, OSError(errno.EBUSY, "Device or resource busy"))
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    pass

  def command(self, command: str, timeout: float | None = None) -> AtResponse:
    del timeout
    self.commands.append(command)
    if command == "AT+CGMI":
      return AtResponse(command, ("Quectel",), "OK")
    if command == "AT+GMM":
      return AtResponse(command, ("EC25",), "OK")
    if command == "AT+GMR":
      return AtResponse(command, ("EC25AFXDGAR07A01M1G",), "OK")
    if command == "AT+CFUN?":
      return AtResponse(command, (f"+CFUN: {self.radio_state}",), "OK")
    if command == "AT+CFUN=0":
      self.radio_state = 0
      return AtResponse(command, (), "OK")
    if command == "AT+CPIN?":
      raise AtCommandError(AtResponse(command, (), "+CME ERROR: 10"))
    if command == "AT+QSIMSTAT?":
      return AtResponse(command, ("+QSIMSTAT: 0,0",), "OK")
    if command == 'AT+QCFG="usbnet"':
      return AtResponse(command, ('+QCFG: "usbnet",0',), "OK")
    return AtResponse(command, (), "OK")


def test_probe_falls_back_to_secondary_and_detects_missing_sim(tmp_path: Path):
  FakeChannel.commands = []
  FakeChannel.radio_state = 0

  result = probe_modem(_endpoints(tmp_path), channel_factory=FakeChannel)

  assert result.selected_at_port.name == "ttyUSB3"
  assert result.port_attempts[0].port.name == "ttyUSB2"
  assert "Device or resource busy" in (result.port_attempts[0].error or "")
  assert result.manufacturer == "Quectel"
  assert result.model == "EC25"
  assert result.firmware == "EC25AFXDGAR07A01M1G"
  assert result.sim_state == "missing"
  assert result.radio_state == 0
  assert result.usb_mode == 0
  assert result.usb_mode_name == "qmi"
  assert "AT+CGSN" not in FakeChannel.commands
  assert "AT+CIMI" not in FakeChannel.commands
  assert "AT+QCCID" not in FakeChannel.commands


def test_keep_radio_off_only_moves_toward_cfun_zero(tmp_path: Path):
  FakeChannel.commands = []
  FakeChannel.radio_state = 1

  result = probe_modem(_endpoints(tmp_path), keep_radio_off=True, channel_factory=FakeChannel)

  assert result.radio_changed_to_off is True
  assert result.radio_state == 0
  assert FakeChannel.commands.count("AT+CFUN=0") == 1
  assert "AT+CFUN=1" not in FakeChannel.commands
