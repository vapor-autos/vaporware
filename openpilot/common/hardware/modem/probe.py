from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from openpilot.common.hardware.modem.at import AtChannel, AtCommandError, AtError, AtResponse
from openpilot.common.hardware.modem.discovery import ModemEndpoints, modem_manager_state


RADIO_STATES = {
  0: "minimum",
  1: "full",
  4: "airplane",
}

USB_MODES = {
  0: "qmi",
  1: "ecm",
  2: "mbim",
}


@dataclass(frozen=True)
class PortAttempt:
  port: Path
  error: str | None

  def as_dict(self) -> dict[str, str | None]:
    return {"port": str(self.port), "error": self.error}


@dataclass(frozen=True)
class ProbeResult:
  endpoints: ModemEndpoints
  selected_at_port: Path
  port_attempts: tuple[PortAttempt, ...]
  modem_manager: str
  manufacturer: str | None
  model: str | None
  firmware: str | None
  sim_state: str
  radio_state: int | None
  radio_state_name: str
  usb_mode: int | None
  usb_mode_name: str
  radio_changed_to_off: bool
  query_errors: dict[str, str]

  def as_dict(self) -> dict[str, Any]:
    return {
      "endpoints": self.endpoints.as_dict(),
      "selected_at_port": str(self.selected_at_port),
      "port_attempts": [attempt.as_dict() for attempt in self.port_attempts],
      "modem_manager": self.modem_manager,
      "manufacturer": self.manufacturer,
      "model": self.model,
      "firmware": self.firmware,
      "sim_state": self.sim_state,
      "radio_state": self.radio_state,
      "radio_state_name": self.radio_state_name,
      "usb_mode": self.usb_mode,
      "usb_mode_name": self.usb_mode_name,
      "radio_changed_to_off": self.radio_changed_to_off,
      "query_errors": self.query_errors,
    }


class ProbeError(Exception):
  def __init__(self, message: str, attempts: tuple[PortAttempt, ...] = ()):
    super().__init__(message)
    self.attempts = attempts


ChannelFactory = Callable[..., AtChannel]


def _query(channel: AtChannel, command: str, errors: dict[str, str], timeout: float | None = None) -> AtResponse | None:
  try:
    return channel.command(command, timeout=timeout)
  except AtCommandError as e:
    errors[command] = e.response.final
    return e.response
  except AtError as e:
    errors[command] = str(e)
    return None


def _plain_value(response: AtResponse | None) -> str | None:
  if response is None:
    return None
  for line in response.lines:
    if not line.startswith("+"):
      return line
  return None


def _int_value(response: AtResponse | None, prefix: str) -> int | None:
  if response is None:
    return None
  value = response.value(prefix)
  if value is None:
    return None
  try:
    return int(value.split(",", 1)[0].strip())
  except ValueError:
    return None


def _sim_state(cpin: AtResponse | None, qsimstat: AtResponse | None) -> str:
  if cpin is not None:
    value = cpin.value("+CPIN:")
    if value == "READY":
      return "ready"
    if value:
      return "locked"
    if cpin.final.startswith("+CME ERROR") and cpin.final.rsplit(":", 1)[-1].strip() == "10":
      return "missing"

  if qsimstat is not None:
    value = qsimstat.value("+QSIMSTAT:")
    if value is not None:
      fields = [field.strip() for field in value.split(",")]
      if len(fields) >= 2:
        return "ready" if fields[1] == "1" else "missing"
  return "unknown"


def _probe_channel(
  channel: AtChannel,
  endpoints: ModemEndpoints,
  selected_port: Path,
  attempts: tuple[PortAttempt, ...],
  keep_radio_off: bool,
) -> ProbeResult:
  errors: dict[str, str] = {}
  manufacturer = _plain_value(_query(channel, "AT+CGMI", errors))
  model = _plain_value(_query(channel, "AT+GMM", errors))
  firmware = _plain_value(_query(channel, "AT+GMR", errors))
  cfun = _query(channel, "AT+CFUN?", errors)
  cpin = _query(channel, "AT+CPIN?", errors)
  qsimstat = _query(channel, "AT+QSIMSTAT?", errors)
  usbnet = _query(channel, 'AT+QCFG="usbnet"', errors)

  radio_state = _int_value(cfun, "+CFUN:")
  radio_changed_to_off = False
  if keep_radio_off and radio_state != 0:
    response = _query(channel, "AT+CFUN=0", errors, timeout=15)
    if response is not None and response.final == "OK":
      radio_changed_to_off = True
      cfun = _query(channel, "AT+CFUN?", errors)
      radio_state = _int_value(cfun, "+CFUN:")

  usb_mode = _int_value(usbnet, "+QCFG:")
  if usb_mode is None and usbnet is not None:
    value = usbnet.value('+QCFG: "usbnet",')
    try:
      usb_mode = int(value) if value is not None else None
    except ValueError:
      usb_mode = None

  return ProbeResult(
    endpoints=endpoints,
    selected_at_port=selected_port,
    port_attempts=attempts,
    modem_manager=modem_manager_state(),
    manufacturer=manufacturer,
    model=model,
    firmware=firmware,
    sim_state=_sim_state(cpin, qsimstat),
    radio_state=radio_state,
    radio_state_name=RADIO_STATES.get(radio_state, "unknown"),
    usb_mode=usb_mode,
    usb_mode_name=USB_MODES.get(usb_mode, "unknown"),
    radio_changed_to_off=radio_changed_to_off,
    query_errors=errors,
  )


def probe_modem(
  endpoints: ModemEndpoints,
  timeout: float = 3.0,
  keep_radio_off: bool = False,
  channel_factory: ChannelFactory = AtChannel,
) -> ProbeResult:
  candidates = endpoints.at_candidates
  if not candidates:
    raise ProbeError("EC25 was found, but no AT ports are available")

  attempts: list[PortAttempt] = []
  for port in candidates:
    try:
      with channel_factory(port=port, timeout=timeout) as channel:
        channel.command("AT")
        attempts.append(PortAttempt(port=port, error=None))
        return _probe_channel(channel, endpoints, port, tuple(attempts), keep_radio_off)
    except AtError as e:
      attempts.append(PortAttempt(port=port, error=str(e)))

  raise ProbeError("none of the EC25 AT ports could be opened and queried", tuple(attempts))
