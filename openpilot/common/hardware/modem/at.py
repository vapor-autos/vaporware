import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial


AT_ERROR_FINALS = frozenset({"ERROR", "NO CARRIER", "NO DIALTONE", "BUSY", "NO ANSWER"})


@dataclass(frozen=True)
class AtResponse:
  command: str
  lines: tuple[str, ...]
  final: str

  def value(self, prefix: str) -> str | None:
    for line in self.lines:
      if line.startswith(prefix):
        return line[len(prefix) :].strip()
    return None


class AtError(Exception):
  pass


class AtOpenError(AtError):
  def __init__(self, port: Path, cause: Exception):
    super().__init__(f"cannot open {port}: {cause}")
    self.port = port
    self.cause = cause


class AtTimeoutError(AtError):
  def __init__(self, command: str, lines: tuple[str, ...]):
    super().__init__(f"AT command timed out: {command}")
    self.command = command
    self.lines = lines


class AtCommandError(AtError):
  def __init__(self, response: AtResponse):
    super().__init__(f"AT command failed: {response.command}: {response.final}")
    self.response = response


SerialFactory = Callable[..., Any]


class AtChannel:
  def __init__(
    self,
    port: Path,
    baudrate: int = 115200,
    timeout: float = 3.0,
    serial_factory: SerialFactory = serial.Serial,
  ):
    self.port = port
    self.baudrate = baudrate
    self.timeout = timeout
    self._serial_factory = serial_factory
    self._serial: Any | None = None

  def __enter__(self) -> "AtChannel":
    try:
      self._serial = self._serial_factory(
        port=str(self.port),
        baudrate=self.baudrate,
        timeout=min(0.2, max(0.01, self.timeout)),
        write_timeout=1,
        exclusive=True,
      )
    except (serial.SerialException, OSError) as e:
      raise AtOpenError(self.port, e) from e
    return self

  def __exit__(self, exc_type, exc_value, traceback) -> None:
    if self._serial is not None:
      self._serial.close()
      self._serial = None

  def command(self, command: str, timeout: float | None = None) -> AtResponse:
    if self._serial is None:
      raise AtError("AT channel is not open")
    if "\r" in command or "\n" in command:
      raise ValueError("AT command must not contain line endings")

    command_timeout = self.timeout if timeout is None else timeout
    self._serial.reset_input_buffer()
    try:
      self._serial.write((command + "\r").encode())
      self._serial.flush()
    except (serial.SerialException, OSError) as e:
      raise AtError(f"failed to write {command!r} to {self.port}: {e}") from e

    lines: list[str] = []
    deadline = time.monotonic() + command_timeout
    while time.monotonic() < deadline:
      try:
        raw = self._serial.readline()
      except (serial.SerialException, OSError) as e:
        raise AtError(f"failed to read {command!r} from {self.port}: {e}") from e
      if not raw:
        continue

      line = raw.decode(errors="replace").strip()
      if not line or line == command:
        continue
      if line == "OK":
        return AtResponse(command=command, lines=tuple(lines), final=line)
      if line in AT_ERROR_FINALS or line.startswith(("+CME ERROR", "+CMS ERROR")):
        response = AtResponse(command=command, lines=tuple(lines), final=line)
        raise AtCommandError(response)
      lines.append(line)

    raise AtTimeoutError(command, tuple(lines))
