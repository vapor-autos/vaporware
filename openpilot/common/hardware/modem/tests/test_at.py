from pathlib import Path

import pytest

from openpilot.common.hardware.modem.at import AtChannel, AtCommandError, AtTimeoutError


class FakeSerial:
  def __init__(self, replies: dict[str, list[bytes]], **kwargs):
    self.replies = replies
    self.kwargs = kwargs
    self.lines: list[bytes] = []
    self.closed = False

  def reset_input_buffer(self):
    self.lines = []

  def write(self, data: bytes):
    command = data.decode().rstrip("\r")
    self.lines = list(self.replies.get(command, []))
    return len(data)

  def flush(self):
    pass

  def readline(self) -> bytes:
    return self.lines.pop(0) if self.lines else b""

  def close(self):
    self.closed = True


def _factory(replies: dict[str, list[bytes]]):
  instances = []

  def create(**kwargs):
    instance = FakeSerial(replies, **kwargs)
    instances.append(instance)
    return instance

  return create, instances


def test_at_channel_removes_echo_and_parses_value():
  factory, instances = _factory({"AT+CFUN?": [b"AT+CFUN?\r\n", b"\r\n", b"+CFUN: 0\r\n", b"OK\r\n"]})

  with AtChannel(Path("/dev/fake"), serial_factory=factory) as channel:
    response = channel.command("AT+CFUN?")

  assert response.lines == ("+CFUN: 0",)
  assert response.final == "OK"
  assert response.value("+CFUN:") == "0"
  assert instances[0].kwargs["exclusive"] is True
  assert instances[0].closed is True


def test_at_channel_exposes_cme_error():
  factory, _ = _factory({"AT+CPIN?": [b"+CME ERROR: 10\r\n"]})

  with AtChannel(Path("/dev/fake"), serial_factory=factory) as channel:
    with pytest.raises(AtCommandError) as exc:
      channel.command("AT+CPIN?")

  assert exc.value.response.final == "+CME ERROR: 10"


def test_at_channel_times_out():
  factory, _ = _factory({})

  with AtChannel(Path("/dev/fake"), serial_factory=factory) as channel:
    with pytest.raises(AtTimeoutError):
      channel.command("AT", timeout=0)


def test_at_channel_rejects_embedded_line_ending():
  factory, _ = _factory({})

  with AtChannel(Path("/dev/fake"), serial_factory=factory) as channel:
    with pytest.raises(ValueError, match="line endings"):
      channel.command("AT\rAT+CFUN=1")
