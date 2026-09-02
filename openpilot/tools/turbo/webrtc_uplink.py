import argparse
import asyncio
import os
import time

import requests

from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.system.webrtc.webrtcd import StreamSession
from openpilot.tools.turbo.webrtc_controls import CerealDatagramProtocol, CerealDataChannelReceiver


CONTROL_UDP_PORT = 8445


class Offer:
  def __init__(
    self,
    body: StreamRequestBody,
    session_id: str | None,
    feedback_udp_endpoint: tuple[str, int] | None = None,
    control_udp_services: list[str] | None = None,
  ):
    self.body = body
    self.session_id = session_id
    self.feedback_udp_endpoint = feedback_udp_endpoint
    self.control_udp_services = [] if control_udp_services is None else control_udp_services


class AnswerRejected(Exception):
  pass


class OfferUnavailable(Exception):
  pass


def join_url(base_url: str, path: str) -> str:
  return f"{base_url.rstrip('/')}/{path.lstrip('/')}"


async def fetch_offer(base_url: str, timeout: float, control_udp_port: int = CONTROL_UDP_PORT) -> Offer:
  def get_offer() -> dict:
    resp = requests.get(join_url(base_url, "/offer"), params={"control_udp_port": control_udp_port}, timeout=timeout)
    if resp.status_code == 409:
      raise OfferUnavailable(resp.text)
    resp.raise_for_status()
    return resp.json()

  payload = await asyncio.to_thread(get_offer)
  session_id = payload.pop("session_id", None)
  feedback_udp_host = payload.pop("feedback_udp_host", "")
  feedback_udp_port = int(payload.pop("feedback_udp_port", 0))
  control_udp_services = payload.pop("control_udp_services", [])
  feedback_udp_endpoint = (feedback_udp_host, feedback_udp_port) if feedback_udp_host and feedback_udp_port > 0 else None
  return Offer(StreamRequestBody(**payload), session_id, feedback_udp_endpoint, control_udp_services)


async def post_answer(base_url: str, session_id: str | None, answer_sdp: str, answer_type: str, timeout: float) -> None:
  def send_answer() -> None:
    payload = {"sdp": answer_sdp, "type": answer_type}
    if session_id is not None:
      payload["session_id"] = session_id
    resp = requests.post(join_url(base_url, "/answer"), json=payload, timeout=timeout)
    if resp.status_code == 409:
      raise AnswerRejected(resp.text)
    resp.raise_for_status()

  await asyncio.to_thread(send_answer)


class ControlDatagramProtocol(CerealDatagramProtocol):
  def __init__(self):
    self.cereal_receiver: CerealDataChannelReceiver | None = None
    super().__init__(lambda: self.cereal_receiver)

  def configure(self, services: list[str]) -> None:
    self.cereal_receiver = CerealDataChannelReceiver(services) if services else None


async def run_once(args: argparse.Namespace, control_protocol: ControlDatagramProtocol) -> None:
  offer = await fetch_offer(args.signaling_url, args.http_timeout, args.control_udp_port)
  body = offer.body
  print(f"received offer session={offer.session_id or 'unknown'} cameras={','.join(body.cameras or [body.init_camera])}", flush=True)
  control_protocol.configure(offer.control_udp_services)
  if offer.control_udp_services:
    print(f"udp controls receiving={','.join(offer.control_udp_services)}", flush=True)

  session = StreamSession(body, feedback_udp_endpoint=offer.feedback_udp_endpoint)
  try:
    answer = await session.get_answer()
    await post_answer(args.signaling_url, offer.session_id, answer.sdp, answer.type, args.http_timeout)
    print(f"posted answer session={offer.session_id or 'unknown'}", flush=True)
    session.start()
    assert session.run_task is not None
    await session.run_task
  finally:
    await session.stop()
    control_protocol.configure([])


async def run(args: argparse.Namespace) -> None:
  loop = asyncio.get_running_loop()
  transport, control_protocol = await loop.create_datagram_endpoint(
    ControlDatagramProtocol,
    local_addr=(args.control_udp_host, args.control_udp_port),
  )
  print(f"udp controls listening on {args.control_udp_host}:{args.control_udp_port}", flush=True)
  try:
    while True:
      start = time.monotonic()
      try:
        await run_once(args, control_protocol)
      except asyncio.CancelledError:
        raise
      except Exception as e:
        print(f"uplink session failed: {type(e).__name__}: {e}", flush=True)

      elapsed = time.monotonic() - start
      sleep_s = args.retry_delay if elapsed >= args.retry_delay else args.retry_delay - elapsed
      await asyncio.sleep(sleep_s)
  finally:
    transport.close()


def main() -> None:
  parser = argparse.ArgumentParser(description="UGV outbound WebRTC signaling client")
  parser.add_argument("--signaling-url", default=os.getenv("GCS_SIGNALING_URL", "http://127.0.0.1:8443"), help="GCS signaling base URL")
  parser.add_argument("--http-timeout", type=float, default=20.0, help="HTTP request timeout in seconds")
  parser.add_argument("--retry-delay", type=float, default=2.0, help="delay between reconnect attempts")
  parser.add_argument("--control-udp-host", default=os.getenv("TURBO_UGV_CONTROL_UDP_HOST", "0.0.0.0"), help="host for latest-state UDP controls")
  parser.add_argument(
    "--control-udp-port",
    type=int,
    default=int(os.getenv("TURBO_UGV_CONTROL_UDP_PORT", str(CONTROL_UDP_PORT))),
    help="port for latest-state UDP controls",
  )
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
