#!/usr/bin/env python3
import argparse
import asyncio
import os
import time

import requests

from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.system.webrtc.webrtcd import StreamSession


class Offer:
  def __init__(self, body: StreamRequestBody, session_id: str | None):
    self.body = body
    self.session_id = session_id


class AnswerRejected(Exception):
  pass


def join_url(base_url: str, path: str) -> str:
  return f"{base_url.rstrip('/')}/{path.lstrip('/')}"


async def fetch_offer(base_url: str, timeout: float) -> Offer:
  def get_offer() -> dict:
    resp = requests.get(join_url(base_url, "/offer"), timeout=timeout)
    resp.raise_for_status()
    return resp.json()

  payload = await asyncio.to_thread(get_offer)
  session_id = payload.pop("session_id", None)
  return Offer(StreamRequestBody(**payload), session_id)


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


async def run_once(args: argparse.Namespace) -> None:
  offer = await fetch_offer(args.signaling_url, args.http_timeout)
  body = offer.body
  print(f"received offer session={offer.session_id or 'unknown'} cameras={','.join(body.cameras or [body.init_camera])}", flush=True)

  session = StreamSession(body)
  try:
    answer = await session.get_answer()
    await post_answer(args.signaling_url, offer.session_id, answer.sdp, answer.type, args.http_timeout)
    print(f"posted answer session={offer.session_id or 'unknown'}", flush=True)
    session.start()
    assert session.run_task is not None
    await session.run_task
  finally:
    await session.stop()


async def run(args: argparse.Namespace) -> None:
  while True:
    start = time.monotonic()
    try:
      await run_once(args)
    except asyncio.CancelledError:
      raise
    except Exception as e:
      print(f"uplink session failed: {type(e).__name__}: {e}", flush=True)

    elapsed = time.monotonic() - start
    sleep_s = args.retry_delay if elapsed >= args.retry_delay else args.retry_delay - elapsed
    await asyncio.sleep(sleep_s)


def main() -> None:
  parser = argparse.ArgumentParser(description="UGV outbound WebRTC signaling client")
  parser.add_argument("--signaling-url", default=os.getenv("GCS_SIGNALING_URL", "http://127.0.0.1:8443"), help="GCS signaling base URL")
  parser.add_argument("--http-timeout", type=float, default=20.0, help="HTTP request timeout in seconds")
  parser.add_argument("--retry-delay", type=float, default=2.0, help="delay between reconnect attempts")
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
