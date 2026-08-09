import argparse
import asyncio
import os

from openpilot.tools.turbo.webrtc_client import build_offer, parse_cameras, send_livestream_quality
from openpilot.tools.turbo.teleop_metrics import default_latest_json_path, default_metrics_jsonl_path, env_bool
from openpilot.tools.turbo.webrtc_controls import CerealDataChannelReceiver, expand_feedback_services
from openpilot.tools.turbo.webrtc_vipc_publisher import print_stats, publish_stream_to_vipc


async def run(args: argparse.Namespace) -> None:
  cameras = parse_cameras(args.cameras)
  feedback_services = expand_feedback_services(args.feedback_services, args.feedback_profile)
  builder = build_offer(args.host, args.port, cameras, feedback_services=feedback_services)
  if args.quality or feedback_services:
    builder.add_messaging()

  stream = builder.stream()
  feedback_receiver = CerealDataChannelReceiver(feedback_services) if feedback_services else None
  if feedback_receiver is not None:
    stream.set_message_handler(feedback_receiver.receive)
  stats_task = None

  try:
    await stream.start()
    await stream.wait_for_connection()
    print(f"connected cameras={','.join(cameras)} server={args.server}", flush=True)
    if feedback_services:
      print(f"feedback={','.join(feedback_services)}", flush=True)

    if args.quality:
      send_livestream_quality(stream, args.quality)
      print(f"quality={args.quality}", flush=True)

    if args.stats:
      stats_task = asyncio.create_task(print_stats(stream, args.stats_interval, args.stats_file, args.stats_latest_file))

    await publish_stream_to_vipc(stream, cameras, args.server, args.num_buffers, args.duration, args.log_interval)
  finally:
    if stats_task is not None:
      stats_task.cancel()

    pending = [task for task in [stats_task] if task is not None]
    if pending:
      await asyncio.gather(*pending, return_exceptions=True)
    await stream.stop()


def main() -> None:
  parser = argparse.ArgumentParser(description="Receive WebRTC camera tracks and republish them as local VisionIPC streams")
  parser.add_argument("--host", default=os.getenv("TURBO_UGV_IP", "127.0.0.1"), help="UGV/webrtcd host")
  parser.add_argument("--port", type=int, default=5001, help="UGV/webrtcd HTTP signaling port")
  parser.add_argument("--cameras", default=os.getenv("TURBO_GCS_WEBRTC_CAMS", "wideRoad,driver"), help="comma-separated cameras to request")
  parser.add_argument("--server", default="camerad", help="local VisionIPC server name")
  parser.add_argument(
    "--feedback-services",
    default=os.getenv("TURBO_GCS_WEBRTC_FEEDBACK_SERVICES", "carState"),
    help="comma-separated UGV msgq services to receive over the WebRTC data channel and republish locally",
  )
  parser.add_argument(
    "--feedback-profile",
    default=os.getenv("TURBO_GCS_WEBRTC_FEEDBACK_PROFILE", ""),
    help="comma-separated named UGV feedback profiles to receive over the WebRTC data channel",
  )
  parser.add_argument(
    "--quality",
    choices=("low", "med", "high", "auto"),
    default=os.getenv("TURBO_GCS_WEBRTC_QUALITY", "med"),
    help="livestream quality sent over WebRTC data channel",
  )
  parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; <=0 runs until disconnected")
  parser.add_argument("--num-buffers", type=int, default=4, help="VisionIPC buffers per stream")
  parser.add_argument("--log-interval", type=float, default=1.0, help="frame log interval in seconds")
  parser.add_argument(
    "--stats",
    action="store_true",
    default=env_bool("TURBO_GCS_WEBRTC_STATS"),
    help="print periodic WebRTC stats",
  )
  parser.add_argument(
    "--stats-interval",
    type=float,
    default=float(os.getenv("TURBO_GCS_WEBRTC_STATS_INTERVAL", "2.0")),
    help="WebRTC stats log interval in seconds",
  )
  parser.add_argument(
    "--stats-file",
    default=os.getenv("TURBO_GCS_WEBRTC_STATS_FILE") or default_metrics_jsonl_path("gcs_webrtc"),
    help="optional JSONL file for periodic WebRTC stats",
  )
  parser.add_argument(
    "--stats-latest-file",
    default=os.getenv("TURBO_GCS_WEBRTC_STATS_LATEST_FILE") or default_latest_json_path("gcs_webrtc"),
    help="optional JSON file for the latest WebRTC stats snapshot",
  )
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
