import os
import platform

from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.hardware import PC, TICI
from openpilot.system.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None
TURBO_UGV_IP = os.getenv("TURBO_UGV_IP")
GCS_SIGNALING_PORT = os.getenv("GCS_SIGNALING_PORT")
GCS_SIGNALING_URL = os.getenv("GCS_SIGNALING_URL")
TURBO_GCS_DEBUG_UI = os.getenv("TURBO_GCS_DEBUG_UI") is not None

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return ((started and not params.get_bool("GCS")) or params.get_bool("IsDriverViewEnabled")) or livestream(started, params, CP)

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar and not params.get_bool("GCS")

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar and not params.get_bool("GCS")

def gcs(started: bool, params: Params, CP: car.CarParams) -> bool:
  return params.get_bool("GCS")

def ugv(started: bool, params: Params, CP: car.CarParams) -> bool:
  return params.get_bool("UGV")

def logging(started: bool, params: Params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started: bool, params: Params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox, block=True)
  return started and use_ublox

def joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode") and not params.get_bool("GCS")

def lat_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LateralManeuverMode") and not params.get_bool("GCS")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode") and not params.get_bool("GCS")

def qcomgps(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

def always_run(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not params.get_bool("GCS")

def only_onroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("GCS")

def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started

def livestream(started: bool, params: Params, CP: car.CarParams) -> bool:
  return params.get_bool("IsLiveStreaming") and not params.get_bool("GCS")

def or_(*fns):
  return lambda *args: any(fn(*args) for fn in fns)

def and_(*fns):
  return lambda *args: all(fn(*args) for fn in fns)

def not_(fn):
  return lambda *args: not fn(*args)

procs = [
  DaemonProcess("manage_athenad", "openpilot.system.athena.manage_athenad", "AthenadPid"),

  NativeProcess("loggerd", "openpilot/system/loggerd", ["./loggerd"], logging),
  NativeProcess("encoderd", "openpilot/system/loggerd", ["./encoderd"], only_onroad),
  NativeProcess("stream_encoderd", "openpilot/system/loggerd", ["./encoderd", "--stream"], or_(and_(livestream, not_(iscar)), notcar, ugv)),
  PythonProcess("logmessaged", "openpilot.system.logmessaged", always_run),

  NativeProcess("camerad", "openpilot/system/camerad", ["./camerad"], or_(driverview, livestream), enabled=not WEBCAM),
  PythonProcess("webcamerad", "openpilot.system.camerad.webcam.camerad", driverview, enabled=WEBCAM),
  PythonProcess("proclogd", "openpilot.system.proclogd", only_onroad, enabled=platform.system() != "Darwin"),
  PythonProcess("journald", "openpilot.system.journald", only_onroad, platform.system() != "Darwin"),
  PythonProcess("micd", "openpilot.system.micd", iscar),
  PythonProcess("timed", "openpilot.system.timed", always_run, enabled=not PC),

  PythonProcess("modeld", "openpilot.selfdrive.modeld.modeld", only_onroad),
  PythonProcess("dmonitoringmodeld", "openpilot.selfdrive.modeld.dmonitoringmodeld", driverview, enabled=(WEBCAM or not PC)),

  PythonProcess("sensord", "openpilot.system.sensord.sensord", only_onroad, enabled=not PC),
  PythonProcess("ui", "openpilot.selfdrive.ui.ui", always_run, restart_if_crash=True),
  PythonProcess("soundd", "openpilot.selfdrive.ui.soundd", driverview),
  PythonProcess("locationd", "openpilot.selfdrive.locationd.locationd", only_onroad),
  NativeProcess("_pandad", "openpilot/selfdrive/pandad", ["./pandad"], always_run, enabled=False),
  PythonProcess("calibrationd", "openpilot.selfdrive.locationd.calibrationd", only_onroad),
  PythonProcess("torqued", "openpilot.selfdrive.locationd.torqued", only_onroad),
  PythonProcess("controlsd", "openpilot.selfdrive.controls.controlsd", and_(not_joystick, iscar)),
  PythonProcess("joystickd", "openpilot.tools.joystick.joystickd", or_(joystick, notcar)),
  PythonProcess("selfdrived", "openpilot.selfdrive.selfdrived.selfdrived", only_onroad),
  PythonProcess("card", "openpilot.selfdrive.car.card", only_onroad),
  PythonProcess("deleter", "openpilot.system.loggerd.deleter", always_run),
  PythonProcess("dmonitoringd", "openpilot.selfdrive.monitoring.dmonitoringd", driverview, enabled=(WEBCAM or not PC)),
  PythonProcess("qcomgpsd", "openpilot.system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI),
  PythonProcess("pandad", "openpilot.selfdrive.pandad.pandad", always_run),
  PythonProcess("paramsd", "openpilot.selfdrive.locationd.paramsd", only_onroad),
  PythonProcess("lagd", "openpilot.selfdrive.locationd.lagd", only_onroad),
  PythonProcess("ubloxd", "openpilot.system.ubloxd.ubloxd", ublox, enabled=TICI),
  PythonProcess("pigeond", "openpilot.system.ubloxd.pigeond", ublox, enabled=TICI),
  PythonProcess("plannerd", "openpilot.selfdrive.controls.plannerd", not_long_maneuver),
  PythonProcess("maneuversd", "openpilot.tools.longitudinal_maneuvers.maneuversd", long_maneuver),
  PythonProcess("lateral_maneuversd", "openpilot.tools.lateral_maneuvers.lateral_maneuversd", lat_maneuver),
  PythonProcess("radard", "openpilot.selfdrive.controls.radard", only_onroad),
  PythonProcess("hardwared", "openpilot.system.hardware.hardwared", always_run),
  PythonProcess("modem", "openpilot.common.hardware.tici.modem", always_run, enabled=TICI),
  PythonProcess("tombstoned", "openpilot.system.tombstoned", always_run, enabled=not PC),
  PythonProcess("updated", "openpilot.system.updated.updated", only_offroad, enabled=not PC),
  PythonProcess("uploader", "openpilot.system.loggerd.uploader", always_run),
  PythonProcess("feedbackd", "openpilot.selfdrive.ui.feedback.feedbackd", only_onroad),

  # debug procs
  NativeProcess("bridge", "openpilot/cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "openpilot.system.webrtc.webrtcd", or_(and_(livestream, not_(iscar)), notcar, ugv)),
  PythonProcess("joystick", "openpilot.tools.joystick.joystick_control", and_(joystick, iscar)),

  # turbo gcs procs
  PythonProcess("g29d", "openpilot.tools.turbo.g29d", gcs, enabled=PC),
  PythonProcess("turbo_webrtc_vipc", "openpilot.tools.turbo.webrtc_vipc",
                gcs, enabled=PC and TURBO_UGV_IP is not None and GCS_SIGNALING_PORT is None, restart_if_crash=True),
  PythonProcess("turbo_webrtc_signald", "openpilot.tools.turbo.webrtc_signald",
                gcs, enabled=PC and GCS_SIGNALING_PORT is not None, restart_if_crash=True),
  PythonProcess("gcs_ui", "openpilot.tools.turbo.gcs_ui", gcs, enabled=PC, restart_if_crash=True),
  PythonProcess("gcs_debug_ui", "openpilot.selfdrive.ui.ui", gcs, enabled=PC and TURBO_GCS_DEBUG_UI, restart_if_crash=True),

  # turbo ugv procs
  PythonProcess("turbo_webrtc_uplink", "openpilot.tools.turbo.webrtc_uplink",
                ugv, enabled=not PC and GCS_SIGNALING_URL is not None, restart_if_crash=True),
  PythonProcess("teleopd", "openpilot.tools.turbo.teleopd", ugv, enabled=not PC),
]

managed_processes = {p.name: p for p in procs}
