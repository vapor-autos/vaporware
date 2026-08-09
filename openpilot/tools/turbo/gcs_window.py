import ctypes
import os
from dataclasses import dataclass

import pyray as rl


@dataclass(frozen=True)
class MonitorGeometry:
  index: int
  x: int
  y: int
  width: int
  height: int


def monitor_geometry(name: str) -> MonitorGeometry | None:
  rl.init_window(1, 1, "")
  try:
    for i in range(rl.get_monitor_count()):
      if rl.get_monitor_name(i) == name:
        pos = rl.get_monitor_position(i)
        return MonitorGeometry(i, int(pos.x), int(pos.y), rl.get_monitor_width(i), rl.get_monitor_height(i))
  finally:
    rl.close_window()

  return None


def patch_undecorated_window(decorated_env: str) -> None:
  if os.getenv(decorated_env, "0") != "0":
    return

  set_config_flags = rl.set_config_flags

  def _set_config_flags(flags: int) -> None:
    set_config_flags(flags | rl.ConfigFlags.FLAG_WINDOW_HIDDEN | rl.ConfigFlags.FLAG_WINDOW_UNDECORATED)

  rl.set_config_flags = _set_config_flags


def place_window(title: str, monitor: MonitorGeometry | None) -> None:
  if monitor is None:
    rl.clear_window_state(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
    return

  rl.set_window_position(monitor.x, monitor.y)
  _force_unmanaged_geometry(title, monitor)
  rl.clear_window_state(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)


class _XSetWindowAttributes(ctypes.Structure):
  _fields_ = [
    ("background_pixmap", ctypes.c_ulong),
    ("background_pixel", ctypes.c_ulong),
    ("border_pixmap", ctypes.c_ulong),
    ("border_pixel", ctypes.c_ulong),
    ("bit_gravity", ctypes.c_int),
    ("win_gravity", ctypes.c_int),
    ("backing_store", ctypes.c_int),
    ("backing_planes", ctypes.c_ulong),
    ("backing_pixel", ctypes.c_ulong),
    ("save_under", ctypes.c_int),
    ("event_mask", ctypes.c_long),
    ("do_not_propagate_mask", ctypes.c_long),
    ("override_redirect", ctypes.c_int),
    ("colormap", ctypes.c_ulong),
    ("cursor", ctypes.c_ulong),
  ]


def _force_unmanaged_geometry(title: str, monitor: MonitorGeometry) -> None:
  try:
    lib = ctypes.cdll.LoadLibrary("libX11.so.6")
    _configure_xlib(lib)

    display = lib.XOpenDisplay(os.environ["DISPLAY"].encode())
    if not display:
      return

    try:
      root = lib.XDefaultRootWindow(display)
      window = _find_client_window(lib, display, root, title)
      if window is None:
        return

      attributes = _XSetWindowAttributes()
      attributes.override_redirect = 1
      lib.XChangeWindowAttributes(display, window, 1 << 9, ctypes.byref(attributes))
      lib.XMoveResizeWindow(display, window, monitor.x, monitor.y, monitor.width, monitor.height)
      lib.XRaiseWindow(display, window)
      lib.XFlush(display)
    finally:
      lib.XCloseDisplay(display)
  except Exception:
    return


def _configure_xlib(lib) -> None:
  lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
  lib.XOpenDisplay.restype = ctypes.c_void_p
  lib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
  lib.XDefaultRootWindow.restype = ctypes.c_ulong
  lib.XQueryTree.argtypes = [
    ctypes.c_void_p,
    ctypes.c_ulong,
    ctypes.POINTER(ctypes.c_ulong),
    ctypes.POINTER(ctypes.c_ulong),
    ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
    ctypes.POINTER(ctypes.c_uint),
  ]
  lib.XQueryTree.restype = ctypes.c_int
  lib.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]
  lib.XFetchName.restype = ctypes.c_int
  lib.XFree.argtypes = [ctypes.c_void_p]
  lib.XChangeWindowAttributes.argtypes = [
    ctypes.c_void_p,
    ctypes.c_ulong,
    ctypes.c_ulong,
    ctypes.POINTER(_XSetWindowAttributes),
  ]
  lib.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
  lib.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
  lib.XFlush.argtypes = [ctypes.c_void_p]
  lib.XCloseDisplay.argtypes = [ctypes.c_void_p]


def _find_client_window(lib, display, root: int, title: str) -> int | None:
  matches: list[int] = []
  _walk_windows(lib, display, root, title, matches)
  return matches[-1] if matches else None


def _walk_windows(lib, display, window: int, title: str, matches: list[int]) -> None:
  if _window_name(lib, display, window) == title:
    matches.append(window)

  root_return = ctypes.c_ulong()
  parent_return = ctypes.c_ulong()
  children = ctypes.POINTER(ctypes.c_ulong)()
  child_count = ctypes.c_uint()
  if not lib.XQueryTree(display, window, ctypes.byref(root_return), ctypes.byref(parent_return),
                        ctypes.byref(children), ctypes.byref(child_count)):
    return

  try:
    for i in range(child_count.value):
      _walk_windows(lib, display, children[i], title, matches)
  finally:
    if children:
      lib.XFree(children)


def _window_name(lib, display, window: int) -> str:
  name = ctypes.c_char_p()
  if lib.XFetchName(display, window, ctypes.byref(name)) and name.value:
    value = name.value.decode(errors="replace")
    lib.XFree(name)
    return value
  return ""
