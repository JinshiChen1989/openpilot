"""Regression tests for system/hardware/__init__.py's platform flags.

HARDWARE/ROCKCHIP/TICI/etc. are computed once at import time from
PlatformRegistry.create(), so exercising a different platform means a fresh
interpreter per case (subprocess), not just re-calling a function -- reload()
would risk stale submodule state (e.g. PlatformRegistry's class-level
_platforms dict persisting across reloads).
"""

from __future__ import annotations

import os
import subprocess
import sys


def _flags_for(hardware_env: str | None) -> dict[str, bool]:
  env = dict(os.environ)
  if hardware_env is None:
    env.pop('HARDWARE', None)
  else:
    env['HARDWARE'] = hardware_env
  code = (
    "from openpilot.system.hardware import RK3588, RK3576, ROCKCHIP, TICI, PC\n" +
    "print(RK3588, RK3576, ROCKCHIP, TICI, PC)"
  )
  result = subprocess.run(
    [sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=30,
    cwd=os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
  )
  assert result.returncode == 0, result.stderr
  values = result.stdout.strip().split()
  keys = ['RK3588', 'RK3576', 'ROCKCHIP', 'TICI', 'PC']
  return dict(zip(keys, (v == 'True' for v in values), strict=True))


def test_rockchip_and_tici_are_true_on_rk3588():
  flags = _flags_for('rk3588')
  assert flags == {'RK3588': True, 'RK3576': False, 'ROCKCHIP': True, 'TICI': True, 'PC': False}


def test_rockchip_and_tici_are_true_on_rk3576():
  """Regression test: before the 2026-08-26 fix, ROCKCHIP/TICI were defined
  as `ROCKCHIP = RK3588` only, so this would have incorrectly been False on
  real RK3576 hardware -- silently disabling EGL rendering
  (cameraview.py), the recordd.py encoding path, and updated.py/conftest.py
  hardware-gated behavior on that platform."""
  flags = _flags_for('rk3576')
  assert flags == {'RK3588': False, 'RK3576': True, 'ROCKCHIP': True, 'TICI': True, 'PC': False}


def test_pc_fallback_when_no_hardware_env():
  flags = _flags_for(None)
  assert flags == {'RK3588': False, 'RK3576': False, 'ROCKCHIP': False, 'TICI': False, 'PC': True}


class TestPlatformIndependence:
  """RK3588 and RK3576 must stay siblings, not parent and child.

  RK3576Hardware used to subclass RK3588Hardware. That made 01M's class
  load-bearing for 02M -- neither board's support could be removed from a
  branch without breaking the other's -- and it made `ROCKCHIP` answer "is
  this an 01M" while claiming to answer "is this a Rockchip board".
  """

  def test_rk3576_does_not_inherit_from_rk3588(self):
    from openpilot.system.hardware.rk3576.hardware import RK3576Hardware
    from openpilot.system.hardware.rk3588.hardware import RK3588Hardware
    assert not issubclass(RK3576Hardware, RK3588Hardware)
    assert not issubclass(RK3588Hardware, RK3576Hardware)

  def test_both_boards_share_the_rockchip_base(self):
    from openpilot.system.hardware.rk3576.hardware import RK3576Hardware
    from openpilot.system.hardware.rk3588.hardware import RK3588Hardware
    from openpilot.system.hardware.rockchip_base import RockchipHardware
    assert issubclass(RK3588Hardware, RockchipHardware)
    assert issubclass(RK3576Hardware, RockchipHardware)

  def test_rockchip_flag_tracks_the_base_not_a_board(self):
    # The check that broke when the two became siblings.
    from openpilot.system.hardware.rk3576.hardware import RK3576Hardware
    from openpilot.system.hardware.rockchip_base import RockchipHardware
    assert isinstance(RK3576Hardware(), RockchipHardware)

  def test_the_shared_half_is_actually_shared(self):
    # Each of these lives on the base now. If one silently reappeared on a
    # board class, the two would have started drifting apart again.
    from openpilot.system.hardware.rk3576.hardware import RK3576Hardware
    from openpilot.system.hardware.rk3588.hardware import RK3588Hardware
    from openpilot.system.hardware.rockchip_base import RockchipHardware
    shared = ("reboot", "shutdown", "get_serial", "get_dongle_id",
              "get_camera_array_config", "get_stereo_baseline_mm",
              "has_side_cameras", "has_rear_camera", "get_cellular_interface",
              "get_modem_type", "get_rga", "get_mpp", "get_rknn")
    for name in shared:
      assert name not in vars(RK3588Hardware), f"{name} should live on the base"
      assert name not in vars(RK3576Hardware), f"{name} should live on the base"
      assert name in vars(RockchipHardware), f"{name} missing from the base"

  def test_each_board_still_owns_what_differs(self):
    from openpilot.system.hardware.rk3576.hardware import RK3576Hardware
    from openpilot.system.hardware.rk3588.hardware import RK3588Hardware
    # Modem power control is the real electrical difference: 01M switches a
    # Mini-PCIe USB-mode mux, 02M bit-bangs the EC25 enable line directly.
    for name in ("modem_power_on", "modem_power_off", "detect",
                 "get_device_type", "get_platform", "get_capabilities"):
      assert name in vars(RK3588Hardware), f"RK3588 should own {name}"
      assert name in vars(RK3576Hardware), f"RK3576 should own {name}"

  def test_camera_arrays_differ_as_the_boards_do(self):
    from openpilot.system.hardware.rk3576.hardware import RK3576Hardware
    from openpilot.system.hardware.rk3588.hardware import RK3588Hardware
    assert not RK3588Hardware.HAS_TELE_ROAD          # 01M: 4 MIPI, no tele
    assert RK3576Hardware.HAS_TELE_ROAD              # 02M: 5 MIPI, with tele
    assert len(RK3588Hardware.MIPI_CAMERA_NAMES) == 4
    assert len(RK3576Hardware.MIPI_CAMERA_NAMES) == 5
