"""Tests for the board-resolved HAL module accessor.

Daemons used to import `hal.platform.rk3588_*` by name. That is wrong twice
over: on the other Rockchip board it loads the wrong board's pin map, and on
a branch that does not carry RK3588 at all the import is simply absent. The
accessor makes the module follow whichever board is running.
"""

from __future__ import annotations

import sys
import types

import pytest

from openpilot.system.hardware.base import HardwareBase
from openpilot.system.hardware.pc.hardware import Pc
from openpilot.system.hardware.rockchip_base import RockchipHardware


class _FakeBoard(RockchipHardware):
  HAL_PREFIX = "rk9999"

  def get_device_type(self):
    return "rk9999"


@pytest.fixture
def fake_hal():
  """Install a throwaway `hal.platform.rk9999_pins` for the duration."""
  hal = types.ModuleType("hal")
  platform = types.ModuleType("hal.platform")
  pins = types.ModuleType("hal.platform.rk9999_pins")
  pins.UART = {"RADAR3D": {"device": "/dev/ttyS9", "baud": 115200}}
  hal.platform = platform
  platform.rk9999_pins = pins
  added = {"hal": hal, "hal.platform": platform, "hal.platform.rk9999_pins": pins}
  saved = {k: sys.modules.get(k) for k in added}
  sys.modules.update(added)
  yield pins
  for k, v in saved.items():
    if v is None:
      sys.modules.pop(k, None)
    else:
      sys.modules[k] = v


def test_resolves_the_running_boards_module(fake_hal):
  assert _FakeBoard.hal_module("pins") is fake_hal
  assert _FakeBoard.hal_module("pins").UART["RADAR3D"]["baud"] == 115200


def test_a_module_this_board_does_not_have_is_none_not_an_exception(fake_hal):
  """Boards differ in which hal modules exist. A missing one is an ordinary
  state the caller covers with its in-repo default, not an error."""
  assert _FakeBoard.hal_module("thermal") is None


def test_board_without_a_prefix_resolves_nothing(fake_hal):
  """RockchipHardware itself is abstract w.r.t. board data -- it must not
  accidentally resolve some other board's module."""
  assert RockchipHardware.HAL_PREFIX == ""
  assert RockchipHardware.hal_module("pins") is None


def test_non_rockchip_hardware_answers_none(fake_hal):
  """A dev PC has no hal package at all; daemons must still import."""
  assert Pc.hal_module("pins") is None


def test_every_hardware_class_answers_the_accessor():
  """The hook is on HardwareBase, so no platform can be missing it -- a
  daemon calling HARDWARE.hal_module() must never hit AttributeError."""
  assert hasattr(HardwareBase, "hal_module")
  for cls in (Pc, RockchipHardware, _FakeBoard):
    assert callable(cls.hal_module)


def test_absent_hal_package_is_none_not_importerror():
  """With no `hal` installed at all -- the normal dev-PC and CI state."""
  saved = {k: v for k, v in sys.modules.items() if k == "hal" or k.startswith("hal.")}
  for k in saved:
    del sys.modules[k]
  try:
    assert _FakeBoard.hal_module("pins") is None
  finally:
    sys.modules.update(saved)


def test_the_real_board_declares_a_prefix():
  """Whichever board this branch carries must declare HAL_PREFIX, or every
  daemon silently falls back to its defaults on real hardware."""
  from openpilot.system.hardware.registry import PlatformRegistry
  for name in PlatformRegistry._platforms:
    cls = type(PlatformRegistry.create(name))
    if issubclass(cls, RockchipHardware):
      assert cls.HAL_PREFIX, f"{cls.__name__} declares no HAL_PREFIX"
      assert cls.HAL_PREFIX == cls().get_device_type()
