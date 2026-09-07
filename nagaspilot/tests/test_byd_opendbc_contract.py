"""NGP10 drives the car through opendbc on the comma 3's onboard panda.

NGP10 runs on real comma 3 hardware, which already ships a panda, so it does
not use the BrownPanda gateway - that is EOP10's arrangement, needed because
EOP10 runs on a homegrown RK3588 board with no comma panda. NGP10 therefore
talks to the vehicle with the stock openpilot car stack (selfdrive/car/card.py)
plus the opendbc BYD port, and the panda runs opendbc's BYD safety mode.

These tests pin that contract: the BYD port and its safety mode must be present
in the pinned opendbc, and NGP10 must not depend on the Tesla-over-BrownPanda
gateway contract it used previously.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_ngp10_uses_the_opendbc_byd_port():
  byd = ROOT / "opendbc" / "car" / "byd"
  values = (byd / "values.py").read_text()
  interface = (byd / "interface.py").read_text()

  assert (byd / "carcontroller.py").exists() or (byd / "cam_lka" / "carcontroller.py").exists()
  assert (byd / "carstate.py").exists() or (byd / "cam_lka" / "carstate.py").exists()
  assert (byd / "fingerprints.py").exists()
  assert "class CAR(Platforms)" in values
  assert "BYD_ATTO3" in values
  assert "CarInterface" in interface


def test_byd_safety_mode_is_available_for_the_onboard_panda():
  # The comma panda enforces the limits, so the mode has to ship with opendbc.
  assert (ROOT / "opendbc" / "safety" / "modes" / "byd.h").exists()


def test_ngp10_does_not_depend_on_the_brownpanda_gateway_contract():
  # BrownPanda presents as a Tesla; NGP10 no longer goes through it, so nothing
  # here may require the Tesla port to carry BrownPanda's radar additions.
  for path in sorted((ROOT / "nagaspilot").rglob("*.py")):
    if path.name == Path(__file__).name:
      continue
    body = path.read_text()
    assert "BROWNPANDA_RADAR_CARS" not in body, f"{path} still requires the BrownPanda radar contract"
    assert "brownpanda_radar_present" not in body, f"{path} still requires the BrownPanda radar contract"
