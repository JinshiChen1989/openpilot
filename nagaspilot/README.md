# NagasPilot NGP10

NGP10 is the comma 3-focused, minimized successor path derived from EOP10 and
kept feature-compatible with EDP10 where the hardware can support it.

Runtime features are integrated directly into the normal openpilot processes.
There is no parallel control daemon. Feature settings use the `ngp_` prefix,
while NGP-owned controller modules live under `nagaspilot/controls/` and enter
the upstream runtime through small integration hooks.

NGP10 runs on real comma 3 hardware, which already ships an onboard panda, so
it drives the car through opendbc directly: the stock openpilot car stack
(`selfdrive/car/card.py`) plus the opendbc vehicle port, with the panda running
that port's safety mode. It does not use the BrownPanda gateway.

BrownPanda remains EOP10's arrangement, where it is needed - EOP10 runs on a
homegrown RK3588 board with no comma panda, so it reaches the vehicle over
BrownPanda's Tesla-compatible party bus 0 / autopilot-party bus 2 through
`system/socketd`. Adding a second gateway in front of a panda that is already
present would be redundant hardware on a comma 3, and would put two safety
layers with different limits in series.

Start with [`docs/00_READ_ORDER.md`](docs/00_READ_ORDER.md).
