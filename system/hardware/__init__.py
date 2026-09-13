#!/usr/bin/env python3
"""
Hardware Abstraction Layer for Rockchip platforms: RK3588 (ExoPilot 01M) and
RK3576 (ExoPilot 02M).

Provides platform detection, hardware access, and device configuration.
"""

from __future__ import annotations

from typing import cast

# Core hardware exports
from openpilot.system.hardware.base import HardwareBase, HardwareCapability
from openpilot.system.hardware.registry import PlatformRegistry

# Platform exports
from openpilot.system.hardware.rockchip_base import RockchipHardware
from openpilot.system.hardware.rk3588.hardware import RK3588Hardware
from openpilot.system.hardware.rk3576.hardware import RK3576Hardware

# Singleton hardware instance
HARDWARE = cast(HardwareBase, PlatformRegistry.create())

# Platform detection flags
RK3588 = HARDWARE.get_device_type() == 'rk3588'
RK3588_DETECTED = RK3588
RK3576 = HARDWARE.get_device_type() == 'rk3576'
RK3576_DETECTED = RK3576

# Combined Rockchip platform flag. Asks the shared base, not RK3588Hardware:
# RK3576Hardware used to subclass RK3588Hardware, so "is this a Rockchip
# board" was answered by "is this an 01M" -- true only by accident of the
# class hierarchy, and false the moment the two became siblings. Testing the
# base also picks up any future Rockchip board for free, which an enumerated
# `RK3588 or RK3576` check would silently miss.
ROCKCHIP = isinstance(HARDWARE, RockchipHardware)

# Legacy compatibility alias (TICI = any Rockchip platform)
TICI = ROCKCHIP

# Platform detection helper
PC = not ROCKCHIP

# Speaker detection (for alert tones, TTS output)
HAS_SPEAKER = HARDWARE.has_speaker() if hasattr(HARDWARE, 'has_speaker') else False

# Voice input detection (mic + PCIe accelerator for Whisper STT) — per-platform:
# False on RK3588 (no on-board mic), True on RK3576 (has a mic array)
HAS_VOICE_INPUT = HARDWARE.has_voice_input() if hasattr(HARDWARE, 'has_voice_input') else False

# Side camera detection (UVC via USB 3.0 hub RTS5411S)
HAS_SIDE_CAMERAS = HARDWARE.has_side_cameras() if hasattr(HARDWARE, 'has_side_cameras') else False

# Rear camera detection (USB UVC)
HAS_REAR_CAMERA = HARDWARE.has_rear_camera() if hasattr(HARDWARE, 'has_rear_camera') else False

__all__ = [
    # Core
    'HARDWARE',
    'HardwareBase',
    'HardwareCapability',
    'PlatformRegistry',
    # Platforms
    'RK3588',
    'RK3588_DETECTED',
    'RK3588Hardware',
    'RK3576',
    'RK3576_DETECTED',
    'RK3576Hardware',
    # Combined flags
    'ROCKCHIP',
    'TICI',  # Legacy compatibility
    # Detection
    'PC',
    'HAS_SPEAKER',
    'HAS_VOICE_INPUT',
    'HAS_SIDE_CAMERAS',
    'HAS_REAR_CAMERA',
]
