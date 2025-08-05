#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT GRADIENT COMPENSATION FACTOR (GCF) - SLOPE-AWARE SPEED CONTROL
====================================================================

OVERVIEW:
GCF provides intelligent gradient-aware speed control for VTSC and MTSC
controllers, using high-quality IMU data from locationd's Extended Kalman
Filter to detect road slopes and automatically adjust cruise speeds for
optimal comfort and efficiency on hills.

CORE FUNCTIONALITY:
- Slope detection using calibrated pitch angle measurements
- 5-second averaging window for stable gradient detection
- Adaptive speed reduction based on slope severity
- Shared state management between multiple controllers
- Real-time gradient analysis with noise filtering

OPERATIONAL LOGIC:
┌─────────────────────────────────────────────────────────────────┐
│ IMU DATA: Hardware IMU → sensord → locationd Kalman Filter     │
│ PITCH EXTRACTION: calibratedOrientationNED.value[1] (radians)  │
│ SLOPE AVERAGING: 5-second rolling window for stability         │
│ THRESHOLD DETECTION: 2° minimum for gradient significance      │
│ SPEED ADJUSTMENT: Up to 20% reduction on steep grades (6°+)    │
└─────────────────────────────────────────────────────────────────┘

GRADIENT RESPONSE:
- Gentle slopes (2-4°): Minimal speed reduction (5-10%)
- Moderate slopes (4-6°): Progressive reduction (10-15%)
- Steep slopes (6°+): Maximum reduction (up to 20%)
- Downhill sections: Maintained speed with enhanced safety margins

INTEGRATION ARCHITECTURE:
┌─────────────────────────────────────────────────────────────────┐
│                    LocationD (Kalman Filter)                   │
│                            │                                   │
│                            ▼                                   │
│                      GCF Helper                               │
│                     Shared State                              │
│                    ┌─────────────┐                            │
│                    │             │                            │
│                    ▼             ▼                            │
│               VTSC Controller  MTSC Controller                │
│               (Vision Curves) (Map Curves)                    │
└─────────────────────────────────────────────────────────────────┘

SAFETY FEATURES:
- Conservative gradient thresholds prevent false positives
- Smooth filtering eliminates noise-induced speed oscillations
- Maximum speed reduction caps prevent excessive deceleration
- Fail-safe defaults when sensor data unavailable
- Compatible with existing DCP filter layer architecture

TECHNICAL SPECIFICATIONS:
- Pitch angle source: liveLocationKalman.calibratedOrientationNED.value[1]
- Averaging window: 5.0 seconds (100 samples at 20Hz)
- Detection threshold: 2.0° (0.0349 radians, ~3.5% grade)
- Maximum reduction: 20% of target cruise speed
- Update frequency: 20Hz (matching locationd frequency)
- Memory usage: ~400 bytes for pitch buffer storage

MIT Non-Commercial License

Copyright (c) 2019-, rav4kumar, Rick Lan, dragonpilot community, and a number of other of contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only,
subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from dragonpilot. Contact ricklan@gmail.com for inquiries.
 * Any project that uses the Software must visibly mention the following acknowledgment: "This project uses software from dragonpilot and is licensed under a custom license requiring permission for use."

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

from collections import deque
from typing import Optional, Dict, Any
import math
import numpy as np
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

# ========================================================================
# CENTRALIZED LOGGING
# ========================================================================
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for GCF Helper module
np_logger = NpLogger('gcf')

# ========================================================================
# CONSTANTS & CONFIGURATION
# ========================================================================

# Gradient Detection Thresholds
GRADIENT_DETECTION_THRESHOLD = 0.0349  # radians (2.0°, ~3.5% grade)
STEEP_GRADIENT_THRESHOLD = 0.1047      # radians (6.0°, ~10.5% grade)
MAX_SPEED_REDUCTION = 0.8              # Maximum 20% speed reduction
MIN_SPEED_REDUCTION = 0.95             # Minimum 5% speed reduction

# Averaging and Filtering
PITCH_BUFFER_SIZE = 100                # 5 seconds at 20Hz
PITCH_AVERAGING_SECONDS = 5.0          # Averaging window duration
FILTER_TIME_CONSTANT = 2.0             # Seconds for first-order filter

# Safety Limits
MAX_REASONABLE_PITCH = 0.3491          # 20° maximum reasonable pitch angle
MIN_PITCH_VALIDITY = 0.0175            # 1° minimum pitch for validity

# ========================================================================
# STATE MANAGEMENT
# ========================================================================

class GCFState:
  """Global GCF state management shared between VTSC and MTSC controllers"""
  
  def __init__(self):
    """Initialize GCF state with filtering and averaging parameters"""
    # Detection and filtering parameters
    self.slope_detection_window = PITCH_AVERAGING_SECONDS
    self.slope_threshold_rad = GRADIENT_DETECTION_THRESHOLD
    self.max_speed_reduction = MAX_SPEED_REDUCTION
    self.slope_smoothing_factor = 0.2  # First-order filter factor
    
    # Initialize pitch buffer for averaging (20Hz * 5 seconds = 100 samples)
    self.pitch_buffer = deque(maxlen=PITCH_BUFFER_SIZE)
    
    # Initialize slope smoothing filter
    self.slope_filter = FirstOrderFilter(0.0, self.slope_smoothing_factor, DT_MDL)
    
    # Current state variables
    self.current_slope_rad = 0.0      # Current slope in radians
    self.current_slope_deg = 0.0      # Current slope in degrees  
    self.current_slope_percent = 0.0  # Current slope as percentage grade
    self.gradient_factor = 1.0        # Speed modifier (1.0 = no reduction)
    self.last_update_time = 0.0       # Update timing tracking
    
    # Performance and quality metrics
    self.samples_processed = 0
    self.active_gradient_count = 0
    self.max_slope_detected_deg = 0.0
    self.valid_samples = 0
    self.invalid_samples = 0
    
    np_logger.info("GCF state initialized with locationd IMU integration")

# ========================================================================
# GLOBAL STATE INSTANCE
# ========================================================================

# Global GCF state instance (shared between VTSC and MTSC)
# Note: Single-threaded access assumed in OpenPilot control loop
_gcf_state = GCFState()

# ========================================================================
# CORE FUNCTIONALITY
# ========================================================================

def get_gradient_speed_factor(driving_context: Dict[str, Any], params=None, enabled: bool = False) -> float:
    """
    Main GCF helper function for VTSC and MTSC controllers
    
    Args:
        driving_context: Dictionary with 'sm' (SubMaster) containing liveLocationKalman
        params: Params object for reading GCF configuration (optional - reserved for future use)
        enabled: Whether GCF is enabled for this controller
        
    Returns:
        float: Speed reduction factor (0.80-1.0, where 1.0 = no reduction)
    """
    global _gcf_state
    
    # Early exit if GCF disabled
    if not enabled:
        return 1.0
    
    try:
        # Get locationd data from driving context
        sm = driving_context.get('sm')
        if not sm or 'liveLocationKalman' not in sm:
            return _gcf_state.gradient_factor  # Return last valid factor
        
        llk = sm['liveLocationKalman']
        if not hasattr(llk, 'calibratedOrientationNED') or not llk.calibratedOrientationNED.valid:
            return _gcf_state.gradient_factor
            
        if len(llk.calibratedOrientationNED.value) < 2:
            return _gcf_state.gradient_factor
        
        # Extract calibrated pitch angle (radians)
        pitch_angle_rad = llk.calibratedOrientationNED.value[1]
        data_valid = llk.calibratedOrientationNED.valid
        
        # Update slope detection
        return _update_slope_detection(pitch_angle_rad, data_valid)
        
    except Exception as e:
        np_logger.warning(f"GCF helper function error: {e}")
        return _gcf_state.gradient_factor

# ========================================================================
# SLOPE DETECTION & PROCESSING
# ========================================================================

def _update_slope_detection(pitch_angle_rad: float, data_valid: bool = True) -> float:
  """Update slope detection and calculate gradient speed factor"""
  global _gcf_state
  
  # Input validation with locationd data quality checks
  if not data_valid or not isinstance(pitch_angle_rad, (int, float)) or np.isnan(pitch_angle_rad):
    _gcf_state.invalid_samples += 1
    return _gcf_state.gradient_factor  # Return last valid factor
      
  # Sanity check for reasonable pitch values (±20 degrees max)
  if abs(pitch_angle_rad) > MAX_REASONABLE_PITCH:
    _gcf_state.invalid_samples += 1
    np_logger.warning(f"Extreme pitch angle detected: {math.degrees(pitch_angle_rad):.1f}°")
    return _gcf_state.gradient_factor
      
  # Add valid sample to rolling buffer for averaging
  _gcf_state.pitch_buffer.append(float(pitch_angle_rad))
  _gcf_state.samples_processed += 1
  _gcf_state.valid_samples += 1
  
  # Need minimum samples for reliable slope calculation
  min_samples = min(20, len(_gcf_state.pitch_buffer))  # 1 second minimum at 20Hz
  if len(_gcf_state.pitch_buffer) < min_samples:
    return 1.0  # No gradient compensation until sufficient data
  
  # Calculate slope magnitude from recent samples
  slope_raw_rad = _calculate_slope_magnitude()
    
    # Apply smoothing filter
    _gcf_state.slope_filter.update(slope_raw_rad)
    _gcf_state.current_slope_rad = _gcf_state.slope_filter.x
    
    # Convert to degrees and percentage for intuitive understanding
    _gcf_state.current_slope_deg = math.degrees(_gcf_state.current_slope_rad)
    _gcf_state.current_slope_percent = math.tan(_gcf_state.current_slope_rad) * 100
    
    # Update maximum detected slope for debugging
    _gcf_state.max_slope_detected_deg = max(_gcf_state.max_slope_detected_deg, _gcf_state.current_slope_deg)
    
    # Calculate speed reduction factor
    _gcf_state.gradient_factor = _calculate_speed_factor()
    
    # Periodic logging when GCF is active (every ~1 second at 20Hz)
    if _gcf_state.gradient_factor < 0.99:
        _gcf_state.active_gradient_count += 1
        if _gcf_state.active_gradient_count % 20 == 1:  # Log every ~1 second when active
            np_logger.info(f"Slope compensation active: "
                         f"{_gcf_state.current_slope_deg:.1f}° ({_gcf_state.current_slope_percent:.1f}% grade), "
                         f"factor={_gcf_state.gradient_factor:.3f}")
    
    _gcf_state.last_update_time += DT_MDL
    return _gcf_state.gradient_factor


def _calculate_slope_magnitude() -> float:
    """Calculate slope magnitude from rolling buffer of calibrated pitch angles"""
    global _gcf_state
    
    if len(_gcf_state.pitch_buffer) < 5:
        return 0.0
        
    # Convert to numpy for efficient calculation
    samples = np.array(_gcf_state.pitch_buffer)
    
    # Calculate absolute value of mean pitch angle (handles both uphill/downhill)
    slope_magnitude_rad = abs(np.mean(samples))
    
    return slope_magnitude_rad


def _calculate_speed_factor() -> float:
    """Calculate speed reduction factor based on current slope"""
    global _gcf_state
    
    if _gcf_state.current_slope_rad <= _gcf_state.SLOPE_THRESHOLD_RAD:
        return 1.0  # No reduction below threshold
        
    # Linear scaling above threshold
    # 2° (3.5% grade) = 0% reduction, 6° (10.5% grade) = 20% max reduction
    excess_slope_rad = _gcf_state.current_slope_rad - _gcf_state.SLOPE_THRESHOLD_RAD
    max_excess_rad = math.radians(4.0)  # 4 degrees (6° - 2°)
    
    reduction_ratio = min(1.0, excess_slope_rad / max_excess_rad)
    speed_reduction = reduction_ratio * _gcf_state.MAX_SPEED_REDUCTION
    
    speed_factor = 1.0 - speed_reduction
    
    # Ensure factor is within valid range
    return max(1.0 - _gcf_state.MAX_SPEED_REDUCTION, min(1.0, speed_factor))


def get_gcf_status() -> Dict[str, Any]:
    """Get GCF status for debugging and telemetry"""
    global _gcf_state
    
    return {
        'current_slope_deg': round(_gcf_state.current_slope_deg, 2),
        'current_slope_percent': round(_gcf_state.current_slope_percent, 1),
        'gradient_factor': round(_gcf_state.gradient_factor, 4),
        'buffer_size': len(_gcf_state.pitch_buffer),
        'samples_processed': _gcf_state.samples_processed,
        'valid_samples': _gcf_state.valid_samples,
        'invalid_samples': _gcf_state.invalid_samples,
        'active_count': _gcf_state.active_gradient_count,
        'max_slope_deg': round(_gcf_state.max_slope_detected_deg, 2),
        'is_active': _gcf_state.gradient_factor < 0.99,
        'data_quality': round(_gcf_state.valid_samples / max(1, _gcf_state.samples_processed), 3),
        'threshold_deg': _gcf_state.SLOPE_THRESHOLD_DEG,
        'max_reduction': _gcf_state.MAX_SPEED_REDUCTION
    }


def reset_gcf_stats():
    """Reset GCF performance statistics (for testing/debugging)"""
    global _gcf_state
    
    _gcf_state.active_gradient_count = 0
    _gcf_state.max_slope_detected_deg = 0.0
    _gcf_state.samples_processed = 0
    _gcf_state.valid_samples = 0
    _gcf_state.invalid_samples = 0
    
    np_logger.info("Performance statistics reset")


def get_gcf_thresholds() -> Dict[str, float]:
    """Get current GCF thresholds for parameter tuning and debugging"""
    global _gcf_state
    
    return {
        'threshold_deg': _gcf_state.SLOPE_THRESHOLD_DEG,
        'threshold_rad': _gcf_state.SLOPE_THRESHOLD_RAD,  
        'threshold_percent': math.tan(_gcf_state.SLOPE_THRESHOLD_RAD) * 100,
        'max_reduction': _gcf_state.MAX_SPEED_REDUCTION,
        'window_seconds': _gcf_state.SLOPE_DETECTION_WINDOW,
        'smoothing_factor': _gcf_state.SLOPE_SMOOTHING_FACTOR
    }