#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT ROAD EDGE DETECTOR - BOUNDARY DETECTION SYSTEM
====================================================================

OVERVIEW:
Road Edge Detector provides intelligent road boundary detection using vision
model analysis to identify when the vehicle approaches road edges, enhancing
safety for lane changes and lateral positioning in various road conditions.

CORE FUNCTIONALITY:
- Vision-based road edge probability analysis
- Lane line confidence correlation with edge detection
- Left and right road boundary identification
- Integration with lane change safety systems
- Enhanced support for roads without outer lane markings

DETECTION ALGORITHM:
┌─────────────────────────────────────────────────────────────────┐
│ VISION ANALYSIS: Process road edge standard deviations         │
│ PROBABILITY CORRELATION: Combine with lane line probabilities  │
│ THRESHOLD ANALYSIS: Apply configurable detection thresholds    │
│ EDGE IDENTIFICATION: Determine left/right road boundary status │
│ SAFETY INTEGRATION: Provide data for lane change validation   │
└─────────────────────────────────────────────────────────────────┘

SAFETY APPLICATIONS:
- Enhanced lane change safety validation
- Road boundary awareness for lateral positioning
- Support for countries without consistent outer lane markings
- Integration with LCA system for comprehensive safety checks
- Fallback detection when lane lines are insufficient

INTEGRATION POINTS:
- ModelD: Integrated into model daemon for real-time processing
- LCA System: Provides road edge data for lane change validation
- Vision Model: Uses road edge standard deviations and probabilities
- Safety Systems: Enhances overall lateral control safety margins

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

import numpy as np

# TEMPORARY DEBUG LOGGING:
# To remove all debug logging after testing is complete:
# 1. Comment out sections marked with "DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE"
# 2. Search for "np_logger.debug" and comment out any debug logging calls
# 3. Keep only essential error/warning logs for production
# 4. Note: Uses centralized NpLogger - no direct cloudlog imports needed

# ============================================================================
# CENTRALIZED LOGGING - Uses NagasPilot centralized logging system
# ============================================================================
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for Road Edge Detection module
np_logger = NpLogger('red')
# ============================================================================

NEARSIDE_PROB = 0.2
EDGE_PROB = 0.35

class RoadEdgeDetector:
  def __init__(self, enabled = False):
    self._is_enabled = enabled
    self.left_edge_detected = False
    self.right_edge_detected = False

  def update(self, road_edge_stds, lane_line_probs):
    if not self._is_enabled:
      # ============================================================================
      # DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE
      np_logger.debug("Road edge detection disabled")
      # ============================================================================
      return

    left_road_edge_prob = np.clip(1.0 - road_edge_stds[0], 0.0, 1.0)
    left_lane_nearside_prob = lane_line_probs[0]

    right_road_edge_prob = np.clip(1.0 - road_edge_stds[1], 0.0, 1.0)
    right_lane_nearside_prob = lane_line_probs[3]

    # Store previous states for change detection
    prev_left = self.left_edge_detected
    prev_right = self.right_edge_detected

    self.left_edge_detected = bool(
      left_road_edge_prob > EDGE_PROB and
      left_lane_nearside_prob < NEARSIDE_PROB and
      right_lane_nearside_prob >= left_lane_nearside_prob
    )

    self.right_edge_detected = bool(
      right_road_edge_prob > EDGE_PROB and
      right_lane_nearside_prob < NEARSIDE_PROB and
      left_lane_nearside_prob >= right_lane_nearside_prob
    )

    # ============================================================================
    # DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE
    # Log edge detection changes and detailed analysis
    if self.left_edge_detected != prev_left:
      if self.left_edge_detected:
        np_logger.warning(f"LEFT EDGE DETECTED - road_prob: {left_road_edge_prob:.3f}, "
                        f"lane_prob: {left_lane_nearside_prob:.3f}")
      else:
        np_logger.info(f"Left edge cleared - road_prob: {left_road_edge_prob:.3f}")
    
    if self.right_edge_detected != prev_right:
      if self.right_edge_detected:
        np_logger.warning(f"RIGHT EDGE DETECTED - road_prob: {right_road_edge_prob:.3f}, "
                        f"lane_prob: {right_lane_nearside_prob:.3f}")
      else:
        np_logger.info(f"Right edge cleared - road_prob: {right_road_edge_prob:.3f}")
    
    # Periodic detailed logging for debugging
    import time
    if not hasattr(self, '_last_debug_log') or (time.time() - self._last_debug_log) > 5.0:
      np_logger.debug(f"Edge status - Left: {self.left_edge_detected}, Right: {self.right_edge_detected}, "
                    f"L_road: {left_road_edge_prob:.3f}, R_road: {right_road_edge_prob:.3f}, "
                    f"L_lane: {left_lane_nearside_prob:.3f}, R_lane: {right_lane_nearside_prob:.3f}")
      self._last_debug_log = time.time()
    # ============================================================================

  def set_enabled(self, enabled):
    # ============================================================================
    # DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE
    if enabled != self._is_enabled:
      np_logger.info(f"Road edge detection {'ENABLED' if enabled else 'DISABLED'}")
    # ============================================================================
    self._is_enabled = enabled

  def is_enabled(self):
    return self._is_enabled
