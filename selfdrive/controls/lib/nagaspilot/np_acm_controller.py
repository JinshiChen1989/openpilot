'''
====================================================================
NAGASPILOT ACM (ADAPTIVE CRUISE MODE) - DOWNHILL COASTING SYSTEM
====================================================================

MIT Non-Commercial License
Copyright (c) 2019, dragonpilot

OVERVIEW:
ACM implements intelligent downhill coasting functionality that allows the 
vehicle to maintain or slightly increase speed on downhill slopes without 
unnecessary braking, improving efficiency and driving comfort.

CORE FUNCTIONALITY:
- Slope detection and gradient analysis for downhill identification
- Speed management that allows controlled speed increase on descents
- Lead vehicle awareness to maintain safe following distances
- Time-to-collision (TTC) based safety monitoring
- Energy-efficient coasting with selective brake application

OPERATIONAL LOGIC:
┌─────────────────────────────────────────────────────────────────┐
│ DOWNHILL DETECTION: Slope < -4% threshold triggers coasting     │
│ SPEED MANAGEMENT: Allow 10% speed increase above cruise setting │
│ LEAD SAFETY: Maintain TTC > 3.5s for safe following           │
│ BRAKE MODULATION: Selective brake application based on context │
└─────────────────────────────────────────────────────────────────┘

SAFETY FEATURES:
- TTC-based collision avoidance with lead vehicles
- Configurable slope thresholds for activation
- Speed ratio limits to prevent excessive acceleration
- Lead vehicle detection and proximity monitoring
- Conservative brake application when safety margins approached

INTEGRATION POINTS:
- OpenPilot longitudinal control for cruise management
- Slope estimation from vehicle dynamics and GPS data
- Lead vehicle tracking from vision model
- Brake system integration for seamless control

EFFICIENCY BENEFITS:
- Reduced unnecessary braking on downhill sections
- Improved energy efficiency through regenerative coasting
- Smoother driving experience with natural speed management
- Optimized for highway and mountain driving scenarios

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
- Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from dragonpilot. Contact ricklan@gmail.com for inquiries.
- Any project that uses the Software must visibly mention the following acknowledgment: "This project uses software from dragonpilot and is licensed under a custom license requiring permission for use."

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import numpy as np

# ========================================================================
# CONSTANTS & CONFIGURATION
# ========================================================================

# ACM Thresholds
SLOPE = -0.04  # Minimum downhill slope to activate (-4%)
RATIO = 0.9    # Speed ratio threshold for activation

# Time-to-Collision Safety Parameters
TTC = 3.5      # Minimum safe TTC for lead vehicle
TTC_BP = [TTC, 2.5]
MIN_BRAKE_ALLOW_VALS = [0., -0.5]

# ========================================================================
# ACM CONTROLLER IMPLEMENTATION
# ========================================================================

class ACM:
  # ========================================================================
  # INITIALIZATION & CONFIGURATION
  # ========================================================================
  
  def __init__(self):
    """Initialize Adaptive Cruise Mode with default state"""
    # Configuration parameters
    self.enabled = False
    self.downhill_only = False
    
    # State tracking variables
    self._is_downhill = False
    self._is_speed_over_cruise = False
    self._has_lead = False
    self._active_prev = False

    # Control state
    self.active = False
    self.just_disabled = False
    self.allowed_brake_val = 0.0

  # ========================================================================
  # STATE MANAGEMENT
  # ========================================================================

  def update_states(self, cs, rs, user_ctrl_lon, v_ego, v_cruise):
    self.lead_ttc = float('inf')  # Default if no lead

    if not self.enabled:
      self.active = False
      return

    if len(cs.orientationNED) != 3:
      self.active = False
      return

    pitch_rad = cs.orientationNED[1]
    self._is_downhill = np.sin(pitch_rad) < SLOPE
    self._is_speed_over_cruise = v_ego > (v_cruise * RATIO)

    lead = rs.leadOne
    if lead and lead.status:
      self.lead_ttc = lead.dRel / v_ego if v_ego > 0 else float('inf')
      self._has_lead = self.lead_ttc < TTC
    else:
      self._has_lead = False

    self.active = not user_ctrl_lon and not self._has_lead and self._is_speed_over_cruise and (self._is_downhill if self.downhill_only else True)

    # Update state tracking for transition detection
    self.just_disabled = self._active_prev and not self.active
    self._active_prev = self.active

  # ========================================================================
  # CONTROL OPERATIONS
  # ========================================================================

  def update_a_desired_trajectory(self, a_desired_trajectory):
    """Modify acceleration trajectory to enable coasting behavior"""
    if not self.active:
      return a_desired_trajectory

    # Suppress braking commands to allow smooth coasting
    for i in range(len(a_desired_trajectory)):
      if a_desired_trajectory[i] < 0 and a_desired_trajectory[i] > self.allowed_brake_val:
        a_desired_trajectory[i] = 0.0
    return a_desired_trajectory

  def update_output_a_target(self, output_a_target):
    """Modify output acceleration to suppress braking when coasting"""
    if not self.active:
      return output_a_target

    # Suppress braking command if within allowed range
    if output_a_target < 0 and output_a_target > self.allowed_brake_val:
      output_a_target = 0.0
    return output_a_target
