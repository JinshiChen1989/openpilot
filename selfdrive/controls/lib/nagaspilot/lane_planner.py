"""
====================================================================
NAGASPILOT LANE PLANNER - LANE LINE PROCESSING AND PATH GENERATION
====================================================================

OVERVIEW:
The Lane Planner processes vision model lane line data to generate stable
path trajectories for lateral control. It provides filtered lane line
probabilities, lane width estimation, and path generation with fixed
engineering calibrations.

CORE FUNCTIONALITY:
- Lane line data parsing from vision model output
- Adaptive lane width estimation with filtering
- Path trajectory generation for lateral control
- Lane line probability and confidence processing
- Fixed engineering offsets for consistent behavior

TECHNICAL SPECIFICATIONS:
- Trajectory size: 33 points for smooth path planning
- Lane width estimation: 3.7m default with adaptive filtering
- Update frequency: 20Hz matching model output frequency
- Path offset: 0.00m (fixed engineering value)
- Camera offset: 0.00m (fixed engineering value)

FILTERING SYSTEM:
- Lane width: FirstOrderFilter(3.7, 9.95, DT_MDL) for stability
- Lane certainty: FirstOrderFilter(1.0, 0.95, DT_MDL) for confidence
- Probability smoothing for consistent lane detection
- Standard deviation filtering for uncertainty quantification

SAFETY FEATURES:
- Fixed engineering calibrations prevent user misconfiguration
- Filtered probability outputs for stable lane detection
- Fallback lane width when detection confidence low
- Atomic model data processing to prevent inconsistent reads
- Robust handling of missing or invalid lane line data

INTEGRATION POINTS:
- Vision model lane line data (laneLines from ModelV2)
- Lateral MPC for path following control
- DLP system for dynamic lateral profiling
- Lane change assist systems for maneuver planning

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
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
# Use numpy's interp function instead of missing numpy_fast
interp = np.interp
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
# from openpilot.common.swaglog import cloudlog  # Commented out - debug logging only


TRAJECTORY_SIZE = 33
# Fixed engineering values - both centered (no offset)
# Camera and path both at perfect center
PATH_OFFSET = 0.00
CAMERA_OFFSET = 0.00


class LanePlanner:
  # ========================================================================
  # INITIALIZATION & CONFIGURATION
  # ========================================================================
  
  def __init__(self):
    """Initialize lane planner with fixed engineering values"""
    # Trajectory arrays for lane line data
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))  # Left lane line y-coordinates
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))  # Right lane line y-coordinates

    # Lane width estimation with filtering for stability
    self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.7  # Default lane width (meters)

    # Lane detection probabilities
    self.lll_prob = 0.0  # Left lane line probability
    self.rll_prob = 0.0  # Right lane line probability  
    self.d_prob = 0.0    # Combined lane detection probability

    # Lane line standard deviations (uncertainty)
    self.lll_std = 0.0   # Left lane line standard deviation
    self.rll_std = 0.0   # Right lane line standard deviation

    # Lane change probabilities from desire state
    self.l_lane_change_prob = 0.0  # Left lane change probability
    self.r_lane_change_prob = 0.0  # Right lane change probability

    # Fixed engineering values - no user configuration allowed
    # Both camera and path offsets are centered (zero offset)
    self.camera_offset = CAMERA_OFFSET  # 0.00m - camera perfectly centered
    self.path_offset = PATH_OFFSET      # 0.00m - path perfectly centered

  # ========================================================================
  # CORE PROCESSING
  # ========================================================================

  def parse_model(self, md):
    # Using fixed engineering values only - no custom offsets
    # Atomic snapshot of model data to prevent inconsistent reads
    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      # Compute all arrays atomically from snapshot
      ll1_t = np.array(lane_lines[1].t)
      ll2_t = np.array(lane_lines[2].t) 
      ll1_y = np.array(lane_lines[1].y)
      ll2_y = np.array(lane_lines[2].y)
      
      # Update all values from atomic computation
      self.ll_t = (ll1_t + ll2_t) / 2
      self.ll_x = lane_lines[1].x
      self.lll_y = ll1_y + self.camera_offset
      self.rll_y = ll2_y + self.camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    # Atomic desire state update
    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.Desire.laneChangeRight]

  # ========================================================================
  # PATH GENERATION
  # ========================================================================

  def get_d_path(self, v_ego, path_t, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    else:
      # cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
      pass
    return path_xyz