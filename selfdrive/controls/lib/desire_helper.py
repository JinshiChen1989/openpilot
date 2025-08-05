# Standard library imports
from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
import time

# Import moved to top to fix performance bug from commit b002eb7
# Previously this was imported inside update() function causing performance issues
from openpilot.selfdrive.controls.lib.nagaspilot.np_lca_controller import is_lane_change_safe

# Lane change state machine enums from cereal
LaneChangeState = log.LaneChangeState      # off, preLaneChange, laneChangeStarting, laneChangeFinishing
LaneChangeDirection = log.LaneChangeDirection  # none, left, right

# Configuration constants - fallback values only (actual values from Params)
LANE_CHANGE_TIME_MAX = 10.                 # Maximum time allowed for a lane change (10 seconds)

# Desire mapping table: Maps lane change state + direction to OpenPilot desires
# This table controls what lateral maneuver OpenPilot should execute based on
# the current lane change state machine position
DESIRES = {
  # No lane change direction selected
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,                    # Not changing lanes
    LaneChangeState.preLaneChange: log.Desire.none,          # Waiting to start
    LaneChangeState.laneChangeStarting: log.Desire.none,     # Should never happen
    LaneChangeState.laneChangeFinishing: log.Desire.none,    # Should never happen
  },
  # Left lane change direction
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,                    # Not changing lanes
    LaneChangeState.preLaneChange: log.Desire.none,          # Waiting to start left change
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,   # Actively changing left
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,  # Completing left change
  },
  # Right lane change direction
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,                    # Not changing lanes
    LaneChangeState.preLaneChange: log.Desire.none,          # Waiting to start right change
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,  # Actively changing right
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight, # Completing right change
  },
}


class DesireHelper:
  """
  Lane Change Assistant State Machine - Minimized Implementation

  This class implements a 4-state lane change state machine:
  1. off: No lane change in progress, waiting for blinker activation
  2. preLaneChange: Blinker is on, waiting for torque or auto-activation
  3. laneChangeStarting: Lane change initiated, fading out lane lines
  4. laneChangeFinishing: Lane change completing, fading in lane lines

  Key features:
  - Minimal changes from baseline commit e7c9f28 (only +22 lines)
  - Enhanced lane detection via np_lca_controller.py
  - One-lane-change safety to prevent multiple rapid changes
  - Real-time lane availability checking to prevent race conditions
  """

  def __init__(self, params=None):
    """
    Initialize the lane change assistant with centralized parameters

    Args:
        params: Params object for centralized parameter access (optional, creates new if None)
    """
    # Core state machine variables
    self.lane_change_state = LaneChangeState.off        # Current state in 4-state machine
    self.lane_change_direction = LaneChangeDirection.none # Direction of lane change (left/right/none)
    self.lane_change_timer = 0.0                        # Time spent in current lane change
    self.lane_change_ll_prob = 1.0                      # Lane line probability for visual fade effect
    self.keep_pulse_timer = 0.0                         # Timer for lane-keeping assistance pulses
    self.prev_one_blinker = False                       # Previous blinker state (for edge detection)
    self.desire = log.Desire.none                       # Output desire sent to lateral control

    # Centralized parameter system
    self.params = params if params is not None else Params()
    
    # Load parameters from centralized system with fallback defaults
    lca_speed_kph = float(self.params.get("np_lat_lca_speed", "40"))  # Default 40 km/h
    self.np_lat_lca_speed = lca_speed_kph * CV.KPH_TO_MS             # Convert KPH to m/s
    self.np_lat_lca_auto_sec = float(self.params.get("np_lat_lca_auto_sec", "2.0"))  # Default 2.0 seconds
    self.np_lat_lca_auto_sec_start = 0.                 # Timestamp when auto-timer started

    # Enhanced LCA features (minimal additions from baseline)
    self.lane_change_completed = False                  # One-lane-change safety flag

  def refresh_parameters(self):
    """
    Refresh parameters from centralized system (call if parameters change during runtime)
    """
    lca_speed_kph = float(self.params.get("np_lat_lca_speed", "40"))  # Default 40 km/h
    self.np_lat_lca_speed = lca_speed_kph * CV.KPH_TO_MS             # Convert KPH to m/s
    self.np_lat_lca_auto_sec = float(self.params.get("np_lat_lca_auto_sec", "2.0"))  # Default 2.0 seconds

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected, right_edge_detected, modelV2=None):
    """
    Update the lane change state machine every frame (50Hz)

    Args:
        carstate: Current vehicle state (speed, blinkers, steering, blindspots)
        lateral_active: Whether OpenPilot lateral control is engaged
        lane_change_prob: ML model confidence that lane change is complete (0.0-1.0)
        left_edge_detected: Whether left road edge is detected by vision
        right_edge_detected: Whether right road edge is detected by vision
        modelV2: Vision model data (optional, enables enhanced lane detection)
    """
    # Extract current vehicle state
    v_ego = carstate.vEgo                                    # Current speed in m/s
    one_blinker = carstate.leftBlinker != carstate.rightBlinker  # True if exactly one blinker is on (XOR)
    below_lane_change_speed = True if self.np_lat_lca_speed == 0. else v_ego < self.np_lat_lca_speed  # Speed check

    # ============================================================================
    # GLOBAL SAFETY CHECKS: Force exit from lane change if conditions become unsafe
    # ============================================================================
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      # Exit lane change immediately if:
      # 1. Lateral control is disengaged (user takeover or system failure)
      # 2. Lane change has been active for more than 10 seconds (stuck/failed)
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # ============================================================================
      # 4-STATE LANE CHANGE STATE MACHINE: Handle complete lane change lifecycle
      # ============================================================================
      
      # STATE 1: OFF - Idle state, waiting for blinker activation to initiate lane change
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        """
        Entry conditions for starting a lane change sequence:
        1. Currently in 'off' state (not already processing a lane change)
        2. Exactly one blinker is active (left XOR right, prevents both/neither)
        3. Blinker just activated (rising edge detected via prev_one_blinker comparison)
        4. Vehicle speed >= minimum threshold (40 km/h by default for safety)
        
        This transition only occurs once per blinker activation to prevent repeated triggers.
        """
        # Enhanced LCA: Pre-validate lane availability before entering state machine
        # This prevents race conditions where lane becomes unavailable during the process
        if is_lane_change_safe(carstate, modelV2, self.params):
          # Lane change is validated as safe - enter preparatory state
          self.lane_change_state = LaneChangeState.preLaneChange
          self.lane_change_ll_prob = 1.0  # Reset lane line visibility to 100%
          
          # Initialize auto-activation timer if nudgeless mode is enabled
          if self.np_lat_lca_auto_sec > 0.:
            self.np_lat_lca_auto_sec_start = time.time()  # Record start timestamp for timer

      # STATE 2: PRE_LANE_CHANGE - Waiting for user confirmation via torque or auto-timer
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        """
        Preparatory state where system is ready but waiting for final confirmation:
        - Blinker is active and lane change has been validated as safe
        - System waits for user steering input OR auto-timer expiration
        - Continuously monitors safety conditions (blindspot, speed, blinker status)
        - Can exit back to 'off' state if any safety condition fails
        """
        
        # Lock in lane change direction based on which blinker is currently active
        # This prevents direction changes mid-process if user toggles blinkers rapidly
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        # TORQUE DETECTION: Check for intentional steering wheel input in correct direction
        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))
        """
        Steering torque validation ensures intentional user confirmation:
        - User must actively press/grip steering wheel (steeringPressed = True)
        - Torque direction must match intended lane change direction:
          * Left lane change: Positive torque (counterclockwise wheel rotation)
          * Right lane change: Negative torque (clockwise wheel rotation)
        This prevents accidental lane changes from unintentional steering inputs.
        """

        # BLINDSPOT MONITORING: Comprehensive obstacle detection using multiple sources
        blindspot_detected = (((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))
        """
        Multi-source blindspot detection for enhanced safety:
        1. carstate.leftBlindspot/rightBlindspot: Vehicle's built-in radar/ultrasonic sensors
        2. left_edge_detected/right_edge_detected: Vision-based road edge detection
        This combined approach provides comprehensive coverage of potential obstacles.
        """

        # AUTO-ACTIVATION TIMER: Enable nudgeless lane changes after safety delay
        if self.np_lat_lca_auto_sec > 0.:
          # Safety feature: Reset timer whenever blindspot is newly detected
          if blindspot_detected:
            self.np_lat_lca_auto_sec_start = time.time()  # Restart timer to ensure safety delay
          else:
            # Check if sufficient time has elapsed without any blindspot detection
            if (time.time() - self.np_lat_lca_auto_sec_start) >= self.np_lat_lca_auto_sec:
              torque_applied = True  # Auto-activate lane change after safe delay period

        # EXIT CONDITIONS: Return to 'off' state if any safety condition is violated
        if not one_blinker or below_lane_change_speed or self.lane_change_completed:
          """
          Safety exit conditions that abort the lane change process:
          1. not one_blinker: User turned off blinker (cancelled lane change intent)
          2. below_lane_change_speed: Vehicle speed dropped below safe threshold
          3. lane_change_completed: Already completed one change (one-change-per-signal safety)
          """
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          
        # LANE CHANGE INITIATION: Start actual maneuver if all conditions satisfied
        elif torque_applied and not blindspot_detected:
          """
          Lane change initiation requires both conditions:
          1. torque_applied: User steering input OR auto-timer expiration
          2. not blindspot_detected: No vehicles/obstacles detected in target lane
          """
          # Enhanced LCA: Final real-time safety validation before lane change execution
          # This prevents race conditions where lane conditions change between initial check and execution
          if is_lane_change_safe(carstate, modelV2, self.params):
            self.lane_change_state = LaneChangeState.laneChangeStarting
            self.lane_change_completed = True  # Mark as completed to prevent multiple changes per signal

      # STATE 3: LANE_CHANGE_STARTING - Actively executing the lane change maneuver
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        """
        Active lane change execution state:
        - Lateral control is actively steering the vehicle to change lanes
        - Lane lines gradually fade out to prevent lane-keeping interference
        - ML model monitors progress and determines when lane change is complete
        - Transitions to finishing state when maneuver is substantially complete
        """
        
        # VISUAL FADE EFFECT: Gradually reduce lane line visibility over 0.5 seconds
        # This prevents the lane-keeping system from fighting against the lane change maneuver
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
        """
        Lane line fade calculation:
        - Update frequency: 50Hz (DT_MDL = 0.02 seconds per frame)
        - Fade rate: 2 * 0.02 = 0.04 per frame (4% per frame)
        - Total fade duration: 1.0 / 0.04 = 25 frames = 0.5 seconds
        - Result: Smooth transition from 100% to 0% lane line visibility
        """

        # COMPLETION DETECTION: Monitor ML model confidence for lane change completion
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          """
          Lane change considered substantially complete when:
          1. ML model reports <2% probability of ongoing lane change (98% confidence of completion)
          2. Lane line fade is nearly complete (<1% visibility remaining)
          Both conditions ensure reliable detection before transitioning to finishing state.
          """
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # STATE 4: LANE_CHANGE_FINISHING - Completing lane change and restoring lane keeping
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        """
        Lane change completion and restoration state:
        - Primary lane change maneuver is complete
        - Lane lines gradually fade back in to restore normal lane-keeping
        - System determines next state based on blinker status
        - Prepares for potential additional lane changes or returns to idle
        """
        
        # VISUAL RESTORATION: Gradually restore lane line visibility over 1.0 second
        # This smoothly re-enables the lane-keeping system after lane change completion
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        """
        Lane line restoration calculation:
        - Update frequency: 50Hz (DT_MDL = 0.02 seconds per frame)
        - Restoration rate: 0.02 per frame (2% per frame)
        - Total restoration duration: 1.0 / 0.02 = 50 frames = 1.0 second
        - Result: Smooth transition from 0% to 100% lane line visibility
        """

        # STATE TRANSITION: Determine next state when restoration is complete
        if self.lane_change_ll_prob > 0.99:
          """
          When lane line visibility is fully restored (>99%):
          """
          self.lane_change_direction = LaneChangeDirection.none  # Clear lane change direction
          
          if one_blinker:
            # Blinker still active: User may want additional lane changes
            # Return to preLaneChange state to allow multiple changes with single blinker activation
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            # Blinker deactivated: Lane change sequence complete
            # Return to off state and wait for next blinker activation
            self.lane_change_state = LaneChangeState.off

    # ============================================================================
    # TIMER MANAGEMENT: Update lane change duration tracking
    # ============================================================================
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      # Reset timer when not actively changing lanes (off or waiting for confirmation)
      self.lane_change_timer = 0.0
    else:
      # Increment timer during active lane change (starting + finishing states)
      # This tracks total time spent in lane change maneuver for timeout detection
      self.lane_change_timer += DT_MDL

    # ============================================================================
    # STATE TRACKING: Update previous state variables for edge detection
    # ============================================================================
    self.lane_change_completed &= one_blinker  # Reset completion flag when blinker turns off
    self.prev_one_blinker = one_blinker        # Store current blinker state for next frame comparison

    # ============================================================================
    # DESIRE OUTPUT: Generate final lateral control command
    # ============================================================================
    # Look up the appropriate OpenPilot desire based on current state and direction
    # This tells the lateral controller what maneuver to execute
    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # ============================================================================
    # LANE-KEEPING ASSISTANCE: Manage keep pulse functionality during preLaneChange
    # ============================================================================
    # Send periodic keep pulses during preLaneChange to maintain lane position
    # while waiting for user confirmation or auto-timer expiration
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      # Reset pulse timer when not in preLaneChange state
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      # Increment pulse timer during preLaneChange state
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        # Send a keep pulse every 1.0 second to maintain lane position
        self.keep_pulse_timer = 0.0  # Reset timer for next pulse
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        # Clear keep desires to prevent interference with lane change logic
        self.desire = log.Desire.none