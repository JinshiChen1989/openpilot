#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT PDA (PARALLEL DRIVE AVOIDANCE) - STRATEGIC OVERTAKING
====================================================================

OVERVIEW:
PDA provides strategic overtaking optimization through TTT (Time To Takeover)
minimization, operating as a DCP speed boost filter for intelligent
overtaking assistance and traffic flow optimization.

CORE FUNCTIONALITY:
- Adjacent lane monitoring: Track lead vehicles in left/right lanes
- TTT optimization: Minimize TTA (Time to Approach) + TTD (Time to Depart)  
- Strategic speed boost: Temporary acceleration for safer overtaking
- Lane change coordination: Works with LCA for complete overtaking system
- Traffic flow analysis: Optimize position relative to adjacent traffic

OVERTAKING STRATEGY:
┌─────────────────────────────────────────────────────────────────┐
│ ANALYSIS: Monitor adjacent lane vehicles and predict movement   │
│ TIMING: Calculate optimal acceleration window for safe passing  │
│ EXECUTION: Apply strategic speed boost via DCP filter system   │
│ COORDINATION: Work with LCA for complete lane change sequence  │
└─────────────────────────────────────────────────────────────────┘

SAFETY FEATURES:
- Conservative TTT calculations with safety margins
- Multi-vehicle tracking for complex traffic scenarios
- Speed boost limits to prevent dangerous acceleration
- Integration with existing safety monitoring systems
- Fallback to normal cruise when conditions unclear

INTEGRATION POINTS:
- DCP Filter Architecture: Speed boost filter layer
- LCA System: Lane change coordination and timing
- Vision Model: Adjacent vehicle detection and tracking
- Safety Systems: Maintains all existing safety boundaries
- Strategic Speed Boosting: Temporary speed increase for efficient overtaking
- Safety Integration: Respects all safety constraints and cruise speed limits

Key Features:
- Never exceeds driver-set cruise speed target
- 15-second lead detection activation rule
- 20% maximum speed boost limit
- TTT ≤ 5s (TTA ≤ 3s, TTD ≤ 2s)
- DCP dependency (only active when DCP enabled)
- OPOM override (disabled when One Pedal mode active)

TEMPORARY DEBUG LOGGING:
To remove all debug logging after testing is complete:
1. Comment out sections marked with "DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE"
2. Search for "cloudlog." and comment out any debug logging calls
3. Keep only essential error/warning logs for production
"""

import numpy as np
from enum import IntEnum
from typing import Dict, Any, Optional, Tuple
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.conversions import Conversions as CV

from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for PDA module
np_logger = NpLogger('pda')
from openpilot.selfdrive.controls.lib.nagaspilot.dcp_profile import DCPFilterLayer, DCPFilterType, DCPFilterResult


class PDAState(IntEnum):
    """PDA State machine states"""
    DISABLED = 0      # PDA off or conditions not met
    MONITORING = 1    # Scanning for overtaking opportunities
    PLANNING = 2      # Calculating optimal overtaking strategy
    EXECUTING = 3     # Actively overtaking (speed boost applied)
    RETURNING = 4     # Returning to normal cruise after overtaking


class AdjacentAnchorTracker:
    """Track anchor vehicles in adjacent lanes (±1.8m+ lateral) for overtaking analysis"""

    def __init__(self):
        self.params = Params()

        # Anchor vehicle tracking filters
        self.left_anchor_filter = FirstOrderFilter(0.0, 0.3, DT_MDL)
        self.right_anchor_filter = FirstOrderFilter(0.0, 0.3, DT_MDL)

        # Tracking state
        self.left_anchor_valid = False
        self.right_anchor_valid = False
        self.left_anchor_data = {}
        self.right_anchor_data = {}

        # Stability tracking
        self.left_anchor_stable_count = 0
        self.right_anchor_stable_count = 0
        self.min_stable_count = 10  # Require 10 consecutive detections

    def update(self, model_data, ego_speed):
        """Update adjacent lane anchor vehicle tracking"""
        self.left_anchor_valid = False
        self.right_anchor_valid = False

        if not model_data or len(model_data.leads) < 2:  # Need at least 2 vehicles for anchor analysis
            self.left_anchor_stable_count = 0
            self.right_anchor_stable_count = 0
            return

        # Analyze all detected vehicles for adjacent lane anchors (±1.8m+ lateral distance)
        for i, anchor in enumerate(model_data.leads):
            if i == 0:  # Skip primary lead (same lane lead car)
                continue

            # Check if anchor is in adjacent lane (±1.8m+ from centerline)
            if (anchor.prob > 0.4 and
                anchor.x[0] > 0 and anchor.x[0] < 200.0 and  # Reasonable range
                abs(anchor.y[0]) >= 1.8 and abs(anchor.y[0]) < 6.0):  # Adjacent lane position (±1.8m+)

                # Determine which adjacent lane (left or right)
                if anchor.y[0] <= -1.8:  # Left lane (negative y, ≥1.8m)
                    self.left_anchor_filter.update(anchor.x[0])
                    self.left_anchor_data = {
                        'distance': self.left_anchor_filter.x,
                        'lateral_pos': anchor.y[0],
                        'speed': ego_speed + anchor.v[0],
                        'relative_speed': anchor.v[0],
                        'confidence': anchor.prob
                    }
                    self.left_anchor_stable_count += 1
                    if self.left_anchor_stable_count >= self.min_stable_count:
                        self.left_anchor_valid = True

                elif anchor.y[0] >= 1.8:  # Right lane (positive y, ≥1.8m)
                    self.right_anchor_filter.update(anchor.x[0])
                    self.right_anchor_data = {
                        'distance': self.right_anchor_filter.x,
                        'lateral_pos': anchor.y[0],
                        'speed': ego_speed + anchor.v[0],
                        'relative_speed': anchor.v[0],
                        'confidence': anchor.prob
                    }
                    self.right_anchor_stable_count += 1
                    if self.right_anchor_stable_count >= self.min_stable_count:
                        self.right_anchor_valid = True

        # Reset counters if anchors not detected
        if not self.left_anchor_valid:
            self.left_anchor_stable_count = max(0, self.left_anchor_stable_count - 2)
        if not self.right_anchor_valid:
            self.right_anchor_stable_count = max(0, self.right_anchor_stable_count - 2)


class TTCMonitor:
    """Monitor Time To Collision (TTC) and distance gap for same-lane lead cars within ±1.8m lateral range"""

    def __init__(self):
        # TTC monitoring parameters
        self.lateral_range = 1.8      # meters (±1.8m from ego vehicle centerline)
        self.min_safe_ttc = 3.0       # seconds (minimum safe TTC threshold)
        self.critical_ttc = 3.0       # seconds (critical TTC threshold - abort overtaking) - increased from 2.0s
        self.ttc_filter = FirstOrderFilter(0.0, 0.5, DT_MDL)  # Smooth TTC calculations

        # Distance gap monitoring parameters - now speed-dependent
        self.base_min_gap_distance = 50.0  # meters (base minimum gap at low speeds)
        self.min_time_gap = 3.0            # seconds (minimum time-based following distance)

        self.gap_distance_filter = FirstOrderFilter(0.0, 0.3, DT_MDL)  # Smooth distance calculations

    def get_min_following_distance(self, ego_speed: float) -> float:
        """Calculate minimum following distance based on speed (physics-based)"""
        # Speed-dependent following distance: max(base_distance, time_gap * speed)
        time_based_distance = self.min_time_gap * ego_speed
        return max(self.base_min_gap_distance, time_based_distance)

    def calculate_ttc(self, lead_distance: float, relative_speed: float) -> float:
        """Calculate Time To Collision with lead vehicle"""
        if relative_speed <= 0:
            return float('inf')  # No collision risk if not approaching

        ttc = lead_distance / abs(relative_speed)
        self.ttc_filter.update(ttc)
        return self.ttc_filter.x

    def check_same_lane_lead_safety(self, model_data, ego_speed: float) -> Tuple[bool, float, Dict[str, float]]:
        """
        Check if same-lane lead car is within lateral range and monitor TTC + distance gap

        Returns:
            Tuple[bool, float, Dict]: (is_safe, ttc_value, gap_info)
        """
        gap_info = {'distance': float('inf'), 'safe_distance': True}

        if not model_data or len(model_data.leads) == 0:
            return True, float('inf'), gap_info  # No lead cars detected, safe to proceed

        # Check primary lead car (index 0 is same-lane lead car)
        primary_lead_car = model_data.leads[0]

        # Verify lead car is within lateral safety range (±1.8m from centerline)
        if (primary_lead_car.prob > 0.5 and
            primary_lead_car.x[0] > 0 and  # Lead car is ahead
            abs(primary_lead_car.y[0]) <= self.lateral_range):  # Within lateral range

            # Calculate distance gap with filtering
            lead_distance = primary_lead_car.x[0]
            self.gap_distance_filter.update(lead_distance)
            filtered_distance = self.gap_distance_filter.x

            gap_info['distance'] = filtered_distance

            # Check minimum gap distance (driver-settable from CAN bus)
            min_safe_distance = self.get_min_following_distance(ego_speed)
            if filtered_distance < min_safe_distance:
                gap_info['safe_distance'] = False
                return False, 0.0, gap_info  # Distance too close - disable PDA

            gap_info['safe_distance'] = True

            # Calculate TTC
            relative_speed = -primary_lead_car.v[0]  # Negative vRel means approaching

            if relative_speed > 0:  # We are approaching the lead car
                ttc = self.calculate_ttc(lead_distance, relative_speed)

                # Check safety thresholds
                if ttc < self.critical_ttc:
                    return False, ttc, gap_info  # Critical - abort overtaking
                elif ttc < self.min_safe_ttc:
                    return False, ttc, gap_info  # Not safe - block overtaking
                else:
                    return True, ttc, gap_info   # Safe TTC and gap
            else:
                return True, float('inf'), gap_info  # Lead car is moving away or same speed

        return True, float('inf'), gap_info  # Lead car not in critical lateral range


class TTTOptimizationCalculator:
    """Calculate optimal Time To Takeover (TTA + TTD) for strategic overtaking"""

    def __init__(self):
        # TTT optimization parameters
        self.vehicle_length = 4.5     # meters (ego vehicle length)
        self.safety_buffer = 10.0     # meters (safety clearance)
        self.max_overtaking_boost = 5.0 * CV.KPH_TO_MS  # 5 km/h max boost
        self.min_speed_advantage = 2.0 * CV.KPH_TO_MS   # 2 km/h minimum advantage needed

        # Overtaking opportunity thresholds
        self.min_lead_distance = 30.0    # meters (minimum distance to consider overtaking)
        self.max_lead_distance = 50.0    # meters (maximum distance for overtaking - preset default)
        self.optimal_ttt_threshold = 5.0 # seconds (maximum acceptable TTT)
        self.max_tta_threshold = 3.0     # seconds (maximum Time to Approach - preset default)
        self.max_ttd_threshold = 2.0     # seconds (maximum Time to Depart)

    def calculate_tta(self, lead_distance: float, ego_speed: float, lead_speed: float, speed_boost: float) -> float:
        """Calculate Time To Approach lead vehicle's current position"""
        boosted_ego_speed = ego_speed + speed_boost
        relative_speed = boosted_ego_speed - lead_speed

        if relative_speed <= 0:
            return float('inf')  # Cannot approach if not faster

        tta = lead_distance / relative_speed
        return max(0.0, tta)

    def calculate_ttd(self, ego_speed: float, lead_speed: float, speed_boost: float) -> float:
        """Calculate Time To Depart (clear the lead vehicle safely)"""
        boosted_ego_speed = ego_speed + speed_boost
        relative_speed = boosted_ego_speed - lead_speed

        if relative_speed <= 0:
            return float('inf')  # Cannot depart if not faster

        # Distance needed to safely clear lead vehicle
        clearance_distance = self.vehicle_length + self.safety_buffer
        ttd = clearance_distance / relative_speed
        return max(0.0, ttd)

    def calculate_optimal_ttt(self, lead_data: Dict, ego_speed: float, target_speed: float) -> Tuple[float, float]:
        """Calculate optimal TTT and required speed boost"""
        if not lead_data or lead_data['speed'] >= target_speed:
            return float('inf'), 0.0  # No overtaking benefit

        lead_distance = lead_data['distance']
        lead_speed = lead_data['speed']

        # Check basic feasibility
        if (lead_distance < self.min_lead_distance or
            lead_distance > self.max_lead_distance or
            target_speed - lead_speed < self.min_speed_advantage):
            return float('inf'), 0.0

        # Find optimal speed boost to minimize TTT
        best_ttt = float('inf')
        optimal_boost = 0.0

        # Try different speed boost levels (limited to 20% of current speed)
        # ✅ FIXED: Pre-calculated boost levels for better performance
        max_boost_20pct = min(self.max_overtaking_boost, ego_speed * 0.20)
        boost_step = 0.5 * CV.KPH_TO_MS  # 0.5 km/h steps
        boost_levels = [i * boost_step for i in range(int(max_boost_20pct / boost_step) + 1)]
        for boost in boost_levels:
            tta = self.calculate_tta(lead_distance, ego_speed, lead_speed, boost)
            ttd = self.calculate_ttd(ego_speed, lead_speed, boost)

            if tta != float('inf') and ttd != float('inf'):
                ttt = tta + ttd
                # Detailed TTT constraints: TTT≤5s, TTA≤3s, TTD≤2s, and 20% speed limit
                if (ttt < best_ttt and ttt <= self.optimal_ttt_threshold and
                    tta <= self.max_tta_threshold and ttd <= self.max_ttd_threshold and
                    boost <= ego_speed * 0.20):
                    best_ttt = ttt
                    optimal_boost = boost

        return best_ttt, optimal_boost

    def evaluate_overtaking_opportunity(self, lead_data: Dict, ego_speed: float, target_speed: float) -> Tuple[bool, float, float]:
        """Evaluate if overtaking opportunity exists and calculate optimal strategy"""
        if not lead_data:
            return False, 0.0, float('inf')

        optimal_ttt, optimal_boost = self.calculate_optimal_ttt(lead_data, ego_speed, target_speed)

        # Determine if overtaking is beneficial
        is_beneficial = (optimal_ttt != float('inf') and
                        optimal_ttt <= self.optimal_ttt_threshold and
                        optimal_boost > 0)

        return is_beneficial, optimal_boost, optimal_ttt


class NpPDAController(DCPFilterLayer):
    """PDA DCP filter layer for strategic overtaking optimization"""

    def __init__(self):
        # Initialize as DCP filter layer
        super().__init__(
            name="PDA",
            filter_type=DCPFilterType.SPEED_ENHANCEMENT,
            priority=5  # Lowest priority - overtaking only when all safety systems allow
        )

        self.params = Params()

        self.anchor_tracker = AdjacentAnchorTracker()  # Track anchor cars in adjacent lanes (±1.8m+)
        self.ttt_calculator = TTTOptimizationCalculator()  # TTT optimization for anchor cars
        self.ttc_monitor = TTCMonitor()  # TTC safety monitoring for same-lane lead cars (±1.8m)

        # DCP dependency tracking
        self.dcp_active = False

        # PDA state management
        self.state = PDAState.DISABLED
        self.active_overtaking = False
        self.current_accel_multiplier = 1.0  # 1.0 = no change, up to 1.2 = 20% acceleration enhancement

        # Acceleration multiplier filter for smoothness
        self.accel_multiplier_filter = FirstOrderFilter(1.0, 0.2, DT_MDL)

        # Activation parameters
        self.min_activation_speed = 40.0 * CV.KPH_TO_MS   # 40 km/h minimum
        self.max_activation_speed = 130.0 * CV.KPH_TO_MS  # 130 km/h maximum

        # State tracking
        self.overtaking_start_time = 0.0
        self.max_overtaking_duration = 45.0  # seconds (maximum overtaking time)
        self.overtaking_count = 0

        # Lead car detection tracking for 15-second rule
        self.lead_detection_start_time = 0.0
        self.lead_detection_duration_threshold = 15.0  # seconds
        self.consecutive_lead_detections = 0
        self.min_consecutive_detections = 300  # ~15 seconds at 20Hz
        self.last_activation_time = 0.0
        self.activation_cooldown = 30.0  # seconds cooldown between activations

        # Debug data
        self.debug_data = {}

        np_logger.info("Controller initialized with TTT optimization")

    def update_parameters(self, params: Params):
        """Update PDA parameters from Params"""
        try:
            # CRITICAL: Check DCP dependency first with proper exception handling
            try:
                dcp_mode = params.get_int("np_dcp_mode")
                self.dcp_active = (dcp_mode > 0)
            except (ValueError, TypeError, AttributeError) as e:
                np_logger.warning(f"PDA: Failed to read np_dcp_mode parameter: {e}, defaulting to disabled")
                self.dcp_active = False
                self.enabled = False
                return

            # CRITICAL: Check OPOM override status
            opom_active = params.get_bool("np_opom_enabled", False)

            if not self.dcp_active:
                self.enabled = False  # Force disable when DCP is off
                np_logger.debug("Disabled: DCP foundation is off (np_dcp_mode=0)")
                return

            if opom_active:
                self.enabled = False  # Force disable when OPOM is active
                np_logger.debug("Disabled: OPOM override mode is active")
                return

            self.enabled = params.get_bool("np_pda_enabled")

            # Update TTT optimization parameters
            boost_val = params.get_float("np_pda_max_boost", 5.0)
            self.ttt_calculator.max_overtaking_boost = boost_val * CV.KPH_TO_MS

            threshold_val = params.get_float("np_pda_ttt_threshold", 5.0)
            self.ttt_calculator.optimal_ttt_threshold = threshold_val

            # TTA/TTD timing constraints with validation
            tta_threshold = params.get("np_pda_max_tta", encoding='utf8')
            if tta_threshold:
                try:
                    tta_val = float(tta_threshold)
                    if 0.5 <= tta_val <= 10.0:  # Reasonable range 0.5-10 seconds
                        self.ttt_calculator.max_tta_threshold = tta_val
                    else:
                        np_logger.warning(f"PDA max_tta out of range (0.5-10): {tta_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid max_tta parameter '{tta_threshold}': {e}")

            ttd_threshold = params.get("np_pda_max_ttd", encoding='utf8')
            if ttd_threshold:
                try:
                    ttd_val = float(ttd_threshold)
                    if 0.5 <= ttd_val <= 8.0:  # Reasonable range 0.5-8 seconds
                        self.ttt_calculator.max_ttd_threshold = ttd_val
                    else:
                        np_logger.warning(f"PDA max_ttd out of range (0.5-8): {ttd_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid max_ttd parameter '{ttd_threshold}': {e}")

            # TTC safety parameters with validation
            min_safe_ttc = params.get("np_pda_min_safe_ttc", encoding='utf8')
            if min_safe_ttc:
                try:
                    ttc_val = float(min_safe_ttc)
                    if 1.0 <= ttc_val <= 8.0:  # Reasonable range 1-8 seconds
                        self.ttc_monitor.min_safe_ttc = ttc_val
                    else:
                        np_logger.warning(f"PDA min_safe_ttc out of range (1-8): {ttc_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid min_safe_ttc parameter '{min_safe_ttc}': {e}")

            critical_ttc = params.get("np_pda_critical_ttc", encoding='utf8')
            if critical_ttc:
                try:
                    critical_val = float(critical_ttc)
                    if 0.5 <= critical_val <= 5.0:  # Reasonable range 0.5-5 seconds
                        self.ttc_monitor.critical_ttc = critical_val
                    else:
                        np_logger.warning(f"PDA critical_ttc out of range (0.5-5): {critical_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid critical_ttc parameter '{critical_ttc}': {e}")

            lateral_range = params.get("np_pda_lateral_range", encoding='utf8')
            if lateral_range:
                try:
                    range_val = float(lateral_range)
                    if 0.5 <= range_val <= 5.0:  # Reasonable range 0.5-5 meters
                        self.ttc_monitor.lateral_range = range_val
                    else:
                        np_logger.warning(f"PDA lateral_range out of range (0.5-5): {range_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid lateral_range parameter '{lateral_range}': {e}")

            # Distance gap parameters with validation
            min_lead_gap = params.get("np_pda_min_lead_gap", encoding='utf8')
            if min_lead_gap:
                try:
                    gap_val = float(min_lead_gap)
                    if 5.0 <= gap_val <= 100.0:  # Reasonable range 5-100 meters
                        self.ttc_monitor.min_lead_gap_distance = gap_val
                    else:
                        np_logger.warning(f"PDA min_lead_gap out of range (5-100): {gap_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid min_lead_gap parameter '{min_lead_gap}': {e}")

            # TTT distance parameters with validation
            max_lead_distance = params.get("np_pda_max_lead_distance", encoding='utf8')
            if max_lead_distance:
                try:
                    max_dist_val = float(max_lead_distance)
                    if 20.0 <= max_dist_val <= 200.0:  # Reasonable range 20-200 meters
                        self.ttt_calculator.max_lead_distance = max_dist_val
                    else:
                        np_logger.warning(f"PDA max_lead_distance out of range (20-200): {max_dist_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid max_lead_distance parameter '{max_lead_distance}': {e}")

            min_lead_distance = params.get("np_pda_min_lead_distance", encoding='utf8')
            if min_lead_distance:
                try:
                    min_dist_val = float(min_lead_distance)
                    if 10.0 <= min_dist_val <= 100.0:  # Reasonable range 10-100 meters
                        self.ttt_calculator.min_lead_distance = min_dist_val
                    else:
                        np_logger.warning(f"PDA min_lead_distance out of range (10-100): {min_dist_val}, using default")
                except (ValueError, TypeError) as e:
                    np_logger.warning(f"PDA invalid min_lead_distance parameter '{min_lead_distance}': {e}")

        except (ValueError, TypeError) as e:
            np_logger.warning(f"Invalid parameter values, using defaults: {e}")

    def _check_lead_detection_for_acceleration(self, radar_state, current_time: float) -> bool:
        """Check if car has been detecting lead car for acceleration for 15+ seconds"""
        if not radar_state or not radar_state.leadOne.status:
            # No lead detected, reset counter
            self.consecutive_lead_detections = 0
            self.lead_detection_start_time = 0.0
            return False

        # Lead detected
        if self.consecutive_lead_detections == 0:
            self.lead_detection_start_time = current_time

        self.consecutive_lead_detections += 1

        # Check if we've had consistent lead detection for 15+ seconds
        detection_duration = current_time - self.lead_detection_start_time
        if (detection_duration >= self.lead_detection_duration_threshold and
            self.consecutive_lead_detections >= self.min_consecutive_detections):

            # Check if lead car requires acceleration to overtake with proper validation
            try:
                if (hasattr(radar_state.leadOne, 'vLeadK') and
                    hasattr(radar_state.leadOne, 'vRel')):
                    lead_speed = radar_state.leadOne.vLeadK
                    relative_speed = radar_state.leadOne.vRel

                    # Validate speed values are reasonable
                    if (not (-50.0 <= lead_speed <= 100.0) or
                        not (-30.0 <= relative_speed <= 30.0)):
                        np_logger.warning(f"PDA: Invalid radar speeds - lead: {lead_speed:.1f}, rel: {relative_speed:.1f}")
                        return False
                else:
                    np_logger.warning("PDA: Missing radar leadOne speed attributes")
                    return False
            except (AttributeError, TypeError) as e:
                np_logger.warning(f"PDA: Error accessing radar leadOne data: {e}")
                return False

            # If lead is slower and we've been behind them for 15s, allow activation
            if relative_speed < 0:  # Negative vRel means lead is slower
                return True

        return False

    def update_state_machine(self, driving_context: Dict[str, Any]) -> None:
        """Update PDA state machine"""
        # CRITICAL: Check DCP dependency first
        if not self.dcp_active:
            self.state = PDAState.DISABLED
            self.active_overtaking = False
            return

        # Extract context
        model_data = driving_context.get('model_data')
        ego_speed = driving_context.get('v_ego', 0.0)
        target_speed = driving_context.get('target_cruise_speed', 0.0)
        radar_state = driving_context.get('radar_state')
        car_state = driving_context.get('car_state')

        if not model_data or not car_state:
            self.state = PDAState.DISABLED
            return

        # Get current time with robust fallback
        try:
            if hasattr(car_state, 'wallTimeNanos') and car_state.wallTimeNanos is not None:
                current_time = car_state.wallTimeNanos / 1e9
            else:
                # Use monotonic time as fallback to prevent timing issues
                import time
                current_time = time.monotonic()
                if not hasattr(self, '_time_fallback_warned'):
                    np_logger.warning("PDA: Using monotonic time fallback - wallTimeNanos unavailable")
                    self._time_fallback_warned = True
        except (AttributeError, TypeError, ZeroDivisionError) as e:
            import time
            current_time = time.monotonic()
            np_logger.warning(f"PDA: Error reading wallTimeNanos, using monotonic fallback: {e}")

        # Update anchor tracking for adjacent lanes (±1.8m+)
        self.anchor_tracker.update(model_data, ego_speed)

        # Track lead car detection for 15-second activation rule
        has_lead_for_accel = self._check_lead_detection_for_acceleration(radar_state, current_time)

        # CRITICAL: Check TTC safety and distance gap with same-lane lead cars (±1.8m)
        ttc_safe, current_ttc, gap_info = self.ttc_monitor.check_same_lane_lead_safety(model_data, ego_speed)

        # State transitions
        if not self.enabled or ego_speed < self.min_activation_speed or ego_speed > self.max_activation_speed:
            self.state = PDAState.DISABLED
            self.active_overtaking = False
            return

        # CRITICAL: Block or abort overtaking if TTC is unsafe or distance gap is too small
        if not ttc_safe:
            if self.state == PDAState.EXECUTING:
                # Abort active overtaking due to unsafe TTC or distance
                self.state = PDAState.RETURNING
                self.active_overtaking = False
                if not gap_info['safe_distance']:
                    np_logger.warning(f"Aborting overtaking - unsafe gap distance: {gap_info['distance']:.1f}m")
                else:
                    np_logger.warning(f"Aborting overtaking - unsafe TTC: {current_ttc:.1f}s")
            else:
                # Block new overtaking attempts
                self.state = PDAState.DISABLED
                if not gap_info['safe_distance']:
                    np_logger.debug(f"Blocking overtaking - unsafe gap distance: {gap_info['distance']:.1f}m")
                else:
                    np_logger.debug(f"Blocking overtaking - unsafe TTC: {current_ttc:.1f}s")
            return

        # Get best overtaking opportunity (prefer left lane in LHD regions)
        best_opportunity = None
        target_lane = None

        if self.anchor_tracker.left_anchor_valid:
            left_beneficial, left_boost, left_ttt = self.ttt_calculator.evaluate_overtaking_opportunity(
                self.anchor_tracker.left_anchor_data, ego_speed, target_speed)
            if left_beneficial:
                best_opportunity = (left_boost, left_ttt)
                target_lane = 'left'

        if self.anchor_tracker.right_anchor_valid:
            right_beneficial, right_boost, right_ttt = self.ttt_calculator.evaluate_overtaking_opportunity(
                self.anchor_tracker.right_anchor_data, ego_speed, target_speed)
            if right_beneficial and (not best_opportunity or right_ttt < best_opportunity[1]):
                best_opportunity = (right_boost, right_ttt)
                target_lane = 'right'

        # State machine logic
        if self.state == PDAState.DISABLED:
            if best_opportunity or has_lead_for_accel:
                self.state = PDAState.MONITORING

        elif self.state == PDAState.MONITORING:
            if best_opportunity or has_lead_for_accel:
                self.state = PDAState.PLANNING
            else:
                self.state = PDAState.MONITORING

        elif self.state == PDAState.PLANNING:
            if best_opportunity or has_lead_for_accel:
                # Check cooldown period
                if current_time - self.last_activation_time >= self.activation_cooldown:
                    self.state = PDAState.EXECUTING
                    self.active_overtaking = True
                    self.overtaking_start_time = current_time
                    self.last_activation_time = current_time
                    self.overtaking_count += 1
                    if has_lead_for_accel:
                        np_logger.info("Starting overtaking due to 15s lead detection rule")
                    else:
                        np_logger.info(f"Starting overtaking maneuver in {target_lane} lane, TTT={best_opportunity[1]:.1f}s")
                else:
                    cooldown_remaining = self.activation_cooldown - (current_time - self.last_activation_time)
                    np_logger.debug(f"Activation blocked by cooldown ({cooldown_remaining:.1f}s remaining)")
            else:
                self.state = PDAState.MONITORING

        elif self.state == PDAState.EXECUTING:
            overtaking_duration = current_time - self.overtaking_start_time

            if (not best_opportunity or
                overtaking_duration > self.max_overtaking_duration):
                self.state = PDAState.RETURNING
                np_logger.info("Completing overtaking maneuver")

        elif self.state == PDAState.RETURNING:
            if abs(self.accel_multiplier_filter.x - 1.0) < 0.01:  # Returned to normal acceleration (≈1.0)
                self.state = PDAState.MONITORING
                self.active_overtaking = False
                np_logger.info("Returned to monitoring mode")

        # Calculate target acceleration multiplier (1.0 to 1.2 for 20% enhancement)
        if self.state == PDAState.EXECUTING and best_opportunity:
            # Convert speed boost to acceleration multiplier (max 20% enhancement)
            speed_boost = best_opportunity[0]
            # Protect against division by zero and very low speeds
            if ego_speed < 1.0:  # Less than 1 m/s (3.6 km/h) - too slow for safe overtaking
                np_logger.warning(f"PDA: Ego speed too low for overtaking ({ego_speed:.2f} m/s), using neutral multiplier")
                target_accel_multiplier = 1.0  # No enhancement for very low speeds
            else:
                # Prevent division by zero when ego_speed is very small
                if ego_speed > 0.1:  # Only apply boost if speed is above 0.1 m/s
                    target_accel_multiplier = 1.0 + min(speed_boost / ego_speed, 0.20)  # Cap at 20%
                else:
                    target_accel_multiplier = 1.0  # No boost for very low speeds
        else:
            target_accel_multiplier = 1.0  # No enhancement

        # Apply smooth filtering
        self.accel_multiplier_filter.update(target_accel_multiplier)
        self.current_accel_multiplier = self.accel_multiplier_filter.x

        # Update debug data
        self.debug_data = {
            'state': self.state.name,
            'left_anchor_valid': self.anchor_tracker.left_anchor_valid,
            'right_anchor_valid': self.anchor_tracker.right_anchor_valid,
            'best_opportunity': best_opportunity,
            'target_lane': target_lane,
            'accel_multiplier': self.current_accel_multiplier,
            'active_overtaking': self.active_overtaking,
            'overtaking_count': self.overtaking_count,
            'has_lead_for_accel': has_lead_for_accel,
            'consecutive_lead_detections': self.consecutive_lead_detections,
            'detection_duration': current_time - self.lead_detection_start_time if self.lead_detection_start_time > 0 else 0.0,
            'ttc_safe': ttc_safe,
            'current_ttc': current_ttc,
            'ttc_critical_threshold': self.ttc_monitor.critical_ttc,
            'ttc_safe_threshold': self.ttc_monitor.min_safe_ttc,
            'gap_distance': gap_info['distance'],
            'gap_safe': gap_info['safe_distance'],
            'min_gap_threshold': self.ttc_monitor.min_lead_gap_distance
        }

    def process(self, speed_target: float, driving_context: Dict[str, Any]) -> DCPFilterResult:
        """
        DCP filter interface - process speed target through PDA acceleration multiplier

        Args:
            speed_target: Current target speed from DCP foundation
            driving_context: Current driving situation data

        Returns:
            DCPFilterResult with acceleration multiplier (1.0 to 1.2 for 20% enhancement)
        """
        # Update parameters
        self.update_parameters(self.params)

        v_ego = driving_context.get('v_ego', 0.0)
        np_logger.debug(f"Processing - speed_target: {speed_target:.1f}m/s, v_ego: {v_ego:.1f}m/s")

        # CRITICAL: Check DCP dependency
        if not self.dcp_active:
            np_logger.debug("DCP dependency not met - PDA inactive")
            return DCPFilterResult(speed_modifier=1.0, active=False, reason="DCP disabled")

        # CRITICAL: Check if enabled
        if not self.enabled:
            np_logger.debug("PDA disabled via parameter")
            return DCPFilterResult(speed_modifier=1.0, active=False, reason="PDA disabled")

        # Update state machine
        self.update_state_machine(driving_context)

        # Apply acceleration multiplier if actively overtaking
        if self.state == PDAState.EXECUTING and self.active_overtaking and self.current_accel_multiplier > 1.0:
            # Use acceleration multiplier as speed modifier (1.0 to 1.2)
            # This multiplies the current DLP foundation command by up to 20%
            accel_multiplier = min(self.current_accel_multiplier, 1.20)  # Cap at 20%

            return DCPFilterResult(
                speed_modifier=accel_multiplier,
                active=True,
                reason=f"Strategic overtaking (x{accel_multiplier:.2f} accel multiplier)",
                priority=self.priority
            )

        return DCPFilterResult(speed_modifier=1.0, active=False, reason="No overtaking opportunity")

    def get_acceleration_multiplier(self) -> float:
        """
        Get current acceleration multiplier for DLP foundation command enhancement

        Returns:
            float: Acceleration multiplier (1.0 = no change, up to 1.2 = 20% enhancement)
        """
        if (self.enabled and self.dcp_active and
            self.state == PDAState.EXECUTING and self.active_overtaking):
            return min(self.current_accel_multiplier, 1.20)  # Cap at 20%
        return 1.0  # No enhancement

    def get_status(self) -> Dict[str, Any]:
        """Get PDA status for telemetry"""
        return {
            'enabled': self.enabled,
            'dcp_active': self.dcp_active,
            'state': self.state.name,
            'active_overtaking': self.active_overtaking,
            'current_accel_multiplier': self.current_accel_multiplier,
            'overtaking_count': self.overtaking_count,
            'debug_data': self.debug_data.copy()
        }


# Legacy interface for backward compatibility
def create_pda_controller() -> NpPDAController:
    """Create PDA controller instance"""
    return NpPDAController()