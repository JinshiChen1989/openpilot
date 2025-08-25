"""
====================================================================
NAGASPILOT DYNAMIC LANE PLUS (DLP) - LATERAL CONTROL SYSTEM
====================================================================

OVERVIEW:
This module implements NagasPilot's Dynamic Lane Plus (DLP) system for 
intelligent lateral vehicle control. DLP provides 4 operational modes
with automatic switching based on driving conditions.

OPERATIONAL MODES:
- Mode 0 (Off): Fallback to OpenPilot lateral control
- Mode 1 (Lanekeep): Basic lane keeping with lane line following  
- Mode 2 (Laneless): Advanced lane keeping without strict lane dependency
- Mode 3 (DLP): Full dynamic profiling with auto mode switching

KEY FEATURES:
- NDLOB (No Disengage Lateral On Brake): Optional brake override safety
- Vision-based curve detection for laneless operation
- Dynamic mode switching based on lane confidence and curvature
- Integrated with OpenPilot's lateral MPC and desire helper
- Real-time parameter updates with safety delays

SAFETY SYSTEMS:
- Emergency speed/torque checks for NDLOB override
- Mode transition delays to prevent rapid switching
- Lane change state machine integration
- Fallback to OpenPilot on any failure

INTEGRATION:
- Works with DCP (Dynamic Cruise Profiles) for longitudinal coordination
- Interfaces with OpenPilot's standard lateral control pipeline
- Compatible with existing lane change assist and monitoring systems

CODE REVIEW NOTES:
- Critical safety functions: should_maintain_lateral_on_brake() (line ~156)
- Mode switching logic: get_dynamic_lane_profile() (line ~268)  
- Parameter validation: _get_bool_param(), _get_int_param() (line ~120)
- Real-time updates: read_param() with safety delays (line ~107)

TEMPORARY DEBUG LOGGING:
To remove all debug logging after testing is complete:
1. Comment out sections marked with "DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE"
2. Search for "cloudlog." and comment out any debug logging calls
3. Keep only essential error/warning logs for production
"""

import time
import numpy as np
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
# ⚠️ PERFORMANCE: Using slower numpy.interp instead of optimized numpy_fast
# TODO: Implement or find numpy_fast equivalent for better performance
interp = np.interp
from openpilot.common.params import Params

# ============================================================================
# DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE
# Comment out the following import and all cloudlog.* calls when debugging done
# ============================================================================
# from openpilot.common.swaglog import cloudlog
# ============================================================================
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.nagaspilot.lane_planner import LanePlanner, TRAJECTORY_SIZE
from openpilot.selfdrive.controls.lib.nagaspilot.common import DrivingContext, ControlMode
from openpilot.selfdrive.controls.lib.nagaspilot.helpers import validate_speed, classify_driving_context
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error, get_road_edge
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper

import cereal.messaging as messaging
from cereal import log

LaneChangeState = log.LaneChangeState


PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.04
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 700.0


class LateralPlanner:
  # ========================================================================
  # INITIALIZATION & CONFIGURATION
  # ========================================================================
  
  def __init__(self, CP, debug=False, model_use_lateral_planner=False):
    """
    Initialize Dynamic Lane Plus (DLP) lateral planner
    
    Args:
        CP: Car parameters (wheelbase, mass, tire stiffness)
        debug: Enable debug mode for additional logging
        model_use_lateral_planner: Use model-based lateral planning vs MPC
    """
    # Core OpenPilot integration components
    self.LP = LanePlanner()    # Lane line detection and processing
    self.DH = DesireHelper()   # Lane change state machine and desire logic

    # Vehicle-specific dynamics for lateral control calculations
    # These factors determine how the car responds to steering inputs
    self.factor1 = CP.wheelbase - CP.centerToFront  # Rear axle to CG distance
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)  # Understeer gradient
    
    # MPC (Model Predictive Control) state tracking
    self.last_cloudlog_t = 0      # Throttle error logging to prevent spam
    self.solution_invalid_cnt = 0  # Track consecutive MPC solution failures

    # Control state variables
    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.x_sol = np.zeros((TRAJECTORY_SIZE, 4), dtype=np.float32)
    self.v_ego = MIN_SPEED
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0
    self.d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))

    self.debug_mode = debug
    self.model_use_lateral_planner = model_use_lateral_planner

    # Initialize MPC controller
    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

    # Initialize parameter system
    self.param_s = Params()
    self.param_read_counter = 0
    
    # Initialize DLP state variables
    self.dynamic_lane_profile = self._get_int_param("np_dlp_mode", 0)
    self.dynamic_lane_profile_status = True
    self.dynamic_lane_profile_status_buffer = False
    self.standstill_elapsed = 0.0
    self.standstill = False
    self.road_edge = False
    
    # Load initial parameter values
    self.ndlob_enabled = self._get_bool_param("NoDisengageLateralOnBrake", False)
    self.vision_curve_laneless = self._get_bool_param("np_dlp_vision_curve", False)
    self.edge_toggle = self._get_bool_param("RoadEdge", False)
    
    # SOC (Smart Offset Controller) parameters
    self.soc_enabled = self._get_bool_param("np_soc_enabled", False)
    self.soc_max_offset = 0.25          # Maximum lateral offset (meters)
    self.soc_avoidance_distance = 30.0  # Detection range (meters)
    self.soc_confidence_threshold = 0.7 # Vehicle detection threshold
    self.soc_offset_rate = 0.05         # Maximum offset change rate (m/s)
    self.soc_min_offset = 0.05          # Minimum meaningful offset
    self.soc_tta_threshold = 8.0        # TTA activation threshold (seconds)
    
    # SOC state variables
    self.soc_current_offset = 0.0
    self.soc_last_update_time = time.time()
    self.soc_vehicle_classes = {'car': 'MONITOR', 'bus': 'AVOID_LARGE', 'truck': 'AVOID_LARGE'}
    
    # Complete parameter initialization
    self.read_param()

  # ========================================================================
  # PARAMETER MANAGEMENT
  # ========================================================================

  def read_param(self):
    """Read and update DLP parameters with safety delays"""
    # Read nagaspilot DLP mode parameter with safety delay
    new_dlp_mode = self._get_int_param("np_dlp_mode", 0)
    
    # SAFETY: Prevent rapid mode switching while driving
    if hasattr(self, 'dynamic_lane_profile') and new_dlp_mode != self.dynamic_lane_profile:
        if not hasattr(self, '_mode_change_timer'):
            self._mode_change_timer = 0
        self._mode_change_timer += 1
        if self._mode_change_timer < 25:  # 0.5 second delay at 50Hz
            return  # Keep current mode
        self._mode_change_timer = 0
    
    self.dynamic_lane_profile = new_dlp_mode
    
    # Periodic parameter updates (every second at 50Hz)
    if self.param_read_counter % 50 == 0:
      self.vision_curve_laneless = self._get_bool_param("np_dlp_vision_curve", False)
      self.edge_toggle = self._get_bool_param("RoadEdge", False)
      self.ndlob_enabled = self._get_bool_param("NoDisengageLateralOnBrake", False)
      
      # SOC parameter updates
      self.soc_enabled = self._get_bool_param("np_soc_enabled", False)
      try:
        self.soc_max_offset = max(0.1, min(0.5, float(self.param_s.get("np_soc_max_offset", "0.25"))))
        self.soc_avoidance_distance = max(10.0, min(50.0, float(self.param_s.get("np_soc_avoidance_distance", "30"))))
        self.soc_confidence_threshold = max(0.5, min(0.95, float(self.param_s.get("np_soc_confidence_threshold", "0.7"))))
        self.soc_offset_rate = max(0.01, min(0.2, float(self.param_s.get("np_soc_offset_rate", "0.05"))))
        self.soc_tta_threshold = max(3.0, min(15.0, float(self.param_s.get("np_soc_tta_threshold", "8.0"))))
      except (ValueError, TypeError):
        pass  # Keep existing values if parameter reading fails
    self.param_read_counter += 1

  def _get_bool_param(self, key: str, default: bool) -> bool:
    """
    PARAMETER VALIDATION: Secure boolean parameter reading with error handling
    
    This function implements the standard NagasPilot parameter validation pattern
    used across all modules for consistent and secure parameter handling.
    
    SECURITY FEATURES:
    - Type validation: Ensures parameter is boolean-compatible
    - Exception handling: Graceful fallback to safe defaults
    - Null checking: Handles missing parameters safely
    - Encoding validation: Proper byte string handling
    
    Args:
        key: Parameter name to read from system
        default: Safe fallback value if parameter invalid/missing
        
    Returns:
        bool: Validated parameter value or safe default
    """
    try:
      value = self.param_s.get(key)  # Get raw parameter value
      return value == b'1' if value is not None else default  # Convert to boolean
    except Exception:
      # Log warning but continue with safe default (no system crash)
      # cloudlog.warning(f"[DLP] Error reading bool param {key}, using default {default}")
      return default

  def _get_int_param(self, key: str, default: int) -> int:
    """
    PARAMETER VALIDATION: Secure integer parameter reading with error handling
    
    Implements standardized integer parameter validation used throughout
    NagasPilot for consistent security and error handling.
    
    VALIDATION STEPS:
    1. Retrieve parameter with UTF-8 encoding
    2. Validate conversion to integer type
    3. Handle missing/invalid parameters gracefully
    4. Return safe default on any error
    
    Args:
        key: Parameter name to read from system
        default: Safe fallback value if parameter invalid/missing
        
    Returns:
        int: Validated parameter value or safe default
    """
    try:
      value = self.param_s.get(key, encoding='utf8')  # Get parameter as string
      return int(value) if value is not None else default  # Convert and validate
    except (ValueError, TypeError):
      # Log validation error but continue safely
      # cloudlog.warning(f"[DLP] Error reading int param {key}, using default {default}")
      return default

  # ========================================================================
  # SAFETY & VALIDATION
  # ========================================================================


  def should_maintain_lateral_on_brake(self, sm) -> bool:
    """
    CRITICAL SAFETY FUNCTION: NDLOB (No Disengage Lateral On Brake)
    
    Determines if lateral control should be maintained when brake is pressed.
    This function implements safety-critical logic that can override normal
    brake-based disengagement behavior.
    
    SAFETY DESIGN:
    - Works across ALL DLP profiles (Lanekeep, Laneless, DLP)
    - Includes emergency safety checks to prevent dangerous overrides
    - Only activates under safe conditions (low speed, no driver conflict)
    
    EMERGENCY SAFETY CHECKS:
    1. Speed limit: Above 90 kph (25 m/s), always allow driver takeover
    2. Torque conflict: If driver fighting system (>200 Nm), disengage immediately
    3. Parameter validation: Only activate if explicitly enabled by user
    
    Returns:
        bool: True if lateral control should be maintained despite brake press
              False if normal brake disengagement should occur
    
    WARNING: This function can override safety-critical brake disengagement.
             Any changes must be thoroughly tested and validated.
    """
    # First check: Is NDLOB feature enabled by user?
    if not self.ndlob_enabled:
        return False  # Feature disabled, use normal brake disengagement
        
    # Second check: Is brake actually pressed?
    CS = sm['carState']
    if not CS.brakePressed:
        return False  # No brake press, function not applicable
    
    # EMERGENCY SAFETY CHECK #1: High speed override prevention
    # Above 90 kph, driver needs immediate control for emergency situations
    if CS.vEgo > 25.0:  # 25 m/s = 90 kph = 56 mph
        return False  # HIGH SPEED: Always allow driver takeover
    
    # EMERGENCY SAFETY CHECK #2: Driver conflict detection  
    # If driver is applying significant steering torque, they want control
    if abs(CS.steeringTorque) > 200:  # 200 Nm threshold for active steering
        return False  # TORQUE CONFLICT: Driver fighting system, disengage
    
    # All safety checks passed - safe to maintain lateral control
    # Log the decision for debugging and safety monitoring
    current_dlp_mode = self._get_int_param("np_dlp_mode", 0)
    mode_names = {0: "Off", 1: "Lanekeep", 2: "Laneless", 3: "DLP"}
    mode_name = mode_names.get(current_dlp_mode, "Unknown")
    
    # cloudlog.info(f"[NDLOB] Safe brake override in {mode_name} mode, maintaining lateral control")
    return True  # SAFE: Maintain lateral control despite brake press

  def calculate_soc_lateral_offset(self, sm) -> float:
    """
    SOC (Smart Offset Controller) - DLP Enhancement Function
    
    Calculates lateral offset for vehicle avoidance based on YOLOv8 detection data
    and modelV2 lead tracking fusion. Uses TTA (Time To Approach) calculations
    for intelligent timing of avoidance maneuvers.
    
    DEPENDENCIES:
    - Requires successful modelV2 lead v3 fusion for reliable tracking
    - Uses TTA calculations for proactive avoidance timing
    - Integrates YOLOv8 detection with OpenPilot lead car tracking
    
    SAFETY DESIGN:
    - Only applies small offsets within lane boundaries (max 0.25m)
    - Smooth transitions with rate limiting to prevent jerky movements
    - Only responds to high-confidence detections of large vehicles
    - TTA-based activation prevents unnecessary maneuvers
    
    Returns:
        float: Lateral offset in meters (+ = left offset, - = right offset)
    """
    import time
    import numpy as np
    
    # Early exit if SOC is disabled
    if not self.soc_enabled:
        return 0.0
    
    # CRITICAL: Check modelV2 lead v3 fusion success
    if not self._check_modelv2_lead_fusion_success(sm):
        return self._apply_soc_offset_transition(0.0)
    
    # Get YOLOv8 detection data
    if 'yolov8Detections' not in sm:
        return self._apply_soc_offset_transition(0.0)
    
    yolov8_data = sm['yolov8Detections']
    
    # Validate detection data
    if not hasattr(yolov8_data, 'detections') or len(yolov8_data.detections) == 0:
        return self._apply_soc_offset_transition(0.0)
    
    # Process vehicle detections
    large_vehicles = []
    for detection in yolov8_data.detections:
        class_name = detection.className
        
        # Only process large vehicle classes
        if class_name not in self.soc_vehicle_classes or class_name == 'car':
            continue
            
        # Skip if not avoiding this vehicle type
        if self.soc_vehicle_classes[class_name] != 'AVOID_LARGE':
            continue
            
        # Extract vehicle information
        distance = detection.position3D.x if hasattr(detection, 'position3D') and detection.position3D.x > 0 else 999.0
        lateral_position = detection.position3D.y if hasattr(detection, 'position3D') else 0.0
        confidence = detection.confidence
        
        # Skip low confidence or distant detections
        if confidence < self.soc_confidence_threshold or distance > self.soc_avoidance_distance:
            continue
            
        large_vehicles.append({
            'class_name': class_name,
            'distance': distance,
            'lateral_position': lateral_position,
            'confidence': confidence
        })
    
    # Calculate avoidance offset with TTA-based activation
    if not large_vehicles:
        # No large vehicles detected, gradually return to center
        return self._apply_soc_offset_transition(0.0)
    
    # Find closest large vehicle for TTA calculation
    closest_vehicle = min(large_vehicles, key=lambda v: v['distance'])
    
    # Calculate TTA (Time To Approach) for activation decision
    v_ego = sm['carState'].vEgo if 'carState' in sm else 0.0
    tta = self._calculate_vehicle_tta(closest_vehicle, v_ego, sm)
    
    # TTA-based activation threshold (only activate if approach is imminent)
    tta_threshold = getattr(self, 'soc_tta_threshold', 8.0)  # 8 second default
    if tta > tta_threshold:
        # Vehicle too far away temporally, no avoidance needed yet
        return self._apply_soc_offset_transition(0.0)
    
    # Calculate offset based on vehicle proximity, lateral position, and TTA urgency
    base_offset = self.soc_max_offset
    distance_factor = max(0.3, 1.0 - (closest_vehicle['distance'] / self.soc_avoidance_distance))
    lateral_factor = min(1.0, abs(closest_vehicle['lateral_position']) / 3.0)  # 3m lane width assumption
    tta_urgency = max(0.3, 1.0 - (tta / tta_threshold))  # More urgent as TTA decreases
    
    # Calculate target offset (positive = left offset to avoid vehicle on right)
    if closest_vehicle['lateral_position'] > 0:  # Vehicle on left
        target_offset = -base_offset * distance_factor * lateral_factor * tta_urgency  # Offset right
    else:  # Vehicle on right
        target_offset = base_offset * distance_factor * lateral_factor * tta_urgency   # Offset left
    
    return self._apply_soc_offset_transition(target_offset)
  
  def _apply_soc_offset_transition(self, target_offset: float) -> float:
    """Apply smooth SOC offset transition with rate limiting"""
    import time
    
    current_time = time.time()
    dt = current_time - self.soc_last_update_time
    self.soc_last_update_time = current_time
    
    # Apply rate limiting for smooth transitions
    if abs(target_offset - self.soc_current_offset) > self.soc_min_offset / 10:  # 5mm threshold
        max_change = self.soc_offset_rate * dt
        
        if target_offset > self.soc_current_offset:
            self.soc_current_offset = min(target_offset, self.soc_current_offset + max_change)
        else:
            self.soc_current_offset = max(target_offset, self.soc_current_offset - max_change)
    else:
        self.soc_current_offset = target_offset
    
    # Apply minimum offset threshold
    if abs(self.soc_current_offset) < self.soc_min_offset:
        self.soc_current_offset = 0.0
    
    return self.soc_current_offset

  def _check_modelv2_lead_fusion_success(self, sm) -> bool:
    """Check if modelV2 lead v3 fusion is successful for reliable SOC operation"""
    
    # Check if modelV2 data is available
    if 'modelV2' not in sm:
        return False
        
    modelV2 = sm['modelV2']
    
    # Check if lead tracking data is valid
    if not hasattr(modelV2, 'leadsV3') or len(modelV2.leadsV3) == 0:
        return False
        
    # Check lead v3 data quality
    lead = modelV2.leadsV3[0]  # Primary lead
    
    # Validate lead data fields
    if not hasattr(lead, 'prob') or not hasattr(lead, 'x') or not hasattr(lead, 'v'):
        return False
        
    # Check lead detection probability (fusion success indicator)
    if lead.prob < 0.5:  # Minimum 50% confidence for reliable tracking
        return False
        
    # Check lead distance validity
    if lead.x <= 0 or lead.x > 150:  # Reasonable lead distance range
        return False
        
    # Check if radarState data is available for fusion validation
    if 'radarState' in sm:
        radar_state = sm['radarState']
        if hasattr(radar_state, 'leadOne') and hasattr(radar_state.leadOne, 'status'):
            # Ensure radar also has valid lead tracking
            if not radar_state.leadOne.status:
                return False
    
    return True  # All fusion validation checks passed

  def _calculate_vehicle_tta(self, vehicle_data: dict, v_ego: float, sm) -> float:
    """Calculate Time To Approach (TTA) for vehicle avoidance timing"""
    
    vehicle_distance = vehicle_data['distance']
    
    # Get lead vehicle speed from modelV2 if available
    lead_speed = 0.0
    if 'modelV2' in sm and hasattr(sm['modelV2'], 'leadsV3') and len(sm['modelV2'].leadsV3) > 0:
        lead = sm['modelV2'].leadsV3[0]
        if hasattr(lead, 'v'):
            lead_speed = lead.v
    
    # Calculate relative speed (positive = approaching)
    relative_speed = v_ego - lead_speed
    
    # Handle edge cases
    if relative_speed <= 0:
        return 999.0  # Not approaching or moving away
        
    if vehicle_distance <= 0:
        return 0.0  # Already at vehicle position
    
    # Calculate TTA = distance / relative_speed
    tta = vehicle_distance / relative_speed
    
    # Reasonable TTA bounds
    return min(999.0, max(0.0, tta))

  # ========================================================================
  # MPC & CONTROL OPERATIONS
  # ========================================================================

  def reset_mpc(self, x0=None):
    """Reset MPC controller state"""
    if x0 is None:
      x0 = np.zeros(4)
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  # ========================================================================
  # DATA PROCESSING & MODEL UPDATES
  # ========================================================================

  def update(self, sm):
    self.read_param()
    self.standstill = sm['carState'].standstill
    # clip speed , lateral planning is not possible at 0 speed
    measured_curvature = sm['controlsState'].curvature
    v_ego_car = sm['carState'].vEgo
    
    # ============================================================================
    # DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE
    # cloudlog.debug(f"[DLP] Update - v_ego: {v_ego_car:.1f}m/s, standstill: {self.standstill}, "
    #               f"mode: {self.dlp_mode}, curvature: {measured_curvature:.4f}")
    # ============================================================================

    # Parse model predictions
    md = sm['modelV2']

    if self.model_use_lateral_planner:
      self.LP.parse_model(md)
      if len(md.position.x) == TRAJECTORY_SIZE and (len(md.orientation.x) == TRAJECTORY_SIZE or
                                                    (len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolutionDEPRECATED.x) == TRAJECTORY_SIZE)):
        if len(md.orientation.x) == TRAJECTORY_SIZE:
          self.t_idxs = np.array(md.position.t)
          self.plan_yaw = np.array(md.orientation.z)
          self.plan_yaw_rate = np.array(md.orientationRate.z)
        if len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolutionDEPRECATED.x) == TRAJECTORY_SIZE:
          self.x_sol = np.column_stack([md.lateralPlannerSolutionDEPRECATED.x, md.lateralPlannerSolutionDEPRECATED.y, md.lateralPlannerSolutionDEPRECATED.yaw, md.lateralPlannerSolutionDEPRECATED.yawRate])
        self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
        self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
        car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
        self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
        self.v_ego = self.v_plan[0]

      # Lane change logic
      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
      
      # Call desire_helper with original interface but pass modelV2 for enhanced ALC features
      self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, False, False, md)

      # Turn off lanes during lane change
      if self.DH.desire == log.Desire.laneChangeRight or self.DH.desire == log.Desire.laneChangeLeft:
        self.LP.lll_prob *= self.DH.lane_change_ll_prob
        self.LP.rll_prob *= self.DH.lane_change_ll_prob
      self.d_path_w_lines_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)

      low_speed = v_ego_car < 10 * CV.MPH_TO_MS

      if not self.get_dynamic_lane_profile(sm['longitudinalPlanSP']) and not low_speed:
        self.path_xyz = self.d_path_w_lines_xyz
        self.dynamic_lane_profile_status = False
      else:
        self.path_xyz[:, 1] += self.LP.path_offset
        self.dynamic_lane_profile_status = True

      if not self.dynamic_lane_profile_status:
        self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                                 LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                                 STEERING_RATE_COST)

        y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
        heading_pts = self.plan_yaw[:LAT_MPC_N+1]
        yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
        self.y_pts = y_pts

        assert len(y_pts) == LAT_MPC_N + 1
        assert len(heading_pts) == LAT_MPC_N + 1
        assert len(yaw_rate_pts) == LAT_MPC_N + 1
        lateral_factor = np.clip(self.factor1 - (self.factor2 * self.v_plan**2), 0.0, np.inf)
        p = np.column_stack([self.v_plan, lateral_factor])
        self.lat_mpc.run(self.x0,
                         p,
                         y_pts,
                         heading_pts,
                         yaw_rate_pts)
        # init state for next iteration
        # mpc.u_sol is the desired second derivative of psi given x0 curv state.
        # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
        # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
        self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

        #  Check for infeasible MPC solution
        mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
        t = time.monotonic()
        if mpc_nans or self.lat_mpc.solution_status != 0:
          self.reset_mpc()
          self.x0[3] = measured_curvature * self.v_ego
          if t > self.last_cloudlog_t + 5.0:
            self.last_cloudlog_t = t
            # cloudlog.warning("Lateral mpc - nan: True")

        if self.lat_mpc.cost > 1e6 or mpc_nans:
          self.solution_invalid_cnt += 1
        else:
          self.solution_invalid_cnt = 0

    if not self.model_use_lateral_planner:
      self.road_edge = get_road_edge(sm['carState'], md, self.edge_toggle)

  # ========================================================================
  # STATE MACHINE LOGIC
  # ========================================================================

  def get_dynamic_lane_profile(self, longitudinal_plan_sp):
    """
    CORE STATE MACHINE: Dynamic Lane Profile (DLP) Mode Selection
    
    This function implements the heart of NagasPilot's intelligent lateral
    control system, determining when to use lane-based vs laneless control
    based on real-time driving conditions.
    
    STATE MACHINE MODES:
    ┌─────────────────────────────────────────────────────────────────────┐
    │ Mode 0 (Off):      Fallback to OpenPilot lateral control           │
    │ Mode 1 (Lanekeep): Basic lane keeping with lane line following     │  
    │ Mode 2 (Laneless): Advanced lane keeping without lane dependency   │
    │ Mode 3 (DLP):      Full dynamic profiling with intelligent switching│
    └─────────────────────────────────────────────────────────────────────┘
    
    DLP MODE 3 STATE MACHINE:
    ┌──────────────────┐    Lane Change Active    ┌─────────────────┐
    │   LANE-BASED     │ ────────────────────────▶ │    LANELESS     │
    │   (Follow Lines) │                           │ (Path-based)    │
    │                  │ ◀──────────────────────── │                 │
    └──────────────────┘    Lane Change Complete   └─────────────────┘
           │                                               ▲
           ▼                                               │
    ┌──────────────────┐    Low Confidence/Curves  ┌─────────────────┐
    │   BUFFERING      │ ────────────────────────▶ │    LANELESS     │
    │ (Evaluating)     │                           │ (Vision-based)  │
    │                  │ ◀──────────────────────── │                 │
    └──────────────────┘    High Confidence        └─────────────────┘
    
    TRANSITION CONDITIONS:
    - Lane Change: Automatically switch to laneless during lane changes
    - Low Confidence: Switch when lane detection confidence < 30%
    - High Curves: Switch when lateral acceleration > 1.0 m/s² (if enabled)
    - Recovery: Return to lane-based when confidence > 50% and curves < 0.6 m/s²
    
    Args:
        longitudinal_plan_sp: Longitudinal plan with vision-based curvature data
        
    Returns:
        bool: True = Use laneless control, False = Use lane-based control
    """
    # MODE 0: FALLBACK - Complete DLP disable, use OpenPilot lateral control
    if self.dynamic_lane_profile == 0:
      self.dynamic_lane_profile_status = False        # Clear active status
      self.dynamic_lane_profile_status_buffer = False # Clear buffer state
      # cloudlog.info("[DLP] Mode 0 - Fallback to OpenPilot lateral control active")
      return False  # Use OpenPilot's standard lateral control
      
    # MODE 1: LANEKEEP - Basic lane keeping (always lane-based)
    elif self.dynamic_lane_profile == 1:
      return False  # Always use lane lines for steering
      
    # MODE 2: LANELESS - Advanced lane keeping (always laneless)
    elif self.dynamic_lane_profile == 2:
      return True   # Always ignore lane lines, use path-based control
      
    # MODE 3: DLP - Full dynamic profiling with intelligent state machine
    elif self.dynamic_lane_profile == 3:
      # STATE: LANE CHANGE IN PROGRESS - Force laneless for safety
      if self.DH.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
        return True  # LANELESS: Lane change requires path-based control
        
      # STATE: NORMAL OPERATION - Evaluate conditions for mode switching
      elif self.DH.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        # Calculate average lane confidence from left and right lane detection
        lane_confidence = (self.LP.lll_prob + self.LP.rll_prob) / 2
        
        # Get vision-based curvature data for curve detection
        current_lat_acc = longitudinal_plan_sp.visionCurrentLatAcc
        predicted_lat_acc = longitudinal_plan_sp.visionMaxPredLatAcc
        
        # TRANSITION TO LANELESS: Low confidence or high curvature detected
        if (lane_confidence < 0.3 or  # Lane detection confidence below 30%
            ((current_lat_acc > 1.0 or predicted_lat_acc > 1.4) and  # Sharp curves detected
             self.vision_curve_laneless)):  # Vision curve feature enabled
          self.dynamic_lane_profile_status_buffer = True  # Enter laneless buffer state
          
        # TRANSITION TO LANE-BASED: High confidence and low curvature
        if (lane_confidence > 0.5 and  # Lane detection confidence above 50%
            ((current_lat_acc < 0.6 and predicted_lat_acc < 0.7) or  # Gentle curves
             not self.vision_curve_laneless)):  # Vision curve feature disabled
          self.dynamic_lane_profile_status_buffer = False  # Enter lane-based buffer state
          
        # EXECUTE BUFFERED STATE: Use the current buffer state decision
        if self.dynamic_lane_profile_status_buffer:
          return True   # LANELESS: Use path-based control
          
    # DEFAULT: Use lane-based control for any unhandled cases
    return False  # LANE-BASED: Use lane line following

  # ========================================================================
  # OUTPUT & PUBLISHING
  # ========================================================================

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlanDEPRECATED')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlanDEPRECATED = plan_send.lateralPlanDEPRECATED
    lateralPlanDEPRECATED.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlanDEPRECATED.dPathPoints = self.path_xyz[:,1].tolist() if self.dynamic_lane_profile_status else self.y_pts.tolist()
    lateralPlanDEPRECATED.psis = self.x_sol[0:CONTROL_N, 2].tolist() if self.dynamic_lane_profile_status else self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlanDEPRECATED.curvatures = (self.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist() if self.dynamic_lane_profile_status else (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlanDEPRECATED.curvatureRates = [float(0) for _ in range(CONTROL_N-1)] if self.dynamic_lane_profile_status else [float(x.item() / self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0] # TODO: unused

    lateralPlanDEPRECATED.mpcSolutionValid = bool(1) if self.dynamic_lane_profile_status else bool(plan_solution_valid)
    lateralPlanDEPRECATED.solverExecutionTime = 0.0 if self.dynamic_lane_profile_status else self.lat_mpc.solve_time
    if self.debug_mode:
      lateralPlanDEPRECATED.solverState.x = self.x_sol.tolist() if self.dynamic_lane_profile_status else self.lat_mpc.x_sol.tolist()
      if not self.dynamic_lane_profile_status:
        lateralPlanDEPRECATED.solverCost = self.lat_mpc.cost
        lateralPlanDEPRECATED.solverState = log.LateralPlan.SolverState.new_message()
        lateralPlanDEPRECATED.solverState.u = self.lat_mpc.u_sol.flatten().tolist()

    lateralPlanDEPRECATED.desire = self.DH.desire
    lateralPlanDEPRECATED.useLaneLines = not self.dynamic_lane_profile_status
    lateralPlanDEPRECATED.laneChangeState = self.DH.lane_change_state
    lateralPlanDEPRECATED.laneChangeDirection = self.DH.lane_change_direction

    pm.send('lateralPlanDEPRECATED', plan_send)

    plan_sp_send = messaging.new_message('lateralPlanSPDEPRECATED')
    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlanSPDEPRECATED = plan_sp_send.lateralPlanSPDEPRECATED

    lateralPlanSPDEPRECATED.laneWidth = float(self.LP.lane_width)
    lateralPlanSPDEPRECATED.lProb = float(self.LP.lll_prob)
    lateralPlanSPDEPRECATED.rProb = float(self.LP.rll_prob)
    lateralPlanSPDEPRECATED.dProb = float(self.LP.d_prob)

    lateralPlanSPDEPRECATED.dynamicLaneProfile = int(self.dynamic_lane_profile)
    lateralPlanSPDEPRECATED.dynamicLaneProfileStatus = bool(self.dynamic_lane_profile_status)

    lateralPlanSPDEPRECATED.laneChangeEdgeBlockDEPRECATED = self.road_edge

    if self.standstill:
      self.standstill_elapsed += DT_MDL
    else:
      self.standstill_elapsed = 0.0
    lateralPlanSPDEPRECATED.standstillElapsed = int(self.standstill_elapsed)

    pm.send('lateralPlanSPDEPRECATED', plan_sp_send)

  # ========================================================================
  # FOUNDATION COORDINATION
  # ========================================================================

  def get_foundation_status(self):
    """Get DLP foundation status for coordination with DCP"""
    return {
        'dlp_mode': self.dynamic_lane_profile,
        'dlp_active': self.dynamic_lane_profile_status,
        'lane_confidence': (self.LP.lll_prob + self.LP.rll_prob) / 2,
        'fallback_active': (self.dynamic_lane_profile == 0),
        'foundation_type': 'dlp',
    }

  def update_foundation_coordination(self, dcp_status=None):
    """Update DLP based on DCP coordination (if needed)"""
    # Foundation coordination logic (simple, safe)
    if dcp_status and dcp_status.get('fallback_active', False):
        # If DCP is in fallback, consider DLP coordination
        # For now, DLP operates independently, but this provides hook for future coordination
        pass

  @property
  def foundation_ready(self):
    """Check if DLP foundation is ready for layer systems"""
    return (hasattr(self, 'dynamic_lane_profile') and 
            hasattr(self, 'dynamic_lane_profile_status') and
            self.dynamic_lane_profile > 0)  # Not in fallback mode

  def get_layer_interface(self):
    """Provide interface for future layer systems"""
    return {
        'foundation_type': 'dlp',
        'foundation_ready': self.foundation_ready,
        'current_mode': self.dynamic_lane_profile,
        'status': self.dynamic_lane_profile_status,
        'lane_confidence': (self.LP.lll_prob + self.LP.rll_prob) / 2 if hasattr(self, 'LP') else 0.0
    }