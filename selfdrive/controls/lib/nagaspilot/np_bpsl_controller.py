#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT BPSL (BRAKE PEDAL SPEED LEARNING) - DCP FILTER
====================================================================

OVERVIEW:
BPSL completes the dual-pedal speed learning system alongside APSL,
providing intelligent speed target adaptation based on driver brake
pedal behavior for comprehensive personalized speed control.

CORE FUNCTIONALITY:
BPSL provides 3 essential functions:
1. Learn new target speed from brake pedal release (final achieved speed)
2. Distinguish manual vs system braking for accurate learning
3. Coordinate with APSL for seamless dual-pedal speed control

LEARNING ALGORITHM:
┌─────────────────────────────────────────────────────────────────┐
│ BRAKE PRESS: Driver manually reduces speed below cruise target  │
│ LEARNING PHASE: Monitor driver's intended reduced speed target  │
│ BRAKE RELEASE: Capture final achieved speed as new preference  │
│ SPEED ADOPTION: Update cruise control with learned target      │
└─────────────────────────────────────────────────────────────────┘

INTELLIGENT BRAKE DETECTION:
- Distinguishes manual driver braking from system-initiated braking
- Filters out automatic emergency braking events
- Recognizes intentional speed reduction vs temporary adjustments
- Validates learning scenarios to prevent false speed adoption
- Coordinates with existing brake-based safety systems

DUAL-PEDAL COORDINATION:
- Seamless integration with APSL accelerator learning system
- Unified speed learning interface across both pedal inputs
- Prevents conflicting speed targets from simultaneous learning
- Maintains consistent learning behavior across acceleration/deceleration
- Preserves driver intent regardless of pedal input method

SAFETY FEATURES:
- Conservative brake learning with extensive validation
- Maintains emergency braking capabilities during learning
- Preserves all existing safety intervention systems
- Ensures coordination with NDLOB and other brake-based features
- Fallback protection against invalid learning scenarios

INTEGRATION POINTS:
- APSL Coordination: Unified dual-pedal learning system
- DCP Filter Architecture: Coordinated filter layer operation
- Brake System Integration: Manual vs automatic brake detection
- Safety Systems: Maintains emergency intervention capabilities
- Cruise Control: Updates target speed based on learned preferences
"""

import time
from typing import Dict, Any, Optional
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger
from openpilot.selfdrive.controls.lib.nagaspilot.dcp_profile import (
    DCPFilterLayer, 
    DCPFilterType, 
    DCPFilterResult
)

# Initialize logger
np_logger = NpLogger('bpsl')


class BPSLFilter(DCPFilterLayer):
    """
    BPSL Filter - Brake Pedal Speed Learning
    
    Core Functions:
    1. Learn final speed when driver releases brake → New cruise target
    2. Distinguish manual vs system braking → Only learn from manual braking
    3. Coordinate with APSL → Dual-pedal speed learning system
    """
    
    def __init__(self):
        super().__init__(name="BPSL", filter_type=DCPFilterType.DRIVER_OVERRIDE, priority=80)
        
        self.params = Params()
        
        # Core parameters
        self.enabled = False
        self.BRAKE_THRESHOLD = 0.05         # 5% brake threshold
        self.MIN_ENGAGEMENT_SPEED = 5.0     # 5 m/s minimum speed  
        self.MIN_LEARNING_DECREASE = 2.0    # 2 m/s minimum decrease to learn (7 km/h)
        self.MANUAL_BRAKE_THRESHOLD = 0.15  # 15% brake pressure = likely manual
        
        # Learning state
        self.last_brake_position = 0.0
        self.learning_mode = False
        self.deceleration_start_speed = None
        self.learned_target_speed = None
        self.learning_timestamp = None
        
        # Manual vs system brake detection
        self.system_brake_active = False
        self.lead_car_detected = False
        self.fcw_active = False
        self.following_distance_brake = False
        
        # Smoothing filter for brake position
        self.brake_filter = FirstOrderFilter(0.0, 0.2, 0.01)  # 0.2 Hz cutoff
        
        # Parameter reading optimization
        self.param_read_counter = 0
        self.read_params()
        
        np_logger.info("BPSL initialized - Brake Pedal Speed Learning ready")
    
    def read_params(self):
        """Read parameters from Params with error handling"""
        try:
            self.enabled = self.params.get_bool("np_bpsl_enabled", False)
        except Exception as e:
            np_logger.error(f"BPSL parameter read error: {e}")
            self.enabled = False
    
    def get_driver_brake_position(self, driving_context: Dict[str, Any]) -> float:
        """Get current driver brake pedal position with filtering"""
        try:
            # Get brake position from driving context
            brake_pos = driving_context.get('brake_pressed', False)
            
            # Convert boolean to approximate analog value if needed
            if isinstance(brake_pos, bool):
                brake_pos = 0.3 if brake_pos else 0.0  # Estimate 30% brake when pressed
            
            # Apply smoothing filter
            filtered_brake = self.brake_filter.update(float(brake_pos))
            return max(0.0, min(1.0, filtered_brake))  # Clamp to [0, 1]
            
        except Exception as e:
            np_logger.error(f"BPSL brake position error: {e}")
            return 0.0
    
    def detect_system_braking(self, driving_context: Dict[str, Any]) -> bool:
        """
        Detect if current braking is system-initiated vs manual
        
        System braking indicators:
        - Lead car too close
        - Emergency braking (FCW/AEB)
        - Following distance maintenance
        - Safety system activation
        
        Returns True if system braking, False if manual braking
        """
        try:
            car_state = driving_context.get('car_state')
            longitudinal_plan = driving_context.get('longitudinal_plan')
            
            # Check for lead car proximity
            if longitudinal_plan:
                # Fixed: Added hasattr() checks before getattr() calls for safety
                if hasattr(longitudinal_plan, 'hasLead'):
                    self.lead_car_detected = getattr(longitudinal_plan, 'hasLead', False)
                    if self.lead_car_detected:
                        # If lead car is close and we're braking, likely system braking
                        if hasattr(longitudinal_plan, 'dRel'):
                            lead_distance = getattr(longitudinal_plan, 'dRel', float('inf'))
                            if lead_distance < 30.0:  # Less than 30m following distance
                                return True
                        else:
                            np_logger.warning("BPSL: longitudinal_plan missing dRel attribute")
                else:
                    np_logger.warning("BPSL: longitudinal_plan missing hasLead attribute")
                    self.lead_car_detected = False
            
            # Check for emergency braking indicators
            if car_state:
                # Fixed: Added hasattr() checks before getattr() calls for safety
                # FCW (Forward Collision Warning) active
                if hasattr(car_state, 'fcw'):
                    self.fcw_active = getattr(car_state, 'fcw', False)
                    if self.fcw_active:
                        return True
                else:
                    np_logger.debug("BPSL: car_state missing fcw attribute")
                    self.fcw_active = False
                
                # Emergency braking system active
                if hasattr(car_state, 'emergencyBrakeActive'):
                    emergency_brake = getattr(car_state, 'emergencyBrakeActive', False)
                    if emergency_brake:
                        return True
                else:
                    np_logger.debug("BPSL: car_state missing emergencyBrakeActive attribute")
            
            # Check for following distance braking
            # This would be more sophisticated in production
            v_ego = driving_context.get('v_ego', 0.0)
            if self.lead_car_detected and v_ego > 10.0:  # Highway speeds with lead car
                # Likely following distance maintenance
                return True
            
            # If none of the system braking conditions met, assume manual
            return False
            
        except Exception as e:
            np_logger.error(f"BPSL system brake detection error: {e}")
            # Conservative: assume system braking on error
            return True
    
    def is_manual_braking(self, brake_position: float, driving_context: Dict[str, Any]) -> bool:
        """
        Determine if current braking is manual (driver-initiated) vs system braking
        
        Manual braking criteria:
        1. Brake pressure above manual threshold
        2. No system braking indicators active
        3. No immediate safety threats
        """
        # Must have significant brake pressure for manual braking
        if brake_position < self.MANUAL_BRAKE_THRESHOLD:
            return False
        
        # Check if system braking is active
        if self.detect_system_braking(driving_context):
            return False
        
        # If we get here, likely manual braking
        return True
    
    def detect_learning_events(self, brake_position: float, current_speed: float, driving_context: Dict[str, Any]):
        """
        Detect when to start/stop learning from driver braking behavior
        
        Learning Logic:
        - Manual brake pressed → Start learning mode, record start speed
        - Brake released → Learn final speed if significant decrease and was manual
        """
        # Detect start of deceleration (manual brake press)
        if (brake_position > self.BRAKE_THRESHOLD and 
            self.last_brake_position <= self.BRAKE_THRESHOLD and
            self.is_manual_braking(brake_position, driving_context)):
            
            self.learning_mode = True
            self.deceleration_start_speed = current_speed
            self.system_brake_active = False  # Reset system brake flag
            
            if self.enabled:
                np_logger.info(f"BPSL: Manual brake learning started at {current_speed:.1f} m/s")
        
        # Detect end of deceleration (brake release)
        elif (brake_position <= self.BRAKE_THRESHOLD and 
              self.last_brake_position > self.BRAKE_THRESHOLD):
            
            if (self.learning_mode and 
                self.deceleration_start_speed is not None and
                not self.system_brake_active):
                
                speed_decrease = self.deceleration_start_speed - current_speed
                
                # Learn if meaningful speed decrease
                if speed_decrease >= self.MIN_LEARNING_DECREASE:
                    old_target = self.learned_target_speed
                    # ✅ FIXED: Speed bounds checking with reasonable limits
                    self.learned_target_speed = max(1.0, min(current_speed, 45.0))  # Clamp between 1-45 m/s
                    self.learning_timestamp = time.time()
                    
                    if self.enabled:
                        np_logger.info(f"BPSL: Learned new target {current_speed:.1f} m/s "
                                     f"(was {old_target:.1f if old_target else 'none'}, "
                                     f"decrease: {speed_decrease:.1f} m/s)")
                elif self.enabled:
                    np_logger.debug(f"BPSL: No learning - decrease {speed_decrease:.1f} m/s < threshold")
            
            self.learning_mode = False
        
        # Update system braking detection during braking
        elif brake_position > self.BRAKE_THRESHOLD and self.learning_mode:
            if self.detect_system_braking(driving_context):
                self.system_brake_active = True
                if self.enabled:
                    np_logger.debug("BPSL: System braking detected, will not learn on release")
        
        self.last_brake_position = brake_position
    
    def process(self, speed_target: float, driving_context: Dict[str, Any]) -> DCPFilterResult:
        """
        Process speed target through BPSL logic
        
        Returns learned target speed if available, otherwise original target.
        Coordinates with APSL through timestamp-based priority.
        """
        # Periodic parameter updates
        if hasattr(self, 'param_read_counter'):
            self.param_read_counter += 1
            if self.param_read_counter % 100 == 0:  # Every 1 second at 100Hz
                self.read_params()
        else:
            self.param_read_counter = 0
        
        # Get current context
        v_ego = driving_context.get('v_ego', 0.0)
        brake_position = self.get_driver_brake_position(driving_context)
        
        # Early exit if disabled or too slow
        if not self.enabled:
            return DCPFilterResult(
                speed_modifier=1.0,
                active=False,
                reason="BPSL disabled",
                priority=self.priority
            )
        
        if v_ego < self.MIN_ENGAGEMENT_SPEED:
            return DCPFilterResult(
                speed_modifier=1.0,
                active=False,
                reason="Speed too low for BPSL",
                priority=self.priority
            )
        
        # Core BPSL Logic: Detect learning events
        self.detect_learning_events(brake_position, v_ego, driving_context)
        
        # Function 1: Keep DCP active during manual braking (no disengagement)
        is_manually_braking = (brake_position > self.BRAKE_THRESHOLD and 
                              self.is_manual_braking(brake_position, driving_context))
        
        # Function 2: Return learned target speed (coordinate with APSL)
        if self.learned_target_speed is not None:
            # Check coordination with APSL
            speed_modifier = self._coordinate_with_apsl(speed_target, driving_context)
        else:
            speed_modifier = 1.0
        
        # Generate status
        if is_manually_braking:
            if self.system_brake_active:
                reason = f"System brake detected ({brake_position:.2f}) - not learning"
            else:
                reason = f"Manual brake override ({brake_position:.2f})"
                if self.learning_mode:
                    reason += " - Learning"
        elif self.learned_target_speed:
            reason = f"Learned target: {self.learned_target_speed:.1f} m/s"
        else:
            reason = "Monitoring for manual braking"
        
        return DCPFilterResult(
            speed_modifier=speed_modifier,
            active=is_manually_braking or (self.learned_target_speed is not None),
            reason=reason,
            priority=self.priority
        )
    
    def _coordinate_with_apsl(self, speed_target: float, driving_context: Dict[str, Any]) -> float:
        """
        Coordinate BPSL learned target with APSL
        
        Logic: Most recent learning wins
        - Compare BPSL timestamp with APSL timestamp
        - Use most recent learned target
        """
        try:
            # Try to get APSL status from driving context or controller
            apsl_timestamp = None
            apsl_target = None
            
            # ⚠️ INCOMPLETE: APSL coordination is placeholder implementation
            # TODO: Implement proper inter-controller communication for dual-pedal learning
            # This would be enhanced with direct APSL controller access
            # For now, use simple coordination
            
            if self.learned_target_speed is not None and self.learning_timestamp is not None:
                # BPSL has a learned target
                # Validate speed_target before division to prevent unrealistic calculations
                if speed_target < 1.0:  # Less than 1 m/s (3.6 km/h) - too slow for meaningful speed modification
                    np_logger.warning(f"BPSL: Speed target too low ({speed_target:.2f} m/s), using neutral modifier")
                    return 1.0
                
                speed_modifier = self.learned_target_speed / speed_target
                speed_modifier = max(0.3, min(speed_modifier, 2.0))  # Safety bounds
                return speed_modifier
            
            return 1.0
            
        except Exception as e:
            np_logger.error(f"BPSL coordination error: {e}")
            return 1.0
    
    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive BPSL status"""
        return {
            'enabled': self.enabled,
            'learning_mode': self.learning_mode,
            'learned_target_speed': self.learned_target_speed,
            'learning_timestamp': self.learning_timestamp,
            'last_brake_position': round(self.last_brake_position, 3),
            'system_brake_active': self.system_brake_active,
            'lead_car_detected': self.lead_car_detected,
            'fcw_active': self.fcw_active,
            'manual_brake_threshold': self.MANUAL_BRAKE_THRESHOLD,
            'min_learning_decrease': self.MIN_LEARNING_DECREASE
        }
    
    def update_parameters(self, params: Params):
        """Update parameters from Params (called by DCP)"""
        self.params = params
        self.read_params()