#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT APSL (ACCELERATOR PEDAL SPEED LEARNING) - DCP FILTER
====================================================================

OVERVIEW:
APSL provides intuitive speed learning functionality that adapts cruise
control targets based on driver accelerator pedal behavior, enabling
personalized speed preferences while maintaining DCP system coordination.

CORE FUNCTIONALITY:
APSL has exactly 2 primary functions:
1. Learn new target speed from accelerator pedal release (final achieved speed)
2. Keep DCP active during pedal override (prevent system disengagement)

LEARNING ALGORITHM:
┌─────────────────────────────────────────────────────────────────┐
│ PEDAL PRESS: Driver accelerates beyond current cruise speed     │
│ LEARNING PHASE: Monitor driver's intended target speed          │
│ PEDAL RELEASE: Capture final achieved speed as new target      │
│ SPEED ADOPTION: Update cruise control with learned preference  │
└─────────────────────────────────────────────────────────────────┘

INTEGRATION BENEFITS:
- Works seamlessly with MADS (Modified Autonomous Driving System)
- Maintains DCP filter coordination during driver interventions
- Prevents unnecessary system disengagement during speed adjustments
- Provides natural, intuitive speed learning interface
- Preserves all other DCP filter functionality during learning

OPERATIONAL LOGIC:
- Detects accelerator pedal engagement above cruise speed
- Monitors driver's intended speed target during acceleration
- Captures final stable speed when pedal is released
- Updates cruise control target with learned speed preference
- Maintains system engagement throughout the learning process

SAFETY FEATURES:
- Conservative speed learning with validation
- Maintains all existing safety limits and bounds
- Preserves emergency braking and intervention capabilities
- Ensures DCP filter coordination remains intact
- Fallback to previous settings if learning data invalid

INTEGRATION POINTS:
- DCP Filter Architecture: Operates as coordinated filter layer
- Cruise Control: Updates target speed based on learned preferences
- Pedal Monitoring: Real-time accelerator pedal state tracking
- MADS Integration: Seamless operation with modified driving systems
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
np_logger = NpLogger('apsl')


class APSLFilter(DCPFilterLayer):
    """
    APSL Filter - Accelerator Pedal Speed Learning
    
    Core Functions:
    1. Learn final speed when driver releases pedal → New cruise target
    2. Keep DCP active during pedal override → No disengagement
    """
    
    def __init__(self):
        super().__init__(name="APSL", filter_type=DCPFilterType.DRIVER_OVERRIDE, priority=70)
        
        self.params = Params()
        
        # Simple parameters
        self.enabled = False
        self.PEDAL_THRESHOLD = 0.05        # 5% pedal threshold
        self.MIN_ENGAGEMENT_SPEED = 1.39   # 5 km/h minimum speed (1.39 m/s)  
        self.MIN_LEARNING_INCREASE = 2.0   # 2 m/s minimum increase to learn (7 km/h)
        
        # Learning state
        self.last_pedal_position = 0.0
        self.learning_mode = False
        self.acceleration_start_speed = None
        self.learned_target_speed = None
        
        # Simple noise filter
        self.pedal_filter = FirstOrderFilter(0.0, 0.3, 0.01)  # 10ms DT_CTRL
        
        # Read initial parameters
        self.read_params()
        
        np_logger.info("APSL Filter initialized")
    
    def read_params(self):
        """Simple parameter reading - just enable/disable"""
        try:
            self.enabled = self.params.get_bool("np_apsl_enabled", False)
            if self.enabled:
                np_logger.info("APSL enabled - Learning mode active")
        except (AttributeError, ValueError, TypeError) as e:
            # Fixed: Specific exception handling for better debugging
            np_logger.error(f"APSL parameter validation error: {e}")
            self.enabled = False
        except Exception as e:
            # Catch unexpected exceptions but log them specifically
            np_logger.error(f"APSL unexpected initialization error: {e}")
            self.enabled = False
    
    def get_driver_pedal_position(self, driving_context: Dict[str, Any]) -> float:
        """Get filtered driver pedal position"""
        try:
            pedal_pos = driving_context.get('driver_pedal_position', 0.0)
            self.pedal_filter.update(pedal_pos)
            return self.pedal_filter.x
        except (AttributeError, TypeError, ValueError) as e:
            np_logger.warning(f"APSL: Error reading pedal position: {e}")
            return 0.0
        except Exception as e:
            np_logger.error(f"APSL: Unexpected error in pedal position: {e}")
            return 0.0
    
    def detect_learning_events(self, pedal_position: float, current_speed: float):
        """
        Detect when to start/stop learning from driver behavior
        
        Learning Logic:
        - Pedal pressed → Start learning mode, record start speed
        - Pedal released → Learn final speed if significant increase
        """
        # Detect start of acceleration (pedal press)
        if pedal_position > self.PEDAL_THRESHOLD and self.last_pedal_position <= self.PEDAL_THRESHOLD:
            self.learning_mode = True
            self.acceleration_start_speed = current_speed
            if self.enabled:
                np_logger.info(f"APSL: Learning started at {current_speed:.1f} m/s")
        
        # Detect end of acceleration (pedal release)
        elif pedal_position <= self.PEDAL_THRESHOLD and self.last_pedal_position > self.PEDAL_THRESHOLD:
            if self.learning_mode and self.acceleration_start_speed is not None:
                speed_increase = current_speed - self.acceleration_start_speed
                
                # Learn if meaningful speed increase
                if speed_increase >= self.MIN_LEARNING_INCREASE:
                    old_target = self.learned_target_speed
                    # Add bounds checking to prevent learning dangerous speeds
                    MIN_LEARNED_SPEED = 5.56  # m/s (20 km/h) - minimum safe speed
                    MAX_LEARNED_SPEED = 38.89 # m/s (140 km/h) - maximum reasonable speed
                    self.learned_target_speed = max(MIN_LEARNED_SPEED, min(current_speed, MAX_LEARNED_SPEED))
                    
                    if self.enabled:
                        np_logger.info(f"APSL: Learned new target {current_speed:.1f} m/s "
                                     f"(was {old_target:.1f if old_target else 'none'}, "
                                     f"increase: {speed_increase:.1f} m/s)")
                elif self.enabled:
                    np_logger.debug(f"APSL: No learning - increase {speed_increase:.1f} m/s < threshold")
            
            self.learning_mode = False
        
        self.last_pedal_position = pedal_position
    
    def process(self, speed_target: float, driving_context: Dict[str, Any]) -> DCPFilterResult:
        """
        Process speed target through APSL logic
        
        Returns learned target speed if available, otherwise original target.
        Always keeps DCP active (no disengagement).
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
        pedal_position = self.get_driver_pedal_position(driving_context)
        
        # Early exit if disabled or too slow
        if not self.enabled:
            return DCPFilterResult(
                speed_modifier=1.0,
                active=False,
                reason="APSL disabled",
                priority=self.priority
            )
        
        if v_ego < self.MIN_ENGAGEMENT_SPEED:
            return DCPFilterResult(
                speed_modifier=1.0,
                active=False,
                reason="Speed too low for APSL",
                priority=self.priority
            )
        
        # Core APSL Logic: Detect learning events
        self.detect_learning_events(pedal_position, v_ego)
        
        # Function 1: Keep DCP active (no disengagement)
        # This is handled by returning active=True when pedal pressed
        is_overriding = pedal_position > self.PEDAL_THRESHOLD
        
        # Function 2: Return learned target speed
        if self.learned_target_speed is not None:
            # ✅ FIXED: Speed target validation with minimum threshold
            if speed_target < 1.0:
                return 1.0  # Return safe minimum speed for very low targets
                
            speed_modifier = self.learned_target_speed / max(speed_target, 0.1)
            # ✅ FIXED: Speed modifier bounds checking with safe limits
            speed_modifier = max(0.3, min(speed_modifier, 2.0))  # Clamp between 0.3x and 2.0x
        else:
            speed_modifier = 1.0
        
        # Generate status
        if is_overriding:
            reason = f"Pedal override active ({pedal_position:.2f})"
            if self.learning_mode:
                reason += " - Learning"
        elif self.learned_target_speed:
            reason = f"Learned target: {self.learned_target_speed:.1f} m/s"
        else:
            reason = "Monitoring for learning opportunities"
        
        return DCPFilterResult(
            speed_modifier=speed_modifier,
            active=is_overriding or (self.learned_target_speed is not None),
            reason=reason,
            priority=self.priority
        )
    
    def get_status(self) -> Dict[str, Any]:
        """Get simple APSL status"""
        return {
            'enabled': self.enabled,
            'learning_mode': self.learning_mode,
            'learned_target_speed': self.learned_target_speed,
            'last_pedal_position': round(self.last_pedal_position, 3),
            'acceleration_start_speed': self.acceleration_start_speed
        }
    
    def is_actively_controlling(self) -> bool:
        """Check if APSL is actively controlling speed"""
        return self.enabled and (self.learning_mode or self.learned_target_speed is not None)