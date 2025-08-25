#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT SSD (STAND STILL DURATION) - SAFETY TIMEOUT SYSTEM
====================================================================

OVERVIEW:
SSD implements a configurable standstill timeout system that prevents
automatic cruise control resume after extended stop periods, enhancing
safety and driver awareness during stop-and-go traffic scenarios.

CORE FUNCTIONALITY:
- Simple timer overlay on existing OpenPilot standstill detection
- Configurable timeout functionality: 2min, 5min, 10min, or Forever
- Prevents unintended auto-resume after extended stationary periods
- Integration with existing cruise control resume logic
- Driver notification when timeout period is reached

SAFETY ARCHITECTURE:
┌─────────────────────────────────────────────────────────────────┐
│ STANDSTILL DETECTION: Uses proven OpenPilot standstill logic    │
│ TIMEOUT MONITORING: Configurable duration before auto-disable  │
│ RESUME PREVENTION: Blocks automatic cruise resume after timeout │
│ DRIVER NOTIFICATION: Clear indication when manual resume needed │
└─────────────────────────────────────────────────────────────────┘

TIMEOUT LEVELS:
- Level 0: 2 minutes (conservative, city driving)
- Level 1: 5 minutes (balanced, mixed traffic)
- Level 2: 10 minutes (extended, highway scenarios)  
- Level 3: Forever (no timeout, advanced users)

SAFETY BENEFITS:
- Prevents unintended acceleration after long stops
- Ensures driver awareness before cruise resume
- Reduces risk of surprise acceleration in parking situations
- Provides configurable safety margins for different scenarios
- Maintains driver engagement during extended stops

INTEGRATION POINTS:
- OpenPilot Standstill Detection: Uses existing proven logic
- Cruise Control System: Integrates with resume functionality
- Driver Interface: Clear timeout notifications and status
- NagasPilot Logger: Consistent logging and monitoring

INTEGRATION:
- controlsd.py calls get_status() for complete status information
- Message protocol fields @51-@54 publish telemetry data
- Parameters: np_ssd_enabled, np_ssd_duration_level

SAFETY:
- All conditions checked internally (enabled state, parameter validation)
- Safe defaults on error conditions
- Graceful degradation when disabled
"""
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for SSD module - consistent with nagaspilot architecture
np_logger = NpLogger('ssd')

class SimpleSSDTimer:
    """
    Stand Still Duration Timer - Configurable standstill timeout controller
    
    Manages standstill timeout logic with user-configurable duration settings.
    Integrates with existing OpenPilot standstill detection without replacing core logic.
    """
    def __init__(self):
        """Initialize SSD timer with parameter system access"""
        self.params = Params()  # Access to nagaspilot parameter system
        self.start_time = None  # Track when standstill began
        
    def get_status(self, CS, current_time):
        """
        Get complete SSD status including all telemetry data
        
        Args:
            CS: CarState object with vehicle status
            current_time: Current monotonic time for timer calculations
            
        Returns:
            dict: Complete SSD status with fields:
                - enabled: bool - SSD feature enabled state
                - active: bool - Currently in standstill and timing
                - timeout_reached: bool - Timeout period exceeded
                - time_remaining: float - Seconds remaining until timeout
                - duration_level: int - User-selected duration setting (0-3)
        """
        # Input validation - critical for safety
        if CS is None or current_time is None: 
            np_logger.warning("Missing CS or current_time data - using safe defaults")
            return {
                'enabled': False,
                'active': False,
                'timeout_reached': False,
                'time_remaining': 0.0,
                'duration_level': 0
            }
        
        # Check if SSD is enabled via parameter system
        try:
            enabled = self.params.get_bool("np_ssd_enabled", False)
        except Exception as e:
            np_logger.error(f"Failed to read np_ssd_enabled parameter: {e}")
            enabled = False
        if not enabled:
            np_logger.debug("SSD disabled - returning inactive status")
            return {
                'enabled': False,
                'active': False,
                'timeout_reached': False,
                'time_remaining': 0.0,
                'duration_level': 0
            }
            
        # Get user-configured duration level with bounds checking for safety
        try:
            duration_level = max(0, min(self.params.get_int("np_ssd_duration_level", 0), 3))
        except Exception as e:
            np_logger.error(f"Failed to read np_ssd_duration_level parameter: {e}")
            duration_level = 0  # Safe default
        timeout = [120, 300, 600, float('inf')][duration_level]  # 2min, 5min, 10min, Forever
        
        # Forever mode (level 3) - infinite timeout, never triggers nagaspilot timeout
        if timeout == float('inf'):
            np_logger.debug("Forever mode active - timeout disabled")
            return {
                'enabled': True,
                'active': False,  # Not "active" since no timeout will occur
                'timeout_reached': False,
                'time_remaining': float('inf'),
                'duration_level': duration_level
            }
        
        # Main logic: Check if vehicle is in standstill (reuse existing OpenPilot detection)
        # ✅ FIXED: Added redundant speed-based validation for safety
        ego_speed_ms = max(0.0, CS.vEgo)  # Vehicle speed in m/s
        standstill_confirmed = CS.cruiseState.standstill or ego_speed_ms < 0.1  # <0.36 km/h
        
        if standstill_confirmed:  # Vehicle stopped - start/continue timer
            # Initialize timer on first standstill detection
            if self.start_time is None:
                self.start_time = current_time
                np_logger.info(f"Standstill started - timeout in {timeout}s (level {duration_level})")
            
            # Calculate elapsed time and remaining time until timeout
            elapsed = current_time - self.start_time
            timeout_reached = elapsed >= timeout
            time_remaining = max(0.0, timeout - elapsed)
            
            # Log timeout event for debugging/monitoring
            if timeout_reached:
                np_logger.warning(f"TIMEOUT REACHED - Standstill for {elapsed:.1f}s >= {timeout}s")
            
            return {
                'enabled': True,
                'active': True,  # Currently timing standstill duration
                'timeout_reached': timeout_reached,
                'time_remaining': time_remaining,
                'duration_level': duration_level
            }
        else:  # Vehicle moving - reset timer and return inactive state
            # Log standstill end for debugging (only if timer was active)
            if self.start_time is not None:
                elapsed = current_time - self.start_time
                np_logger.debug(f"Vehicle moving - standstill ended after {elapsed:.1f}s")
            
            # Reset timer state
            self.start_time = None
            return {
                'enabled': True,
                'active': False,  # Not in standstill anymore
                'timeout_reached': False,
                'time_remaining': 0.0,
                'duration_level': duration_level
            }
    
    def update(self, CS, current_time):
        """
        Legacy update method for backward compatibility with existing controlsd.py integration
        
        Args:
            CS: CarState object with vehicle status
            current_time: Current monotonic time for timer calculations
            
        Returns:
            dict: Legacy format with timeout_reached field for existing integration points
        """
        status = self.get_status(CS, current_time)
        return {'timeout_reached': status['timeout_reached']}

# SSD Integration Notes:
# - Imported and instantiated in controlsd.py as self.ssd_timer = SimpleSSDTimer()
# - Called via get_status() for complete telemetry data  
# - Legacy update() method maintained for existing resume logic integration
# - Message protocol fields @51-@54 publish status to external systems