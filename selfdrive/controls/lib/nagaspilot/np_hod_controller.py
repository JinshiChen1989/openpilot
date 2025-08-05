#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT HOD (HANDS OFF DURATION) - SAFETY MONITORING SYSTEM
====================================================================

OVERVIEW:
HOD implements a configurable hands-off driving timer that overlays on top
of OpenPilot's existing steering detection system, allowing extended hands-off
periods with progressive safety warnings and interventions.

CORE FUNCTIONALITY:
- Configurable timeout durations: 2min, 5min, 10min, or Forever
- Two-stage safety system: warning alerts followed by speed reduction
- Integration with existing OpenPilot steering detection logic
- Progressive warning escalation for user awareness
- Safe fallback behavior when limits are exceeded

SAFETY ARCHITECTURE:
┌─────────────────┐    Timer Expired     ┌─────────────────┐
│  HANDS-OFF      │ ───────────────────▶ │   WARNING       │
│  (Normal Op)    │                      │  (Visual/Audio) │
│                 │ ◀─────────────────── │                 │
└─────────────────┘    Hands Detected    └─────────────────┘
                                                  │
                                                  ▼ Warning Ignored
                                         ┌─────────────────┐
                                         │  DECELERATION   │
                                         │ (Speed Reduction)│
                                         └─────────────────┘

OPERATIONAL MODES:
- Level 0: 2 minutes hands-off (conservative, city driving)
- Level 1: 5 minutes hands-off (balanced, mixed driving)
- Level 2: 10 minutes hands-off (extended, highway driving)
- Level 3: Forever hands-off (advanced users, testing)

SAFETY FEATURES:
- Reuses OpenPilot's proven steeringPressed detection logic
- Progressive warning system prevents sudden interventions
- Configurable thresholds for different driving scenarios
- Integration with existing safety monitoring systems
- Graceful degradation when limits exceeded

INTEGRATION POINTS:
- OpenPilot CarState: Uses existing steering detection
- NagasPilot Logger: Consistent logging infrastructure
- Safety Systems: Coordinates with other monitoring modules
- Parameter System: User-configurable timeout settings

INTEGRATION:
- controlsd.py calls get_status() for complete status information
- Message protocol fields @55-@56 publish telemetry data
- Parameters: np_hod_enabled, np_hod_duration_level
- Affects lateral control and driver monitoring awareness

SAFETY:
- Progressive timeout system (Stage 1: Warning, Stage 2: Deceleration)
- All conditions checked internally (enabled state, parameter validation)
- Safe defaults on error conditions
- Immediate recovery on steering input detection
"""
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for HOD module - consistent with nagaspilot architecture
np_logger = NpLogger('hod')

class SimpleHODTimer:
    """
    Hand Off Duration Timer - Configurable hands-off driving controller
    
    Manages hands-off timeout logic with user-configurable duration settings.
    Integrates with existing OpenPilot steering detection and driver monitoring.
    """
    def __init__(self):
        """Initialize HOD timer with parameter system access and state tracking"""
        self.params = Params()  # Access to nagaspilot parameter system
        self.start_time = None  # Track when hands-off period began
        self.first_timeout_reached = False  # Track warning stage activation
        
    def get_status(self, CS, current_time):
        """
        Get complete HOD status including all telemetry data
        
        Args:
            CS: CarState object with steering and vehicle status
            current_time: Current monotonic time for timer calculations
            
        Returns:
            dict: Complete HOD status with fields:
                - enabled: bool - HOD feature enabled state
                - active: bool - Currently hands-off and timing
                - timeout_reached: bool - Warning stage timeout reached
                - awarenessStatus: float - Driver monitoring awareness level (0.0-1.0)
                - session_time: float - Current hands-off session duration
                - duration_level: int - User-selected duration setting (0-3)
        """
        # Input validation - critical for safety
        if CS is None or current_time is None: 
            np_logger.warning("Invalid input - CS or current_time is None")
            return {
                'enabled': False,
                'active': False,
                'timeout_reached': False,
                'awarenessStatus': 1.0,  # Safe default - full awareness
                'session_time': 0.0,
                'duration_level': 0
            }
        
        # Check if HOD is enabled via parameter system
        enabled = self.params.get_bool("np_hod_enabled", False)
        if not enabled:
            np_logger.debug("HOD disabled - returning inactive status")
            return {
                'enabled': False,
                'active': False,
                'timeout_reached': False,
                'awarenessStatus': 1.0,  # Full awareness when disabled
                'session_time': 0.0,
                'duration_level': 0
            }
            
        # Get user-configured duration level with bounds checking for safety
        # ✅ FIXED: Exception handling for parameter access
        try:
            duration_level = max(0, min(self.params.get_int("np_hod_duration_level", 0), 3))
        except (ValueError, TypeError) as e:
            np_logger.warning(f"Invalid HOD duration level parameter: {e}, using default")
            duration_level = 0  # Default to 2-minute timeout
        timeout = [120, 300, 600, float('inf')][duration_level]  # 2min, 5min, 10min, Forever
        
        # Forever mode (level 3) - infinite timeout, never triggers warnings/deceleration
        if timeout == float('inf'):
            np_logger.debug("Forever mode active - timeout disabled")
            return {
                'enabled': True,
                'active': not CS.steeringPressed,  # Active if hands off
                'timeout_reached': False,
                'awarenessStatus': 1.0,  # Full awareness in forever mode
                'session_time': 0.0 if self.start_time is None else current_time - self.start_time,
                'duration_level': duration_level
            }
        
        # Main logic: Check steering input status (reuse existing OpenPilot detection)
        # ⚠️ DEPENDENCY: Relies on OpenPilot's steering detection accuracy
        # TODO: Consider adding torque-based validation for better hand detection
        if not CS.steeringPressed:  # Hands off - start/continue timer
            # Initialize timer on first hands-off detection
            if self.start_time is None:
                self.start_time = current_time
                np_logger.info(f"Hands off detected - timeout in {timeout}s (level {duration_level})")
            
            # Calculate elapsed hands-off time
            elapsed_time = current_time - self.start_time
            
            # Progressive timeout system: Stage 1 (Warning) → Stage 2 (Deceleration)
            if elapsed_time >= timeout:
                # Stage 1: First timeout reached - enter warning state
                if not self.first_timeout_reached:
                    np_logger.warning(f"FIRST TIMEOUT REACHED - Hands off for {elapsed_time:.1f}s >= {timeout}s")
                self.first_timeout_reached = True
                
                # Stage 2: Timeout + grace period - trigger deceleration
                if elapsed_time >= (timeout + 60):  # +60s grace period for driver response
                    np_logger.error(f"DECELERATION TRIGGERED - Hands off for {elapsed_time:.1f}s >= {timeout + 60}s")
                    # ⚠️ SAFETY: Direct awareness manipulation affects vehicle control
                    # TODO: Consider more gradual deceleration profile for safety
                    awarenessStatus = 0.0  # Trigger deceleration (driver monitoring system)
                else:
                    # Grace period - warning state but no deceleration yet
                    remaining_grace = (timeout + 60) - elapsed_time
                    np_logger.warning(f"WARNING STATE - Hands off for {elapsed_time:.1f}s, deceleration in {remaining_grace:.1f}s")
                    awarenessStatus = 0.5  # Warning level (alerts but no deceleration)
            else:
                # Normal operation - within allowed hands-off time
                awarenessStatus = 1.0  # Full awareness - no warnings
                
            return {
                'enabled': True,
                'active': True,  # Currently in hands-off state and timing
                'timeout_reached': self.first_timeout_reached,
                'awarenessStatus': awarenessStatus,
                'session_time': elapsed_time,
                'duration_level': duration_level
            }
        else:  # Hands on steering - recovery mechanism: reset all state
            # Log recovery event for debugging (only if timer was active)
            if self.start_time is not None:
                elapsed = current_time - self.start_time
                np_logger.info(f"Hands on steering detected - recovery after {elapsed:.1f}s")
            
            # Reset all timer state for next hands-off session
            self.start_time = None
            self.first_timeout_reached = False
            return {
                'enabled': True,
                'active': False,  # Not hands-off anymore
                'timeout_reached': False,
                'awarenessStatus': 1.0,  # Full awareness when hands on
                'session_time': 0.0,
                'duration_level': duration_level
            }
    
    def update(self, CS, current_time):
        """
        Legacy update method for backward compatibility with existing controlsd.py integration
        
        Args:
            CS: CarState object with steering and vehicle status
            current_time: Current monotonic time for timer calculations
            
        Returns:
            dict: Legacy format with timeout_reached and awarenessStatus for existing integration points
        """
        status = self.get_status(CS, current_time)
        return {
            'timeout_reached': status['timeout_reached'], 
            'awarenessStatus': status['awarenessStatus']
        }

# HOD Integration Notes:
# - Imported and instantiated in controlsd.py as self.hod_timer = SimpleHODTimer()
# - Called via get_status() for complete telemetry data
# - Legacy update() method maintained for existing lateral control and DM integration
# - Message protocol fields @55-@56 publish status to external systems
# - awarenessStatus affects driver monitoring system behavior (0.0=decel, 0.5=warning, 1.0=normal)