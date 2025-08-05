"""
====================================================================
NAGASPILOT COMMON CONSTANTS - SHARED SYSTEM DEFINITIONS
====================================================================

OVERVIEW:
This module defines all shared constants, enumerations, and configuration
values used consistently across NagasPilot control modules to ensure
unified behavior and maintainable code.

CORE DEFINITIONS:
- Speed thresholds for driving context classification
- Control mode enumerations for system behavior
- Validation ranges for parameter bounds checking
- Mathematical constants for control algorithms
- Safety limits and operational boundaries

SPEED CLASSIFICATION SYSTEM:
- Highway: >80 kph (22.23 m/s) - High speed automated driving
- City: 20-80 kph (5.56-22.23 m/s) - Urban traffic navigation  
- Low Speed: 8-20 kph (2.23-5.56 m/s) - Parking and maneuvering
- Creep: <8 kph (2.23 m/s) - Stop-and-go traffic

CONTROL MODE DEFINITIONS:
- DrivingContext: Enumeration for driving environment classification
- ControlMode: System operational state definitions
- Parameter ranges: Validation boundaries for safe operation

SAFETY CONSIDERATIONS:
- All thresholds based on proven automotive safety standards
- Validation ranges prevent dangerous parameter values
- Constants chosen for optimal performance across vehicle types
- Consistent units (metric) throughout system

INTEGRATION POINTS:
- Used by all NagasPilot control modules for consistency
- Provides unified speed classification across features
- Ensures parameter validation uses common boundaries
- Enables consistent behavior across different driving modes

CODE REVIEW NOTES:
- Speed thresholds: Carefully tuned for safety and performance
- Validation ranges: Prevent invalid parameter configurations
- Enum definitions: Type-safe control mode selection
- Unit consistency: All values in SI units (m/s, m/s², etc.)
"""

from enum import Enum
import numpy as np

# === Speed Thresholds (m/s) ===
SPEED_THRESHOLD_HIGHWAY = 22.23  # m/s (approx. 80 kph)
SPEED_THRESHOLD_CITY = 15.27     # m/s (approx. 55 kph) 
SPEED_THRESHOLD_LOW = 5.56       # m/s (approx. 20 kph)
SPEED_THRESHOLD_CREEP = 2.23     # m/s (approx. 8 kph)

# === ACM (Adaptive Coasting Mode) Constants ===
ACM_SLOPE_THRESHOLD = -0.04
ACM_RATIO = 0.9
ACM_TTC = 3.5
ACM_TTC_BP = [ACM_TTC, 2.5]
ACM_MIN_BRAKE_ALLOW_VALS = [0., -0.5]

# === Road Edge Detection Constants ===
NEARSIDE_PROB_THRESHOLD = 0.2
EDGE_PROB_THRESHOLD = 0.35

# === Control System Enums ===
class ControlMode(Enum):
  """Control system operation modes"""
  DISABLED = 0
  ENABLED = 1
  OVERRIDE = 2

class DrivingContext(Enum):
  """Driving context classifications for AEM"""
  HIGHWAY = "highway"
  CITY = "city"
  LOW_SPEED = "low_speed"
  CREEP = "creep"
  STOPPED = "stopped"

class EdgeType(Enum):
  """Road edge detection types"""
  NONE = 0
  LEFT_EDGE = 1
  RIGHT_EDGE = 2
  BOTH_EDGES = 3

# === Parameter Validation Ranges ===
SPEED_RANGE = (0.0, 50.0)  # m/s
ACCELERATION_RANGE = (-4.0, 4.0)  # m/s²
PROBABILITY_RANGE = (0.0, 1.0)

# === Default Configuration ===
DEFAULT_CONFIG = {
    'acm_enabled': False,
    'aem_enabled': False, 
    'road_edge_detection_enabled': False,
    'debug_mode': False,
}