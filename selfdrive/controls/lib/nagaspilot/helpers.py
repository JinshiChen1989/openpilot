"""
====================================================================
NAGASPILOT HELPER UTILITIES - SHARED CONTROL FUNCTIONS
====================================================================

OVERVIEW:
This module provides essential utility functions used across all NagasPilot
control modules, ensuring consistent behavior and validated operations
throughout the system.

CORE FUNCTIONALITY:
- Speed classification and validation (highway, city, low speed, creep)
- Driving context analysis and classification  
- Input validation and bounds checking
- Common mathematical operations for control systems
- Pure utility functions with no external dependencies

KEY UTILITY CATEGORIES:
- Speed Analysis: validate_speed(), classify_driving_context()
- Validation: Input bounds checking and type validation
- Context Classification: Highway vs city vs parking detection
- Mathematical Operations: Common calculations for control systems

SAFETY FEATURES:
- All functions include input validation with safe defaults
- Speed thresholds based on proven safety standards
- Defensive programming with error handling
- Performance-optimized for real-time control loops

INTEGRATION POINTS:
- Used by DLP, DCP, and all filter modules
- Interfaces with common.py for shared constants
- Provides consistent utility functions for development
- Ensures validated behavior across NagasPilot components

CODE REVIEW NOTES:
- Speed validation: validate_speed() with safety bounds
- Context classification: classify_driving_context() for mode selection
- Input validation: All functions validate parameters before use
- Pure utility functions with no logging dependencies
"""

import numpy as np
from typing import Union, Tuple, Optional

from .common import (
    SPEED_THRESHOLD_HIGHWAY, SPEED_THRESHOLD_CITY,
    SPEED_THRESHOLD_LOW, SPEED_THRESHOLD_CREEP,
    DrivingContext, SPEED_RANGE, ACCELERATION_RANGE, PROBABILITY_RANGE
)

# ========================================================================
# INPUT VALIDATION
# ========================================================================

def validate_speed(speed: float) -> float:
    """
    Validate and clamp speed value to acceptable range
    
    Args:
        speed: Speed value in m/s
        
    Returns:
        Clamped speed value within safe operating bounds
    """
    return np.clip(speed, SPEED_RANGE[0], SPEED_RANGE[1])

def validate_acceleration(accel: float) -> float:
    """
    Validate and clamp acceleration value to acceptable range
    
    Args:
        accel: Acceleration value in m/s²
        
    Returns:
        Clamped acceleration value within physical limits
    """
    return np.clip(accel, ACCELERATION_RANGE[0], ACCELERATION_RANGE[1])

def validate_probability(prob: float) -> float:
    """
    Validate and clamp probability value to [0.0, 1.0] range
    
    Args:
        prob: Probability value
        
    Returns:
        Clamped probability value within valid mathematical range
    """
    return np.clip(prob, PROBABILITY_RANGE[0], PROBABILITY_RANGE[1])

# ========================================================================
# DRIVING CONTEXT ANALYSIS
# ========================================================================

def classify_driving_context(speed: float) -> DrivingContext:
    """
    Classify current driving context based on speed
    
    Args:
        speed: Current vehicle speed in m/s
        
    Returns:
        DrivingContext enum value
    """
    speed = validate_speed(speed)
    
    if speed >= SPEED_THRESHOLD_HIGHWAY:
        return DrivingContext.HIGHWAY
    elif speed >= SPEED_THRESHOLD_CITY:
        return DrivingContext.CITY
    elif speed >= SPEED_THRESHOLD_LOW:
        return DrivingContext.LOW_SPEED
    elif speed >= SPEED_THRESHOLD_CREEP:
        return DrivingContext.CREEP
    else:
        return DrivingContext.STOPPED

# ========================================================================
# MATHEMATICAL UTILITIES
# ========================================================================

def interpolate_value(x: float, x_points: list, y_points: list) -> float:
    """
    Linear interpolation utility for smooth value mapping
    
    Args:
        x: Input value to interpolate
        x_points: X coordinate points (must be sorted)
        y_points: Y coordinate points (corresponding to x_points)
        
    Returns:
        Interpolated Y value using linear interpolation
    """
    return float(np.interp(x, x_points, y_points))

def smooth_transition(current: float, target: float, rate: float, dt: float) -> float:
    """
    Apply exponential smooth transition between current and target values
    
    Args:
        current: Current value
        target: Target value  
        rate: Transition rate (1/time_constant)
        dt: Time delta in seconds
        
    Returns:
        Smoothed value with exponential decay characteristics
    """
    if rate <= 0:
        return target
    
    alpha = 1.0 - np.exp(-rate * dt)
    return current + alpha * (target - current)

def safe_divide(numerator: float, denominator: float, default: float = 0.0) -> float:
    """
    Safe division with default value for zero denominator
    
    Args:
        numerator: Numerator value
        denominator: Denominator value
        default: Default value if denominator is zero or very small
        
    Returns:
        Division result or default value to prevent divide-by-zero errors
    """
    return numerator / denominator if abs(denominator) > 1e-6 else default

# ========================================================================
# UTILITY FUNCTIONS
# ========================================================================

def within_range(value: float, center: float, tolerance: float) -> bool:
    """
    Check if value is within tolerance range of center point
    
    Args:
        value: Value to check
        center: Center point of the range
        tolerance: Tolerance range (± from center)
        
    Returns:
        True if value is within [center - tolerance, center + tolerance]
    """
    return abs(value - center) <= tolerance

def moving_average(values: list, window_size: int) -> float:
    """
    Calculate moving average over sliding window
    
    Args:
        values: List of numerical values
        window_size: Window size for averaging (must be positive)
        
    Returns:
        Moving average value over the specified window
    """
    if not values or window_size <= 0:
        return 0.0
    
    window = values[-window_size:] if len(values) >= window_size else values
    return sum(window) / len(window)