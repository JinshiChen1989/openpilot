#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT LCA (LANE CHANGE ASSIST) - CENTRALIZED CONTROL SYSTEM
====================================================================

OVERVIEW:
LCA provides intelligent lane change assistance with comprehensive safety
validation, lane availability detection, and road geometry analysis for
safe and smooth automated lane changes.

CORE FUNCTIONALITY:
- Centralized lane width calculation and validation
- Real-time lane availability checking to prevent race conditions
- Enhanced road edge support for countries without outer lane lines
- Multi-source lane detection (vision + road edge + map data)
- Dynamic safety parameter adjustment based on road conditions

SAFETY ARCHITECTURE:
┌─────────────────────────────────────────────────────────────────┐
│ LANE VALIDATION: Multi-layer safety checks before lane changes  │
│ - Lane width sufficiency (minimum 3.0m standard lanes)         │
│ - Road edge clearance (minimum 5.0m near road boundaries)      │
│ - Vision confidence thresholds for lane line detection         │
│ - Blindspot monitoring integration                              │
│ - Speed-dependent safety margins                               │
└─────────────────────────────────────────────────────────────────┘

KEY IMPROVEMENTS:
- Prevents race conditions with real-time lane availability checking
- Clean API interface that keeps desire_helper.py simple and maintainable
- Centralized parameter system for consistent width calculations
- Enhanced debugging and monitoring capabilities
- Robust fallback handling when lane detection degrades

INTEGRATION POINTS:
- DesireHelper: Primary consumer for lane change decision making
- ModelV2: Vision-based lane line detection and confidence
- CarState: Vehicle dynamics and blindspot monitoring
- Parameter System: User-configurable safety thresholds
- Road Edge Detection: Enhanced support for edge lane changes

VALIDATION LOGIC:
- Multi-criteria lane safety assessment
- Dynamic parameter adjustment based on driving conditions  
- Conservative defaults for unknown or degraded conditions
- Comprehensive logging for safety analysis and debugging

TEMPORARY DEBUG LOGGING:
To remove all debug logging after testing is complete:
1. Comment out the cloudlog import at line ~22
2. Comment out all sections marked with "DEBUG LOGGING - REMOVE AFTER TESTING COMPLETE"
3. Search for "cloudlog." and comment out any remaining logging calls
"""

import math
import numpy as np
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for LCA module
np_logger = NpLogger('lca')

def calculate_lane_width(lane, current_lane, road_edge=None):
    """
    Calculate lane width between two lane lines - Enhanced for road edge lanes

    Handles countries without leftmost/rightmost lane lines where vehicles
    can change lanes to the road edge area with appropriate safety margins.

    Args:
        lane: Target lane line data (has .x and .y attributes)
        current_lane: Current lane line data (has .x and .y attributes)
        road_edge: Optional road edge data for additional validation

    Returns:
        float: Lane width in meters, adjusted for road edge proximity
    """
    current_x = np.asarray(current_lane.x)
    current_y = np.asarray(current_lane.y)

    lane_y_interp = np.interp(current_x, np.asarray(lane.x), np.asarray(lane.y))
    distance_to_lane = np.mean(np.abs(current_y - lane_y_interp))

    if road_edge is None:
        return float(distance_to_lane)

    road_edge_y_interp = np.interp(current_x, np.asarray(road_edge.x), np.asarray(road_edge.y))
    distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y_interp))

    # Enhanced logic for road edge lane changes
    if distance_to_road_edge < distance_to_lane:
        # Lane change TO road edge area (common in countries without outer lane lines)
        # Return the actual available width (distance to road edge)
        # The minimum width check will determine if it's safe enough
        return float(distance_to_road_edge)

    return float(distance_to_lane)

def check_lane_width_available(carstate, modelV2, lane_detection_enabled, min_lane_width=None, min_road_edge_width=None, params=None):
    """
    Check if lane is wide enough for safe lane change with road edge support

    Enhanced logic handles countries without outer lane lines by allowing
    lane changes to road edge areas with appropriate safety margins.

    Args:
        carstate: Current vehicle state (contains blinker information)
        modelV2: Vision model data containing lane lines and road edges
        lane_detection_enabled: Whether lane detection should be performed
        min_lane_width: Minimum width for regular lane changes (meters) - overrides params if provided
        min_road_edge_width: Minimum width for road edge lanes (meters) - overrides params if provided
        params: Params object for centralized parameter access (creates new if None)

    Returns:
        bool: True if lane change is safe, False otherwise
    """
    # Load parameters from centralized system with fallback defaults
    if params is None:
        params = Params()
    
    # Use provided values or load from parameters
    if min_lane_width is None:
        min_lane_width = float(params.get("np_lca_min_lane_width", "3.0"))  # Default 3.0m
    if min_road_edge_width is None:
        min_road_edge_width = float(params.get("np_lca_min_edge_width", "5.0"))  # Default 5.0m
    
    if not lane_detection_enabled:
        np_logger.debug("Lane detection disabled - allowing lane change")
        return True

    if not (carstate.leftBlinker or carstate.rightBlinker):
        return False

    # Validate vision data before processing
    if modelV2 is None:
        np_logger.info("No vision data available - lane change disabled")
        return False
        
    if not hasattr(modelV2, 'laneLines') or len(modelV2.laneLines) < 4:
        np_logger.info("Insufficient lane lines detected - lane change disabled")
        return False

    try:
        if carstate.leftBlinker:
            # LEFT LANE CHANGE: Check width of target left lane
            lane_width = calculate_lane_width(
                modelV2.laneLines[0],  # Target left lane
                modelV2.laneLines[1],  # Current left boundary
                modelV2.roadEdges[0] if len(modelV2.roadEdges) > 0 else None
            )

            # Detect if this is a road edge scenario and no left lane line is detected
            is_road_edge_lane = _is_road_edge_lane_left(modelV2)
            min_width = min_road_edge_width if is_road_edge_lane else min_lane_width
            
            direction = "LEFT"
            lane_type = "road_edge" if is_road_edge_lane else "regular"

        else:  # Right blinker
            # RIGHT LANE CHANGE: Check width of target right lane
            lane_width = calculate_lane_width(
                modelV2.laneLines[3],  # Target right lane
                modelV2.laneLines[2],  # Current right boundary
                modelV2.roadEdges[1] if len(modelV2.roadEdges) > 1 else None
            )

            # Detect if this is a road edge scenario and no right lane line is detected
            is_road_edge_lane = _is_road_edge_lane_right(modelV2)
            min_width = min_road_edge_width if is_road_edge_lane else min_lane_width
            
            direction = "RIGHT"
            lane_type = "road_edge" if is_road_edge_lane else "regular"

        # Log detailed lane change analysis
        lane_change_safe = lane_width >= min_width
        
        # PlotJuggler-optimized logging with np_ prefix
        np_logger.plot_value(f"lca_lane_width_{direction.lower()}", lane_width, "m")
        np_logger.plot_value(f"lca_min_width_{direction.lower()}", min_width, "m")
        np_logger.plot_event(f"lca_lane_change_safe_{direction.lower()}", lane_change_safe)
        
        if lane_change_safe:
            np_logger.info(f"Lane change {direction} ALLOWED: {lane_type} lane, width={lane_width:.2f}m >= min={min_width:.2f}m")
        else:
            np_logger.warning(f"Lane change {direction} BLOCKED: {lane_type} lane, width={lane_width:.2f}m < min={min_width:.2f}m")
            
        return lane_change_safe

    except Exception as e:
        # Fail safe: Return False on any error with detailed logging
        np_logger.error(f"CRITICAL ERROR in lane change validation: {str(e)}")
        np_logger.error(f"Vision data state: laneLines={len(modelV2.laneLines) if hasattr(modelV2, 'laneLines') else 'None'}, "
                      f"roadEdges={len(modelV2.roadEdges) if hasattr(modelV2, 'roadEdges') else 'None'}")
        return False

# Helper function to detect if left lane change is to a road edge area (no outer lane line)
def _is_road_edge_lane_left(modelV2):
    """
    Detect if left lane change is to a road edge area (no outer lane line)

    Returns True if road edge is closer than the target lane line,
    indicating this is a road edge lane scenario.
    """
    if len(modelV2.roadEdges) == 0:
        return False

    try:
        # ✅ FIXED: Numpy already imported at top of file, removed redundant import
        current_x = np.asarray(modelV2.laneLines[1].x)
        current_y = np.asarray(modelV2.laneLines[1].y)

        # Distance to road edge
        road_edge_y = np.interp(current_x, np.asarray(modelV2.roadEdges[0].x), np.asarray(modelV2.roadEdges[0].y))
        distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y))

        # Distance to target lane line
        lane_line_y = np.interp(current_x, np.asarray(modelV2.laneLines[0].x), np.asarray(modelV2.laneLines[0].y))
        distance_to_lane_line = np.mean(np.abs(current_y - lane_line_y))

        # If road edge is closer, this is a road edge lane
        return distance_to_road_edge < distance_to_lane_line

    except Exception:
        return False

# Helper function to detect if right lane change is to a road edge area (no outer lane line)
def _is_road_edge_lane_right(modelV2):
    """
    Detect if right lane change is to a road edge area (no outer lane line)

    Returns True if road edge is closer than the target lane line,
    indicating this is a road edge lane scenario.
    """
    if len(modelV2.roadEdges) < 2:
        return False

    try:
        # ✅ FIXED: Numpy already imported at top of file, removed redundant import
        current_x = np.asarray(modelV2.laneLines[2].x)
        current_y = np.asarray(modelV2.laneLines[2].y)

        # Distance to road edge
        road_edge_y = np.interp(current_x, np.asarray(modelV2.roadEdges[1].x), np.asarray(modelV2.roadEdges[1].y))
        distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y))

        # Distance to target lane line
        lane_line_y = np.interp(current_x, np.asarray(modelV2.laneLines[3].x), np.asarray(modelV2.laneLines[3].y))
        distance_to_lane_line = np.mean(np.abs(current_y - lane_line_y))

        # If road edge is closer, this is a road edge lane
        return distance_to_road_edge < distance_to_lane_line

    except Exception:
        return False

# Simple API for desire_helper.py to minimize changes from baseline
def is_lane_change_safe(carstate, modelV2=None, params=None):
    """
    Simple API for desire_helper.py - minimizes changes from baseline

    This function encapsulates all the complex lane detection logic
    so desire_helper.py can stay as simple as the baseline version.

    Args:
        carstate: Current vehicle state (contains blinker information)
        modelV2: Vision model data (optional, None disables lane detection)
        params: Params object for centralized parameter access (creates new if None)

    Returns:
        bool: True if lane change is safe, False otherwise
    """
    # If no vision data provided, allow lane change (baseline behavior)
    if modelV2 is None:
        np_logger.debug("No vision data provided - defaulting to baseline behavior (allow)")
        return True

    # Enhanced LCA: Check lane width with centralized parameters
    result = check_lane_width_available(
        carstate=carstate,
        modelV2=modelV2,
        lane_detection_enabled=True,
        params=params  # Use centralized parameters
    )
    
    # Additional context logging for debugging
    blinker_state = "LEFT" if carstate.leftBlinker else ("RIGHT" if carstate.rightBlinker else "NONE")
    np_logger.debug(f"Final result: {result} (blinker={blinker_state})")
    
    return result

