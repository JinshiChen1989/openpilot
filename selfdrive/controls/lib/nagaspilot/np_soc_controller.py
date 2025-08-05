#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT SOC (SMART OFFSET CONTROLLER) - DLP LATERAL ENHANCEMENT
====================================================================

OVERVIEW:
SOC provides intelligent lateral vehicle avoidance through YOLOv8 detection
integration with clean, simple physics-based offset calculation for safe
clearance from large vehicles while maintaining lane discipline.

CORE FUNCTIONALITY:
- Vehicle detection processing from YOLOv8 daemon
- Distance-based lateral offset calculation for buses/trucks
- Smooth offset transitions with configurable rate limiting
- DLP foundation integration for lateral positioning enhancement
- Safe fallback when detection data unavailable

VEHICLE CLASSES:
- car (class 2): Monitor only, no offset
- bus (class 5): Lateral avoidance offset
- truck (class 7): Lateral avoidance offset

SAFETY FEATURES:
- Conservative offset limits (max 0.3m)
- Gradual transitions (configurable rate)
- Input validation and bounds checking
- Graceful degradation on errors
- DLP foundation dependency validation

INTEGRATION:
- DLP Foundation: Requires np_dlp_mode > 0 to operate
- YOLOv8 Detection: Processes vehicle detection messages
- Parameter System: All parameters use np_ prefix
- Message Publishing: SOC status via cereal messaging
"""

import math
import time
from typing import Dict, List, Optional
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger
np_logger = NpLogger('soc')

# SOC vehicle classes (matching YOLOv8 daemon)
SOC_VEHICLE_CLASSES = {
    2: 'car',      # Monitor only - no offset
    5: 'bus',      # Large vehicle - apply offset
    7: 'truck',    # Large vehicle - apply offset
}

class NpSOCController:
    """
    Smart Offset Controller - Intelligent lateral avoidance for overtaking scenarios
    
    Provides smart lateral offset calculation for large vehicle avoidance using YOLOv8 
    detection data with DLP foundation integration. Only activates when actually 
    overtaking slower vehicles, not for faster vehicles pulling away. Features smart 
    return-to-center timing based on relative speed analysis.
    """
    
    def __init__(self):
        """Initialize SOC with safe defaults"""
        self.params = Params()
        
        # SOC state
        self.enabled = False
        self.dlp_dependency_met = False
        self.current_offset = 0.0
        self.target_offset = 0.0
        self.last_update_time = time.time()
        
        # SOC parameters (safe defaults)
        self.max_offset = 0.25           # Maximum lateral offset (meters)
        self.min_offset = 0.02           # Minimum offset to activate (meters)
        self.offset_rate = 0.1           # Maximum offset change rate (m/s)
        self.avoidance_distance = 30.0   # Detection range (meters)
        self.confidence_threshold = 0.7  # Detection confidence threshold
        
        # Lane width tracking and vehicle-based offset
        self.vehicle_width = 1.9         # Our vehicle width (meters) - will be updated from params
        self.safety_margin = 0.20        # Safety margin each side (meters)
        self.current_lane_width = 3.2    # Current detected lane width
        self.lane_width_memory = 3.2     # Last known lane width for laneless mode
        self.lane_width_update_time = 0.0
        self.lane_memory_timeout = 30.0  # Use memory for 30 seconds max
        self.lane_center_offset = 0.0    # Our position relative to lane center
        self.left_lane_line_dist = 1.6   # Distance to left lane line
        self.right_lane_line_dist = 1.6  # Distance to right lane line
        
        # Performance tracking
        self.vehicles_detected = 0
        self.avoidance_activations = 0
        
        # Smart return-to-center logic
        self.last_vehicle_detection = {}        # Track last seen vehicles with speed
        self.return_delay_time = 0.0            # Time to wait before returning
        self.min_return_delay = 2.0             # Minimum delay when detection lost (seconds)
        self.max_return_delay = 5.0             # Maximum delay for safety (seconds)
        
        # Acceleration safety check - prevents SOC during high acceleration
        self.max_safe_acceleration = 2.0       # Maximum acceleration for SOC activation (m/s²)
        self.accel_check_enabled = True        # Enable acceleration safety check
        
        np_logger.info("SOC Controller initialized - Smart vehicle avoidance ready")
    
    def read_params(self):
        """Read SOC parameters with bounds checking and np_ prefix"""
        try:
            # Check DCP dependency (SOC requires lateral control foundation)
            dlp_mode = self.params.get_int("np_dlp_mode")
            self.dlp_dependency_met = (dlp_mode > 0)
            
            if not self.dlp_dependency_met:
                self.enabled = False
                return
                
            # Read SOC enable status
            self.enabled = self.params.get_bool("np_soc_enabled")
            
            # Maximum lateral offset (0.1-0.5m range)
            try:
                max_offset_str = self.params.get("np_soc_max_offset", encoding='utf8')
                if max_offset_str:
                    self.max_offset = max(0.1, min(0.5, float(max_offset_str)))
            except (ValueError, TypeError):
                self.max_offset = 0.25  # Safe default
                
            # Minimum activation offset (0.01-0.1m range)
            try:
                min_offset_str = self.params.get("np_soc_min_offset", encoding='utf8')
                if min_offset_str:
                    self.min_offset = max(0.01, min(0.1, float(min_offset_str)))
            except (ValueError, TypeError):
                self.min_offset = 0.02  # Safe default
                
            # Offset change rate (0.05-0.3 m/s range)
            try:
                offset_rate_str = self.params.get("np_soc_offset_rate", encoding='utf8')
                if offset_rate_str:
                    self.offset_rate = max(0.05, min(0.3, float(offset_rate_str)))
            except (ValueError, TypeError):
                self.offset_rate = 0.1  # Safe default
                
            # Avoidance detection distance (10-50m range)
            try:
                distance_str = self.params.get("np_soc_avoidance_distance", encoding='utf8')
                if distance_str:
                    self.avoidance_distance = max(10.0, min(50.0, float(distance_str)))
            except (ValueError, TypeError):
                self.avoidance_distance = 30.0  # Safe default
                
            # Detection confidence threshold (0.5-0.9 range)
            try:
                confidence_str = self.params.get("np_soc_confidence_threshold", encoding='utf8')
                if confidence_str:
                    self.confidence_threshold = max(0.5, min(0.9, float(confidence_str)))
            except (ValueError, TypeError):
                self.confidence_threshold = 0.7  # Safe default
                
            # Vehicle width from brownpanda settings (1.5-2.2m range)
            try:
                vehicle_width_str = self.params.get("np_vehicle_width", encoding='utf8')
                if vehicle_width_str:
                    self.vehicle_width = max(1.5, min(2.2, float(vehicle_width_str)))
            except (ValueError, TypeError):
                self.vehicle_width = 1.9   # Safe default
                
            # Safety margin (0.2-0.5m range)
            try:
                safety_margin_str = self.params.get("np_soc_safety_margin", encoding='utf8')
                if safety_margin_str:
                    self.safety_margin = max(0.2, min(0.5, float(safety_margin_str)))
            except (ValueError, TypeError):
                self.safety_margin = 0.20  # Safe default
                
        except Exception as e:
            np_logger.error(f"Parameter reading error: {e}, using safe defaults")
            self.enabled = False
    
    def update_lane_information(self, driving_context: Dict) -> None:
        """Simple lane width tracking with safe fallbacks"""
        try:
            current_time = time.time()
            sm = driving_context.get('sm')
            
            # Safe fallback: no data source
            if not sm or 'modelV2' not in sm:
                self._use_fallback_lane_data()
                return
                
            model_data = sm['modelV2']
            
            # Try to get lane lines
            if hasattr(model_data, 'laneLines') and model_data.laneLines:
                left_y = None
                right_y = None
                
                # Simple search for left and right lane lines
                for line in model_data.laneLines:
                    if hasattr(line, 'y') and len(line.y) > 0:
                        y_pos = line.y[0]
                        if -4.0 <= y_pos <= -0.5 and left_y is None:  # Left lane
                            left_y = y_pos
                        elif 0.5 <= y_pos <= 4.0 and right_y is None:  # Right lane
                            right_y = y_pos
                
                # Update if both lines found and reasonable
                if left_y is not None and right_y is not None:
                    lane_width = abs(right_y - left_y)
                    if 2.5 <= lane_width <= 4.5:  # Reasonable lane width
                        self.current_lane_width = lane_width
                        self.lane_width_memory = lane_width
                        self.lane_width_update_time = current_time
                        self.left_lane_line_dist = abs(left_y)
                        self.right_lane_line_dist = abs(right_y)
                        self.lane_center_offset = -(left_y + right_y) / 2
                        return
            
            # No valid lane data - use memory or fallback
            self._use_fallback_lane_data()
                
        except Exception as e:
            np_logger.warning(f"Lane update error: {e}")
            self._use_fallback_lane_data()
    
    def _use_fallback_lane_data(self) -> None:
        """Use lane memory or safe conservative defaults"""
        current_time = time.time()
        time_since_update = current_time - self.lane_width_update_time
        
        if time_since_update < self.lane_memory_timeout:
            # Use memory - assume centered
            self.current_lane_width = self.lane_width_memory
            self.left_lane_line_dist = self.lane_width_memory / 2
            self.right_lane_line_dist = self.lane_width_memory / 2
            self.lane_center_offset = 0.0
        else:
            # Conservative fallback - narrow lane assumption
            self.current_lane_width = 3.0
            self.left_lane_line_dist = 1.5
            self.right_lane_line_dist = 1.5
            self.lane_center_offset = 0.0
    
    def calculate_adaptive_offset_limit(self, direction: str) -> float:
        """Calculate maximum safe offset - simple vehicle size + safety margin"""
        try:
            # Simple calculation: vehicle half-width + safety margin = space needed
            space_needed = (self.vehicle_width / 2) + self.safety_margin
            
            # Get available space to lane line
            if direction == 'left':
                available_space = self.left_lane_line_dist
            else:  # direction == 'right'
                available_space = self.right_lane_line_dist
            
            # Conservative: use only 60% of remaining space after our requirements
            remaining_space = available_space - space_needed
            if remaining_space > 0:
                safe_offset = min(remaining_space * 0.6, self.max_offset)
                return max(self.min_offset, safe_offset) if safe_offset > 0 else 0.0
            else:
                return 0.0  # No safe space available
            
        except Exception as e:
            np_logger.warning(f"Offset limit error: {e}")
            return 0.0  # Safe fallback: no offset
    
    def _validate_detection_data(self, detection: Dict) -> bool:
        """Validate detection data for safety"""
        try:
            # Check required fields
            if not all(key in detection for key in ['className', 'confidence', 'position3D']):
                return False
                
            # Check vehicle class
            if detection['className'] not in ['bus', 'truck']:
                return False
                
            # Check confidence
            if detection['confidence'] < self.confidence_threshold:
                return False
                
            # Check position data
            pos = detection['position3D']
            if not all(key in pos for key in ['x', 'y', 'z']):
                return False
                
            # Check for valid distance (positive, reasonable range)
            distance = pos['x']
            if not (0.1 <= distance <= 100.0):
                return False
                
            # Check for valid lateral position (reasonable range)
            lateral = pos['y']
            if not (-10.0 <= lateral <= 10.0):
                return False
                
            return True
            
        except (TypeError, KeyError, ValueError):
            return False
    
    def calculate_vehicle_offset(self, vehicle_class: str, distance: float, lateral: float) -> float:
        """Simple clean offset calculation with safety fallbacks"""
        try:
            # Only offset for large vehicles
            if vehicle_class == 'truck':
                base_offset = self.safety_margin * 1.2  # 24cm for trucks
            elif vehicle_class == 'bus':
                base_offset = self.safety_margin * 1.0  # 20cm for buses  
            else:
                return 0.0  # No offset for cars
            
            # Simple distance scaling - closer vehicles need more offset
            if distance > 20.0:
                distance_factor = 0.5      # Far vehicles - half offset
            elif distance > 10.0:
                distance_factor = 0.8      # Medium distance
            else:
                distance_factor = 1.0      # Close vehicles - full offset
            
            # Simple lateral check - is vehicle close to our lane?
            abs_lateral = abs(lateral)
            if abs_lateral > self.current_lane_width:
                return 0.0  # Too far away - no offset needed
            elif abs_lateral > self.current_lane_width * 0.6:
                lateral_factor = 0.5  # Adjacent lane - reduced offset
            else:
                lateral_factor = 1.0  # Close to our lane - full offset
            
            # Calculate desired offset
            desired_offset = base_offset * distance_factor * lateral_factor
            
            # Check lane safety limits and apply direction
            if lateral > 0:  # Vehicle to our left - move right (negative)
                max_safe = self.calculate_adaptive_offset_limit('right')
                final_offset = -min(desired_offset, max_safe)
            else:  # Vehicle to our right - move left (positive)
                max_safe = self.calculate_adaptive_offset_limit('left')
                final_offset = min(desired_offset, max_safe)
            
            # Apply minimum threshold for activation
            if abs(final_offset) < self.min_offset:
                return 0.0
                
            return final_offset
                
        except Exception as e:
            np_logger.warning(f"Offset calculation error: {e}")
            return 0.0  # Safe fallback: no offset
    
    def update_vehicle_tracking(self, detections: List[Dict], driving_context: Dict) -> None:
        """
        Track vehicles with speed data for smart return-to-center logic
        Uses driving context to get ego speed for relative speed calculation
        """
        try:
            current_time = time.time()
            ego_speed_ms = driving_context.get('v_ego', 0.0) if driving_context else 0.0
            
            # Get current vehicles in range
            current_vehicles = {}
            
            for detection in detections:
                if not self._validate_detection_data(detection):
                    continue
                
                vehicle_class = detection['className']
                if vehicle_class not in ['bus', 'truck']:  # Only track large vehicles
                    continue
                
                pos = detection['position3D']
                distance = pos['x']
                lateral = pos['y']
                
                if distance > self.avoidance_distance:
                    continue
                
                # Create unique vehicle ID based on position (rounded for stability)
                vehicle_id = f"{vehicle_class}_{distance:.0f}_{lateral:.1f}"
                
                # Store current detection with timestamp
                current_vehicles[vehicle_id] = {
                    'distance': distance,
                    'lateral': lateral,
                    'class': vehicle_class,
                    'time': current_time,
                    'ego_speed': ego_speed_ms
                }
            
            # Calculate relative speeds for previously tracked vehicles
            for vehicle_id, current_data in current_vehicles.items():
                if vehicle_id in self.last_vehicle_detection:
                    prev_data = self.last_vehicle_detection[vehicle_id]
                    
                    # Calculate relative speed
                    dt = current_data['time'] - prev_data['time']
                    if dt > 0.1:  # Minimum time delta for reliable calculation
                        distance_change = current_data['distance'] - prev_data['distance']
                        relative_speed = distance_change / dt  # Positive = vehicle moving away
                        current_data['relative_speed'] = relative_speed
                    else:
                        current_data['relative_speed'] = prev_data.get('relative_speed', 0.0)
                else:
                    current_data['relative_speed'] = 0.0  # First detection
            
            # Update tracking
            self.last_vehicle_detection = current_vehicles
            
        except Exception as e:
            np_logger.warning(f"Vehicle tracking error: {e}")
    
    def calculate_smart_return_delay(self) -> float:
        """
        Calculate how long to delay return-to-center based on relative speeds
        Considers multiple vehicles and convoy scenarios
        Returns delay time in seconds
        """
        try:
            # If no vehicles were being tracked, return immediately
            if not self.last_vehicle_detection:
                return 0.0
            
            # Check if we were likely passing any vehicles
            max_delay = 0.0
            vehicle_count = len(self.last_vehicle_detection)
            
            for vehicle_id, vehicle_data in self.last_vehicle_detection.items():
                relative_speed = vehicle_data.get('relative_speed', 0.0)
                ego_speed = vehicle_data.get('ego_speed', 0.0)
                last_distance = vehicle_data.get('distance', 30.0)
                
                # If we were moving faster than the vehicle (overtaking scenario)
                if relative_speed > 1.0:  # We were gaining on the vehicle (m/s)
                    # Calculate estimated time to fully pass based on vehicle size and our speed
                    vehicle_length = 15.0 if vehicle_data['class'] == 'bus' else 8.0  # Estimated lengths
                    passing_time = vehicle_length / ego_speed if ego_speed > 2.0 else 3.0
                    
                    # Add safety margin and consider last known distance
                    if last_distance < 10.0:  # Vehicle was close when lost
                        delay = min(passing_time + 1.0, self.max_return_delay)
                    else:
                        delay = min(passing_time * 0.5, self.max_return_delay)
                    
                    max_delay = max(max_delay, delay)
                
                elif relative_speed < -2.0:  # Vehicle was pulling away from us
                    # Likely already passed - shorter delay
                    max_delay = max(max_delay, self.min_return_delay * 0.5)
                
                else:  # Similar speeds or uncertain
                    # Use minimum delay for safety
                    max_delay = max(max_delay, self.min_return_delay)
            
            # For multiple vehicles (convoy scenario), add extra caution
            if vehicle_count > 1:
                convoy_bonus = min(1.0, vehicle_count * 0.3)  # Extra 0.3s per vehicle
                max_delay += convoy_bonus
                np_logger.info(f"Convoy detected ({vehicle_count} vehicles), adding {convoy_bonus:.1f}s delay")
            
            return min(max_delay, self.max_return_delay)
            
        except Exception as e:
            np_logger.warning(f"Smart return delay calculation error: {e}")
            return self.min_return_delay
    
    def _check_acceleration_safety(self, driving_context: Dict) -> Dict:
        """
        Check if current acceleration is safe for SOC activation
        Prevents SOC during high acceleration (PDA boost, manual acceleration, etc.)
        
        Returns:
            Dict with safety status and reason
        """
        try:
            if not self.accel_check_enabled:
                return {'safe': True, 'reason': 'Acceleration check disabled'}
            
            # Get current acceleration from driving context
            current_accel = driving_context.get('a_ego', 0.0)
            
            # Check if acceleration is within safe limits
            if abs(current_accel) > self.max_safe_acceleration:
                return {
                    'safe': False,
                    'acceleration': current_accel,
                    'reason': f'High acceleration: {current_accel:.2f} m/s² > {self.max_safe_acceleration:.2f} m/s²'
                }
            
            return {
                'safe': True,
                'acceleration': current_accel,
                'reason': f'Safe acceleration: {current_accel:.2f} m/s²'
            }
            
        except Exception as e:
            np_logger.warning(f"Acceleration safety check error: {e}")
            return {
                'safe': False,
                'acceleration': 0.0,
                'reason': f'Acceleration check error: {e}'
            }
    
    def _should_avoid_vehicle(self, vehicle_id: str, distance: float, relative_speed: float) -> bool:
        """
        Determine if we should avoid this vehicle based on overtaking scenario analysis
        
        Args:
            vehicle_id: Unique vehicle identifier
            distance: Current distance to vehicle (meters)
            relative_speed: Our speed relative to vehicle (m/s, positive = we're faster)
            
        Returns:
            True if we should apply lateral offset for this vehicle
        """
        try:
            # Case 1: First detection - no speed data yet, be conservative and apply offset
            if vehicle_id not in self.last_vehicle_detection:
                np_logger.debug(f"First detection of {vehicle_id}, applying offset (no speed data)")
                return True
            
            # Case 2: Vehicle is significantly faster than us - don't offset
            if relative_speed < -1.0:  # Vehicle pulling away at >1 m/s (~3.6 km/h)
                np_logger.debug(f"Vehicle {vehicle_id} faster than us (rel_speed={relative_speed:.1f}), skipping offset")
                return False
                
            # Case 3: We're approaching the vehicle - definitely apply offset
            if relative_speed > 0.5:  # We're catching up at >0.5 m/s (~1.8 km/h) 
                np_logger.debug(f"Approaching {vehicle_id} (rel_speed={relative_speed:.1f}), applying offset")
                return True
                
            # Case 4: Similar speeds or very close distance - apply offset for safety
            if distance < 15.0:  # Very close - always offset regardless of speed
                np_logger.debug(f"Very close to {vehicle_id} ({distance:.1f}m), applying safety offset")
                return True
                
            # Case 5: Uncertain speed but reasonable distance - slight bias toward safety
            if abs(relative_speed) < 0.5:  # Similar speeds within ±0.5 m/s
                np_logger.debug(f"Similar speeds to {vehicle_id} (rel_speed={relative_speed:.1f}), applying offset")
                return True
                
            # Case 6: Vehicle slower but we're not catching up much - skip offset
            np_logger.debug(f"Ambiguous scenario for {vehicle_id} (rel_speed={relative_speed:.1f}), skipping offset")
            return False
            
        except Exception as e:
            np_logger.warning(f"Vehicle avoidance decision error for {vehicle_id}: {e}")
            return True  # Default to safe behavior
    
    def _is_pda_boost_active(self, driving_context: Dict) -> bool:
        """
        Check if PDA is currently applying speed boost - critical safety check
        SOC should not activate new offsets during PDA boost phases
        """
        try:
            # Check for PDA active status in driving context
            pda_status = driving_context.get('pda_status', {})
            if isinstance(pda_status, dict):
                is_pda_active = pda_status.get('active', False)
                pda_reason = pda_status.get('reason', '')
                
                # Check if PDA is in boost/overtaking phase
                if is_pda_active and any(keyword in pda_reason.lower() for keyword in 
                    ['boost', 'overtaking', 'overtake', 'anchor']):
                    np_logger.info(f"PDA boost detected: {pda_reason}")
                    return True
            
            # Alternative check: look for speed modifications in driving context
            speed_modifier = driving_context.get('speed_modifier', 1.0)
            if speed_modifier > 1.05:  # >5% speed increase suggests active boost
                np_logger.debug(f"Speed boost detected: {speed_modifier:.2f}")
                return True
                
            return False
            
        except Exception as e:
            np_logger.warning(f"PDA boost check error: {e}")
            return False  # Default to allow SOC if uncertain
    
    def process_detections(self, detections: List[Dict]) -> float:
        """Process vehicle detections and calculate target offset"""
        try:
            if not detections:
                return 0.0
                
            max_offset = 0.0
            vehicles_found = 0
            
            for detection in detections:
                # Validate detection data
                if not self._validate_detection_data(detection):
                    continue
                    
                # Extract detection data
                vehicle_class = detection['className']
                confidence = detection['confidence']
                pos = detection['position3D']
                distance = pos['x']
                lateral = pos['y']
                
                # Check if vehicle is within avoidance range
                if distance > self.avoidance_distance:
                    continue
                
                # Check if we're actually approaching this vehicle (relative speed check)
                vehicle_id = f"{vehicle_class}_{distance:.0f}_{lateral:.1f}"
                relative_speed = 0.0
                if vehicle_id in self.last_vehicle_detection:
                    relative_speed = self.last_vehicle_detection[vehicle_id].get('relative_speed', 0.0)
                
                # Only apply offset if we're actually in an overtaking scenario
                should_avoid = self._should_avoid_vehicle(vehicle_id, distance, relative_speed)
                if not should_avoid:
                    continue
                    
                # Calculate offset for this vehicle
                offset = self.calculate_vehicle_offset(vehicle_class, distance, lateral)
                
                # Track maximum offset needed
                if abs(offset) > abs(max_offset):
                    max_offset = offset
                    
                vehicles_found += 1
                
            self.vehicles_detected += vehicles_found
            return max_offset
            
        except Exception as e:
            np_logger.error(f"Detection processing error: {e}")
            return 0.0
    
    def update_offset(self, target_offset: float) -> float:
        """Update current offset with smooth transitions"""
        try:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            # Calculate maximum change this update
            max_change = self.offset_rate * dt
            
            # Apply rate limiting for smooth transitions
            offset_error = target_offset - self.current_offset
            
            if abs(offset_error) <= max_change:
                # Can reach target this update
                self.current_offset = target_offset
            else:
                # Move toward target at maximum rate
                if offset_error > 0:
                    self.current_offset += max_change
                else:
                    self.current_offset -= max_change
            
            # Safety clamp to maximum offset
            self.current_offset = max(-self.max_offset, min(self.max_offset, self.current_offset))
            
            return self.current_offset
            
        except Exception as e:
            np_logger.error(f"Offset update error: {e}")
            return 0.0
    
    def get_status(self, detections: List[Dict] = None, driving_context: Dict = None) -> Dict:
        """
        Get SOC status with detection processing
        
        Args:
            detections: List of vehicle detection dictionaries from YOLOv8
            driving_context: Driving context for lane information
            
        Returns:
            Dictionary with SOC status information
        """
        # Read parameters and check dependencies
        self.read_params()
        
        # Update lane width information if driving context provided
        if driving_context:
            self.update_lane_information(driving_context)
        
        # Default inactive status
        status = {
            'enabled': self.enabled,
            'dlp_dependency_met': self.dlp_dependency_met,
            'active': False,
            'lateral_offset': 0.0,
            'target_offset': 0.0,
            'vehicles_detected': 0,
            'reason': 'SOC inactive'
        }
        
        # Early exit if not enabled or dependency not met
        if not (self.enabled and self.dlp_dependency_met):
            if not self.dlp_dependency_met:
                status['reason'] = 'DLP Foundation disabled'
            else:
                status['reason'] = 'SOC disabled'
            return status
        
        # Process detections if provided  
        if detections:
            try:
                # CRITICAL SAFETY CHECKS: Block SOC during unsafe conditions
                if driving_context:
                    # Check 1: High acceleration (any source)
                    accel_safety = self._check_acceleration_safety(driving_context)
                    if not accel_safety['safe']:
                        if abs(self.current_offset) < self.min_offset:  # Currently not offsetting
                            status.update({
                                'active': False,
                                'lateral_offset': 0.0,
                                'target_offset': 0.0,
                                'vehicles_detected': len([d for d in detections if self._validate_detection_data(d)]),
                                'reason': f'SOC blocked: {accel_safety["reason"]}'
                            })
                            return status
                    
                    # Check 2: PDA boost phase active
                    pda_status = driving_context.get('pda_status', {})
                    if pda_status.get('active', False):
                        if abs(self.current_offset) < self.min_offset:  # Currently not offsetting
                            status.update({
                                'active': False,
                                'lateral_offset': 0.0,
                                'target_offset': 0.0,
                                'vehicles_detected': len([d for d in detections if self._validate_detection_data(d)]),
                                'reason': 'SOC blocked: PDA boost active'
                            })
                            return status
                
                # Update vehicle tracking for smart return logic
                self.update_vehicle_tracking(detections, driving_context)
                
                # Calculate target offset from detections
                new_target_offset = self.process_detections(detections)
                
                # Check if we have new vehicles during return delay
                if hasattr(self, 'return_delay_time') and self.return_delay_time > 0:
                    current_time = time.time()
                    if current_time < self.return_delay_time and new_target_offset > 0:
                        # New vehicle detected during delay - cancel delay and use new target
                        self.return_delay_time = 0.0
                        self.target_offset = new_target_offset
                        np_logger.info("New vehicle detected during return delay - canceling delay")
                    else:
                        self.target_offset = new_target_offset
                else:
                    self.target_offset = new_target_offset
                
                # Update current offset with smooth transitions
                self.current_offset = self.update_offset(self.target_offset)
                
                # Determine if SOC is actively controlling
                is_active = abs(self.current_offset) > self.min_offset
                
                if is_active and not hasattr(self, '_was_active'):
                    self.avoidance_activations += 1
                    self._was_active = True
                elif not is_active:
                    self._was_active = False
                
                # Update status
                status.update({
                    'active': is_active,
                    'lateral_offset': self.current_offset,
                    'target_offset': self.target_offset,
                    'vehicles_detected': len([d for d in detections if self._validate_detection_data(d)]),
                    'reason': f'Avoiding large vehicles: {self.current_offset:.2f}m offset' if is_active else 'No avoidance needed'
                })
                
                
            except Exception as e:
                np_logger.error(f"Status processing error: {e}")
                status['reason'] = f'Processing error: {str(e)}'
        else:
            # No detection data - use smart return logic
            current_time = time.time()
            
            # Check if we should delay return based on relative speed analysis
            if hasattr(self, 'return_delay_time') and self.return_delay_time > 0:
                if current_time < self.return_delay_time:
                    # Still in delay period - maintain current offset
                    self.target_offset = self.current_offset
                    self.current_offset = self.update_offset(self.target_offset)
                    remaining_delay = self.return_delay_time - current_time
                    status['lateral_offset'] = self.current_offset
                    status['reason'] = f'Smart return delay: {remaining_delay:.1f}s remaining'
                else:
                    # Delay period over - start returning to center
                    self.target_offset = 0.0
                    self.current_offset = self.update_offset(0.0)
                    status['lateral_offset'] = self.current_offset
                    status['reason'] = 'Returning to center after delay'
                    if abs(self.current_offset) < self.min_offset:
                        self.return_delay_time = 0.0  # Reset delay
            else:
                # First time losing detection - calculate smart delay
                smart_delay = self.calculate_smart_return_delay()
                if smart_delay > 0:
                    self.return_delay_time = current_time + smart_delay
                    self.target_offset = self.current_offset  # Maintain current offset
                    self.current_offset = self.update_offset(self.target_offset)
                    status['lateral_offset'] = self.current_offset
                    status['reason'] = f'Detection lost, smart delay: {smart_delay:.1f}s'
                else:
                    # No delay needed - return immediately
                    self.target_offset = 0.0
                    self.current_offset = self.update_offset(0.0)
                    status['lateral_offset'] = self.current_offset
                    status['reason'] = 'No detection data - returning to center'
        
        return status
    
    def get_debug_info(self) -> Dict:
        """Get debug information for monitoring"""
        current_time = time.time()
        time_since_lane_update = current_time - self.lane_width_update_time
        
        return {
            'enabled': self.enabled,
            'dlp_dependency_met': self.dlp_dependency_met,
            'current_offset': round(self.current_offset, 3),
            'target_offset': round(self.target_offset, 3),
            'max_offset': self.max_offset,
            'min_offset': self.min_offset,
            'offset_rate': self.offset_rate,
            'avoidance_distance': self.avoidance_distance,
            'confidence_threshold': self.confidence_threshold,
            'vehicles_detected': self.vehicles_detected,
            'avoidance_activations': self.avoidance_activations,
            # Lane width tracking info
            'vehicle_width': self.vehicle_width,
            'safety_margin': self.safety_margin,
            'current_lane_width': round(self.current_lane_width, 2),
            'lane_width_memory': round(self.lane_width_memory, 2),
            'time_since_lane_update': round(time_since_lane_update, 1),
            'using_lane_memory': time_since_lane_update < self.lane_memory_timeout,
            'lane_center_offset': round(self.lane_center_offset, 3),
            'left_lane_line_dist': round(self.left_lane_line_dist, 2),
            'right_lane_line_dist': round(self.right_lane_line_dist, 2)
        }

# SOC Integration Notes:
# - Lane-aware adaptive offset based on vehicle dimensions and lane constraints
# - Remembers lane width when DLP switches to laneless mode (30s memory)
# - Vehicle width follows brownpanda settings parameters (np_vehicle_width)
# - Adaptive safety margins based on available lane space
# - All parameters use np_ prefix for consistency
# - Graceful fallback on errors or missing data
# - DLP foundation dependency ensures proper integration
# - Prevents lane line violations through constraint checking