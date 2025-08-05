#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT EODS (ENHANCED OBSTACLE DETECTION SYSTEM) - DCP FILTER LAYER
====================================================================

OVERVIEW:
EODS implements enhanced obstacle detection using YOLOv8 vision data to provide
immediate enhanced stop/slow responses for people and animals. This filter has
the highest priority in the DCP system to ensure safety-critical response.

CORE FUNCTIONALITY:
- Real-time YOLOv8 detection processing for enhanced obstacle detection
- Physics-based threat assessment and distance calculation
- Enhanced stop/slow override with highest DCP filter priority
- Integration with DCP filter architecture for coordinated control
- Fallback handling when detection data is unavailable

ENHANCED DETECTION CLASSES:
- People: Immediate enhanced stop (threat level 5)
- Large animals (horse, cow, elephant): Enhanced stop (threat level 5)
- Medium animals (dog): Controlled slowdown (threat level 3)
- Small animals (cat): Monitor/slow down (threat level 2)

OPERATIONAL FLOW:
1. Receive YOLOv8 detection data from vision daemon
2. Filter for enhanced detection object classes only
3. Calculate threat level and distance for each object
4. Determine enhanced response (stop/slow/monitor)
5. Apply safety override through DCP filter system

CRITICAL DEPENDENCIES:
- Requires DCP foundation active (np_dcp_mode > 0) to function
- Needs YOLOv8 detection data from vision daemon
- Integration with longitudinal control for speed overrides
- Coordination with other DCP filters (EODS has highest priority)

Architecture Integration:
┌─────────────────────────────────────────────────────────────────┐
│                    DCP Foundation                                │
│              (Core Cruise Control)                               │
├─────────────────────────────────────────────────────────────────┤
│ EODS Filter │ VTSC Filter │ MTSC Filter │ VCSC Filter │ PDA Filter│
│ (Enhanced   │ (Vision-    │ (Map-based  │ (Comfort-   │ (Perform- │
│  Override)  │  Speed ↓)   │  Speed ↓)   │  Speed ↓)   │  Speed ↑) │
│ Priority: 1 │ Priority:100│ Priority:90 │ Priority:80 │ Priority:70│
└─────────────────────────────────────────────────────────────────┘
"""

import time
import numpy as np
from enum import IntEnum
from typing import Dict, Any, Optional, List
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for EODS module
np_logger = NpLogger('eods')
from openpilot.selfdrive.controls.lib.nagaspilot.dcp_profile import DCPFilterLayer, DCPFilterType, DCPFilterResult

# ========================================================================
# CONSTANTS & CONFIGURATION
# ========================================================================

class EODSState(IntEnum):
    """EODS state machine states"""
    DISABLED = 0          # EODS disabled or DCP foundation inactive
    MONITORING = 1        # Monitoring for enhanced detection objects but no threat
    EMERGENCY_DETECTED = 2 # Enhanced detection object detected, calculating response
    EMERGENCY_ACTIVE = 3   # Enhanced response active (stop/slow)

# Enhanced detection threat levels for different object classes
ENHANCED_THREAT_CLASSES = {
    'person': 5,      # Maximum threat - immediate stop
    'horse': 5,       # Large animal - immediate stop  
    'cow': 5,         # Large animal - immediate stop
    'elephant': 5,    # Large wildlife - immediate stop
    'dog': 3,         # Medium threat - slow down
    'cat': 2,         # Low threat - monitor/slow down
}

# ========================================================================
# EODS CONTROLLER IMPLEMENTATION
# ========================================================================

class NpEODSController(DCPFilterLayer):
    """NagasPilot Enhanced Obstacle Detection System - DCP Filter Implementation"""
    
    def __init__(self):
        # Initialize with highest priority for enhanced safety override
        super().__init__(name="EODS", filter_type=DCPFilterType.SAFETY_OVERRIDE, priority=1)
        
        self.params = Params()
        
        # Core EODS parameters
        self.enhanced_stop_distance = 10.0    # meters - enhanced stop distance
        self.slow_down_distance = 20.0         # meters - controlled slowdown distance
        self.confidence_threshold = 0.8        # detection confidence threshold
        self.min_response_speed = 2.0          # m/s - minimum response speed
        
        # Deceleration control parameters (configurable)
        self.max_deceleration_rate = 2.5       # m/s² - maximum deceleration for enhanced stop
        self.emergency_reduction_factor = 0.3  # 70% speed reduction for high threat
        self.moderate_reduction_factor = 0.5   # 50% speed reduction for medium threat
        self.min_speed_modifier = 0.1          # Minimum 10% of original speed (safety limit)
        
        # State management
        self.state = EODSState.DISABLED
        self.current_enhanced_level = 0
        self.last_enhanced_reason = ""
        self.last_detection_time = 0
        self.detection_timeout = 1.0           # seconds - detection validity timeout
        
        # Performance tracking
        self.frame_count = 0
        self.last_params_update = 0
        self.params_update_interval = 1.0      # seconds
        
        # DCP dependency tracking
        self.dcp_dependency_met = False
        
        np_logger.info("[EODS] Enhanced Obstacle Detection System initialized")
    
    def read_params(self):
        """Read EODS parameters from Params with validation"""
        current_time = time.time()
        
        # Rate limit parameter reading for performance
        if current_time - self.last_params_update < self.params_update_interval:
            return
            
        self.last_params_update = current_time
        
        try:
            # Check DCP dependency first
            dcp_mode = self.params.get("np_dcp_mode", encoding='utf8')
            self.dcp_dependency_met = dcp_mode is not None and int(dcp_mode or "0") > 0
            
            # EODS enable status
            self.enabled = self.params.get_bool("np_eods_enabled")
            
            # Enhanced stop distance
            try:
                stop_dist_str = self.params.get("np_eods_enhanced_distance", encoding='utf8')
                if stop_dist_str:
                    self.enhanced_stop_distance = max(5.0, min(20.0, float(stop_dist_str)))  # Bounds: 5-20m
            except (ValueError, TypeError):
                self.enhanced_stop_distance = 10.0  # Default
                
            # Maximum deceleration rate
            try:
                decel_str = self.params.get("np_eods_max_deceleration", encoding='utf8')
                if decel_str:
                    self.max_deceleration_rate = max(1.0, min(4.0, float(decel_str)))  # Bounds: 1.0-4.0 m/s²
            except (ValueError, TypeError):
                self.max_deceleration_rate = 2.5  # Default
                
            # Emergency reduction factor (high threat)
            try:
                emerg_factor_str = self.params.get("np_eods_emergency_reduction", encoding='utf8')
                if emerg_factor_str:
                    self.emergency_reduction_factor = max(0.1, min(0.5, float(emerg_factor_str)))  # Bounds: 10%-50%
            except (ValueError, TypeError):
                self.emergency_reduction_factor = 0.3  # Default (70% reduction)
                
            # Moderate reduction factor (medium threat)
            try:
                mod_factor_str = self.params.get("np_eods_moderate_reduction", encoding='utf8')
                if mod_factor_str:
                    self.moderate_reduction_factor = max(0.3, min(0.8, float(mod_factor_str)))  # Bounds: 30%-80%
            except (ValueError, TypeError):
                self.moderate_reduction_factor = 0.5  # Default (50% reduction)
                
            # Slowdown distance
            try:
                slow_dist_str = self.params.get("np_eods_slow_distance", encoding='utf8')
                if slow_dist_str:
                    self.slow_down_distance = max(10.0, min(50.0, float(slow_dist_str)))  # Bounds: 10-50m
            except (ValueError, TypeError):
                self.slow_down_distance = 20.0  # Default
                
            # Confidence threshold
            try:
                conf_str = self.params.get("np_eods_confidence_threshold", encoding='utf8')
                if conf_str:
                    self.confidence_threshold = max(0.5, min(0.95, float(conf_str)))  # Bounds: 0.5-0.95
            except (ValueError, TypeError):
                self.confidence_threshold = 0.8  # Default
                
        except Exception as e:
            np_logger.warning(f"[EODS] Parameter read error: {e}, using defaults")
            self.enabled = False
    
    def process_yolov8_detections(self, driving_context: Dict[str, Any]) -> Dict[str, Any]:
        """Process YOLOv8 detections and extract enhanced detection objects"""
        
        # Get YOLOv8 data from driving context
        sm = driving_context.get('sm')
        if not sm or 'yolov8Detections' not in sm:
            return {
                'detected_objects': [],
                'threat_level': 0,
                'enhanced_distance': 999.0,
                'detection_confidence': 0.0
            }
            
        yolov8_data = sm['yolov8Detections']
        
        # Validate detection data
        if not hasattr(yolov8_data, 'detections') or len(yolov8_data.detections) == 0:
            return {
                'detected_objects': [],
                'threat_level': 0,
                'enhanced_distance': 999.0,
                'detection_confidence': 0.0
            }
        
        enhanced_objects = []
        max_threat_level = 0
        closest_distance = 999.0
        combined_confidence = 0.0
        
        # Process each detection for enhanced detection objects
        for detection in yolov8_data.detections:
            class_name = detection.className
            
            # Only process enhanced detection classes
            if class_name not in ENHANCED_THREAT_CLASSES:
                continue
                
            # Extract object information
            threat_level = ENHANCED_THREAT_CLASSES[class_name]
            distance = detection.position3D.x if hasattr(detection, 'position3D') and detection.position3D.x > 0 else 999.0
            confidence = detection.confidence
            
            # Skip low confidence detections
            if confidence < self.confidence_threshold:
                continue
                
            # Update enhanced detection metrics
            max_threat_level = max(max_threat_level, threat_level)
            closest_distance = min(closest_distance, distance)
            combined_confidence = max(combined_confidence, confidence)
            
            enhanced_objects.append({
                'class_name': class_name,
                'threat_level': threat_level,
                'distance': distance,
                'confidence': confidence
            })
            
        return {
            'detected_objects': enhanced_objects,
            'threat_level': max_threat_level,
            'enhanced_distance': closest_distance,
            'detection_confidence': combined_confidence
        }
    
    def calculate_enhanced_response(self, enhanced_data: Dict[str, Any], v_ego: float) -> DCPFilterResult:
        """Calculate enhanced response based on detected objects"""
        
        if not enhanced_data['detected_objects']:
            self.state = EODSState.MONITORING
            self.current_enhanced_level = 0
            return DCPFilterResult(
                speed_modifier=1.0,
                active=False,
                reason="No enhanced detection objects detected",
                priority=self.priority
            )
        
        max_threat = enhanced_data['threat_level']
        min_distance = enhanced_data['enhanced_distance']
        confidence = enhanced_data['detection_confidence']
        
        self.state = EODSState.EMERGENCY_DETECTED
        
        # Enhanced stop override (highest threat)
        if max_threat >= 5 and min_distance < self.enhanced_stop_distance:
            self.state = EODSState.EMERGENCY_ACTIVE
            self.current_enhanced_level = 5
            self.last_enhanced_reason = f"Enhanced stop: threat level {max_threat} at {min_distance:.1f}m"
            
            np_logger.warning(f"[EODS] {self.last_enhanced_reason}")
            
            return DCPFilterResult(
                speed_modifier=0.0,  # Complete stop
                active=True,
                reason=self.last_enhanced_reason,
                priority=self.priority
            )
            
        # Controlled slowdown (medium-high threat)
        elif max_threat >= 3 and min_distance < self.slow_down_distance:
            self.state = EODSState.EMERGENCY_ACTIVE
            self.current_enhanced_level = max_threat
            
            # Calculate slowdown factor based on threat and distance (configurable)
            if max_threat >= 4:
                reduction_factor = self.emergency_reduction_factor  # High threat reduction (configurable)
            else:
                reduction_factor = self.moderate_reduction_factor   # Medium threat reduction (configurable)
            
            # Apply minimum speed modifier safety limit
            reduction_factor = max(reduction_factor, self.min_speed_modifier)
                
            self.last_enhanced_reason = f"Enhanced slowdown: threat level {max_threat} at {min_distance:.1f}m"
            
            np_logger.info(f"[EODS] {self.last_enhanced_reason}")
            
            return DCPFilterResult(
                speed_modifier=reduction_factor,
                active=True,
                reason=self.last_enhanced_reason,
                priority=self.priority
            )
            
        # Monitor only (low threat or far distance)
        else:
            self.state = EODSState.MONITORING
            self.current_enhanced_level = 0
            return DCPFilterResult(
                speed_modifier=1.0,
                active=False,
                reason=f"Monitoring: threat level {max_threat} at {min_distance:.1f}m",
                priority=self.priority
            )
    
    def process(self, speed_target: float, driving_context: Dict[str, Any]) -> DCPFilterResult:
        """
        Process speed target through EODS filter layer
        
        Args:
            speed_target: Current target speed from DCP foundation
            driving_context: Driving context with 'sm' (SubMaster), 'v_ego', etc.
            
        Returns:
            DCPFilterResult with emergency speed modification and status
        """
        # Read parameters and check dependencies
        self.read_params()
        
        # Get driving context
        v_ego = driving_context.get('v_ego', 0.0)
        
        # Early exit if DCP dependency not met
        if not self.dcp_dependency_met:
            np_logger.debug("[EODS] DCP dependency not met - EODS inactive")
            return DCPFilterResult(
                speed_modifier=1.0, 
                active=False,
                reason="DCP foundation disabled",
                priority=self.priority
            )
        
        # Early exit if disabled or insufficient speed
        if not self.enabled or v_ego < self.min_response_speed:
            self.state = EODSState.DISABLED
            return DCPFilterResult(
                speed_modifier=1.0,
                active=False,
                reason="EODS disabled or low speed",
                priority=self.priority
            )
        
        # Process YOLOv8 detections
        current_time = time.time()
        enhanced_data = self.process_yolov8_detections(driving_context)
        
        # Check for detection timeout
        if enhanced_data['detected_objects']:
            self.last_detection_time = current_time
        elif current_time - self.last_detection_time > self.detection_timeout:
            # Clear stale detection data
            enhanced_data = {
                'detected_objects': [],
                'threat_level': 0,
                'enhanced_distance': 999.0,
                'detection_confidence': 0.0
            }
        
        # Calculate and return enhanced response
        result = self.calculate_enhanced_response(enhanced_data, v_ego)
        
        # Update performance tracking
        self.frame_count += 1
        
        return result
    
    def update_parameters(self, params: Params):
        """Update EODS parameters - compatibility method for DCP filter system"""
        self.params = params
        self.read_params()