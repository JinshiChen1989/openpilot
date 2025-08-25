#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT DCP (DYNAMIC CRUISE PROFILES) - FILTER ARCHITECTURE
====================================================================

OVERVIEW:
DCP is the foundational filter management system that enables modular,
priority-based speed control for enhanced autonomous driving. It provides
a flexible framework where multiple speed control algorithms can work
together intelligently without conflicts.

CORE ARCHITECTURE:
- Filter Layer System: Independent, reusable speed control modules
- Priority Management: Safety-critical filters take precedence
- Result Aggregation: Intelligent combination of filter outputs
- Parameter Coordination: Centralized configuration management
- Foundation Integration: Works with DLP for lateral/longitudinal sync

SUPPORTED FILTER LAYERS:
- VTSC (Vision Turn Speed Control): Reactive curve speed management
- MTSC (Map Turn Speed Control): Proactive map-based speed control
- GCF (Gradient Compensation Factor): Hill/slope speed adjustments
- PDA (Parallel Drive Avoidance): Strategic overtaking optimization
- APSL (Accelerator Pedal Speed Learning): Driver behavior learning
- BPSL (Brake Pedal Speed Learning): Brake pattern analysis

Architecture:
┌─────────────────────────────────────────────────────────────────┐
│                    DCP Foundation                                │
│              (Core Cruise Control)                               │
├─────────────────────────────────────────────────────────────────┤
│ VTSC │ MTSC │ VCSC │ PDA │ APSL │ BPSL │                          │
│(Spd↓)│(Spd↓)│(Cmf↓)│(Spd↑)│(Gas+)│(Brake-)│                      │
└─────────────────────────────────────────────────────────────────┘

PRODUCTION LOGGING:
Uses centralized NpLogger for essential error, warning, and info logs.
Debug logging has been removed for production deployment.
"""

from enum import IntEnum
from typing import Dict, Any, Optional, List, Callable
import threading
import numpy as np
from openpilot.common.params import Params

from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for DCP module
logger = NpLogger('dcp')
from openpilot.selfdrive.controls.lib.nagaspilot.np_aem_controller import AEM


class APSLSafetyCoordinator:
    """
    APSL Safety Coordination System
    
    Manages integration between APSL driver speed learning and all DCP safety systems.
    Ensures learned speeds are always bounded by safety limits from vision, map,
    following distance, and emergency systems.
    """
    
    def __init__(self):
        self.params = Params()
        
        # Safety limit tracking
        self.active_limits = {}
        self.safety_violations = []
        
        # Emergency braking removed - camera-based systems unreliable
        logger.info("APSLSafetyCoordinator initialized successfully")
    
    def collect_safety_limits(self, driving_context: Dict[str, Any]) -> Dict[str, Optional[float]]:
        """
        Collect all active safety limits from DCP systems
        
        Args:
            driving_context: Enhanced driving context with safety data
            
        Returns:
            Dictionary of safety limits from all sources
        """
        safety_limits = {
            # Vision-based safety (VTSC)
            'vtsc_curve_limit': driving_context.get('vtsc_speed_limit'),
            
            # Map-based safety (MTSC)  
            'mtsc_map_limit': driving_context.get('mtsc_speed_limit'),
            
            # Comfort-based safety (VCSC)
            'vcsc_comfort_limit': driving_context.get('vcsc_speed_limit'),
            
            # Following distance safety
            'following_limit': driving_context.get('following_speed_limit'),
            
            
            # Absolute safety maximum (120 km/h)
            'absolute_limit': driving_context.get('max_safe_speed', 33.5),
            
            # Speed limit from map/GPS data
            'speed_limit_map': self._get_map_speed_limit(driving_context),
        }
        
        # Filter out None values and store active limits
        self.active_limits = {k: v for k, v in safety_limits.items() if v is not None}
        
        return safety_limits
    
    def apply_safety_limits_to_driver_input(self, driver_speed: float, 
                                          safety_limits: Dict[str, Optional[float]]) -> Dict[str, Any]:
        """
        Apply most restrictive safety limit to driver input
        
        Args:
            driver_speed: Speed requested by driver pedal input
            safety_limits: Dictionary of all safety limits
            
        Returns:
            Dictionary with safe_speed and applied_limits information
        """
        # Start with driver requested speed
        safe_speed = driver_speed
        applied_limits = []
        
        # Apply each safety limit (most restrictive wins)
        for limit_name, limit_value in safety_limits.items():
            if limit_value is not None and safe_speed > limit_value:
                safe_speed = limit_value
                applied_limits.append({
                    'type': limit_name,
                    'limit': limit_value,
                    'original_driver_speed': driver_speed,
                    'reason': self._get_limit_reason(limit_name)
                })
                logger.warning(f"APSL limited by {limit_name}: {driver_speed:.1f} → {safe_speed:.1f} m/s")
        
        
        return {
            'safe_speed': max(0.0, safe_speed),  # Ensure non-negative
            'applied_limits': applied_limits,
            'safety_margin': safe_speed - driver_speed
        }
    
    
    def _get_map_speed_limit(self, driving_context: Dict[str, Any]) -> Optional[float]:
        """Get speed limit from map/GPS data"""
        # This would interface with map/GPS speed limit data
        # For now, return None (no map speed limit)
        return None
    
    
    
    def _get_limit_reason(self, limit_name: str) -> str:
        """Get human-readable reason for each limit type"""
        reasons = {
            'vtsc_curve_limit': 'Vision-detected curve ahead',
            'mtsc_map_limit': 'Map-based curve ahead', 
            'vcsc_comfort_limit': 'Comfort/passenger safety',
            'following_limit': 'Following distance safety',
            'absolute_limit': 'Absolute maximum speed (120 km/h)',
            'speed_limit_map': 'Legal speed limit'
        }
        return reasons.get(limit_name, f'Safety limit: {limit_name}')
    
    def get_safety_status(self) -> Dict[str, Any]:
        """Get current safety system status"""
        return {
            'active_limits': self.active_limits,
            'safety_violations': self.safety_violations,
            'total_safety_systems': len(self.active_limits),
            'highest_priority_limit': min(self.active_limits.values()) if self.active_limits else None
        }


class DCPMode(IntEnum):
    """DCP Mode definitions"""
    OFF = 0      # FALLBACK to OpenPilot longitudinal control
    HIGHWAY = 1  # ACC-focused stable cruise
    URBAN = 2    # Blended-focused reactive cruise  
    DCP = 3      # Full adaptive mode switching


class DCPFilterType(IntEnum):
    """Filter layer types for DCP processing"""
    SPEED_REDUCTION = 0    # VTSC, MTSC, VCSC - reduce speed
    SPEED_ENHANCEMENT = 1  # PDA - increase speed
    SAFETY_OVERRIDE = 2    # SOC - safety overrides
    DRIVER_OVERRIDE = 3    # APSL - driver pedal speed learning with DCP integration


class DCPFilterResult:
    """Result from a filter layer processing"""
    
    def __init__(self, speed_modifier: float = 1.0, active: bool = False, 
                 reason: str = "", priority: int = 0):
        self.speed_modifier = speed_modifier  # 1.0 = no change, <1.0 = slower, >1.0 = faster
        self.active = active                  # Whether filter is currently active
        self.reason = reason                  # Human-readable reason for activation
        self.priority = priority              # Filter priority (higher = more important)


class DCPFilterLayer:
    """Base class for DCP filter layers"""
    
    def __init__(self, name: str, filter_type: DCPFilterType, priority: int = 0):
        self.name = name
        self.filter_type = filter_type
        self.priority = priority
        self.enabled = False
        
    def process(self, speed_target: float, driving_context: Dict[str, Any]) -> DCPFilterResult:
        """
        Process the speed target through this filter layer
        
        Args:
            speed_target: Current target speed from DCP foundation
            driving_context: Current driving situation data
            
        Returns:
            DCPFilterResult with speed modification and status
        """
        raise NotImplementedError("Filter layers must implement process()")
    
    def update_parameters(self, params: Params):
        """Update filter parameters from Params"""
        pass


class DCPFilterManager:
    """Manages filter layers and applies them to DCP output"""
    
    def __init__(self):
        self.filters: List[DCPFilterLayer] = []
        self.last_applied_filters = []
        # Enhanced thread safety for filter registration and access
        self._filter_lock = threading.RLock()
        # Atomic flag for filter list state
        self._filters_modified = False
        
    def register_filter(self, filter_layer: DCPFilterLayer):
        """Register a new filter layer with enhanced thread safety"""
        with self._filter_lock:
            self.filters.append(filter_layer)
            # Sort by priority (higher priority first)
            self.filters.sort(key=lambda f: f.priority, reverse=True)
            # Set atomic flag to indicate filter list was modified
            self._filters_modified = True
            logger.info(f"Registered filter: {filter_layer.name} (priority: {filter_layer.priority})")
    
    def apply_filters(self, base_speed: float, driving_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Apply all enabled filters to the base speed from DCP foundation
        
        Args:
            base_speed: Base target speed from DCP foundation
            driving_context: Current driving situation data
            
        Returns:
            Dict with final_speed, active_filters, and filter_details
        """
        current_speed = base_speed
        active_filters = []
        filter_details = []
        
        # Apply filters in priority order with thread safety
        with self._filter_lock:
            for filter_layer in self.filters:
                if not filter_layer.enabled:
                    continue
                    
                try:
                    result = filter_layer.process(current_speed, driving_context)
                    
                    if result.active:
                        # Validate speed_modifier before applying
                        if not isinstance(result.speed_modifier, (int, float)) or not np.isfinite(result.speed_modifier):
                            logger.error(f"Invalid speed_modifier {result.speed_modifier} from {filter_layer.name}, skipping filter")
                            continue
                        
                        # Apply reasonable bounds to speed_modifier (0.1x to 3.0x original speed)
                        if not (0.1 <= result.speed_modifier <= 3.0):
                            logger.error(f"Speed_modifier {result.speed_modifier} from {filter_layer.name} out of bounds [0.1, 3.0], skipping filter")
                            continue
                        
                        # Apply speed modification with validation
                        new_speed = current_speed * result.speed_modifier
                        
                        # Final validation of calculated speed
                        if not np.isfinite(new_speed) or new_speed < 0:
                            logger.error(f"Invalid new_speed {new_speed} calculated from {filter_layer.name}, skipping filter")
                            continue
                        
                        # Speed reduction filters: take the most restrictive
                        if filter_layer.filter_type == DCPFilterType.SPEED_REDUCTION:
                            current_speed = min(current_speed, new_speed)
                            # Store safety limits for APSL integration
                            if filter_layer.name == "VTSC":
                                driving_context['vtsc_speed_limit'] = new_speed
                            elif filter_layer.name == "MTSC":
                                driving_context['mtsc_speed_limit'] = new_speed
                            elif filter_layer.name == "VCSC":
                                driving_context['vcsc_speed_limit'] = new_speed
                        # Speed enhancement filters: take the highest
                        elif filter_layer.filter_type == DCPFilterType.SPEED_ENHANCEMENT:
                            current_speed = max(current_speed, new_speed)
                        # Safety overrides: always apply
                        elif filter_layer.filter_type == DCPFilterType.SAFETY_OVERRIDE:
                            current_speed = new_speed
                        # Driver overrides: highest priority, replace target completely
                        elif filter_layer.filter_type == DCPFilterType.DRIVER_OVERRIDE:
                            current_speed = new_speed
                        
                        active_filters.append(filter_layer.name)
                        filter_details.append({
                            'name': filter_layer.name,
                            'modifier': result.speed_modifier,
                            'reason': result.reason,
                            'priority': filter_layer.priority
                        })
                        
                except (ValueError, TypeError, AttributeError, ZeroDivisionError) as e:
                    logger.error(f"Filter {filter_layer.name} failed with {type(e).__name__}: {e}")
                    continue
                except Exception as e:
                    logger.critical(f"Filter {filter_layer.name} failed with unexpected error: {e}")
                    continue
        
        self.last_applied_filters = active_filters
        
        return {
            'final_speed': current_speed,
            'base_speed': base_speed,
            'active_filters': active_filters,
            'filter_details': filter_details
        }
    
    def get_status(self) -> Dict[str, Any]:
        """Get current filter manager status with thread safety"""
        with self._filter_lock:
            return {
                'total_filters': len(self.filters),
                'enabled_filters': len([f for f in self.filters if f.enabled]),
                'last_applied': self.last_applied_filters.copy(),  # Safe copy
                'filter_names': [f.name for f in self.filters],
                'filters_modified': self._filters_modified
            }
        


class DCPProfile:
    """
    Dynamic Cruise Profile manager with Filter Layer Architecture support.
    
    Provides unified longitudinal control by wrapping the existing AEM system 
    with mode-specific behaviors and supporting filter layers for speed controllers.
    
    Architecture:
    1. DCP Foundation: Provides base cruise control mode (acc/blended)
    2. Filter Layers: Modify speed targets (VTSC, MTSC, VCSC, PDA)
    3. Safety Systems: Override with safety controls (SOC)
    """
    
    def __init__(self, aem_instance: AEM):
        """Initialize DCP Profile with existing AEM instance and filter layer support"""
        self.aem = aem_instance
        self.params = Params()  # Use existing parameter system directly
        
        # Initialize Filter Layer Architecture
        self.filter_manager = DCPFilterManager()
        self.fallback_enabled = True  # Independent fallback control
        
        # Register filter layers
        self._register_eods_filter()      # Enhanced Obstacle Detection System
        self._register_vtsc_filter()
        self._register_mtsc_filter()
        self._register_vcsc_filter()
        self._register_pda_filter()
        self._register_apsl_filter()
        self._register_bpsl_filter()
        
        # Load parameters with atomic validation to prevent TOCTOU
        mode_param = self.params.get("np_dcp_mode", encoding="utf-8") or "1"
        self.mode = self._validate_mode(self._parse_int_safe(mode_param, 1))
        
        personality_param = self.params.get("np_dcp_personality", encoding="utf-8") or "1"
        self.personality = self._validate_personality(self._parse_int_safe(personality_param, 1))
        
        highway_param = self.params.get("np_dcp_highway_bias", encoding="utf-8") or "0.8"
        self.highway_bias = self._validate_bias(self._parse_float_safe(highway_param, 0.8), "highway")
        
        urban_param = self.params.get("np_dcp_urban_bias", encoding="utf-8") or "0.3"
        self.urban_bias = self._validate_bias(self._parse_float_safe(urban_param, 0.3), "urban")
        
        # Validate parameter interactions
        self._validate_parameter_interactions()
        
        # Enhanced features (future implementation)
        self.energy_optimizer_enabled = self.params.get_bool("np_energy_optimizer_enabled", False)
        self.curve_speed_enabled = self.params.get_bool("np_curve_speed_enabled", False)
        self.cutoff_speed_enabled = self.params.get_bool("np_cutoff_speed_enabled", False)
        self.predictive_cruise_enabled = self.params.get_bool("np_predictive_cruise_enabled", False)
        
        # Fallback control parameters
        self.safety_fallback = self.params.get_bool("np_dcp_safety_fallback", True)
        self.fallback_enabled = self.params.get_bool("np_dcp_fallback_enabled", True)
        
        logger.info(f"Initialized with Filter Layer Architecture - Mode: {self.mode}, "
                     f"Personality: {self.personality}, Fallback: {self.fallback_enabled}")
        
    def get_mpc_mode(self, driving_context) -> str:
        """
        Get MPC mode based on DCP profile and driving context.
        
        Args:
            driving_context: Dictionary containing driving situation data
            
        Returns:
            str: MPC mode ('acc', 'blended', or None for off/fallback)
        """
        # Check for fallback mode (Mode 0 = FALLBACK to OpenPilot longitudinal control)
        if self.mode == DCPMode.OFF:
            if self.fallback_enabled:
                logger.debug("Mode 0: FALLBACK to OpenPilot longitudinal control")
                return None  # Return None to signal fallback to OpenPilot
            else:
                # If fallback disabled, default to highway mode for safety
                logger.warning("Mode 0 with fallback disabled, using Highway mode")
                return self._highway_behavior(driving_context)
        elif self.mode == DCPMode.HIGHWAY:
            return self._highway_behavior(driving_context)
        elif self.mode == DCPMode.URBAN:
            return self._urban_behavior(driving_context)
        elif self.mode == DCPMode.DCP:
            return self._adaptive_behavior(driving_context)
        else:
            # Fallback to highway mode for invalid values
            logger.warning(f"Invalid mode {self.mode}, falling back to Highway")
            return self._highway_behavior(driving_context)
    
    def prepare_driving_context_for_apsl(self, base_driving_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Prepare enhanced driving context with APSL-specific data
        
        Args:
            base_driving_context: Basic driving context from controlsd
            
        Returns:
            Enhanced driving context with all data needed for APSL integration
        """
        # Start with base context with size validation
        if len(base_driving_context) > 100:  # Limit context size to prevent memory issues
            logger.warning(f"Base driving context size ({len(base_driving_context)}) exceeds limit, truncating")
            # Keep only essential keys for safety
            essential_keys = {'v_ego', 'lead_one', 'steering_angle', 'standstill', 'brake_pressed', 
                            'driver_pedal_position', 'lateral_control_active', 'car_state', 'events'}
            base_driving_context = {k: v for k, v in base_driving_context.items() if k in essential_keys}
        enhanced_context = base_driving_context.copy()
        
        # Add APSL-specific data
        enhanced_context.update({
            # APSL driver input data
            'driver_pedal_position': base_driving_context.get('driver_pedal_position', 0.0),
            'apsl_enabled': self.params.get_bool("EnableAPSL", False),
            'disengage_on_accelerator': self.params.get_bool("DisengageOnAccelerator", True),
            'ndlob_active': not self.params.get_bool("DisengageLateralOnBrake", False),
            
            # Safety limit sources (will be populated by filters)
            'vtsc_speed_limit': None,  # Set by VTSC filter
            'mtsc_speed_limit': None,  # Set by MTSC filter  
            'vcsc_speed_limit': None,  # Set by VCSC filter
            'following_speed_limit': None,  # Set by following distance system
            'max_safe_speed': 33.5,  # 120 km/h absolute maximum
            
            # Brake and lateral coordination
            'brake_pressed': base_driving_context.get('brake_pressed', False),
            'lateral_control_active': base_driving_context.get('lateral_control_active', True),
            
            # Event coordination
            'events': base_driving_context.get('events'),
            'car_state': base_driving_context.get('car_state'),
        })
        
        return enhanced_context
    
    def get_target_speed_with_filters(self, base_speed: float, driving_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Get target speed with filter layer processing applied.
        
        This is the new Filter Layer Architecture entry point that:
        1. Gets base cruise control mode from DCP foundation
        2. Applies all enabled speed controller filters (VTSC, MTSC, VCSC, PDA, APSL)
        3. Returns final speed target with filter details
        
        Args:
            base_speed: Base target speed from cruise control system
            driving_context: Current driving situation data
            
        Returns:
            Dict containing:
                - final_speed: Speed after all filters applied
                - base_speed: Original speed before filters
                - mpc_mode: Recommended MPC mode (acc/blended/None)
                - active_filters: List of active filter names
                - filter_details: Detailed filter application info
                - fallback_active: Whether fallback mode is active
                - apsl_integration: APSL-specific integration data
        """
        
        # Prepare enhanced driving context for APSL integration
        enhanced_context = self.prepare_driving_context_for_apsl(driving_context)
        
        # Get base MPC mode from DCP foundation
        mpc_mode = self.get_mpc_mode(enhanced_context)
        
        # Check if we're in fallback mode
        if mpc_mode is None:
            return {
                'final_speed': base_speed,
                'base_speed': base_speed,
                'mpc_mode': None,
                'active_filters': [],
                'filter_details': [],
                'fallback_active': True,
                'fallback_reason': 'DCP Mode 0 - OpenPilot longitudinal control',
                'apsl_integration': {'enabled': False, 'reason': 'DCP fallback mode'}
            }
        
        # Apply filter layer architecture with enhanced context
        filter_result = self.filter_manager.apply_filters(base_speed, enhanced_context)
        
        # Extract APSL integration data from enhanced context
        apsl_integration = {
            'enabled': enhanced_context.get('apsl_enabled', False),
            'driver_pedal_active': enhanced_context.get('driver_pedal_position', 0.0) > 0.05,
            'ndlob_active': enhanced_context.get('ndlob_active', False),
            'safety_limits_applied': {
                'vtsc_limit': enhanced_context.get('vtsc_speed_limit'),
                'mtsc_limit': enhanced_context.get('mtsc_speed_limit'),
                'vcsc_limit': enhanced_context.get('vcsc_speed_limit'),
                'following_limit': enhanced_context.get('following_speed_limit'),
                'max_safe_speed': enhanced_context.get('max_safe_speed', 33.5)
            }
        }
        
        return {
            'final_speed': filter_result['final_speed'],
            'base_speed': filter_result['base_speed'],
            'mpc_mode': mpc_mode,
            'active_filters': filter_result['active_filters'],
            'filter_details': filter_result['filter_details'],
            'fallback_active': False,
            'apsl_integration': apsl_integration,
            'fallback_reason': None
        }
    
    def register_filter_layer(self, filter_layer: DCPFilterLayer):
        """
        Register a new filter layer (VTSC, MTSC, VCSC, PDA, etc.)
        
        Args:
            filter_layer: Filter layer instance to register
        """
        self.filter_manager.register_filter(filter_layer)
    
    def enable_filter(self, filter_name: str, enabled: bool = True):
        """
        Enable or disable a specific filter layer
        
        Args:
            filter_name: Name of the filter to enable/disable
            enabled: Whether to enable (True) or disable (False) the filter
        """
        for filter_layer in self.filter_manager.filters:
            if filter_layer.name == filter_name:
                filter_layer.enabled = enabled
                logger.info(f"Filter {filter_name} {'enabled' if enabled else 'disabled'}")
                return
        logger.warning(f"[DCP] Filter {filter_name} not found for enable/disable")
    
    def get_filter_status(self) -> Dict[str, Any]:
        """Get current status of all filter layers"""
        return self.filter_manager.get_status()
    
    def _highway_behavior(self, context) -> str:
        """Highway mode: ACC-focused with configurable bias"""
        # Apply highway bias: 0.0 = always blended, 1.0 = always ACC
        if self.highway_bias >= 0.9:
            return "acc"  # Strong ACC preference
        elif self.highway_bias <= 0.1:
            return "blended"  # Strong blended preference
        else:
            # Use AEM for intelligent switching but bias toward ACC
            aem_mode = self._get_aem_mode(context)
            if aem_mode == "acc":
                return "acc"
            else:
                # Apply bias: higher bias = more likely to override to ACC
                if context.get('v_ego', 0) > 15.0 and self.highway_bias > 0.5:
                    return "acc"  # Override to ACC at highway speeds
                return aem_mode
    
    def _urban_behavior(self, context) -> str:
        """Urban mode: Blended-focused with configurable bias"""
        # Apply urban bias: 0.0 = always blended, 1.0 = always ACC
        if self.urban_bias >= 0.9:
            return "acc"  # Strong ACC preference (rare in urban)
        elif self.urban_bias <= 0.1:
            return "blended"  # Strong blended preference
        else:
            # Use AEM for intelligent switching but bias toward blended
            aem_mode = self._get_aem_mode(context)
            if aem_mode == "blended":
                return "blended"
            else:
                # Apply bias: lower bias = more likely to override to blended
                if context.get('v_ego', 0) < 20.0 and self.urban_bias < 0.5:
                    return "blended"  # Override to blended at urban speeds
                return aem_mode
    
    def _adaptive_behavior(self, context) -> str:
        """DCP mode: Full adaptive mode switching using AEM intelligence"""
        # Use AEM's sophisticated decision-making directly
        return self._get_aem_mode(context)
    
    def _validate_mode(self, mode_value):
        """Validate DCP mode with logging"""
        if mode_value < 0 or mode_value > 3:
            # Standardized logging using centralized logger
            logger.warning(f"[DCP] Invalid mode {mode_value}, using default Highway (1)")
            return DCPMode.HIGHWAY
        return DCPMode(mode_value)

    def _validate_personality(self, personality_value):
        """Validate personality with logging"""
        if personality_value < 0 or personality_value > 2:
            logger.warning(f"[DCP] Invalid personality {personality_value}, using default Standard (1)")
            return 1
        return personality_value

    def _validate_bias(self, bias_value, bias_type):
        """Validate bias parameters with logging"""
        if bias_value < 0.0 or bias_value > 1.0:
            default_bias = 0.8 if bias_type == "highway" else 0.3
            logger.warning(f"[DCP] Invalid {bias_type} bias {bias_value}, using default {default_bias}")
            return default_bias
        return bias_value

    def _parse_int_safe(self, value_str: str, default: int) -> int:
        """Safely parse integer from string with validation"""
        try:
            return int(value_str.strip()) if value_str else default
        except (ValueError, TypeError, AttributeError):
            logger.warning(f"[DCP] Invalid int value '{value_str}', using default {default}")
            return default

    def _parse_float_safe(self, value_str: str, default: float) -> float:
        """Safely parse float from string with validation"""
        try:
            return float(value_str.strip()) if value_str else default
        except (ValueError, TypeError, AttributeError):
            logger.warning(f"[DCP] Invalid float value '{value_str}', using default {default}")
            return default

    def _validate_parameter_interactions(self):
        """Validate parameter interactions and log warnings"""
        # Check for conflicting bias settings
        if self.highway_bias < 0.3 and self.urban_bias > 0.7:
            logger.warning("[DCP] Conflicting bias settings: highway prefers blended but urban prefers ACC")
        
        # Check for extreme personality with conflicting bias
        if self.personality == 0 and (self.highway_bias > 0.8 or self.urban_bias > 0.8):
            logger.warning("[DCP] Relaxed personality with high ACC bias may cause conflicts")
    
    def _register_eods_filter(self):
        """Register EODS filter layer if enabled"""
        try:
            # Check if EODS is enabled
            eods_enabled = self.params.get_bool("np_eods_enabled")
            if eods_enabled:
                from openpilot.selfdrive.controls.lib.nagaspilot.np_eods_controller import NpEODSController
                eods_filter = NpEODSController()
                self.filter_manager.register_filter(eods_filter)
                logger.info("[DCP] EODS filter registered - Enhanced Obstacle Detection System active (non-vehicle obstacles)")
            else:
                logger.info("[DCP] EODS filter not enabled")
        except Exception as e:
            # Handle registration failure gracefully with fallback mechanism
            logger.error(f"[DCP] Failed to register EODS filter: {e}")
            # Track failed filter registration for system health monitoring
            if not hasattr(self, 'failed_filters'):
                self.failed_filters = set()
            self.failed_filters.add('EODS')
            logger.warning("[DCP] EODS filter unavailable - Non-vehicle obstacle detection disabled")
            # Notify system health monitoring
            self.params.put("np_controller_registration_status", "eods_failed")
            # System continues operating with reduced safety functionality

    def _register_vtsc_filter(self):
        """Register VTSC filter layer if enabled"""
        try:
            # Check if VTSC is enabled
            vtsc_enabled = self.params.get_bool("np_vtsc_enabled")
            if vtsc_enabled:
                from openpilot.selfdrive.controls.lib.nagaspilot.np_vtsc_controller import NpVTSCController
                vtsc_filter = NpVTSCController()
                self.filter_manager.register_filter(vtsc_filter)
                logger.info("[DCP] VTSC filter registered - Vision Turn Speed Controller active")
            else:
                logger.info("[DCP] VTSC filter not enabled")
        except Exception as e:
            # Handle registration failure gracefully with fallback mechanism
            logger.error(f"[DCP] Failed to register VTSC filter: {e}")
            # Track failed filter registration for system health monitoring
            if not hasattr(self, 'failed_filters'):
                self.failed_filters = set()
            self.failed_filters.add('VTSC')
            logger.warning("[DCP] VTSC filter unavailable - Vision-based speed control disabled")
            # Notify system health monitoring
            self.params.put("np_controller_registration_status", "vtsc_failed")
            # System continues operating with reduced functionality

    def _register_mtsc_filter(self):
        """Register MTSC filter layer if enabled"""
        try:
            # Check if MTSC is enabled
            mtsc_enabled = self.params.get_bool("np_mtsc_enabled")
            if mtsc_enabled:
                from openpilot.selfdrive.controls.lib.nagaspilot.np_mtsc_controller import NpMTSCController
                mtsc_filter = NpMTSCController()
                self.filter_manager.register_filter(mtsc_filter)
                logger.info("[DCP] MTSC filter registered - Map Turn Speed Controller active")
            else:
                logger.info("[DCP] MTSC filter not enabled")
        except Exception as e:
            # Handle registration failure gracefully with fallback mechanism
            logger.error(f"[DCP] Failed to register MTSC filter: {e}")
            # Track failed filter registration for system health monitoring
            if not hasattr(self, 'failed_filters'):
                self.failed_filters = set()
            self.failed_filters.add('MTSC')
            logger.warning("[DCP] MTSC filter unavailable - Map-based speed control disabled")
            # Notify system health monitoring  
            self.params.put("np_controller_registration_status", "mtsc_failed")
            # System continues operating with reduced functionality

    def _register_vcsc_filter(self):
        """Register VCSC filter layer if enabled"""
        try:
            # Check if VCSC is enabled
            vcsc_enabled = self.params.get_bool("np_vcsc_enabled")
            if vcsc_enabled:
                from openpilot.selfdrive.controls.lib.nagaspilot.np_vcsc_controller import NpVCSCController
                vcsc_filter = NpVCSCController()
                self.filter_manager.register_filter(vcsc_filter)
                logger.info("[DCP] VCSC filter registered - Vertical Comfort Speed Controller active with Kalman filter")
            else:
                logger.info("[DCP] VCSC filter not enabled")
        except Exception as e:
            # Handle registration failure gracefully with fallback mechanism
            logger.error(f"[DCP] Failed to register VCSC filter: {e}")
            # Track failed filter registration for system health monitoring
            if not hasattr(self, 'failed_filters'):
                self.failed_filters = set()
            self.failed_filters.add('VCSC')
            logger.warning("[DCP] VCSC filter unavailable - Comfort-based speed control disabled")
            # System continues operating with reduced functionality

    def _register_pda_filter(self):
        """Register PDA filter layer if enabled"""
        try:
            # Check if PDA is enabled
            pda_enabled = self.params.get_bool("np_pda_enabled")
            if pda_enabled:
                from openpilot.selfdrive.controls.lib.nagaspilot.np_pda_controller import NpPDAController
                pda_filter = NpPDAController()
                self.filter_manager.register_filter(pda_filter)
                logger.info("[DCP] PDA filter registered - Parallel Drive Avoidance active with TTC safety monitoring")
            else:
                logger.info("[DCP] PDA filter not enabled")
        except Exception as e:
            # Handle registration failure gracefully with fallback mechanism
            logger.error(f"[DCP] Failed to register PDA filter: {e}")
            # Track failed filter registration for system health monitoring
            if not hasattr(self, 'failed_filters'):
                self.failed_filters = set()
            self.failed_filters.add('PDA')
            logger.warning("[DCP] PDA filter unavailable - Overtaking optimization disabled")
            # System continues operating with reduced functionality

    def _register_apsl_filter(self):
        """Register APSL filter layer if enabled"""
        try:
            # Check if APSL is enabled
            apsl_enabled = self.params.get_bool("EnableAPSL", False)
            if apsl_enabled:
                from openpilot.selfdrive.controls.lib.nagaspilot.np_apsl_controller import APSLFilter
                apsl_filter = APSLFilter()
                self.filter_manager.register_filter(apsl_filter)
                logger.info("[DCP] APSL filter registered - Accelerator Pedal Speed Learning with DCP integration")
            else:
                logger.info("[DCP] APSL filter not enabled")
        except Exception as e:
            # Handle registration failure gracefully with fallback mechanism
            logger.error(f"[DCP] Failed to register APSL filter: {e}")
            # Track failed filter registration for system health monitoring
            if not hasattr(self, 'failed_filters'):
                self.failed_filters = set()
            self.failed_filters.add('APSL')
            logger.warning("[DCP] APSL filter unavailable - Accelerator pedal learning disabled")
            # System continues operating with reduced functionality

    def _register_bpsl_filter(self):
        """Register BPSL filter layer if enabled"""
        try:
            # Check if BPSL is enabled
            bpsl_enabled = self.params.get_bool("EnableBPSL", False)
            if bpsl_enabled:
                from openpilot.selfdrive.controls.lib.nagaspilot.np_bpsl_controller import BPSLFilter
                bpsl_filter = BPSLFilter()
                self.filter_manager.register_filter(bpsl_filter)
                logger.info("[DCP] BPSL filter registered - Brake Pedal Speed Learning with dual-pedal coordination")
            else:
                logger.info("[DCP] BPSL filter not enabled")
        except Exception as e:
            # Handle registration failure gracefully with fallback mechanism
            logger.error(f"[DCP] Failed to register BPSL filter: {e}")
            # Track failed filter registration for system health monitoring
            if not hasattr(self, 'failed_filters'):
                self.failed_filters = set()
            self.failed_filters.add('BPSL')
            logger.warning("[DCP] BPSL filter unavailable - Brake pedal learning disabled")
            # System continues operating with reduced functionality

    def _get_aem_mode(self, context) -> str:
        """
        Safe wrapper to call AEM.get_mode() with proper parameter mapping.
        Maps DCP driving context to AEM's expected parameters.
        """
        try:
            # Map DCP context to AEM parameters
            return self.aem.get_mode(
                v_ego_raw=context.get('v_ego', 0.0),
                lead_one_data_raw=context.get('lead_one'),
                steering_angle_deg_raw=context.get('steering_angle', 0.0),
                standstill_raw=context.get('standstill', False),
                long_personality=context.get('long_personality', 1),
                v_model_error_raw=context.get('v_model_error', 0.0),
                allow_throttle_planner=context.get('allow_throttle', True),
                model_path_plan_raw=context.get('model_path_plan', {}),
                a_target_from_prev_cycle=context.get('a_target_prev', 0.0),
                model_predicts_stop_prev=context.get('model_predicts_stop_prev', False),
                fcw_active_prev=context.get('fcw_active_prev', False),
                mpc_source_prev=context.get('mpc_source_prev', 'acc')
            )
        except Exception as e:
            logger.error(f"[DCP] AEM.get_mode() failed: {e}")
            # Safe fallback to ACC on AEM failure
            return "acc"
    
    def update_parameters(self):
        """Update parameters from Params with validation (call periodically)"""
        try:
            # Atomic parameter validation to prevent TOCTOU issues
            mode_param = self.params.get("np_dcp_mode", encoding="utf-8") or "1"
            new_mode = self._validate_mode(self._parse_int_safe(mode_param, 1))
            if new_mode != self.mode:
                logger.info(f"[DCP] Mode changed: {self.mode} -> {new_mode}")
                self.mode = new_mode
            
            # Atomic parameter retrieval and validation
            personality_param = self.params.get("np_dcp_personality", encoding="utf-8") or "1"
            self.personality = self._validate_personality(self._parse_int_safe(personality_param, 1))
            
            highway_param = self.params.get("np_dcp_highway_bias", encoding="utf-8") or "0.8"
            self.highway_bias = self._validate_bias(self._parse_float_safe(highway_param, 0.8), "highway")
            
            urban_param = self.params.get("np_dcp_urban_bias", encoding="utf-8") or "0.3"
            self.urban_bias = self._validate_bias(self._parse_float_safe(urban_param, 0.3), "urban")
            
            # Update fallback control parameters
            self.fallback_enabled = self.params.get_bool("np_dcp_fallback_enabled", True)
            self.safety_fallback = self.params.get_bool("np_dcp_safety_fallback", True)
            
            # Update filter layer parameters
            for filter_layer in self.filter_manager.filters:
                filter_layer.update_parameters(self.params)
            
            # Re-validate parameter interactions after updates
            self._validate_parameter_interactions()
            
        except Exception as e:
            logger.warning(f"[DCP] Parameter update failed: {e}")
    
    def get_status_string(self) -> str:
        """Get human-readable status for debugging/UI with filter layer info"""
        mode_names = {
            DCPMode.OFF: "Fallback",
            DCPMode.HIGHWAY: "Highway", 
            DCPMode.URBAN: "Urban",
            DCPMode.DCP: "Adaptive"
        }
        personality_names = ["Relaxed", "Standard", "Aggressive"]
        
        # Get filter status
        filter_status = self.filter_manager.get_status()
        active_filters = " | ".join(self.filter_manager.last_applied_filters) if self.filter_manager.last_applied_filters else "None"
        
        base_status = f"DCP: {mode_names.get(self.mode, 'Unknown')} | " \
                     f"Personality: {personality_names[self.personality]} | " \
                     f"Highway Bias: {self.highway_bias:.1f} | " \
                     f"Urban Bias: {self.urban_bias:.1f}"
        
        filter_info = f" | Filters: {filter_status['enabled_filters']}/{filter_status['total_filters']} | " \
                     f"Active: {active_filters}"
        
        return base_status + filter_info


class DCPSafetyFallback:
    """Enhanced safety fallback system for DCP operations with Filter Layer Architecture support"""
    
    def __init__(self):
        self.consecutive_errors = 0
        self.max_errors_before_fallback = 5
        self.fallback_mode = DCPMode.HIGHWAY  # Safe default
        self.filter_errors = {}  # Track filter-specific errors
        
    def safe_get_mode(self, dcp_profile: DCPProfile, driving_context) -> str:
        """Safe wrapper for getting DCP mode with error handling (legacy method)"""
        try:
            # Reset error counter on successful operation
            mode = dcp_profile.get_mpc_mode(driving_context)
            self.consecutive_errors = 0
            return mode
            
        except Exception as e:
            self.consecutive_errors += 1
            logger.error(f"[DCP] Error in mode calculation: {e}")
            
            # Check if we need to enter fallback mode
            if self.consecutive_errors >= self.max_errors_before_fallback:
                logger.warning(f"[DCP] Too many errors ({self.consecutive_errors}), "
                               f"entering fallback mode: {self.fallback_mode}")
                return "acc" if self.fallback_mode == DCPMode.HIGHWAY else "blended"
            
            # Return safe default for single errors
            return "acc"
    
    def safe_get_target_speed_with_filters(self, dcp_profile: DCPProfile, base_speed: float, 
                                         driving_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Safe wrapper for getting target speed with filter layers applied
        
        Args:
            dcp_profile: DCP profile instance
            base_speed: Base target speed from cruise control
            driving_context: Current driving situation data
            
        Returns:
            Dict with target speed and safety status
        """
        try:
            # Get target speed with filters
            result = dcp_profile.get_target_speed_with_filters(base_speed, driving_context)
            
            # Reset error counter on successful operation
            self.consecutive_errors = 0
            
            # Validate result
            if result['final_speed'] <= 0 or result['final_speed'] > 200:  # Sanity check
                logger.warning(f"[DCP] Invalid final speed {result['final_speed']}, using base speed")
                result['final_speed'] = base_speed
                result['safety_override'] = True
            else:
                result['safety_override'] = False
            
            return result
            
        except Exception as e:
            self.consecutive_errors += 1
            logger.error(f"[DCP] Error in filter layer processing: {e}")
            
            # Check if we need to enter fallback
            if self.consecutive_errors >= self.max_errors_before_fallback:
                logger.warning(f"[DCP] Too many filter errors ({self.consecutive_errors}), "
                               "entering fallback")
                return {
                    'final_speed': base_speed,
                    'base_speed': base_speed,
                    'mpc_mode': "acc",  # Safe default
                    'active_filters': [],
                    'filter_details': [],
                    'fallback_active': True,
                    'fallback_reason': 'Emergency fallback due to filter errors',
                    'safety_override': True
                }
            
            # Return safe fallback for single errors
            return {
                'final_speed': base_speed,
                'base_speed': base_speed,
                'mpc_mode': "acc",
                'active_filters': [],
                'filter_details': [],
                'fallback_active': False,
                'fallback_reason': None,
                'safety_override': True
            }
    
    def log_filter_error(self, filter_name: str, error: Exception):
        """Log and track filter-specific errors"""
        if filter_name not in self.filter_errors:
            self.filter_errors[filter_name] = 0
        
        self.filter_errors[filter_name] += 1
        logger.error(f"[DCP] Filter {filter_name} error #{self.filter_errors[filter_name]}: {error}")
        
        # Disable problematic filters after too many errors
        if self.filter_errors[filter_name] >= 3:
            logger.warning(f"[DCP] Disabling filter {filter_name} due to repeated errors")
            return True  # Signal to disable filter
        
        return False
    
    def get_safety_status(self) -> Dict[str, Any]:
        """Get current safety system status"""
        return {
            'consecutive_errors': self.consecutive_errors,
            'max_errors_threshold': self.max_errors_before_fallback,
            'filter_errors': self.filter_errors.copy(),
            'fallback_mode': self.fallback_mode,
            'safety_active': self.consecutive_errors > 0
        }
        
    def get_system_health(self) -> Dict[str, Any]:
        """Get comprehensive system health including failed filter registrations"""
        failed_filters = getattr(self, 'failed_filters', set())
        filter_manager_status = self.filter_manager.get_status()
        
        return {
            'total_expected_filters': 6,  # VTSC, MTSC, VCSC, PDA, APSL, BPSL
            'successfully_registered': filter_manager_status['total_filters'],
            'failed_registrations': len(failed_filters),
            'failed_filter_names': list(failed_filters),
            'system_health': 'DEGRADED' if failed_filters else 'HEALTHY',
            'missing_capabilities': [f"{name} functionality unavailable" for name in failed_filters],
            'filter_errors': self.filter_errors.copy(),
            'consecutive_errors': self.consecutive_errors,
            'fallback_active': self.fallback_mode
        }