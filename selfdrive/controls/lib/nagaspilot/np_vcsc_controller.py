#!/usr/bin/env python3
"""
====================================================================
NAGASPILOT VCSC - VERTICAL COMFORT SPEED CONTROLLER
====================================================================

OVERVIEW:
VCSC implements an intelligent comfort-based speed control system using
locationd's Extended Kalman Filter IMU data to detect road surface conditions
and automatically reduce cruise speeds on rough terrain for enhanced passenger
comfort while maintaining safety and efficiency.

CORE FUNCTIONALITY:
- Kalman filter-enhanced vertical acceleration analysis
- Real-time road surface roughness detection
- Uncertainty-aware comfort score calculations
- Adaptive speed reduction based on comfort thresholds
- Integration with DCP filter layer architecture

OPERATIONAL LOGIC:
┌─────────────────────────────────────────────────────────────────┐
│ IMU DATA: Hardware sensors → locationd Kalman Filter           │
│ EXTRACTION: Bias-corrected acceleration + uncertainty estimates │
│ ANALYSIS: Vertical acceleration variance + jerk calculations    │
│ COMFORT SCORING: Weighted metrics with confidence intervals    │
│ SPEED ADJUSTMENT: Progressive reduction based on comfort score │
└─────────────────────────────────────────────────────────────────┘

COMFORT ASSESSMENT:
- Smooth roads: No speed reduction, full cruise speed maintained
- Minor roughness: 0.5-2 m/s reduction for subtle comfort improvement
- Moderate roughness: 2-5 m/s reduction for noticeable comfort gain
- Severe roughness: Up to 10 m/s reduction with 70% minimum speed ratio
- Uncertainty handling: Reduced sensitivity when IMU confidence low

KALMAN FILTER INTEGRATION:
┌─────────────────────────────────────────────────────────────────┐
│                    LocationD (Extended Kalman Filter)          │
│                            │                                   │
│                            ▼                                   │
│              Bias-Corrected Acceleration Data                  │
│              + Uncertainty Covariance Matrix                   │
│                            │                                   │
│                            ▼                                   │
│                    VCSC Comfort Analysis                       │
│                  ┌─────────────────────┐                       │
│                  │ Acceleration        │                       │
│                  │ Variance Analysis   │                       │
│                  │ ┌─────────────────┐ │                       │
│                  │ │ Jerk Calculation│ │                       │
│                  │ │ & Smoothing     │ │                       │
│                  │ └─────────────────┘ │                       │
│                  │ ┌─────────────────┐ │                       │
│                  │ │ Confidence      │ │                       │
│                  │ │ Assessment      │ │                       │
│                  │ └─────────────────┘ │                       │
│                  └─────────────────────┘                       │
│                            │                                   │
│                            ▼                                   │
│                    DCP Speed Modifier                          │
└─────────────────────────────────────────────────────────────────┘

TECHNICAL SPECIFICATIONS:
- Data source: livePose messages from locationd (20Hz)
- Buffer size: 2 seconds (40 samples) for memory efficiency
- Analysis window: Minimum 2.5 seconds for stable comfort metrics
- Confidence threshold: 0.3-1.0 range with uncertainty weighting
- Speed reduction: 0.5-10 m/s range with 70% minimum speed ratio
- Update frequency: 20Hz matching locationd output frequency

SAFETY FEATURES:
- Conservative comfort thresholds prevent unnecessary speed reductions
- Confidence-based adjustments for uncertain IMU readings
- Rate limiting prevents abrupt speed changes (1.5 m/s² max deceleration)
- Highway speed protection requires higher confidence (0.6+)
- Minimum speed ratio ensures reasonable cruise speeds (70%)
- Graceful fallback when locationd data unavailable

ALGORITHM WEIGHTS:
- Acceleration variance: 60% (primary comfort metric)
- Jerk variance: 30% (smoothness assessment)
- Uncertainty factor: 10% (confidence adjustment)
- Speed scaling: Variable based on vehicle velocity
- Angular velocity: Additional context for vehicle dynamics

MIT Non-Commercial License

Copyright (c) 2019-, rav4kumar, Rick Lan, dragonpilot community, and a number of other of contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only,
subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from dragonpilot. Contact ricklan@gmail.com for inquiries.
 * Any project that uses the Software must visibly mention the following acknowledgment: "This project uses software from dragonpilot and is licensed under a custom license requiring permission for use."

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import collections
import numpy as np
from typing import Dict, Any, Optional, Deque
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.nagaspilot.np_logger import NpLogger

# Initialize centralized logger for VCSC module
np_logger = NpLogger('vcsc')
from openpilot.selfdrive.controls.lib.nagaspilot.dcp_profile import DCPFilterLayer, DCPFilterType, DCPFilterResult
import openpilot.common.cereal_messaging as messaging


class NpVCSCController(DCPFilterLayer):
    """
    Vertical Comfort Speed Controller using locationd's Kalman filter data
    
    This controller monitors vertical acceleration and road surface conditions
    using high-quality, bias-corrected IMU data from locationd's Extended Kalman
    Filter to provide comfort-optimized speed control.
    """
    
    def __init__(self):
        """Initialize VCSC with Kalman filter data integration"""
        super().__init__(
            name="VCSC", 
            filter_type=DCPFilterType.SPEED_REDUCTION, 
            priority=3  # Lower priority than VTSC/MTSC curve controllers
        )
        
        self.params = Params()
        
        # Kalman filter data integration
        self.sm = messaging.SubMaster(['livePose'])
        self.pose_data_valid = False
        self.last_pose_time = 0.0
        
        # Enhanced acceleration buffers using Kalman filter data - optimized for memory usage
        # OPTIMIZED: 4 buffers × 40 entries × 3 float64 = ~3.8KB (reduced from ~9.6KB)
        # 40 entries = 2 seconds at 20Hz (sufficient for comfort analysis)
        self.accel_buffer: Deque[np.ndarray] = collections.deque(maxlen=40)  # 2 seconds at 20Hz
        self.accel_std_buffer: Deque[np.ndarray] = collections.deque(maxlen=40)  # Uncertainty buffer
        self.velocity_buffer: Deque[np.ndarray] = collections.deque(maxlen=40)  # Velocity context
        self.angular_vel_buffer: Deque[np.ndarray] = collections.deque(maxlen=40)  # Vehicle dynamics
        
        # Jerk calculation using filtered data - reduced buffer sizes
        self.jerk_buffer: Deque[np.ndarray] = collections.deque(maxlen=39)  # n-1 for derivative
        self.filtered_jerk_buffer: Deque[float] = collections.deque(maxlen=10)  # Reduced smoothed jerk buffer
        
        # Comfort calculation state
        self.last_comfort_score = 0.0
        self.last_confidence = 1.0
        self.last_speed_adjustment = 0.0
        self.last_speed_reduction = 0.0  # Previous speed_reduction for rate limiting
        self.comfort_trend = 0.0  # Trending comfort over time
        
        # Load and validate parameters with enhanced defaults
        self.enabled = self.params.get_bool("np_vcsc_enabled", False)
        self._load_parameters()
        
        # Debug and monitoring
        self.debug_enabled = self.params.get_bool("np_vcsc_debug_enabled", False)
        self.update_counter = 0
        # Fixed: Add exception handling consistent with other parameter reads
        try:
            self.log_interval = self.params.get_int("np_vcsc_log_interval", 100)
        except Exception as e:
            np_logger.warning(f"VCSC: Error reading log_interval parameter: {e}, using default 100")
            self.log_interval = 100
        
        np_logger.info(f"Initialized with Kalman filter integration - "
                     f"Enabled: {self.enabled}, Debug: {self.debug_enabled}")
    
    def _load_parameters(self):
        """Load and validate VCSC parameters with safety fallbacks"""
        try:
            # Core comfort thresholds
            self.comfort_threshold = max(0.5, min(10.0, 
                self.params.get_float("np_vcsc_comfort_threshold", 2.5)))
            self.jerk_threshold = max(0.1, min(5.0,
                self.params.get_float("np_vcsc_jerk_threshold", 1.0)))
            
            # Speed reduction parameters
            self.min_speed_reduction = max(0.0, min(5.0,
                self.params.get_float("np_vcsc_min_reduction", 0.5)))
            self.max_speed_reduction = max(self.min_speed_reduction, min(20.0,
                self.params.get_float("np_vcsc_max_reduction", 10.0)))
            
            # Safety constraints
            self.min_speed_ratio = max(0.3, min(0.95,
                self.params.get_float("np_vcsc_min_speed_ratio", 0.7)))
            self.max_decel_rate = max(0.1, min(3.0,
                self.params.get_float("np_vcsc_max_decel_rate", 1.5)))
            
            # Algorithm weights for Kalman filter enhancement
            self.accel_variance_weight = max(0.0, min(1.0,
                self.params.get_float("np_vcsc_accel_weight", 0.6)))
            self.jerk_variance_weight = max(0.0, min(1.0,
                self.params.get_float("np_vcsc_jerk_weight", 0.3)))
            self.uncertainty_weight = max(0.0, min(1.0,
                self.params.get_float("np_vcsc_uncertainty_weight", 0.1)))
            
            # Normalize weights to sum to 1.0
            total_weight = (self.accel_variance_weight + self.jerk_variance_weight + 
                          self.uncertainty_weight)
            if total_weight > 0:
                self.accel_variance_weight /= total_weight
                self.jerk_variance_weight /= total_weight
                self.uncertainty_weight /= total_weight
            
            # Speed-dependent comfort scaling
            self.speed_comfort_scaling = max(0.1, min(2.0,
                self.params.get_float("np_vcsc_speed_scaling", 0.8)))
            
            # Confidence and uncertainty handling
            self.min_confidence = max(0.1, min(1.0,
                self.params.get_float("np_vcsc_min_confidence", 0.3)))
            self.uncertainty_factor = max(0.1, min(5.0,
                self.params.get_float("np_vcsc_uncertainty_factor", 2.0)))
            
            np_logger.info(f"Parameters loaded - Comfort threshold: {self.comfort_threshold:.2f}, "
                         f"Speed reduction: {self.min_speed_reduction:.1f}-{self.max_speed_reduction:.1f} m/s")
            
        except Exception as e:
            np_logger.error(f"Parameter loading failed: {e}")
            # Use safe defaults
            self.comfort_threshold = 2.5
            self.jerk_threshold = 1.0
            self.min_speed_reduction = 0.5
            self.max_speed_reduction = 10.0
            self.min_speed_ratio = 0.7
            self.max_decel_rate = 1.5
            self.accel_variance_weight = 0.6
            self.jerk_variance_weight = 0.3
            self.uncertainty_weight = 0.1
            self.speed_comfort_scaling = 0.8
            self.min_confidence = 0.3
            self.uncertainty_factor = 2.0
    
    def process(self, speed_target: float, driving_context: Dict[str, Any]) -> DCPFilterResult:
        """
        Main VCSC processing using Kalman filter enhanced data
        
        Args:
            speed_target: Current target speed from DCP foundation
            driving_context: Current driving situation data
            
        Returns:
            DCPFilterResult with comfort-based speed adjustment
        """
        self.update_counter += 1
        
        # Update locationd data
        if not self._update_kalman_data():
            return DCPFilterResult(speed_modifier=1.0, active=False, 
                                 reason="No valid locationd data", priority=self.priority)
        
        # Check if we have sufficient data
        if len(self.accel_buffer) < 50:  # Need at least 2.5 seconds of data
            return DCPFilterResult(speed_modifier=1.0, active=False,
                                 reason="Insufficient acceleration history", priority=self.priority)
        
        try:
            # Calculate enhanced comfort metrics using Kalman filter data
            comfort_metrics = self._calculate_enhanced_comfort_metrics()
            
            # Assess road conditions with uncertainty consideration
            comfort_assessment = self._assess_road_conditions(comfort_metrics, driving_context)
            
            # Calculate speed adjustment if needed
            if comfort_assessment['requires_adjustment']:
                speed_modifier = self._calculate_comfort_speed_modifier(
                    comfort_assessment, speed_target, driving_context)
                
                self.last_speed_adjustment = (1.0 - speed_modifier) * speed_target
                
                # Debug logging
                if self.debug_enabled and self.update_counter % self.log_interval == 0:
                    self._debug_log_comfort_state(comfort_metrics, comfort_assessment, speed_modifier)
                
                return DCPFilterResult(
                    speed_modifier=speed_modifier,
                    active=True,
                    reason=f"Comfort: {comfort_assessment['comfort_score']:.2f}, "
                           f"Confidence: {comfort_assessment['confidence']:.2f}",
                    priority=self.priority
                )
            
            return DCPFilterResult(speed_modifier=1.0, active=False,
                                 reason="Road conditions acceptable for comfort", priority=self.priority)
            
        except Exception as e:
            np_logger.error(f"Processing error: {e}")
            return DCPFilterResult(speed_modifier=1.0, active=False,
                                 reason=f"Processing error: {str(e)}", priority=self.priority)
    
    def _update_kalman_data(self) -> bool:
        """
        Update IMU data from locationd's Kalman filter
        
        Returns:
            bool: True if valid data was obtained
        """
        try:
            self.sm.update(0)  # Non-blocking update
            
            if not self.sm.updated['livePose']:
                return self.pose_data_valid  # Return previous validity state
            
            pose_msg = self.sm['livePose']
            
            # Check data validity and freshness
            current_time = pose_msg.logMonoTime / 1e9  # Convert to seconds
            if current_time <= self.last_pose_time:
                return self.pose_data_valid  # Stale data
            
            self.last_pose_time = current_time
            
            # Extract Kalman filter states (bias-corrected)
            pose_data = pose_msg.pose
            
            # Device frame acceleration (bias-corrected from Kalman filter)
            device_accel = np.array([
                pose_data.acceleration.x,
                pose_data.acceleration.y, 
                pose_data.acceleration.z
            ])
            
            # Acceleration uncertainty (from Kalman filter covariance)
            accel_std = np.array([
                pose_data.accelerationStd.x,
                pose_data.accelerationStd.y,
                pose_data.accelerationStd.z
            ])
            
            # Device frame velocity for context
            device_velocity = np.array([
                pose_data.velocityDevice.x,
                pose_data.velocityDevice.y,
                pose_data.velocityDevice.z
            ])
            
            # Angular velocity for vehicle dynamics context
            angular_velocity = np.array([
                pose_data.angularVelocityDevice.x,
                pose_data.angularVelocityDevice.y,
                pose_data.angularVelocityDevice.z
            ])
            
            # Sanity checks on Kalman filter data
            # ⚠️ STRICT: May reject valid data due to occasional NaN in uncertainty estimates
            # TODO: Consider allowing NaN in std but not in acceleration values
            if np.any(np.isnan(device_accel)) or np.any(np.isnan(accel_std)):
                np_logger.warning("NaN values in locationd data")
                return False
            
            if np.linalg.norm(device_accel) > 50.0:  # Reasonable acceleration limit
                np_logger.warning(f"Excessive acceleration: {np.linalg.norm(device_accel):.2f} m/s²")
                return False
            
            # Add to buffers
            self.accel_buffer.append(device_accel)
            self.accel_std_buffer.append(accel_std)
            self.velocity_buffer.append(device_velocity)
            self.angular_vel_buffer.append(angular_velocity)
            
            # Calculate and store jerk from filtered acceleration
            if len(self.accel_buffer) >= 2:
                # ⚠️ ASSUMPTION: Hardcoded 20Hz update rate may not match actual timing
                # TODO: Calculate actual dt from pose message timestamps
                dt = 0.05  # 20Hz update rate
                jerk = (device_accel - self.accel_buffer[-2]) / dt
                self.jerk_buffer.append(jerk)
                
                # Smooth jerk calculation
                if len(self.jerk_buffer) >= 5:
                    recent_jerk = np.array(list(self.jerk_buffer)[-5:])
                    smooth_jerk_magnitude = np.mean(np.linalg.norm(recent_jerk, axis=1))
                    self.filtered_jerk_buffer.append(smooth_jerk_magnitude)
            
            self.pose_data_valid = True
            return True
            
        except Exception as e:
            np_logger.warning(f"LocationD data unavailable - using fallback behavior: {e}")
            self.pose_data_valid = False
            return False
    
    def _calculate_enhanced_comfort_metrics(self) -> Dict[str, Any]:
        """
        Calculate enhanced comfort metrics using Kalman filter data
        
        Returns:
            Dict with comprehensive comfort analysis
        """
        if len(self.accel_buffer) < 10:
            return {'valid': False}
        
        # Convert buffers to numpy arrays efficiently
        # OPTIMIZED: Use numpy.stack() which is faster than np.array(list()) for structured data
        accel_array = np.stack(self.accel_buffer)  # Direct stack without list conversion
        accel_std_array = np.stack(self.accel_std_buffer)  # More efficient for deques of arrays
        velocity_array = np.stack(self.velocity_buffer)
        
        # Focus on vertical (Z-axis) acceleration for comfort
        vertical_accel = accel_array[:, 2]  # Z-axis (up/down)
        vertical_accel_std = accel_std_array[:, 2]
        
        # Bias-corrected acceleration variance (primary comfort metric)
        accel_variance = np.var(vertical_accel)
        accel_std_dev = np.std(vertical_accel)
        
        # Jerk analysis using filtered data
        jerk_variance = 0.0
        jerk_std_dev = 0.0
        if len(self.filtered_jerk_buffer) >= 10:
            jerk_array = np.array(list(self.filtered_jerk_buffer))
            jerk_variance = np.var(jerk_array)
            jerk_std_dev = np.std(jerk_array)
        
        # Uncertainty-based confidence calculation
        mean_uncertainty = np.mean(vertical_accel_std)
        confidence = max(self.min_confidence, 
                        1.0 - min(1.0, mean_uncertainty * self.uncertainty_factor))
        
        # Speed-dependent comfort scaling
        current_speed = np.linalg.norm(velocity_array[-1]) if len(velocity_array) > 0 else 0.0
        speed_factor = 1.0 + (current_speed / 30.0) * self.speed_comfort_scaling
        
        # Vehicle dynamics context (angular velocity impact)
        angular_impact = 0.0
        if len(self.angular_vel_buffer) >= 10:
            angular_array = np.array(list(self.angular_vel_buffer))
            angular_magnitude = np.mean(np.linalg.norm(angular_array, axis=1))
            angular_impact = min(0.5, angular_magnitude * 0.1)  # Scale and limit impact
        
        # Combined comfort score with Kalman filter enhancement
        weighted_comfort = (
            self.accel_variance_weight * accel_variance * speed_factor +
            self.jerk_variance_weight * jerk_variance +
            self.uncertainty_weight * mean_uncertainty +
            angular_impact
        )
        
        return {
            'valid': True,
            'accel_variance': accel_variance,
            'accel_std_dev': accel_std_dev,
            'jerk_variance': jerk_variance,
            'jerk_std_dev': jerk_std_dev,
            'weighted_comfort': weighted_comfort,
            'confidence': confidence,
            'mean_uncertainty': mean_uncertainty,
            'current_speed': current_speed,
            'speed_factor': speed_factor,
            'angular_impact': angular_impact,
            'sample_count': len(vertical_accel)
        }
    
    def _assess_road_conditions(self, comfort_metrics: Dict[str, Any], 
                              driving_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Assess road conditions and determine if speed adjustment is needed
        
        Args:
            comfort_metrics: Calculated comfort metrics
            driving_context: Current driving situation
            
        Returns:
            Dict with road condition assessment
        """
        if not comfort_metrics.get('valid', False):
            return {'requires_adjustment': False, 'reason': 'Invalid comfort metrics'}
        
        # Primary comfort assessment
        comfort_score = comfort_metrics['weighted_comfort']
        confidence = comfort_metrics['confidence']
        
        # Adjust thresholds based on confidence
        effective_comfort_threshold = self.comfort_threshold / max(0.5, confidence)
        effective_jerk_threshold = self.jerk_threshold / max(0.5, confidence)
        
        # Multi-criteria assessment
        comfort_exceeded = comfort_score > effective_comfort_threshold
        jerk_exceeded = comfort_metrics['jerk_variance'] > effective_jerk_threshold
        
        # Context-based adjustments
        vehicle_speed = driving_context.get('v_ego', 0.0)
        is_highway_speed = vehicle_speed > 25.0  # ~55 mph
        
        # Require higher confidence for highway speeds
        min_highway_confidence = 0.6
        if is_highway_speed and confidence < min_highway_confidence:
            return {
                'requires_adjustment': False,
                'reason': f'Insufficient confidence {confidence:.2f} for highway speed',
                'comfort_score': comfort_score,
                'confidence': confidence
            }
        
        # Trend analysis for stability
        self.comfort_trend = 0.9 * self.comfort_trend + 0.1 * comfort_score
        trend_stable = abs(comfort_score - self.comfort_trend) < (self.comfort_threshold * 0.3)
        
        # Final decision
        requires_adjustment = (comfort_exceeded or jerk_exceeded) and trend_stable
        
        return {
            'requires_adjustment': requires_adjustment,
            'comfort_score': comfort_score,
            'confidence': confidence,
            'comfort_exceeded': comfort_exceeded,
            'jerk_exceeded': jerk_exceeded,
            'trend_stable': trend_stable,
            'effective_comfort_threshold': effective_comfort_threshold,
            'effective_jerk_threshold': effective_jerk_threshold,
            'reason': f'Comfort: {comfort_score:.2f}>{effective_comfort_threshold:.2f}, '
                     f'Jerk: {jerk_exceeded}, Stable: {trend_stable}'
        }
    
    def _calculate_comfort_speed_modifier(self, comfort_assessment: Dict[str, Any],
                                        speed_target: float, 
                                        driving_context: Dict[str, Any]) -> float:
        """
        Calculate speed modifier based on comfort assessment
        
        Args:
            comfort_assessment: Road condition assessment
            speed_target: Current target speed
            driving_context: Driving context data
            
        Returns:
            float: Speed modifier (1.0 = no change, <1.0 = slower)
        """
        comfort_score = comfort_assessment['comfort_score']
        confidence = comfort_assessment['confidence']
        
        # Calculate base reduction factor
        excess_comfort = comfort_score - comfort_assessment['effective_comfort_threshold']
        reduction_factor = min(1.0, excess_comfort / comfort_assessment['effective_comfort_threshold'])
        
        # Apply confidence weighting
        confidence_weighted_factor = reduction_factor * confidence
        
        # Calculate speed reduction
        speed_reduction = (self.min_speed_reduction + 
                          confidence_weighted_factor * (self.max_speed_reduction - self.min_speed_reduction))
        
        # Limit maximum deceleration rate for passenger comfort (fixed rate limiting logic)
        if hasattr(self, 'last_speed_reduction'):
            max_change_per_cycle = self.max_decel_rate * 0.05  # 0.05s cycle time (20Hz)
            
            # Calculate maximum allowed speed_reduction based on previous value
            max_allowed_reduction = self.last_speed_reduction + max_change_per_cycle
            
            # Limit rate of increase in speed reduction for smooth comfort adjustments
            if speed_reduction > max_allowed_reduction:
                speed_reduction = max_allowed_reduction
                np_logger.debug(f"VCSC: Rate limited speed_reduction to {speed_reduction:.3f} m/s")
        
        # Store current speed_reduction for next cycle rate limiting
        self.last_speed_reduction = speed_reduction
        
        # Apply speed modifier with safety constraints and division by zero protection
        if speed_target < 1.0:  # Protect against division by zero and very small speeds
            np_logger.warning(f"VCSC: Speed target too low ({speed_target:.2f} m/s), returning neutral modifier")
            return 1.0
        
        speed_modifier = max(self.min_speed_ratio, 
                           (speed_target - speed_reduction) / speed_target)
        
        # Store comfort state for next cycle
        self.last_comfort_score = comfort_score
        self.last_confidence = confidence
        
        return speed_modifier
    
    def _debug_log_comfort_state(self, comfort_metrics: Dict[str, Any], 
                               comfort_assessment: Dict[str, Any],
                               speed_modifier: float):
        """Log detailed debug information about comfort state"""
        np_logger.debug(
            f"[VCSC] Comfort State - "
            f"Score: {comfort_metrics['weighted_comfort']:.3f}, "
            f"Confidence: {comfort_metrics['confidence']:.3f}, "
            f"Accel Var: {comfort_metrics['accel_variance']:.3f}, "
            f"Jerk Var: {comfort_metrics['jerk_variance']:.3f}, "
            f"Uncertainty: {comfort_metrics['mean_uncertainty']:.3f}, "
            f"Speed Mod: {speed_modifier:.3f}, "
            f"Samples: {comfort_metrics['sample_count']}"
        )
    
    def update_parameters(self, params: Params):
        """Update parameters from Params system"""
        self.params = params
        old_enabled = self.enabled
        self.enabled = params.get_bool("np_vcsc_enabled", False)
        
        if old_enabled != self.enabled:
            np_logger.info(f"Enabled state changed: {old_enabled} -> {self.enabled}")
            
        self._load_parameters()
    
    def get_status(self) -> Dict[str, Any]:
        """Get current VCSC status for monitoring"""
        return {
            'enabled': self.enabled,
            'pose_data_valid': self.pose_data_valid,
            'accel_buffer_size': len(self.accel_buffer),
            'jerk_buffer_size': len(self.jerk_buffer),
            'last_comfort_score': self.last_comfort_score,
            'last_confidence': self.last_confidence,
            'last_speed_adjustment': self.last_speed_adjustment,
            'comfort_trend': self.comfort_trend,
            'update_counter': self.update_counter,
            'parameters': {
                'comfort_threshold': self.comfort_threshold,
                'jerk_threshold': self.jerk_threshold,
                'min_speed_reduction': self.min_speed_reduction,
                'max_speed_reduction': self.max_speed_reduction,
                'min_speed_ratio': self.min_speed_ratio
            }
        }

    def is_active(self) -> bool:
        """Check if VCSC is currently active"""
        return self.enabled and self.pose_data_valid and len(self.accel_buffer) >= 50