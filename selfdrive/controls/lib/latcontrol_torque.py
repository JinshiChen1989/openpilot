import math

from cereal import log
from common.numpy_fast import interp
from common.filter_simple import FirstOrderFilter
from selfdrive.controls.lib.latcontrol import LatControl
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    
    # DTSA (Dynamic Torque Steering Adjustment) for vehicles that support it
    # Adaptive timing compensation for EPS delay variations
    self.enable_dtsa = self._check_dtsa_enabled(CP)
    if self.enable_dtsa:
      self._init_dtsa(CP)
      
  def _check_dtsa_enabled(self, CP):
    """Check if DTSA is enabled for this vehicle"""
    return (hasattr(CP.lateralParams, 'dtsaEnable') and 
            CP.lateralParams.dtsaEnable and
            hasattr(CP.lateralTuning, 'torque'))  # Only for torque mode
      
  def _init_dtsa(self, CP):
    """Initialize DTSA system using CarParams configuration"""
    # DTSA timing parameters from CarParams
    self._dtsa_tau_s = CP.lateralParams.dtsaTau if hasattr(CP.lateralParams, 'dtsaTau') else 0.5
    self._dtsa_dt_s = 0.01  # control timestep
    self._dtsa_filter = FirstOrderFilter(0.0, self._dtsa_tau_s, self._dtsa_dt_s)
    self._dtsa = 0.0  # current timing adjustment
    self._dtsa_max = CP.lateralParams.dtsaMaxTrim if hasattr(CP.lateralParams, 'dtsaMaxTrim') else 0.4
    self._dtsa_scale = CP.lateralParams.dtsaScale if hasattr(CP.lateralParams, 'dtsaScale') else 0.02
    self._dtsa_deadband = CP.lateralParams.dtsaDeadband if hasattr(CP.lateralParams, 'dtsaDeadband') else 0.05
    self._dtsa_min_speed = CP.lateralParams.dtsaMinSpeed if hasattr(CP.lateralParams, 'dtsaMinSpeed') else 5.0
    
    # State tracking
    self._last_req_torque = 0.0
    self._last_applied_torque = None

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk):
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params, setpoint,
                                                     lateral_accel_deadzone, friction_compensation=False)
      torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params, measurement,
                                                     lateral_accel_deadzone, friction_compensation=False)
      pid_log.error = torque_from_setpoint - torque_from_measurement
      ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
                                          desired_lateral_accel - actual_lateral_accel,
                                          lateral_accel_deadzone, friction_compensation=True)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

      # DTSA: Update adaptive timing adjustment for supported vehicles
      if self.enable_dtsa:
        self._update_dtsa(output_torque, CS, steer_limited)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
    
  def _update_dtsa(self, requested_torque, CS, steer_limited):
    """Update DTSA timing adjustment based on torque mismatch"""
    # Track requested torque (before sign flip)
    self._last_req_torque = float(-requested_torque)
    
    # Get actual applied torque from EPS (if available)
    applied = getattr(CS, 'steeringTorque', None)
    if applied is not None:
      self._last_applied_torque = float(applied)
      
      # Only learn when conditions are good:
      # - High enough speed (avoid low-speed noise)
      # - No driver interference 
      # - Not torque-limited by safety
      learning_conditions = (
        CS.vEgo > self._dtsa_min_speed and
        not CS.steeringPressed and 
        not steer_limited
      )
      
      if learning_conditions:
        # Measure torque mismatch: if EPS perfectly tracks controller, 
        # requested + applied ≈ 0 (opposite signs cancel out)
        mismatch = abs(self._last_req_torque + self._last_applied_torque)
        
        # Apply configurable deadband to ignore sensor noise
        if mismatch < self._dtsa_deadband:
          mismatch = 0.0
          
        # Convert mismatch to timing adjustment using configurable scaling
        dtsa_raw = mismatch * self._dtsa_scale
        
        # Smooth with low-pass filter and apply limits
        self._dtsa = max(0.0, min(self._dtsa_filter.update(dtsa_raw), self._dtsa_max))
