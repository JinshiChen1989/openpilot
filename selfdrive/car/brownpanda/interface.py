# OpenPilot framework imports
from cereal import car
from selfdrive.car import get_safety_config, scale_tire_stiffness
from selfdrive.car.brownpanda.carcontroller import CarController
from selfdrive.car.brownpanda.carstate import CarState
from selfdrive.car.brownpanda.values import CAR, BrownPandaFlags, CarControllerParams
from selfdrive.car.interfaces import CarInterfaceBase
from common.conversions import Conversions as CV

# Framework type aliases
SteerControlType = car.CarParams.SteerControlType


# =============================================================================
# [Section] Class
# [Brief ] Configure BrownPanda CarParams and expose control hooks
# ---------------------------------------------------------------------------
# [Class ] CarInterface
# [Brief ] Sets safety, tuning, and capability flags
# [Bases ] CarInterfaceBase
# =============================================================================
class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  # =============================================================================
  # [Section] Accel Limits
  # [Brief ] Provide PID accel bounds for longitudinal controller
  # ---------------------------------------------------------------------------
  # [Function] get_pid_accel_limits
  # [Brief ] Provide PID accel bounds for longitudinal controller
  # [Params] CP: CarParams, current_speed: float, cruise_speed: float
  # [Returns] (min_accel: float, max_accel: float)
  # =============================================================================
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  # =============================================================================
  # [Section] Params
  # [Brief ] Populate BrownPanda CarParams with platform-specific settings
  # ---------------------------------------------------------------------------
  # [Function] _get_params
  # [Brief ] Populate BrownPanda CarParams with platform-specific settings
  # [Params] ret: CarParams, candidate: CAR, fingerprint: dict, car_fw: list, experimental_long: bool, docs: bool
  # [Returns] CarParams
  # =============================================================================
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    # Basic vehicle identification
    ret.carName = "BrownPanda"

    # Safety configuration (hardware-level, following Honda Bosch pattern)
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.brownpanda)]

    # Extract vehicle-specific specs from CAR_SPECS (DragonPilot pattern)
    from selfdrive.car.brownpanda.values import CAR_SPECS
    specs = CAR_SPECS.get(candidate, {})
    
    # Vehicle physical specs using DragonPilot-style dictionary
    ret.mass = specs.get('mass', 1700.)                               # Vehicle curb weight (kg)
    ret.wheelbase = specs.get('wheelbase', 2.72)                      # Distance between axles (m)
    ret.steerRatio = specs.get('steerRatio', 16.0)                    # Steering wheel to wheel angle ratio
    ret.centerToFrontRatio = specs.get('centerToFrontRatio', 0.4)     # CG position (0.5 = center)
    tire_stiffness_factor = specs.get('tireStiffnessFactor', 0.7)     # Tire grip factor
    ret.minSteerSpeed = specs.get('minSteerSpeed', 5.0 * CV.KPH_TO_MS) # Min speed for steering
    ret.minEnableSpeed = specs.get('minEnableSpeed', 5.0 * CV.KPH_TO_MS) # Min speed for ADAS

    ret.centerToFront = ret.wheelbase * ret.centerToFrontRatio        # Distance from CG to front axle
    ret.dashcamOnly = False                                           # Full ADAS capability

    # Control type by model: S05 uses angle control, others use torque (with DTSA)
    if candidate == CAR.DEEPAL_S05:
      ret.steerControlType = SteerControlType.angle   # DEEPAL S05 uses steering angle control
    else:
      ret.steerControlType = SteerControlType.torque  # BYD models use torque control (enables DTSA)

    # Steering actuator delay by powertrain
    if ret.flags & BrownPandaFlags.EV:
      ret.steerActuatorDelay = 0.08  # Fast EV steering response (electric power steering)
    elif ret.flags & BrownPandaFlags.HYBRID:
      ret.steerActuatorDelay = 0.12  # Moderate hybrid response
    else:
      ret.steerActuatorDelay = 0.15  # Slower ICE response

    ret.steerLimitTimer = 0.4        # Standard automotive steering limit timer

    # Longitudinal control (camera-based)
    ret.openpilotLongitudinalControl = True                                 # Use openpilot for speed control
    ret.stoppingControl = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)     # Can stop/start automatically
    ret.startingState = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)       # Automatic starting capability
    ret.radarUnavailable = True                                             # Camera-only system

    # Low-speed behavior by powertrain
    if ret.flags & BrownPandaFlags.EV:
      ret.vEgoStopping = 0.3     # EVs can control precisely at low speeds
      ret.vEgoStarting = 0.3     # Smooth EV acceleration from stop
      ret.stopAccel = -2.5       # Strong regenerative braking capability
    else:
      ret.vEgoStopping = 0.5     # ICE less precise at very low speeds
      ret.vEgoStarting = 0.5     # Standard ICE starting threshold
      ret.stopAccel = -2.0       # Standard friction braking

    ret.stoppingDecelRate = 0.8    # Deceleration rate when approaching stop
    ret.maxSteeringAngleDeg = 1080 # Maximum steering wheel angle (3 turns)

    # Longitudinal PID tuning (per model)
    if candidate == CAR.BYD_ATTO3:
      # BYD ATTO3: Balanced SUV tuning for comfort and efficiency
      ret.longitudinalTuning.kf = 1.0                                     # Feedforward gain
      ret.longitudinalTuning.kpBP = [0., 5., 15., 30.]                    # Speed breakpoints (m/s)
      ret.longitudinalTuning.kpV = [1.6, 1.1, 0.7, 0.4]                  # Proportional gains
      ret.longitudinalTuning.kiBP = [0., 5., 15., 30.]                    # Integral breakpoints
      ret.longitudinalTuning.kiV = [0.20, 0.15, 0.10, 0.06]              # Integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.15                      # Min actuator delay
      ret.longitudinalActuatorDelayUpperBound = 0.25                      # Max actuator delay
    elif candidate == CAR.DEEPAL_S05:
      # DEEPAL S05: Performance sedan tuning for responsive acceleration
      ret.longitudinalTuning.kf = 1.2                                     # Higher feedforward for performance
      ret.longitudinalTuning.kpBP = [0., 8., 20., 35.]                    # Performance-oriented breakpoints
      ret.longitudinalTuning.kpV = [2.0, 1.4, 0.9, 0.6]                  # More aggressive gains
      ret.longitudinalTuning.kiBP = [0., 8., 20., 35.]                    # Matching integral points
      ret.longitudinalTuning.kiV = [0.30, 0.20, 0.14, 0.10]              # Higher integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.10                      # Fast performance response
      ret.longitudinalActuatorDelayUpperBound = 0.18
    elif candidate == CAR.BYD_DOLPHIN:
      # BYD DOLPHIN: City-focused compact EV with smooth, comfortable tuning
      ret.longitudinalTuning.kf = 1.1                                     # Balanced feedforward
      ret.longitudinalTuning.kpBP = [0., 6., 15., 30.]                    # City-optimized breakpoints
      ret.longitudinalTuning.kpV = [1.8, 1.2, 0.8, 0.5]                  # Smooth responsive gains
      ret.longitudinalTuning.kiBP = [0., 6., 15., 30.]                    # Matching integral points
      ret.longitudinalTuning.kiV = [0.24, 0.18, 0.12, 0.08]              # Moderate integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.12                      # Comfortable response
      ret.longitudinalActuatorDelayUpperBound = 0.22
    else:
      # Conservative fallback tuning for unknown vehicles
      ret.longitudinalTuning.kf = 1.0                                     # Safe feedforward
      ret.longitudinalTuning.kpBP = [0., 5., 35.]                         # Simple breakpoints
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]                       # Conservative gains
      ret.longitudinalTuning.kiBP = [0., 35.]                             # Basic integral points
      ret.longitudinalTuning.kiV = [0.18, 0.12]                          # Safe integral gains
      ret.longitudinalActuatorDelayLowerBound = 0.20                      # Conservative delays
      ret.longitudinalActuatorDelayUpperBound = 0.30

    # Model-specific lateral control tuning for optimal steering feel
    if ret.steerControlType == SteerControlType.torque:
      ret.lateralTuning.init('torque')
      ret.lateralTuning.torque.useSteeringAngle = True
      ret.lateralTuning.torque.kf = 1.0
      ret.lateralTuning.torque.steeringAngleDeadzoneDeg = 0.0

      # Torque-controller tuning (per model) - Following DragonPilot patterns
      if candidate == CAR.BYD_ATTO3:
        ret.lateralTuning.torque.kp = 2.2
        ret.lateralTuning.torque.ki = 0.18
        ret.lateralTuning.torque.friction = 0.02
        ret.lateralParams.torqueBP = [0., 5., 15., 30., 50.]
        ret.lateralParams.torqueV = [0., 80., 160., 320., 500.]
      elif candidate == CAR.BYD_DOLPHIN:
        ret.lateralTuning.torque.kp = 2.4
        ret.lateralTuning.torque.ki = 0.20
        ret.lateralTuning.torque.friction = 0.015
        ret.lateralParams.torqueBP = [0., 5., 12., 22., 35.]
        ret.lateralParams.torqueV = [0., 70., 140., 280., 430.]
      else:
        # Generic EV/ICE defaults
        if ret.flags & BrownPandaFlags.EV:
          ret.lateralTuning.torque.kp = 1.2
          ret.lateralTuning.torque.ki = 0.12
          ret.lateralTuning.torque.friction = 0.08
          ret.lateralParams.torqueBP = [0., 8., 15., 25., 40.]
          ret.lateralParams.torqueV = [0., 60., 120., 250., 400.]
        else:
          ret.lateralTuning.torque.kp = 1.0
          ret.lateralTuning.torque.ki = 0.1
          ret.lateralTuning.torque.friction = 0.1
          ret.lateralParams.torqueBP = [0., 5., 10., 20., 30.]
          ret.lateralParams.torqueV = [0., 50., 100., 200., 300.]

      # DTSA configuration using DragonPilot patterns (applies to all torque control)
      ret.lateralParams.dtsaEnable = specs.get('dtsaEnable', False)
      ret.lateralParams.dtsaTau = 0.5
      ret.lateralParams.dtsaMaxTrim = specs.get('dtsaMaxTorque', 0.4) 
      ret.lateralParams.dtsaScale = 0.02
      ret.lateralParams.dtsaDeadband = 0.05
      ret.lateralParams.dtsaMinSpeed = 5.0
    else:
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kf = 0.00008
      ret.lateralTuning.pid.kdBP = [0.]
      ret.lateralTuning.pid.kdV = [0.]

      # PID (angle) tuning (per model)
      if candidate == CAR.BYD_ATTO3:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.18, 0.30, 0.45]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.009, 0.017, 0.027]
      elif candidate == CAR.BYD_DOLPHIN:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.22, 0.38, 0.55]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.012, 0.022, 0.032]
      elif candidate == CAR.DEEPAL_S05:
        ret.lateralTuning.pid.kpBP = [0., 9., 20.]
        ret.lateralTuning.pid.kpV = [0.16, 0.28, 0.42]
        ret.lateralTuning.pid.kiBP = [0., 9., 20.]
        ret.lateralTuning.pid.kiV = [0.008, 0.015, 0.024]
      else:
        if ret.flags & BrownPandaFlags.EV:
          ret.lateralTuning.pid.kpBP = [0., 9., 20.]
          ret.lateralTuning.pid.kpV = [0.2, 0.35, 0.5]
          ret.lateralTuning.pid.kiBP = [0., 9., 20.]
          ret.lateralTuning.pid.kiV = [0.01, 0.02, 0.03]
        else:
          ret.lateralTuning.pid.kpBP = [0., 9., 20.]
          ret.lateralTuning.pid.kpV = [0.15, 0.25, 0.4]
          ret.lateralTuning.pid.kiBP = [0., 9., 20.]
          ret.lateralTuning.pid.kiV = [0.008, 0.015, 0.025]

    # -----------------------------------------------------------------------------
    # [Section] ADAS Feature Configuration
    # [Brief ] Set explicit feature flags and capabilities. Keep minimal; tuning
    #          logic is centralized in values/CarControllerParams.
    # -----------------------------------------------------------------------------
    ret.autoResumeSng = bool(ret.flags & BrownPandaFlags.STOP_AND_GO)
    ret.enableBsm = bool(ret.flags & BrownPandaFlags.FOUR_WHEEL_SENSORS)
    ret.enableApgs = False
    ret.enableDsu = False
    ret.enableGasInterceptor = False
    ret.experimentalLongitudinalAvailable = ret.openpilotLongitudinalControl
    ret.pcmCruise = False

    # -----------------------------------------------------------------------------
    # [Section] Camera-based ADAS
    # [Brief ] Inform openpilot that the platform uses a stock camera system.
    # -----------------------------------------------------------------------------
    ret.hasStockCamera = True

    # -----------------------------------------------------------------------------
    # [Section] Safety Parameter
    # [Brief ] Base safety parameter (0..65535). BrownPanda uses base value.
    # -----------------------------------------------------------------------------
    ret.safetyParam = 0
    assert 0 <= ret.safetyParam <= 65535, f"Invalid safetyParam: {ret.safetyParam}"

    # -----------------------------------------------------------------------------
    # [Section] DragonPilot Integration
    # [Brief ] Integrate with DragonPilot's lateral tuning system
    # -----------------------------------------------------------------------------
    # Apply tire stiffness factor
    ret = scale_tire_stiffness(ret, tire_stiffness_factor)
    
    # DragonPilot lateral tune collection and configuration
    CarInterfaceBase.dp_lat_tune_collection(candidate, ret.latTuneCollection)
    CarInterfaceBase.configure_dp_tune(ret.lateralTuning, ret.latTuneCollection)

    return ret

  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    # Note: DragonPilot-specific initialization is already handled in CarInterfaceBase

  def _update(self, c):
    return self.CS.update(self.cp, self.cp_cam, self.cp_body)

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos, self.dragonconf)

  # Radarless compatibility helper removed; rely on standard CarParams fields.
