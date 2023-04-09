#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.conversions import Conversions as CV

from selfdrive.car import STD_CARGO_KG,scale_tire_stiffness, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.wuling.values import CAR, CruiseButtons, PREGLOBAL_CARS, CarControllerParams, CanBus
from common.params import Params

TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
CRUISE_OVERRIDE_SPEED_MIN = 5 * CV.KPH_TO_MS

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.dp_cruise_speed = 0. # km/h
    self.dp_override_speed_last = 0. # km/h
    self.dp_override_speed = 0. # m/s

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "wuling"
    ret.radarUnavailable = True
    ret.dashcamOnly = candidate in PREGLOBAL_CARS
    ret.autoResumeSng = False
    ret.notCar = False

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.wuling)]

    ret.mass = 1950. + STD_CARGO_KG
    ret.wheelbase = 2.75
    ret.steerRatio = 17.7  # Stock 15.7, LiveParameters
    tire_stiffness_factor = 1  # Stock Michelin Energy Saver A/S, LiveParameters
    ret.centerToFront = ret.wheelbase * 0.5 
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 500], [0, 500]]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1

    ret.transmissionType = TransmissionType.automatic

    CarInterfaceBase.dp_lat_tune_collection(candidate, ret.latTuneCollection)
    CarInterfaceBase.configure_dp_tune(ret.lateralTuning, ret.latTuneCollection)
    
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 41.0], [0., 41.0]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.0002, 0.004], [0.1, 0.7]]
    ret.lateralTuning.pid.kf = 0.00006   # full torque for 20 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
    tire_stiffness_factor = 0.444  # not optimized yet
    
    ret.minEnableSpeed = -1
    
    params = Params()
    if int(params.get("dp_atl").decode('utf-8')) == 1:
      ret.openpilotLongitudinalControl = False
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def _update(self, c):

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)

    events = self.create_common_events(ret,extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic])

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed
    if below_min_enable_speed and not (ret.standstill and ret.brake >= 20):
      events.add(EventName.belowEngageSpeed)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(EventName.belowSteerSpeed)

    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
