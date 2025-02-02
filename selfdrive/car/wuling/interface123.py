#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.conversions import Conversions as CV

from selfdrive.car import STD_CARGO_KG,scale_tire_stiffness,create_button_event, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.wuling.values import CAR, CruiseButtons, PREGLOBAL_CARS, CarControllerParams, CanBus
from common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}

CRUISE_OVERRIDE_SPEED_MIN = 5 * CV.KPH_TO_MS

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.dp_cruise_speed = 0. # km/h
    self.dp_override_speed_last = 0. # km/h
    self.dp_override_speed = 0. # m/s

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "wuling"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.wuling)]
    ret.autoResumeSng = False
    
    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    ret.steerRatio = 17.7
    
    ret.steerControlType = car.CarParams.SteerControlType.angle
    
    ret.radarUnavailable = True
    
    ret.mass = 1950. + STD_CARGO_KG
    ret.wheelbase = 2.75
    ret.centerToFront = ret.wheelbase * 0.4

    CarInterfaceBase.dp_lat_tune_collection(candidate, ret.latTuneCollection)
    CarInterfaceBase.configure_dp_tune(ret.lateralTuning, ret.latTuneCollection)

    return ret

  # returns a car.CarState
  def _update(self, c):

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)
    ret.engineRPM = self.CS.engineRPM
    
    # print('enter first check prev_cruise_buttons', self.CS.prev_cruise_buttons)
    # print('enter first check cruise_buttons', self.CS.cruise_buttons)
    
    # if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
    #   print('enter first check')
    #   buttonEvents = [create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS)]
    # #   # Handle ACCButtons changing buttons mid-press
    # #   if self.CS.cruise_buttons != CruiseButtons.UNPRESS and self.CS.prev_cruise_buttons != CruiseButtons.UNPRESS:
    # #     buttonEvents.append(create_button_event(CruiseButtons.UNPRESS, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS))
    # be = car.CarState.ButtonEvent(pressed=True)
    # be.type = buttons_dict.get(but, ButtonType.unknown)
    
    # buttonEvents = car.CarState.ButtonEvent(pressed=True)
    # ret.buttonEvents = buttonEvents
    # print(ret.buttonEvents)

    
    if self.CS.cruise_buttons != CruiseButtons.UNPRESS or self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      buttonEvents = [create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS)]
     
      ret.buttonEvents = buttonEvents

    # buttonEvents = []
    # be = car.CarState.ButtonEvent.new_message()
    # be.type = car.CarState.ButtonEvent.Type.accelCruise
    # buttonEvents.append(be)
    
    # ret.buttonEvents = buttonEvents

    # print("Print cruise_button: ", self.CS.cruise_buttons)
    # print("Print prev_cruise_button: ", self.CS.prev_cruise_buttons)

    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.low, GearShifter.eco, GearShifter.manumatic], pcm_enable=self.CP.pcmCruise, enable_buttons=(ButtonType.decelCruise,))

    # if not self.CP.pcmCruise:
    #   events.add(EventName.buttonEnable)

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    # print('last event', events)
    
    # below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed
    # if below_min_enable_speed and not (ret.standstill and ret.brake >= 20):
    #   events.add(EventName.belowEngageSpeed)
    # if self.CS.park_brake:
    #   events.add(EventName.parkBrake)
    # if ret.cruiseState.standstill:
    #   events.add(EventName.resumeRequired)
    # if ret.vEgo < self.CP.minSteerSpeed:
    #   events.add(EventName.belowSteerSpeed)
  
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)