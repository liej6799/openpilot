#!/usr/bin/env python3
from cereal import car
from selfdrive.car.wuling.values import CAR, CruiseButtons,AccState
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from common.dp_common import common_interface_atl, common_interface_get_params_lqr

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "wuling"
    ret.radarOffCan = True
    ret.lateralTuning.init('pid')
    ret.pcmCruise = False 
    
    # ret.dashcamOnly = False
    # ret.dashcamOnly = candidate not in (CAR.CX5_2022, CAR.CX9_2021)
    ret.openpilotLongitudinalControl = True

    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.4
    ret.mass = 3000. + STD_CARGO_KG
    ret.wheelbase = 2.75
    ret.centerToFront = ret.wheelbase * 0.5
    ret.steerRatio = 13.5
    ret.steerActuatorDelay = 0.3   # end-to-end angle controller
    ret.lateralTuning.pid.kf = 0.00003
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20.], [0., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.0025, 0.1], [0.00025, 0.01]]

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    # dp
    ret = common_interface_get_params_lqr(ret)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings, dragonconf):
    self.cp.update_strings(can_strings)
    self.cp_loopback.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_loopback)
    # dp
    self.dragonconf = dragonconf
    ret.cruiseState.enabled = common_interface_atl(ret, dragonconf.dpAtl)
    ret.canValid = self.cp.can_valid and self.cp_loopback.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

   # events
    events = self.create_common_events(ret)

    # if self.CS.lkas_disabled:
    #   events.add(EventName.lkasDisabled)
    # elif self.CS.low_speed_alert:
    #   events.add(EventName.belowSteerSpeed)
    
    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if self.CS.pcm_acc_status == AccState.FAULTED:
      events.add(EventName.accFaulted)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    # handle button presses
    for b in ret.buttonEvents:
      # do enable on both accel and decel buttons
      if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
        events.add(EventName.buttonEnable)
      # do disable on button down
      if b.type == ButtonType.cancel and b.pressed:
        events.add(EventName.buttonCancel)


    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c, c.enabled, self.CS, self.frame, c.actuators,
                         c.cruiseControl.cancel, 
                         hud_control.visualAlert,
                         hud_control.setSpeed,
                         hud_control.leftLaneVisible, 
                         hud_control.rightLaneVisible, 
                         hud_control.leftLaneDepart, 
                         hud_control.rightLaneDepart,
                         self.dragonconf)
    self.frame += 1
    return ret
