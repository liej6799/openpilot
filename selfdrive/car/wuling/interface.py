#!/usr/bin/env python3
from cereal import car
from selfdrive.car.wuling.values import CAR, CruiseButtons,AccState
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from common.dp_common import common_interface_atl, common_interface_get_params_lqr
from selfdrive.config import Conversions as CV
from common.op_params import opParams

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    op_params = opParams("wuling car_interface.py for lateral override")

    ret.carName = "wuling"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.wuling)]

    ret.radarOffCan = True
    ret.lateralTuning.init('pid')
    ret.pcmCruise = True
    
    tire_stiffness_factor = 0.444

    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [530]]

    # ret.dashcamOnly = False
    # ret.dashcamOnly = candidate not in (CAR.CX5_2022, CAR.CX9_2021)
    ret.openpilotLongitudinalControl = False

    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.4
    ret.mass = 1900. + STD_CARGO_KG
    ret.wheelbase = 2.75
    ret.centerToFront = ret.wheelbase * 0.4
    # ret.steerRatio = 16.3
    ret.steerRatio = op_params.get('steer_ratio', force_update=True)
    
    ret.steerActuatorDelay = 0.2 # end-to-end angle controller
    
    # ret.lateralTuning.pid.kf = 0
    # ret.lateralTuning.pid.kf = 0.00003
    # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0, 0], [0, 0]]
    # ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 1.], [0., 1.]]
    
    # v1
    # ret.lateralTuning.pid.kpBP = [0., 30.]
    # ret.lateralTuning.pid.kpV = [0.0025, 0.08]
    # ret.lateralTuning.pid.kiBP = [0., 30.]
    # ret.lateralTuning.pid.kiV = [0., 0.60]
    
    # ret.lateralTuning.pid.kpBP = [0., 30.]
    # ret.lateralTuning.pid.kpV = [0.0025, 0.08]
    # ret.lateralTuning.pid.kiBP = [0., 30.]
    # ret.lateralTuning.pid.kiV = [0., 0.60]
    
    #v2
    # ret.lateralTuning.pid.kpBP = [0.,40.]
    # ret.lateralTuning.pid.kiBP = [0., 40.]
    # ret.lateralTuning.pid.kpV = [0.0005, 0.07]
    # ret.lateralTuning.pid.kiV = [0.25, 0.05]
    
    # ret.lateralTuning.pid.kpBP = [0.,40.]
    # ret.lateralTuning.pid.kiBP = [0., 40.]
    # ret.lateralTuning.pid.kpV = [0.006, 0.025]
    # ret.lateralTuning.pid.kiV = [0.10, 0.60]

    bp = [i * CV.MPH_TO_MS for i in op_params.get("TUNE_LAT_PID_bp_mph", force_update=True)]
    kpV = [i for i in op_params.get("TUNE_LAT_PID_kp", force_update=True)]
    kiV = [i for i in op_params.get("TUNE_LAT_PID_ki", force_update=True)]
    ret.lateralTuning.pid.kpV = kpV
    ret.lateralTuning.pid.kiV = kiV
    ret.lateralTuning.pid.kpBP = bp
    ret.lateralTuning.pid.kiBP = bp
    ret.lateralTuning.pid.kf = op_params.get('TUNE_LAT_PID_kf', force_update=True)

    # ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.,0.], [0.,0.]]
    # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.0,0.0], [0.0,0.0]]
    # ret.lateralTuning.pid.kiBP = [0., 0]
    # ret.lateralTuning.pid.kiV = [0., 0]
    
    # ret.longitudinalTuning.kpBP = [5., 35.]
    # ret.longitudinalTuning.kiBP = [0.]
    # ret.longitudinalTuning.kpV = [2.4, 1.5]
    # ret.longitudinalTuning.kiV = [0.36]


    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

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
    # print("dp atl : %s" % dragonconf.dpAtl)
    # print('Cruise enable :  %s' % ret.cruiseState.enabled)

    #dp
    ret.engineRPM = self.CS.engineRPM
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
    # print("Events : %s" % events.to_msg())

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
