from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams
from selfdrive.car import apply_std_steer_torque_limits
from common.dp_common import common_controller_ctrl
from selfdrive.car import make_can_msg

from opendbc.can.packer import CANPacker
from common.dp_common import common_controller_ctrl
from selfdrive.config import Conversions as CV
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.

    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_lkas_cnt = -1
    self.cruise_button_prev = 0
    self.steer_rate_limited = False
    self.steer_alert_last = False
    self.lkas_action = 0
    
    self.apply_gas = 0
    self.apply_brake = 0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)

    print(CP.carFingerprint)
    print(DBC[CP.carFingerprint])
    
    self.p = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed, left_line, right_line, left_lane_depart, right_lane_depart, dragonconf):

    P = self.p

    can_sends = []
    steer_alert = visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw)

    apply_steer = actuators.steer
    
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % P.STEER_STEP) == 0:
      lkas_enabled = c.active and not (CS.out.steerWarning or CS.out.steerError) and CS.out.vEgo > P.MIN_STEER_SPEED
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      # dp
      blinker_on = CS.out.leftBlinker or CS.out.rightBlinker
      if not enabled:
        self.blinker_end_frame = 0
      if self.last_blinker_on and not blinker_on:
        self.blinker_end_frame = frame + dragonconf.dpSignalOffDelay
      apply_steer = common_controller_ctrl(enabled,
                                           dragonconf,
                                           blinker_on or frame < self.blinker_end_frame,
                                           apply_steer, CS.out.vEgo)
      self.last_blinker_on = blinker_on

      self.apply_steer_last = apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(wulingcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

      
    if (frame % 4) == 0:
      print('UI Command HUD Speed :  %s' % hud_speed)
      # can_sends.append(make_can_msg(0x373, b"\x82\x01\x00\x00\xac\x90\x02\xc1", 0))

      # can_sends.append(wulingcan.create_acc_dashboard_command(self.packer, CanBus.POWERTRAIN, enabled, hud_speed * CV.MS_TO_KPH, 0, 0))

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.p.STEER_MAX
    
    print('Last enable :  %s' % self.enabled_last)
    
    if (enabled):
        print('enable adas')

    # if enabled and  (frame % 100) == 0:
      #  can_sends.append(make_can_msg(0x373, b"\xc6\x3d\x01\x00\xac\x90\x02\x42", 0))
      
    self.enabled_last = enabled
    self.main_on_last = CS.out.cruiseState.available
    self.steer_alert_last = steer_alert

    print('Steer :  %s' % apply_steer)

    return new_actuators, can_sends
