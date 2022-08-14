from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams
from selfdrive.car import apply_std_steer_torque_limits
from common.dp_common import common_controller_ctrl

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

    can_sends = []
    steer_alert = visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw)

    apply_steer = actuators.steer
    
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    
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

    if (frame % 4) == 0:
      can_sends.append(wulingcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, enabled, hud_speed * CV.MS_TO_KPH, 0, 0))

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.p.STEER_MAX

    return new_actuators, can_sends
