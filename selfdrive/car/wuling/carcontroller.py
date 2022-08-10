from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, PREGLOBAL_CARS, CarControllerParams
from opendbc.can.packer import CANPacker
from common.dp_common import common_controller_ctrl

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

    print(CP.carFingerprint)
    print(DBC[CP.carFingerprint])
    
    self.p = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart, dragonconf):

    can_sends = []


    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.p.STEER_MAX

    return new_actuators, can_sends
