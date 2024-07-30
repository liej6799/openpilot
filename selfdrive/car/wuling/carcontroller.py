from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams
from common.numpy_fast import clip

def apply_wuling_steer_angle_limits(apply_angle, actual_angle, v_ego):
  # pick angle rate limits based on wind up/down
  ANGLE_RATE_LIMIT_UP = 3       # maximum allow 150 degree per second, 100Hz loop means 1.5
  ANGLE_RATE_LIMIT_DOWN = 3
  
  steer_up = actual_angle * apply_angle >= 0. and abs(apply_angle) > abs(actual_angle)
  rate_limits = ANGLE_RATE_LIMIT_UP if steer_up else ANGLE_RATE_LIMIT_DOWN

  return clip(apply_angle, actual_angle - rate_limits, actual_angle + rate_limits)

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.CP = CP
    
    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False
    self.frame = 0
    self.apply_angle_last = 0
    self.params = CarControllerParams(self.CP)
    
    self.packer_pt = CANPacker(DBC[self.CP.carFingerprint]['pt'])
    


  def update(self, CC, CS, now_nanos):
    P = self.params
    
    # Send CAN commands.
    can_sends = []
    actuators = CC.actuators
    
    apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)

    self.apply_angle_last = apply_angle
    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (self.frame % P.STEER_STEP) == 0:
      lkas_enabled = True
      # if !lkas_enabled:
      # else:
      #   apply_angle = CS.out.steeringAngleDeg
        
      idx = (CS.lka_steering_cmd_counter + 1) % 4
    
      can_sends.append(wulingcan.create_steering_control(self.packer_pt, apply_angle, idx, lkas_enabled))
    
    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle
    
    print('Steer :  %s' % apply_angle)
    self.frame += 1
    return new_actuators, can_sends