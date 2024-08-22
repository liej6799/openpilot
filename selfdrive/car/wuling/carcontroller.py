from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams
from common.numpy_fast import clip

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    
    self.lka_steering_cmd_counter_last = -1
    self.apply_angle_last = 0
    
    self.frame = 0

    self.params = CarControllerParams(self.CP)
    
    self.packer_pt = CANPacker(DBC[self.CP.carFingerprint]['pt'])
    
  def update(self, CC, CS, now_nanos):
    P = self.params
    
    # Send CAN commands.
    can_sends = []
    actuators = CC.actuators
    
    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
    else:
      apply_angle = CS.out.steeringAngleDeg
      
    self.apply_angle_last = apply_angle
    
 
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (self.frame % P.STEER_STEP) == 0:
      lkas_enabled = CC.latActive

      idx = (self.frame/2) % 4
      
      can_sends.append(wulingcan.create_steering_control(self.packer_pt, apply_angle, idx, lkas_enabled))
      can_sends.append(wulingcan.create_brake_command(self.packer_pt, True, idx, 100))
   
      
    
    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle
    
    self.frame += 1
    return new_actuators, can_sends