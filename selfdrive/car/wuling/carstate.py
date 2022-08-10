import copy
from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.car.wuling.values import DBC, CanBus, STEER_THRESHOLD, CAR, PREGLOBAL_CARS 

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
   
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.lka_steering_cmd_counter = 0
    self.engineRPM = 0



  def update(self, pt_cp, cp_cam):
    ret = car.CarState.new_message()
    self.prev_cruise_buttons = self.cruise_buttons
    
    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']*0.25

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2
    
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)
    
    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      
      ("TurnSignals", "BCMTurnSignals"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("EngineRPM", "ECMEngineStatus"),
      
      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("PSCMSteeringAngle", 100),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "PSCMSteeringAngle"),
    ]

    checks = [
      ("PSCMSteeringAngle", 10),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK)
