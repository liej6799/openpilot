import copy
from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.car.wuling.values import DBC, CanBus,HUD_MULTIPLIER, STEER_THRESHOLD, CAR, PREGLOBAL_CARS 
from time import time

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
   
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["TRANSMISSION_STATE"]

    self.lka_steering_cmd_counter = 0
    self.engineRPM = 0
    
    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.low_speed_alert = False
    self.lkas_allowed_speed = False
    self.lkas_disabled = False
    self.cruise_speed = 30 * CV.KPH_TO_MS
    self.cruise_speed_counter = 0
    self.is_cruise_latch = False
    self.rising_edge_since = 0
    self.last_frame = time() # todo: existing infra to reuse?
    self.dt = 0



  def update(self, pt_cp, cp_cam):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons

    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]) * HUD_MULTIPLIER
    # ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"] * -1
    # ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]    
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBelt"]["RIGHTSEATBEALT"] == 0
    
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2
    
    # ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1,
    #                                                                   pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2)

    
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)
    
    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    ret.brakeHoldActive = pt_cp.vl["EPBStatus"]["AVH_STATUS"] != 0
    # ret.brake = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]["TRANSMISSION_STATE"], None))

    # print('Gear Shifter :  %s' % ret.gearShifter)
    ret.gas = pt_cp.vl["GAS_PEDAL"]["GAS_POS"]
    ret.gasPressed = ret.gas > 0
    
    ret.vEgoCluster = ret.vEgoRaw

    ret.cruiseState.available = pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 or pt_cp.vl["AccStatus"]["CruiseState"] != 0
    self.is_cruise_latch = pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 or pt_cp.vl["AccStatus"]["CruiseState"] != 0

    if pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 and ret.brakePressed:
      self.is_cruise_latch = False
    else:
      pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 and not ret.brakePressed
      self.is_cruise_latch = True

    self.park_brake = pt_cp.vl["EPBStatus"]["EPBSTATUS"]
    self.pcm_acc_status = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"]
    # dp - brake lights
    ret.brakeLights = ret.brakePressed
    
    if not ret.cruiseState.available:
      self.is_cruise_latch = False
      
    # ret.cruiseState.enabled = pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 or pt_cp.vl["AccStatus"]["CruiseState"] != 0
    ret.cruiseState.enabled = self.is_cruise_latch
    # ret.cruiseState.enabled = True
    
    ret.cruiseActualEnabled = ret.cruiseState.enabled
    # ret.cruiseState.available =  pt_cp.vl["AccStatus"]["CruiseState"] != 0
    # ret.cruiseState.available = True

    ret.cruiseState.speed = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
    ret.steeringTorque = pt_cp.vl["PSCMSteeringAngle"]["SteeringTorque"]
    ret.steeringTorqueEps = pt_cp.vl["STEER_RELATED"]["STEER_TORQUE"]

    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.genericToggle = bool(pt_cp.vl["BCMTurnSignals"]["HighBeamsActive"])

    # print("Steering torque : %d" % ret.steeringTorque)

    if ret.steeringPressed:
      print("Steering pressed")

    # print('Cruise speed :  %s' % ret.cruiseState.speed)
    # print('Cruise state enable :  %s' % ret.cruiseState.enabled)
    # print('Cruise state available :  %s' % ret.cruiseState.available)

    #trans state 15 "PARKING" 1 "DRIVE" 14 "BACKWARD" 13 "NORMAL"
    
    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      
      ("TurnSignals", "BCMTurnSignals"),
      ("HighBeamsActive", "BCMTurnSignals"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("SteeringTorque", "PSCMSteeringAngle"),
      
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
      ("RIGHTSEATBEALT", "BCMDoorBelt"),
      ("LEFTSEATBEALT", "BCMDoorBelt"),
      ("EPBClosed", "EPBStatus"),
      ("Brake_Pressed", "ECMEngineStatus"),
      ("EPBSTATUS", "EPBStatus"),
      ("AVH_STATUS", "EPBStatus"),
      ("ACCBUTTON", "ASCMActiveCruiseControlStatus"),
      ("ACCSTATE", "ASCMActiveCruiseControlStatus"),
      ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
      ("TRANSMISSION_STATE", "ECMPRDNL"),
      ("LKAS_STATE", "LkasHud"),
      ("LKA_ACTIVE", "LkasHud"),
      ("CruiseMainOn", "AccStatus"),
      ("CruiseState", "AccStatus"),
      ("STEER_TORQUE", "STEER_RELATED"),
      ("GAS_POS", "GAS_PEDAL"),
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("ECMEngineStatus", 10),
      ("EPBStatus", 10),
      ("ECMPRDNL", 10),
      ("BCMDoorBeltStatus", 10),
      ("BCMDoorBelt", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("ASCMActiveCruiseControlStatus", 20),
      ("PSCMSteeringAngle", 100),
      ("LkasHud", 20),
      ("AccStatus", 20),
      ("GAS_PEDAL", 10),
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
