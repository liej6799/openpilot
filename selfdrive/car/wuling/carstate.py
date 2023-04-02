import copy
from cereal import car
from collections import deque

from common.conversions import Conversions as CV
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.wuling.values import DBC, AccState, CanBus,HUD_MULTIPLIER, STEER_THRESHOLD, CAR, PREGLOBAL_CARS 

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS

TORQUE_SAMPLES = 12

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["TRANSMISSION_STATE"]
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    self.steeringTorqueSamples = deque(TORQUE_SAMPLES*[0], TORQUE_SAMPLES)

    self.loopback_lka_steering_cmd_updated = False
    self.loopback_lka_steering_cmd_ts_nanos = 0
    self.pt_lka_steering_cmd_counter = 0
    self.cam_lka_steering_cmd_counter = 0
    self.buttons_counter = 0
    self.engineRPM = 0
    self.cruise_speed = 30 * CV.KPH_TO_MS

  def update(self, pt_cp, cam_cp, loopback_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM'] * 0.25

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
    )
    
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]) * HUD_MULTIPLIER
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # sample rear wheel speeds, standstill=True if ECM allows engagement with brake
    ret.standstill = ret.wheelSpeeds.rl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.seatbeltUnlatched = False
    ret.doorOpen = False
   
    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0

    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2
    
    print('Gear Shifter :  %s' % ret.gearShifter)

    self.park_brake = pt_cp.vl["EPBStatus"]["EPBSTATUS"]
    self.pcm_acc_status = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"]
    # dp - brake lights
    ret.brakeLights = ret.brakePressed
    
    # ret.cruiseState.enabled = pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 or pt_cp.vl["AccStatus"]["CruiseState"] != 0
    ret.cruiseState.enabled = pt_cp.vl["AccStatus"]["CruiseState"] != 0
    # ret.cruiseState.enabled = True
    
    ret.cruiseActualEnabled = ret.cruiseState.enabled
    ret.cruiseState.available = pt_cp.vl["AccStatus"]["CruiseMainOn"] != 0 or pt_cp.vl["AccStatus"]["CruiseState"] != 0
    # ret.cruiseState.available =  pt_cp.vl["AccStatus"]["CruiseState"] != 0
    # ret.cruiseState.available = True

    ret.cruiseState.speed = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
    ret.steeringTorque = pt_cp.vl["PSCMSteeringAngle"]["SteeringTorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.cruiseState.standstill = ret.cruiseState.enabled == 0 and ret.cruiseState.enabled != 0

    self.steeringTorqueSamples.append(ret.steeringTorque)

    print('Cruise speed :  %s' % ret.cruiseState.speed)
    print('Cruise state enable :  %s' % ret.cruiseState.enabled)
    print('Cruise state available :  %s' % ret.cruiseState.available)

    #trans state 15 "PARKING" 1 "DRIVE" 14 "BACKWARD" 13 "NORMAL"
    return ret

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("AEBCmdActive", "AEBCmd"),
        ("RollingCounter", "ASCMLKASteeringCmd"),
        ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
        ("ACCCruiseState", "ASCMActiveCruiseControlStatus"),
      ]
      checks += [
        ("AEBCmd", 10),
        ("ASCMLKASteeringCmd", 10),
        ("ASCMActiveCruiseControlStatus", 25),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.CAMERA)

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
     ("TurnSignals", "BCMTurnSignals"),
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
      ("EPBClosed", "EPBStatus"),
      ("Brake_Pressed", "ECMEngineStatus"),
      ("EPBSTATUS", "EPBStatus"),
      ("ACCBUTTON", "ASCMActiveCruiseControlStatus"),
      ("ACCSTATE", "ASCMActiveCruiseControlStatus"),
      ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
      ("TRANSMISSION_STATE", "ECMPRDNL"),
      ("LKAS_STATE", "LkasHud"),
      ("LKA_ACTIVE", "LkasHud"),
      ("CruiseMainOn", "AccStatus"),
      ("CruiseState", "AccStatus"),
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("ECMEngineStatus", 10),
      ("EPBStatus", 10),
      ("ECMPRDNL", 10),
      ("BCMDoorBeltStatus", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("ASCMActiveCruiseControlStatus", 20),
      ("PSCMSteeringAngle", 100),
      ("LkasHud", 20),
      ("AccStatus", 20),
    ]

    # Used to read back last counter sent to PT by camera
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("RollingCounter", "ASCMLKASteeringCmd"),
      ]
      checks += [
        ("ASCMLKASteeringCmd", 0),
      ]

    if CP.transmissionType == TransmissionType.direct:
      signals.append(("RegenPaddle", "EBCMRegenPaddle"))
      checks.append(("EBCMRegenPaddle", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "ASCMLKASteeringCmd"),
    ]

    checks = [
      ("PSCMSteeringAngle", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK, enforce_checks=False)
