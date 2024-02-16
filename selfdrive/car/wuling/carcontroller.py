from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert
NetworkLocation = car.CarParams.NetworkLocation
LongCtrlState = car.CarControl.Actuators.LongControlState

# Camera cancels up to 0.1s after brake is pressed, ECM allows 0.5s
CAMERA_CANCEL_DELAY_FRAMES = 10
# Enforce a minimum interval between steering messages to avoid a fault
MIN_STEER_MSG_INTERVAL_MS = 15


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.start_time = 0.
    self.apply_steer_last = 0
    self.apply_gas = 0
    self.apply_brake = 0
    self.frame = 0
    self.last_steer_frame = 0
    self.last_button_frame = 0
    self.brake_counter = 0

    self.cancel_counter = 0

    self.lka_steering_cmd_counter = 0
    self.lka_steering_cmd_counter_last = -1

    self.lka_icon_status_last = (False, False)

    self.params = CarControllerParams(self.CP)
    self.packer_pt = CANPacker(DBC[self.CP.carFingerprint]['pt'])

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed
    
    # Send CAN commands.
    can_sends = []

    if CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      # if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
      #   # Cancel Stock ACC if it's enabled while OP is disengaged
      #   # Send at a rate of 10hz until we sync with stock ACC state
      #   can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
#      print("Cruize button %s " % CC.cruiseControl.resume)
      if self.frame % 5 == 0 and CS.resume_alert == 1:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(wulingcan.create_resume_cmd(self.packer_pt, CS.crz_btns_counter, 1))
        print("Send Resume")

    # if CS.steeringPressed:
    #     can_sends.append(wulingcan.create_resume_button())
    #     print("Send Resume")
    # Steering (Active: 50Hz
    steer_step = self.params.STEER_STEP

    self.lka_steering_cmd_counter += 1 if CS.loopback_lka_steering_cmd_updated else 0

    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we
    # received the ASCMLKASteeringCmd loopback confirmation too recently
    last_lka_steer_msg_ms = (now_nanos - CS.loopback_lka_steering_cmd_ts_nanos) * 1e-6
    # if (self.frame - self.last_steer_frame) >= steer_step and last_lka_steer_msg_ms > MIN_STEER_MSG_INTERVAL_MS:
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif  (self.frame  % self.params.STEER_STEP) == 0:
      # Initialize ASCMLKASteeringCmd counter using the camera until we get a msg on the bus
      if CS.loopback_lka_steering_cmd_ts_nanos == 0:
        self.lka_steering_cmd_counter = CS.pt_lka_steering_cmd_counter + 1
   
      CC.latActive = True
      if CC.latActive:
        new_steer = int(round(actuators.steer * self.params.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(-new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
      else:
        apply_steer = 0
      
      print('car controller CS.out.steeringTorque', CS.out.steeringTorque)
      print('car controller CC.latActive', CC.latActive)
      self.last_steer_frame = self.frame
      self.apply_steer_last = apply_steer
      # idx = self.lka_steering_cmd_counter % 4
      can_sends.append(wulingcan.create_steering_control(self.packer_pt, apply_steer, self.frame))

    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)

    # # SW_GMLAN not yet on cam harness, no HUD alerts
    # if self.CP.networkLocation != NetworkLocation.fwdCamera and (self.frame % self.params.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last):
    #   steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
    #   can_sends.append(wulingcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
    #   self.lka_icon_status_last = lka_icon_status

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake

    self.frame += 1
    return new_actuators, can_sends
