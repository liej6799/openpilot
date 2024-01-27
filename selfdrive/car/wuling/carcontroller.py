from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS,CruiseButtons, CarControllerParams
import cereal.messaging as messaging
from common.params import Params, put_bool_nonblocking

VisualAlert = car.CarControl.HUDControl.VisualAlert
NetworkLocation = car.CarParams.NetworkLocation
LongCtrlState = car.CarControl.Actuators.LongControlState
BUTTONS_STATES = ["accelCruise", "decelCruise", "cancel", "resumeCruise"]

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
    self.param_s = Params()

    self.sm = messaging.SubMaster(['longitudinalPlan'])
    self.is_metric = self.param_s.get_bool("IsMetric")
    self.speed_limit_control_enabled = False
    self.last_speed_limit_sign_tap = False
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed

    # if not self.CP.pcmCruiseSpeed:
    #   self.sm.update(0)

    #   if self.sm.updated['longitudinalPlan']:
    #     self.v_tsc_state = self.sm['longitudinalPlan'].visionTurnControllerState
    #     self.slc_state = self.sm['longitudinalPlan'].speedLimitControlState
    #     self.m_tsc_state = self.sm['longitudinalPlan'].turnSpeedControlState
    #     self.speed_limit = self.sm['longitudinalPlan'].speedLimit
    #     self.speed_limit_offset = self.sm['longitudinalPlan'].speedLimitOffset
    #     self.v_tsc = self.sm['longitudinalPlan'].visionTurnSpeed
    #     self.m_tsc = self.sm['longitudinalPlan'].turnSpeed

    #   if self.frame % 200 == 0:
    #     self.speed_limit_control_enabled = self.param_s.get_bool("SpeedLimitControl")
    #     self.is_metric = self.param_s.get_bool("IsMetric")
    #   self.last_speed_limit_sign_tap = self.param_s.get_bool("LastSpeedLimitSignTap")
    #   self.v_cruise_min = MAZDA_V_CRUISE_MIN[self.is_metric] * (CV.KPH_TO_MPH if not self.is_metric else 1)

    # Send CAN commands.
    can_sends = []

    # if not self.CP.pcmCruiseSpeed:
    #       if not self.last_speed_limit_sign_tap_prev and self.last_speed_limit_sign_tap:
    #         self.sl_force_active_timer = self.frame
    #         put_bool_nonblocking("LastSpeedLimitSignTap", False)
    #       self.last_speed_limit_sign_tap_prev = self.last_speed_limit_sign_tap

    #       sl_force_active = self.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
    #       sl_inactive = not sl_force_active and (not self.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
    #       sl_temp_inactive = not sl_force_active and (self.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
    #       slc_active = not sl_inactive and not sl_temp_inactive

    #       self.slc_active_stock = slc_active

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

      # if CC.cruiseControl.resume and self.frame % 2 == 0:
      
      if (CS.resume_alert == 1 or CC.cruiseControl.resume) and self.frame % 2 == 0:
          print("Cruize button %s " % CC.cruiseControl.resume)
          print("Resule Alert %s " % CS.resume_alert)
        # Send Resume button when planner wants car to move
          can_sends.extend([wulingcan.create_buttons(self.packer_pt, CS.buttons_counter, CruiseButtons.RES_ACCEL)]*25)
          self.last_button_frame = self.frame

    # if CS.out.steeringPressed:
    #     can_sends.extend([wulingcan.create_buttons(self.packer_pt, CS.cruise_buttons,0, CruiseButtons.RES_ACCEL)]*25)
    #     can_sends.extend([wulingcan.create_buttons(self.packer_pt, CS.cruise_buttons,1, CruiseButtons.RES_ACCEL)]*25)
    #     can_sends.extend([wulingcan.create_buttons(self.packer_pt, CS.cruise_buttons,2, CruiseButtons.RES_ACCEL)]*25)
    #     print("Send Resume")
    # Steering (Active: 50Hz
    steer_step = self.params.STEER_STEP
    lat_active = CC.latActive
    
    self.lka_steering_cmd_counter += 1 if CS.loopback_lka_steering_cmd_updated else 0


    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we
    # received the ASCMLKASteeringCmd loopback confirmation too recently
    last_lka_steer_msg_ms = (now_nanos - CS.loopback_lka_steering_cmd_ts_nanos) * 1e-6
    if  (self.frame  % self.params.STEER_STEP) == 0:
      if CC.latActive:
        new_steer = int(round(actuators.steer * self.params.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
      else:
        apply_steer = 0
      
      self.apply_steer_last = apply_steer
      self.last_steer_frame = self.frame
      
      can_sends.append(wulingcan.create_steering_control(self.packer_pt, apply_steer, self.frame, CC.latActive))
    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)

    # send Acc dashboard message
    if self.frame % 5 == 0:
      #  print(CS.acc_cmd)
       lead_distance = 0
       set_speed = int(round(hud_v_cruise * CV.MS_TO_KPH))
       can_sends.append(wulingcan.create_acc_hud_control(self.packer_pt, 0, CS.ascm_cc_status,CC.enabled and CS.out.cruiseState.enabled, set_speed, 0, CS.gac_tr_cluster))

    # send HUD alerts
    if self.frame % 5 == 0:
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      can_sends.extend(wulingcan.create_lkas_hud(self.packer_pt, 0, CS.lkas_hud, CC.latActive, steer_required))

    """ACC RADAR COMMAND"""
    if self.frame % 2 == 0:
      lead_distance = 0
      if self.CP.openpilotLongitudinalControl:
        stopping = actuators.longControlState == LongCtrlState.stopping
        if not CC.longActive:
          # ASCM sends max regen when not enabled
          self.apply_gas = 0
          self.apply_brake = 0
        else:
          self.apply_gas = int(round(interp(actuators.accel, self.params.ACCEL_LOOKUP_BP, self.params.ACCEL_LOOKUP_V)))

        idx = (self.frame // 4) % 4

        at_full_stop = CC.longActive and CS.out.standstill
        near_stop = CC.longActive and (CS.out.vEgo < self.params.NEAR_STOP_BRAKE_PHASE)
      print(self.apply_gas)
      can_sends.extend(wulingcan.create_radar_command(self.packer_pt, CS.acc_cmd, 0, self.frame, CC, CS))
      # if CS.out.steeringPressed:
      #   can_sends.extend(wulingcan.create_radar_command(self.packer_pt, CS.acc_cmd, 1, self.frame, CC, CS))
      # else:
      #   can_sends.extend(wulingcan.create_radar_command(self.packer_pt, CS.acc_cmd, 0, self.frame, CC, CS))
      
    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake

    self.frame += 1
    return new_actuators, can_sends

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)
