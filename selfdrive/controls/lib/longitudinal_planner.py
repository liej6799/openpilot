#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params, put_bool_nonblocking
from cereal import car, log

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import T_IDXS
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.system.swaglog import cloudlog

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Time threshold for Conditional Experimental Mode (Code runs at 20hz, so: THRESHOLD / 20 = seconds)
THRESHOLD = 5 # 0.25s

# Acceleration profiles - Credit goes to the DragonPilot team!
                 # MPH = [0.,  35,   35,  40,    40,  45,    45,  67,    67,   67, 123]
A_CRUISE_MIN_BP_CUSTOM = [0., 2.0, 2.01, 11., 11.01, 18., 18.01, 28., 28.01,  33., 55.]
                 # MPH = [0., 6.71, 13.4, 17.9, 24.6, 33.6, 44.7, 55.9, 67.1, 123]
A_CRUISE_MAX_BP_CUSTOM = [0.,    3,   6.,   8.,  11.,  15.,  20.,  25.,  30., 55.]

A_CRUISE_MIN_VALS_ECO_TUNE = [-0.730, -0.730, -0.68, -0.68, -0.50, -0.36, -0.32, -0.28, -0.28, -0.25, -0.25]
A_CRUISE_MAX_VALS_ECO_TUNE = [3.5, 3.0, 1.6, 1.2, .76, .62, .48, .36, .28, .09]

A_CRUISE_MIN_VALS_SPORT_TUNE = [-0.750, -0.750, -0.75, -0.75, -0.54, -0.42, -0.40, -0.35, -0.35, -0.30, -0.30]
A_CRUISE_MAX_VALS_SPORT_TUNE = [3.5, 3.5, 3.5, 3.5, 2.8, 2.0, 1.5, 1.0, .7, .5]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# Lookup table for approaching slower leads
LEAD_DISTANCE = [10., 100.]
LEAD_SPEED_DIFF = [-1., -10.]

# Lookup table for stop sign / stop light detection
STOP_SIGN_BP = [0., 10., 20., 30., 40., 50., 55.]
STOP_SIGN_DISTANCE = [10, 30., 50., 70., 80., 90., 120.]

# VTSC variables
TARGET_LAT_A = 1.9  # m/s^2
MIN_TARGET_V = 5    # m/s

def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_min_accel_eco_tune(v_ego):
  return interp(v_ego, A_CRUISE_MIN_BP_CUSTOM, A_CRUISE_MIN_VALS_ECO_TUNE)

def get_max_accel_eco_tune(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_ECO_TUNE)

def get_min_accel_sport_tune(v_ego):
  return interp(v_ego, A_CRUISE_MIN_BP_CUSTOM, A_CRUISE_MIN_VALS_SPORT_TUNE)

def get_max_accel_sport_tune(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_SPORT_TUNE)

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)
    self.v_model_error = 0.0

    self.x_desired_trajectory = np.zeros(CONTROL_N)
    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.param_read_counter = 0
    self.read_param()
    self.personality = log.LongitudinalPersonality.standard
    self.is_metric = self.params.get_bool("IsMetric")

    # FrogPilot variables
    self.longitudinal_tuning = self.params.get_bool("LongitudinalTuning")
    self.acceleration_profile = self.params.get_int("AccelerationProfile") if self.longitudinal_tuning else 2
    self.aggressive_acceleration = self.params.get_bool("AggressiveAcceleration") and self.longitudinal_tuning
    self.increased_stopping_distance = self.params.get_int("IncreasedStoppingDistance") * (1 if self.is_metric else 0.3048) if self.longitudinal_tuning else 0
    self.smoother_braking = self.params.get_bool("SmootherBraking") and self.longitudinal_tuning

    self.custom_personalities = self.params.get_bool("CustomPersonalities")
    self.aggressive_follow = self.params.get_int("AggressivePersonality") / 10
    self.standard_follow = self.params.get_int("StandardPersonality") / 10
    self.relaxed_follow = self.params.get_int("RelaxedPersonality") / 10
    self.aggressive_jerk = self.params.get_int("AggressiveJerk") / 10
    self.standard_jerk = self.params.get_int("StandardJerk") / 10
    self.relaxed_jerk = self.params.get_int("RelaxedJerk") / 10

    self.green_light_alert = self.params.get_bool("GreenLightAlert")

    self.vision_turn_controller = self.params.get_bool("VisionTurnControl")
    self.curve_sensitivity = self.params.get_int("CurveSensitivity") / 100
    self.turn_aggressiveness = self.params.get_int("TurnAggressiveness") / 100

    self.green_light = False
    self.previously_driving = False
    self.stopped_for_light_previously = False

    self.v_offset = 0
    self.v_target = MIN_TARGET_V

    self.update_frogpilot_params()
    # Set variables for Conditional Experimental Mode
    self.conditional_experimental_mode = self.params.get_bool("ConditionalExperimental")
    if self.conditional_experimental_mode:
      put_bool_nonblocking("ExperimentalMode", True)
    self.curves = self.params.get_bool("ConditionalCurves")
    self.curves_lead = self.params.get_bool("ConditionalCurvesLead")
    self.experimental_mode_via_wheel = self.params.get_bool("ExperimentalModeViaWheel")
    self.limit = self.params.get_int("ConditionalSpeed") * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
    self.limit_lead = self.params.get_int("ConditionalSpeedLead") * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
    self.signal = self.params.get_bool("ConditionalSignal")
    self.slower_lead = self.params.get_bool("ConditionalSlowerLead")
    self.stop_lights = self.params.get_bool("ConditionalStopLights")
    self.curve_detected = False
    self.experimental_mode = False
    self.curvature_count = 0
    self.lead_status_count = 0
    self.previous_lead_speed = 0
    self.previous_status_bar = 0
    self.previous_yRel = 0
    self.status_value = 0
    self.stop_light_count = 0

  def read_param(self):
    try:
      self.personality = int(self.params.get('LongitudinalPersonality'))
    except (ValueError, TypeError):
      self.personality = log.LongitudinalPersonality.standard

  @staticmethod
  def parse_model(model_msg, model_error):
    if (len(model_msg.position.x) == 33 and
       len(model_msg.velocity.x) == 33 and
       len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    return x, v, a, j

  def update(self, sm, frogpilot_toggles_updated):
    # Update FrogPilot variables when they are changed
    if frogpilot_toggles_updated:
      self.update_frogpilot_params()

    if self.param_read_counter % 50 == 0:
      self.read_param()
    self.param_read_counter += 1
    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    v_lead = sm['radarState'].leadOne.vLead
    v_cruise_kph = min(sm['controlsState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      if self.acceleration_profile == 1:
        accel_limits = [get_min_accel_eco_tune(v_ego), get_max_accel_eco_tune(v_ego)]
      elif self.acceleration_profile == 2:
        accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      elif self.acceleration_profile == 3:
        accel_limits = [get_min_accel_sport_tune(v_ego), get_max_accel_sport_tune(v_ego)]
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    # Pfeiferj's Vision Turn Controller
    if self.vision_turn_controller and prev_accel_constraint and v_ego >= 1:
      # Adjust the rate plan with curve sensitivity
      orientation_rate = np.array(np.abs(sm['modelV2'].orientationRate.z)) * self.curve_sensitivity
      velocity = np.array(sm['modelV2'].velocity.x)

      # Get the maximum lat accel from the model
      self.max_pred_lat_acc = np.amax(orientation_rate * velocity)

      # Get the maximum curve based on the current velocity
      max_curve = self.max_pred_lat_acc / (v_ego**2)

      # Adjust the target lateral acceleration with turn aggressiveness
      adjusted_target_lat_a = TARGET_LAT_A * self.turn_aggressiveness

      # Get the target velocity for the maximum curve
      self.v_target = (adjusted_target_lat_a / max_curve) ** 0.5
      self.v_target = 0 if np.isnan(self.v_target) else self.v_target
      self.v_target = max(self.v_target, MIN_TARGET_V)

      # Configure the offset value for the UI
      self.v_offset = max(0, int(v_cruise - self.v_target))

      # Set v_cruise to the desired speed
      v_cruise = min(v_cruise, self.v_target)
    else:
      self.v_offset = 0

    self.mpc.set_weights(prev_accel_constraint, self.custom_personalities, self.aggressive_jerk, self.standard_jerk, self.relaxed_jerk, personality=self.personality)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error)
    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, self.aggressive_acceleration, self.increased_stopping_distance, self.smoother_braking,
                    self.custom_personalities, self.aggressive_follow, self.standard_follow, self.relaxed_follow, personality=self.personality)

    self.x_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.x_solution)
    self.v_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.x_desired_trajectory = self.x_desired_trajectory_full[:CONTROL_N]
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

    # Set the current driving states for FrogPilot functions
    carstate, modeldata, radarstate = sm['carState'], sm['modelV2'], sm['radarState']
    gear = car.CarState.GearShifter
    gear_check = carstate.gearShifter not in (gear.neutral, gear.park, gear.reverse, gear.unknown)
    lead = self.detect_lead(radarstate)
    lead_distance = radarstate.leadOne.dRel
    speed_difference = radarstate.leadOne.vRel * 3.6
    standstill = carstate.standstill

    # Green light alert
    if self.green_light_alert and gear_check:
      self.previously_driving |= not standstill

      stopped_for_light = self.stop_sign_and_light(carstate, lead, lead_distance, modeldata, v_ego, v_lead) and standstill and self.previously_driving
      self.green_light = not stopped_for_light and self.stopped_for_light_previously and not lead
      self.stopped_for_light_previously = stopped_for_light

    # Conditional Experimental Mode
    if self.conditional_experimental_mode and sm['controlsState'].enabled:
      # Set the value of "overridden"
      if self.experimental_mode_via_wheel:
        overridden = self.params_memory.get_int("ConditionalStatus")
      else:
        overridden = 0

      # Update Experimental Mode based on the current driving conditions
      condition_met = self.check_conditions(sm, v_ego, v_lead, carstate, modeldata, lead, lead_distance, speed_difference, standstill)
      if (not self.experimental_mode and condition_met and overridden != 1) or overridden == 2:
        self.experimental_mode = True
      elif (self.experimental_mode and not condition_met and overridden != 2) or overridden == 1:
        self.experimental_mode = False

      # Set parameter for on-road status bar
      status_bar = overridden if overridden in (1, 2) else self.status_value if self.status_value >= 3 and self.experimental_mode else 0
      # Update the status bar if the status value has changed
      if status_bar != self.previous_status_bar:
        self.previous_status_bar = status_bar
        self.params_memory.put_int("ConditionalStatus", status_bar)

  # Check conditions for the appropriate state of Experimental Mode
  def check_conditions(self, sm, v_ego, v_lead, carstate, modeldata, lead, lead_distance, speed_difference, standstill):
    # Prevent Experimental Mode from deactivating at a standstill so we don't accidentally run red lights/stop signs
    if standstill and self.experimental_mode:
      return True

    # Speed check
    speed = (self.limit != 0 and not lead and v_ego < self.limit) or (self.limit_lead != 0 and lead and v_ego < self.limit_lead)
    if speed:
      self.status_value = 3 if lead else 4
      return True

    # Slower lead check
    approaching_lead = self.slower_lead and lead and speed_difference < interp(lead_distance, LEAD_DISTANCE, LEAD_SPEED_DIFF)
    if approaching_lead:
      self.status_value = 5
      return True

    # Turn signal check
    signal_active = self.signal and v_ego < 25 and (carstate.leftBlinker or carstate.rightBlinker)
    if signal_active:
      self.status_value = 6
      return True

    # Stop sign and light check
    stop_light_detected = self.stop_lights and self.stop_sign_and_light(carstate, lead, lead_distance, modeldata, v_ego, v_lead) and not standstill
    if stop_light_detected:
      self.status_value = 7
      return True

    # Road curvature check - Need to check for stop lights/stop signs since the curve function also detects them
    self.curve_detected = self.curves and self.road_curvature(lead, modeldata, v_ego) and not standstill and self.v_offset == 0
    if self.curve_detected:
      self.status_value = 8
      return True

  # Conditional Experimental Mode functions
  def detect_lead(self, radarstate):
    if radarstate.leadOne.status:
      # Check to make sure the lead isn't crossing the intersection
      if abs(radarstate.leadOne.yRel - self.previous_yRel) < 0.25:
        self.lead_status_count = max(10, self.lead_status_count + 1)
      else:
        self.lead_status_count = 0
      self.previous_yRel = radarstate.leadOne.yRel
    else:
      self.lead_status_count = min(0, self.lead_status_count - 1)
    # Check if lead is detected for > 0.25s
    return self.lead_status_count >= THRESHOLD

  # Determine the road curvature - Credit goes to to Pfeiferj!
  def road_curvature(self, lead, modeldata, v_ego):
    # Check if the lead toggle is on or we don't have a lead if not
    if self.curves_lead or not lead:
      predicted_lateral_accelerations = np.abs(np.array(modeldata.acceleration.y))
      predicted_velocities = np.array(modeldata.velocity.x)
      if len(predicted_lateral_accelerations) == len(predicted_velocities) != 0:
        curvature_ratios = predicted_lateral_accelerations / (predicted_velocities ** 2)
        predicted_lateral_accelerations = curvature_ratios * (v_ego ** 2)
        curvature = np.amax(predicted_lateral_accelerations)
        # Setting an upper limit of "5.0" helps prevent it activating at stop lights
        if 5.0 > curvature >= 1.6 or (self.curve_detected and 5.0 > curvature >= 1.1):
          # Setting the maximum to 10 lets it hold the status for 0.25s after it goes "False" to help prevent false negatives
          self.curvature_count = min(10, self.curvature_count + 1)
        else:
          self.curvature_count = max(0, self.curvature_count - 1)
        # Check if curve is detected for > 0.25s
        return self.curvature_count >= THRESHOLD
    return False

  # Stop sign and stop light detection - Credit goes to the DragonPilot team!
  def stop_sign_and_light(self, carstate, lead, lead_distance, modeldata, v_ego, v_lead):
    if abs(carstate.steeringAngleDeg) <= 60 or self.stop_light_count >= THRESHOLD:
      # Check to make sure we don't have a lead that's stopping for the red light / stop sign
      if not lead or not (self.previous_lead_speed >= v_lead or lead_distance <= 10 or v_lead <= 1):
        if len(modeldata.orientation.x) == len(modeldata.position.x) == TRAJECTORY_SIZE:
          if modeldata.position.x[TRAJECTORY_SIZE - 1] < interp(v_ego * 3.6, STOP_SIGN_BP, STOP_SIGN_DISTANCE):
            self.stop_light_count = min(10, self.stop_light_count + 1)
          else:
            self.stop_light_count = max(0, self.stop_light_count - 1)
        else:
          self.stop_light_count = max(0, self.stop_light_count - 1)
      else:
        self.stop_light_count = max(0, self.stop_light_count - 1)
    else:
      self.stop_light_count = max(0, self.stop_light_count - 1)
    self.previous_lead_speed = v_lead
    # Check if stop sign / stop light is detected for > 0.25s
    return self.stop_light_count >= THRESHOLD

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.distances = self.x_desired_trajectory.tolist()
    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time
    longitudinalPlan.personality = self.personality

    # FrogPilot longitudinalPlan variables
    longitudinalPlan.conditionalExperimental = self.experimental_mode
    longitudinalPlan.greenLight = self.green_light
    longitudinalPlan.vtscOffset = self.v_offset
    # LongitudinalPlan variables for onroad driving insights
    have_lead = self.detect_lead(sm['radarState'])
    longitudinalPlan.safeObstacleDistance = self.mpc.safe_obstacle_distance if have_lead else 0
    longitudinalPlan.stoppedEquivalenceFactor = self.mpc.stopped_equivalence_factor if have_lead else 0
    longitudinalPlan.desiredFollowDistance = self.mpc.safe_obstacle_distance - self.mpc.stopped_equivalence_factor if have_lead else 0
    longitudinalPlan.safeObstacleDistanceStock = self.mpc.safe_obstacle_distance_stock if have_lead else 0
    longitudinalPlan.stoppedEquivalenceFactorStock = self.mpc.stopped_equivalence_factor_stock if have_lead else 0

    pm.send('longitudinalPlan', plan_send)
    
  def update_frogpilot_params(self):
    self.longitudinal_tuning = self.params.get_bool("LongitudinalTuning")
    self.acceleration_profile = self.params.get_int("AccelerationProfile") if self.longitudinal_tuning else 2
    self.aggressive_acceleration = self.params.get_bool("AggressiveAcceleration") and self.longitudinal_tuning
    self.increased_stopping_distance = self.params.get_int("IncreasedStoppingDistance") * (1 if self.is_metric else 0.3048) if self.longitudinal_tuning else 0
    self.smoother_braking = self.params.get_bool("SmootherBraking") and self.longitudinal_tuning

    self.conditional_experimental_mode = self.params.get_bool("ConditionalExperimental")
    if self.conditional_experimental_mode:
      self.curves = self.params.get_bool("ConditionalCurves")
      self.curves_lead = self.params.get_bool("ConditionalCurvesLead")
      self.limit = self.params.get_int("ConditionalSpeed") * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      self.limit_lead = self.params.get_int("ConditionalSpeedLead") * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      self.signal = self.params.get_bool("ConditionalSignal")
      self.slower_lead = self.params.get_bool("ConditionalSlowerLead")
      self.stop_lights = self.params.get_bool("ConditionalStopLights")

    self.custom_personalities = self.params.get_bool("CustomPersonalities")
    if self.custom_personalities:
      self.aggressive_follow = self.params.get_int("AggressivePersonality") / 10
      self.standard_follow = self.params.get_int("StandardPersonality") / 10
      self.relaxed_follow = self.params.get_int("RelaxedPersonality") / 10
      self.aggressive_jerk = self.params.get_int("AggressiveJerk") / 10
      self.standard_jerk = self.params.get_int("StandardJerk") / 10
      self.relaxed_jerk = self.params.get_int("RelaxedJerk") / 10

    self.experimental_mode_via_wheel = self.params.get_bool("ExperimentalModeViaWheel")
    self.green_light_alert = self.params.get_bool("GreenLightAlert")

    self.vision_turn_controller = self.params.get_bool("VisionTurnControl")
    if self.vision_turn_controller:
      self.curve_sensitivity = self.params.get_int("CurveSensitivity") / 100
      self.turn_aggressiveness = self.params.get_int("TurnAggressiveness") / 100
