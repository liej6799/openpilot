from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CarControllerParams:
  STEER_MAX = 300  # Safety limit, not LKA max. Trucks use 600.
  STEER_STEP = 2  # control frames per command
  STEER_MAX = 261         # 262 faults
  STEER_DELTA_UP = 3      # 3 is stock. 100 is fine. 200 is too much it seems
  STEER_DELTA_DOWN = 3    # no faults on the way down it seems
  STEER_ERROR_MAX = 80
  MIN_STEER_SPEED = 3.  # m/s

  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc
  NEAR_STOP_BRAKE_PHASE = 0.5  # m/s

  # Heartbeat for dash "Service Adaptive Cruise" and "Service Front Camera"
  ADAS_KEEPALIVE_STEP = 100
  CAMERA_KEEPALIVE_STEP = 100
  
  
  # Volt gasbrake lookups
  # TODO: These values should be confirmed on non-Volt vehicles
  MAX_GAS = 3072  # Safety limit, not ACC max. Stock ACC >4096 from standstill.
  ZERO_GAS = 2048  # Coasting
  MAX_BRAKE = 350  # ~ -3.5 m/s^2 with regen
  MAX_ACC_REGEN = 1404  # Max ACC regen is slightly less than max paddle regen

  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  ACCEL_MAX = 2.  # m/s^2
  ACCEL_MIN = -4.  # m/s^2

  EV_GAS_LOOKUP_BP = [-1., 0., ACCEL_MAX]
  EV_BRAKE_LOOKUP_BP = [ACCEL_MIN, -1.]

  # ICE has much less engine braking force compared to regen in EVs,
  # lower threshold removes some braking deadzone
  GAS_LOOKUP_BP = [-0.1, 0., ACCEL_MAX]
  BRAKE_LOOKUP_BP = [ACCEL_MIN, -0.1]

  GAS_LOOKUP_V = [MAX_ACC_REGEN, ZERO_GAS, MAX_GAS]
  BRAKE_LOOKUP_V = [MAX_BRAKE, 0.]
class CAR:
  ALMAS_RS_PRO = "WULING ALMAZ RS PRO 2022"
class CruiseButtons:
  INIT = 0
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  MAIN = 5
  CANCEL = 6

class AccState:
  OFF = 0
  ACTIVE = 1
  FAULTED = 3
class CanBus:
  POWERTRAIN = 0
  OBSTACLE = 1
  CHASSIS = 2
  SW_GMLAN = 3
  LOOPBACK = 128
  
FINGERPRINTS = {
  CAR.ALMAS_RS_PRO: [{
    193: 8, 197: 8, 201: 8, 225: 8, 288: 5, 296: 8, 298: 8, 320: 4, 381: 8, 401: 8, 404: 8, 413: 8, 451: 8, 454: 8, 481: 8, 485: 8, 489: 8, 497: 8, 501: 8, 549: 8, 560: 8, 608: 8, 611: 8, 617: 8, 840: 6, 842: 6, 844: 6, 846: 6, 880: 8, 883: 8, 996: 8, 997: 8, 1041: 8, 1043: 8, 1045: 8, 1047: 8, 1053: 8, 1065: 8, 1217: 8, 1225: 8, 1341: 8, 1381: 8, 1406: 8, 1417: 8, 1538: 8, 1541: 8, 1543: 8, 1552: 8, 1569: 8
  }]
}

FW_VERSIONS = {
   CAR.ALMAS_RS_PRO: {}
}
DBC = {
  CAR.ALMAS_RS_PRO: dbc_dict('wuling_almazrs_generated', None),
}

STEER_THRESHOLD = 0
HUD_MULTIPLIER = 0.70

PREGLOBAL_CARS = []
