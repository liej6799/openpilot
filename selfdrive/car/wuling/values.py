from collections import defaultdict
from dataclasses import dataclass
from enum import Enum, IntFlag, StrEnum
from typing import Dict, List, Union
from panda.python import uds

from cereal import car
from selfdrive.car import dbc_dict
from selfdrive.car.docs_definitions import CarFootnote, CarHarness, CarInfo, CarParts, Column
from selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries, p16

Ecu = car.CarParams.Ecu



class CarControllerParams:

  STEER_STEP = 2          # control frames per command 50Hz
  LKAS_HUD_STEP = 5                     # LkasHUD frequency 20Hz

  STEER_MAX = 150  # Safety limit, not LKA max. Trucks use 600.
  STEER_DELTA_UP = 4      # 3 is stock. 100 is fine. 200 is too much it seems
  STEER_DELTA_DOWN = 3    # no faults on the way down it seems
  
  STEER_ERROR_MAX = 80
  MIN_STEER_SPEED = 3.  # m/s

  STEER_DRIVER_ALLOWANCE = 35
  STEER_DRIVER_MULTIPLIER = 2    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc
  NEAR_STOP_BRAKE_PHASE = 0.5  # m/s
  INACTIVE_STEER_STEP = 10  # Inactive control frames per command (10hz)
  
  # Heartbeat for dash "Service Adaptive Cruise" and "Service Front Camera"
  ADAS_KEEPALIVE_STEP = 100
  CAMERA_KEEPALIVE_STEP = 100
  STEER_THRESHOLD = 40
  HUD_MULTIPLIER = 0.685
  
  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  ACCEL_MAX = 2.  # m/s^2
  ACCEL_MIN = -4.  # m/s^2

  def __init__(self, CP):
   pass

class CAR:
 ALMAS_RS_PRO = "WULING ALMAZ RS PRO 2022"


class Footnote(Enum):
  OBD_II = CarFootnote(
    'Wuling Almaz RS WITH Acc and LKAS',
    Column.MODEL)


@dataclass
class GMCarInfo(CarInfo):
  package: str = "Adaptive Cruise Control (ACC)"
  
  def init_make(self, CP: car.CarParams):
    if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera:
      self.car_parts = CarParts.common([CarHarness.gm])
    else:
      self.car_parts = CarParts.common([CarHarness.obd_ii])
      self.footnotes.append(Footnote.OBD_II)


CAR_INFO: Dict[str, Union[GMCarInfo, List[GMCarInfo]]] = {
  CAR.ALMAS_RS_PRO: GMCarInfo("Wuling Almaz RS Pro 2022"),
}

class CruiseButtons:
  INIT = 0
  NONE = 0
  UNPRESS = 0
  GAP_DOWN = 1
  GAP_UP = 2
  DECEL_SET = 4
  RES_ACCEL = 8
  MAIN = 16
  CANCEL = 32
  TJA = 32

class AccState:
  OFF = 0
  ACTIVE = 1
  FAULTED = 3
  STANDSTILL = 4

class CanBus:
  POWERTRAIN = 0
  OBSTACLE = 1
  CAMERA = 2
  CHASSIS = 2
  SW_GMLAN = 3
  LOOPBACK = 128
  DROPPED = 192

FINGERPRINTS = {
  CAR.ALMAS_RS_PRO: [{
    193: 8, 197: 8, 201: 8, 225: 8, 288: 5, 296: 8, 298: 8, 320: 4, 381: 8, 401: 8, 404: 8, 413: 8, 451: 8, 454: 8, 481: 8, 485: 8, 489: 8, 497: 8, 501: 8, 549: 8, 560: 8, 608: 8, 611: 8, 617: 8, 840: 6, 842: 6, 844: 6, 846: 6, 880: 8, 883: 8, 996: 8, 997: 8, 1041: 8, 1043: 8, 1045: 8, 1047: 8, 1053: 8, 1065: 8, 1217: 8, 1225: 8, 1341: 8, 1381: 8, 1406: 8, 1417: 8, 1538: 8, 1541: 8, 1543: 8, 1552: 8, 1569: 8
  }]
}

WULING_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.ECU_MANUFACTURING_DATE) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_HARDWARE_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER)
WULING_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
     Request(
      [WULING_VERSION_REQUEST],
      [WULING_VERSION_RESPONSE],
    ),
  ],
)


FW_VERSIONS = {
   CAR.ALMAS_RS_PRO: {
     (Ecu.engine, 0x7e0, None): [
       b'\xf1\x8b !\t\x10\xf1\x9410436987AA      '
     ],
     (Ecu.transmission, 0x7e1, None): [
       b'\xf1\x8b !\x10\x07\xf1\x95C0390011\xf1\x94  bfqa0501'
     ],
     (Ecu.fwdRadar, 0x726, None): [
       b'\xf1\x8b\x00\x00\x00\x00\xf1\x95SGMW.SW.A.3.0\xf1\x91\x01jz\xca'
     ],
     (Ecu.eps, 0x720, None): [
       b'\xf1\x8b210704\xf1\x95\x00\x00\x00y\xf1\x91\x01i\x0c\xf9\xf1\x94\x08"\t'
     ],
   }
}
DBC = {
  CAR.ALMAS_RS_PRO: dbc_dict('wuling_almazrs_generated', None),
}
PREGLOBAL_CARS = ()
