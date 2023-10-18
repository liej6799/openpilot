from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV
import json

mem_params = Params("/dev/shm/params")
params = Params()

class SpeedLimitController:
  nav_speed_limit: float = 0 # m/s
  map_speed_limit: float = 0 # m/s
  car_speed_limit: float = 0 # m/s
  offset: float = 0 # m/s
  nav_enabled: bool = False
  car_enabled: bool = False
  current_max_velocity_update_count: int = 0

  def __init__(self) -> None:
    self.load_persistent_enabled()
    self.write_nav_state()
    self.write_map_state()
    self.write_car_state()
    self.write_offset_state()

  def update_current_max_velocity(self, max_v: float, load_state: bool = True, write_state: bool = True) -> None:
    self.current_max_velocity_update_count += 1
    self.current_max_velocity_update_count = self.current_max_velocity_update_count % 100
    if load_state:
      self.load_state()
      if self.current_max_velocity_update_count == 0:
        self.load_persistent_enabled()

  @property
  def speed_limit(self) -> float:
    limit: float = 0
    if self.nav_enabled and self.nav_speed_limit != 0:
      limit = self.nav_speed_limit
    elif self.map_enabled and self.map_speed_limit != 0:
      limit = self.map_speed_limit
    elif self.car_enabled and self.car_speed_limit != 0:
      limit = self.car_speed_limit

    return limit

  @property
  def speed_limit_mph(self) -> float:
    return self.speed_limit * CV.MS_TO_MPH

  @property
  def speed_limit_kph(self) -> float:
    return self.speed_limit * CV.MS_TO_KPH

  @property
  def offset_mph(self) -> float:
    return self.offset * CV.MS_TO_MPH

  @property
  def offset_kph(self) -> float:
    return self.offset * CV.MS_TO_KPH

  def write_nav_state(self):
    mem_params.put("NavSpeedLimit", json.dumps(self.nav_speed_limit))
    mem_params.put_bool("NavSpeedLimitControl", self.nav_enabled)

  def write_map_state(self):
    mem_params.put("MapSpeedLimit", json.dumps(self.map_speed_limit))
    mem_params.put_bool("MapSpeedLimitControl", self.map_enabled)

  def write_car_state(self):
    mem_params.put("CarSpeedLimit", json.dumps(self.car_speed_limit))
    mem_params.put_bool("CarSpeedLimitControl", self.car_enabled)

  def write_offset_state(self):
    mem_params.put("SpeedLimitOffset", json.dumps(self.offset))

  def load_state(self, load_persistent_enabled=False):
    self.nav_enabled = mem_params.get_bool("NavSpeedLimitControl")
    self.car_enabled = mem_params.get_bool("CarSpeedLimitControl")
    self.map_enabled = mem_params.get_bool("MapSpeedLimitControl")
    self.offset = json.loads(mem_params.get("SpeedLimitOffset"))
    self.nav_speed_limit = json.loads(mem_params.get("NavSpeedLimit"))
    self.map_speed_limit = json.loads(mem_params.get("MapSpeedLimit"))
    self.car_speed_limit = json.loads(mem_params.get("CarSpeedLimit"))

    if load_persistent_enabled:
      self.load_persistent_enabled()

  def load_persistent_enabled(self):
    self.nav_enabled = True
    self.car_enabled = True
    self.map_enabled = True
    mem_params.put_bool("NavSpeedLimitControl", self.nav_enabled)
    mem_params.put_bool("MapSpeedLimitControl", self.map_enabled)
    mem_params.put_bool("CarSpeedLimitControl", self.car_enabled)


  def write_persistent_enabled(self):
    params.put_bool("NavSpeedLimitControl", self.nav_enabled)
    params.put_bool("CarSpeedLimitControl", self.car_enabled)
    params.put_bool("MapSpeedLimitControl", self.map_enabled)


slc = SpeedLimitController()
