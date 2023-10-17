import copy
from cereal import car
from common.numpy_fast import clip
from selfdrive.car import make_can_msg
from openpilot.selfdrive.car.wuling.values import CruiseButtons

VisualAlert = car.CarControl.HUDControl.VisualAlert

def wuling_checksum(dat):
  return sum(dat) & 0xFF

def create_steering_control(packer, apply_steer, frame, steer_req):


  idx = (apply_steer) % 255
  # apply_steer  = clip(apply_steer,-100,100);
  values = {
      "STEER_TORQUE_CMD": -apply_steer,
      "SET_ME_X0": 0x00,
      "COUNTER": (frame/2) % 4,
      "STEER_REQUEST": steer_req,
  }
  values["COUNTER"] = (values["COUNTER"] + 1) % 0x11
  
  dat = packer.make_can_msg("STEERING_LKA", 0, values)[2]

  crc = wuling_checksum(dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("STEERING_LKA", 0, values)

def create_steering_status(packer, apply_steer, frame, steer_step):
  return packer.make_can_msg("ES_LKAS_State", 0, {})


def create_gas_command(packer,throttle, idx, acc_engaged, at_full_stop):
  values = {
    "ENABLE": acc_engaged,
    "COUNTER": idx,
    "COUNTER_2": idx,
    "GAS_CMD": throttle,
    "NEW_SIGNAL_2": 0x04,
    "NEW_SIGNAL_1": 0x0b,
  }

  return packer.make_can_msg("GasCmd", 0, values)

def create_brake_command(packer, apply_brake, idx, acc_engaged, at_full_stop):
  values = {
    "ENABLE": acc_engaged,
    "COUNTER": idx,
    "COUNTER_2": idx,
    "BRAKE_CMD": apply_brake,
    "BRAKE_CMD_2": 27,
    "NEW_SIGNAL_8": 0x50,
    "NEW_SIGNAL_5": 0x15,
    "NEW_SIGNAL_4": 0x01,
    "NEW_SIGNAL_2": 0x06,
    "NEW_SIGNAL_6": 0x01,
    "NEW_SIGNAL_3": 0x35,
  }

  return packer.make_can_msg("BRAKE_MODULE", 0, values)


def create_acc_dashboard_command(packer, acc_engaged, idx, target_speed_kph, resume_button, lead_car_in_sight, fcw):
  # Not a bit shift, dash can round up based on low 4 bits.
  target_speed = int(target_speed_kph) & 0xfff

  values = {
    "ACCAlwaysOne" : 1,
    "COUNTER_1" : idx,
    "COUNTER_2" : (idx+1) % 4,
    "ACCSTATE" : acc_engaged,
    "ACCResumeButton" : resume_button,
    "ACCSpeedSetpoint" : target_speed,
    "ACCGapLevel" : 6 * acc_engaged,  # 3 "far", 0 "inactive"
    "ACCCmdActive" : acc_engaged,
    "ACCAlwaysOne2" : 3,
    "ACCLeadCar" : lead_car_in_sight,
    "SET_ME_X16" : 0x16,
    "FCWAlert": 0x3 if fcw else 0
  }

  return packer.make_can_msg("ASCMActiveCruiseControlStatus", 0, values)

def create_buttons(packer, idx, button):
  resume = 0;
  acc_btn_2 = button;
  if (button == CruiseButtons.RES_ACCEL): 
    resume = 1;
    acc_btn_2 = CruiseButtons.MAIN
  
  values = {
    "RESUME_BTN_2" : resume,
    "ACC_BTN_1" : button,
    "ACC_BTN_2" : acc_btn_2,
    "COUNTER_1" : (idx+1) % 0x11,
    "COUNTER_2" : (idx+1) % 0x11,
  }

  return packer.make_can_msg("STEER_BTN", 0, values)


def create_lkas_hud(packer, bus, lkas_hud_stock_values, steer_warning=0):
  values = {s: lkas_hud_stock_values[s] for s in [
    "LKA_ACTIVE",
    "LKAS_STATE",
    "LKA_LINE",
    "LKA_LINE_2",
    "ALERT_DEPART1",
    "STEER_WARNING",
    "COUNTER_1",
    "COUNTER_2",
    "NEW_SIGNAL_1",
    "NEW_SIGNAL_2",
    "NEW_SIGNAL_3",
    "NEW_SIGNAL_4",
    "NEW_SIGNAL_6",
    "NEW_SIGNAL_7",
    "NEW_SIGNAL_8",
    "NEW_SIGNAL_9",
  ]}

  # print('Send to Lkas');
  # print(values)
  values.update({
    "COUNTER_1": (lkas_hud_stock_values["COUNTER_1"] + 1) % 0x11,
    "COUNTER_2": (lkas_hud_stock_values["COUNTER_2"] + 1) % 0x11,
    "STEER_WARNING": steer_warning,
  })

  return packer.make_can_msg("LkasHud", bus, values)

def create_adas_time_status(bus, tt, idx):
  dat = [(tt >> 20) & 0xff, (tt >> 12) & 0xff, (tt >> 4) & 0xff,
    ((tt & 0xf) << 4) + (idx << 2)]
  chksum = 0x1000 - dat[0] - dat[1] - dat[2] - dat[3]
  chksum = chksum & 0xfff
  dat += [0x40 + (chksum >> 8), chksum & 0xff, 0x12]
  return make_can_msg(0xa1, bytes(dat), bus)

def create_adas_steering_status(bus, idx):
  dat = [idx << 6, 0xf0, 0x20, 0, 0, 0]
  chksum = 0x60 + sum(dat)
  dat += [chksum >> 8, chksum & 0xff]
  return make_can_msg(0x306, bytes(dat), bus)

def create_adas_accelerometer_speed_status(bus, speed_ms, idx):
  spd = int(speed_ms * 16) & 0xfff
  accel = 0 & 0xfff
  # 0 if in park/neutral, 0x10 if in reverse, 0x08 for D/L
  #stick = 0x08
  near_range_cutoff = 0x27
  near_range_mode = 1 if spd <= near_range_cutoff else 0
  far_range_mode = 1 - near_range_mode
  dat = [0x08, spd >> 4, ((spd & 0xf) << 4) | (accel >> 8), accel & 0xff, 0]
  chksum = 0x62 + far_range_mode + (idx << 2) + dat[0] + dat[1] + dat[2] + dat[3] + dat[4]
  dat += [(idx << 5) + (far_range_mode << 4) + (near_range_mode << 3) + (chksum >> 8), chksum & 0xff]
  return make_can_msg(0x308, bytes(dat), bus)

def create_adas_headlights_status(packer, bus):
  values = {
    "Always42": 0x42,
    "Always4": 0x4,
  }
  return packer.make_can_msg("ASCMHeadlight", bus, values)

def create_lka_icon_command(bus, active, critical, steer):
  dat = b"\xc6\x3d\x01\x00\xac\x90\x02\x42"
  return make_can_msg(0x104c006c, dat, bus)

def create_resume_button(bus, active, critical, steer):
  dat = b"\x48\x08\x00\x00\x00\x00\x00\x50"
  return make_can_msg(0x1e1, dat, bus)

