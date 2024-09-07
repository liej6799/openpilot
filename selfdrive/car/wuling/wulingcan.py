
def wuling_checksum(dat):
  return sum(dat) & 0xFF

def create_steering_control(packer, apply_steer, idx, steer_req):

  values = {
        "ACTIVE": 0x64, # Always active 
        "ACTIVE_2": 0x64 if steer_req else 0,
        "STEER_LOCK": 1 if steer_req else 0,
        "COUNTER": idx,
        "STEER_ANGLE_CMD": -apply_steer
  }
  
  values["COUNTER"] = (values["COUNTER"] + 1) % 0x11
  
  dat = packer.make_can_msg("STEERING_LKA", 0, values)[2]

  crc = wuling_checksum(dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("STEERING_LKA", 0, values)


def create_gas_command(packer, idx, acc_req, apply_start, throttle):
    
  values = {
    "ALWAYS_ON_1": 5,
    "ALWAYS_ON_2": 0x78,
    "ACC_ACTIVE_1": 32 if acc_req else 0,
    "ACC_ACTIVE_2": 1 if acc_req else 0,
    
    "GAS_ACTIVE_2": 1 if acc_req and throttle != 1696 else 0, # change this later
    "INV_GAS_ACTIVE_2": 1 if acc_req and throttle == 1696 else 0, #change this later

    "COUNTER": idx,
    "GAS_ACTIVE": 4 if acc_req and throttle != 1696 else 0, # need to check is it based on throttle.
    "GAS_THROTTLE": throttle # need to change later
  }
  
  values["COUNTER"] = (values["COUNTER"] + 1) % 0x11
  
  dat = packer.make_can_msg("AccStatus", 0, values)[2]

  crc = wuling_checksum(dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("AccStatus", 0, values)


def create_brake_command(packer, apply_brake, idx, brake_value):
  values = {
    "BRAKE_CMD": 15 if brake_value > 20 else 0,
    "BRAKE_CMD_2": 16 if brake_value > 20 else 0,
    "COUNTER": idx, # 0 - 255
    "BRAKE_VAL": brake_value,
  }
  
  values["COUNTER"] = (values["COUNTER"] + 1) % 0x11
  
  dat = packer.make_can_msg("BrakeCMD", 0, values)[2]

  crc = wuling_checksum(dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("BrakeCMD", 0, values)
 