#define WHEEL_DATA   0x348
#define STEER_DATA   0x1e5
#define ENGINE_DATA  0xc9
#define GAS_DATA     0x191

// Message from Camera
#define STEERING_LKAS 0x225 // STEER
#define BRAKE_DATA    0x260 // BRAKE
#define ACC_DATA      0x263 // ACC
#define TEST_DATA_1   0x370 // LKAS HUD 1
#define TEST_DATA_2   0x373 // LKAS HUD 2

// CAN bus numbers
#define BUS_MAIN 0
#define BUS_RADAR  1
#define BUS_CAM  2

const CanMsg WULING_TX_MSGS[] = {
    {STEERING_LKAS, 0, 8}, 
    {BRAKE_DATA, 0, 8},
    {ACC_DATA, 0, 8}
};

AddrCheckStruct wl_addr_checks[] = {
  {.msg = {{WHEEL_DATA,   0, 4, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{STEER_DATA,   0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{ENGINE_DATA,  0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{GAS_DATA,     0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}}
};

#define WL_RX_CHECK_LEN (sizeof(wl_addr_checks) / sizeof(wl_addr_checks[0]))
addr_checks wl_rx_checks = {wl_addr_checks, WL_RX_CHECK_LEN};
// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static int wuling_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &wl_rx_checks, NULL, NULL, NULL, NULL);

   if (valid && ((int)GET_BUS(to_push) == BUS_MAIN)) {
      int addr = GET_ADDR(to_push);
      // UNUSED(addr);
      if (addr == WHEEL_DATA) {
        int left_rear_speed = (GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1);
        int right_rear_speed = (GET_BYTE(to_push, 2) << 8) | GET_BYTE(to_push, 3);
        vehicle_moving = (left_rear_speed > WL_STANDSTILL_THRESHOLD) || (right_rear_speed > WL_STANDSTILL_THRESHOLD);
    }

      if (addr == STEER_DATA) {
      int torque_driver_new = GET_BYTE(to_push, 6);
      update_sample(&torque_driver, torque_driver_new);
    }
      
       if (addr == ENGINE_DATA) {
        brake_pressed = GET_BIT(to_push, 40U) != 0U;
    }

    if (addr == GAS_DATA) {
        gas_pressed = GET_BYTE(to_push, 6) != 0U;
    }

      generic_rx_checks((addr == STEERING_LKAS));
   }

  controls_allowed = 1;
  return true;
}

static int wuling_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  // Check if msg is sent on the main BUS

  UNUSED(addr);
  UNUSED(bus);
  controls_allowed = 1;

  // 1 allows the message through
  return tx;
}

static int wuling_fwd_hook(int bus, int addr) {
  // fwd from car to camera. also fwd certain msgs from camera to car

  int bus_fwd = -1;

  if (bus == BUS_MAIN) {
    bus_fwd = BUS_CAM;
  } else if (bus == BUS_CAM) {
    bool block =  (addr == STEERING_LKAS || addr == BRAKE_DATA || addr == ACC_DATA);
    if (!block) {
      bus_fwd = BUS_MAIN;
    }
  } else {
    // don't fwd
  }

  return bus_fwd;
}

static const addr_checks* wuling_init(uint16_t param) {
  UNUSED(param);
  controls_allowed = 1;
  relay_malfunction_reset();
  return &wl_rx_checks;
}

const safety_hooks wuling_hooks = {
  .init = wuling_init,
  .rx = wuling_rx_hook,
  .tx = wuling_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = wuling_fwd_hook,
};
