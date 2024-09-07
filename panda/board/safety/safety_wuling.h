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

const int WL_STANDSTILL_THRESHOLD = 10;  // 0.311kph

const CanMsg WULING_TX_MSGS[] = {
    {STEERING_LKAS, 0, 8}, 
    {BRAKE_DATA, 0, 8},
    {ACC_DATA, 0, 8}
};

RxCheck wl_addr_checks[] = {
  {.msg = {{WHEEL_DATA,   0, 4, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{STEER_DATA,   0, 8, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{ENGINE_DATA,  0, 8, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{GAS_DATA,     0, 8, .frequency = 100U}, { 0 }, { 0 }}}
};

static void wuling_rx_hook(const CANPacket_t *to_push) {
  if ((int)GET_BUS(to_push) == BUS_MAIN) {
    int addr = GET_ADDR(to_push);

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
}

static bool wuling_tx_hook(CANPacket_t *to_send) {
  bool tx = true;
  UNUSED(to_send);
  return tx;
}

static int wuling_fwd_hook(int bus, int addr) {
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

static safety_config wuling_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(wl_addr_checks, WULING_TX_MSGS);
}

const safety_hooks wuling_hooks = {
  .init = wuling_init,
  .rx = wuling_rx_hook,
  .tx = wuling_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = wuling_fwd_hook,
};