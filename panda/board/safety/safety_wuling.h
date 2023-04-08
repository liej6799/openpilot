// CAN msgs we care about
//safety wuling
#define ENGINE_DATA   0xc9
#define LKAS_HUD      0x373
#define STEERING_LKAS      0x225

// CAN bus numbers
#define BUS_MAIN 0
#define BUS_RADAR  1
#define BUS_CAM  2

const CanMsg WULING_TX_MSGS[] = {{ENGINE_DATA, 0, 8}, {LKAS_HUD, 0, 8}};

AddrCheckStruct wl_addr_checks[] = {
  {.msg = {{ENGINE_DATA, 0, 5, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{STEERING_LKAS, 2, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{LKAS_HUD, 2, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define WL_RX_CHECK_LEN (sizeof(wl_addr_checks) / sizeof(wl_addr_checks[0]))
addr_checks wl_rx_checks = {wl_addr_checks, WL_RX_CHECK_LEN};
// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static int wuling_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &wl_rx_checks, NULL, NULL, NULL, NULL);

   if (valid && ((int)GET_BUS(to_push) == BUS_MAIN)) {
      int addr = GET_ADDR(to_push);

      if (addr == 842) {
        vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
      }
      
      if (addr == 201) {
        brake_pressed = GET_BIT(to_push, 40U) != 0U;
      }

      if (addr == 0x191) {
        gas_pressed = GET_BYTE(to_push, 6) != 0U;
      }

      if ((addr == 0x263)) {
        acc_main_on = GET_BIT(to_push, 38U) != 0U;
        if (!acc_main_on) {
          if (!cruise_engaged_prev) {
            controls_allowed = 1;
          }
        }else {
          controls_allowed = 0;
        }
        cruise_engaged_prev = acc_main_on;
      }

      generic_rx_checks((addr == LKAS_HUD || addr == STEERING_LKAS));
   }

  controls_allowed = 1;
  
  return valid;
}

static int wuling_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  // Check if msg is sent on the main BUS

  UNUSED(addr);
  UNUSED(bus);

  // 1 allows the message through
  return tx;
}

static int wuling_fwd_hook(int bus, CANPacket_t *to_fwd) {
  // fwd from car to camera. also fwd certain msgs from camera to car

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (bus == BUS_MAIN) {
    bus_fwd = BUS_CAM;
  } else if (bus == BUS_CAM) {
    // bool block = (addr == LKAS_HUD) || (addr == STEERING_LKAS);
    bool block =  (addr == STEERING_LKAS);
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
  return &mazda_rx_checks;
}

const safety_hooks wuling_hooks = {
  .init = wuling_init,
  .rx = wuling_rx_hook,
  .tx = wuling_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = wuling_fwd_hook,
};
