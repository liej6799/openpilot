//safety wuling
#define ENGINE_DATA   0xc9
#define LKAS_HUD      0x373

// CAN bus numbers
#define BUS_MAIN 0
#define BUS_RADAR  1
#define BUS_CAM  2

const CanMsg WULING_TX_MSGS[] = {{ENGINE_DATA, 0, 8}, {LKAS_HUD, 0, 8}};

AddrCheckStruct wl_addr_checks[] = {
  {.msg = {{ENGINE_DATA, 0, 5, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{LKAS_HUD, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
};


#define WL_RX_CHECK_LEN (sizeof(wl_addr_checks) / sizeof(wl_addr_checks[0]))
addr_checks wl_rx_checks = {wl_addr_checks, WL_RX_CHECK_LEN};

static const addr_checks* wuling_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 0;
  relay_malfunction_reset();
  return &wl_rx_checks;
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static int wuling_rx_hook(CANPacket_t *to_send) {
  UNUSED(to_send);

  bool valid = addr_safety_check(to_send, &wl_rx_checks, NULL, NULL, NULL);

   if (valid && ((int)GET_BUS(to_send) == BUS_MAIN)) {
      int addr = GET_ADDR(to_send);

      generic_rx_checks((addr == LKAS_HUD));
   }

  controls_allowed = 1;
  
  return true;
}

// all commands: gas, brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation
// Message send to main bus

static int wuling_tx_hook(CANPacket_t *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  UNUSED(addr);
  UNUSED(bus);
  controls_allowed = 1;

  // 1 allows the message through
  return tx;
}

static int wuling_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  // fwd from car to camera. also fwd certain msgs from camera to car
  // 0xE4 is steering on all cars except CRV and RDX, 0x194 for CRV and RDX,
  // 0x1FA is brake control, 0x30C is acc hud, 0x33D is lkas hud,
  int bus_fwd = -1;

  if (bus_num == 0) {
    bus_fwd = 2;
  }

  if (bus_num == 2) {
    // block stock lkas messages and stock acc messages (if OP is doing ACC)
    int addr = GET_ADDR(to_fwd);
    // bool is_lkas_msg = (addr == 0x373) || (addr == 0x370) || (addr == 0x33D);
    bool is_lkas_msg = (addr == 0x225) || (addr == 0x194) || (addr == 0x33D);
    // bool is_lkas_msg = (addr == 0x225) || (addr == 0x194) || (addr == 0x33D);
    bool is_acc_hud_msg = addr == 0x30C;
    bool is_brake_msg = addr == 0x1FA;
    bool block_fwd = is_lkas_msg || is_acc_hud_msg || (is_brake_msg);
    if (!block_fwd) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

const safety_hooks wuling_hooks = {
  .init = wuling_init,
  .rx = wuling_rx_hook,
  .tx = wuling_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = wuling_fwd_hook,
};
