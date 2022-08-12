static const addr_checks* wuling_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
  return &default_rx_checks;
}

static int wuling_rx_hook(CANPacket_t *to_send) {
  UNUSED(to_send);

  controls_allowed = 1;
  
  return false;
}

// all commands: gas, brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int wuling_tx_hook(CANPacket_t *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  UNUSED(addr);
  UNUSED(bus);
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
    bool is_lkas_msg = (addr == 0x373) || (addr == 0x370) || (addr == 0x33D);
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
