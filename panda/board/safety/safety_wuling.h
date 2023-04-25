//safety wuling
#define ENGINE_DATA   0xc9
#define LKAS_HUD      0x373
#define STEERING_LKAS      0x225
#define BRAKE_DATA      0x269
#define GAS_DATA      0x260

// CAN bus numbers
#define BUS_MAIN 0
#define BUS_RADAR  1
#define BUS_CAM  2U

const CanMsg WULING_IGNITION_ID = {0x225, 0, 8};
const CanMsg WULING_TX_MSGS[] = {{ENGINE_DATA, 0, 8}, {LKAS_HUD, 0, 8}};

AddrCheckStruct wl_addr_checks[] = {
  {.msg = {{ENGINE_DATA, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{STEERING_LKAS, 2, 8, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{LKAS_HUD, 2, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},

};
// AddrCheckStruct wl_addr_checks[] = {};


#define WL_RX_CHECK_LEN (sizeof(wl_addr_checks) / sizeof(wl_addr_checks[0]))
addr_checks wl_rx_checks = {wl_addr_checks, WL_RX_CHECK_LEN};

static const addr_checks* wuling_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 1;
  relay_malfunction_reset();
  return &wl_rx_checks;
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static int wuling_rx_hook(CANPacket_t *to_send) {
  UNUSED(to_send);

  bool valid = addr_safety_check(to_send, &wl_rx_checks, NULL, NULL, NULL);
	// bool valid = true;

   if (valid && ((int)GET_BUS(to_send) == BUS_MAIN)) {
      int addr = GET_ADDR(to_send);

      if (addr == 842) {
        vehicle_moving = GET_BYTE(to_send, 0) | GET_BYTE(to_send, 1);
      }
      
      if (addr == 201) {
        brake_pressed = GET_BIT(to_send, 40U) != 0U;
      }

      if (addr == 0x191) {
        gas_pressed = GET_BYTE(to_send, 6) != 0U;
      }

      if ((addr == 0x263)) {
        acc_main_on = GET_BIT(to_send, 38U) != 0U;
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
    // bool is_lkas_msg = (addr == 0x225) || (addr == 0x373) || (addr == 0x370);
    // bool is_lkas_msg = (addr == 0x225) || (addr == 0x269) || (addr == 0x260) || (addr == 0x191) || (addr == 0x1c3);
    bool is_lkas_msg = (addr == 0x225);
    bool is_acc_hud_msg = addr == 0x30C;
    bool is_brake_msg = addr == 0x610;
    bool is_gas_msg = addr == 0x415;
    bool block_fwd = is_lkas_msg || is_acc_hud_msg || is_brake_msg || is_gas_msg;
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
