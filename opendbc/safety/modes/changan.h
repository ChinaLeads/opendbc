#pragma once

#include "opendbc/safety/declarations.h"

// Changan Z6 iDD safety mode.
//
// This mode only permits the messages openpilot is expected to send for the
// Z6 iDD ADAS path. Everything else is blocked, so a bug in the car port
// cannot push arbitrary frames onto the vehicle buses.
//
// Bus layout (matches opendbc/car/changan/values.py DBC map):
//   bus 0 : pt  (powertrain / EPS)   - steering command + EPS status
//   bus 2 : cam (camera / ACC)       - longitudinal + cruise status
//
// Signals derived from opendbc/dbc/changan.dbc.

// ---------------------------------------------------------------- helpers --

// CRC-8/J1850, same polynomial and init as changan_checksum() in changancan.py
static uint8_t changan_compute_crc(uint8_t crc) {
  for (int i = 0; i < 8; i++) {
    if ((crc & 0x80U) != 0U) {
      crc = (uint8_t)((crc << 1) ^ 0x1DU);
    } else {
      crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static uint32_t changan_get_checksum(const CANPacket_t *msg) {
  // ACC_CRCCheck_1BA / 17E / 244 live in the last byte, over the first 7
  return (uint32_t)msg->data[7];
}

static uint32_t changan_compute_checksum(const CANPacket_t *msg) {
  uint8_t crc = 0xFFU;
  for (int i = 0; i < 7; i++) {
    crc = changan_compute_crc((uint8_t)(crc ^ msg->data[i]));
  }
  return (uint32_t)(crc ^ 0xFFU);
}

static uint8_t changan_get_counter(const CANPacket_t *msg) {
  // rolling counter is the high nibble of byte 6 on all transmitted frames
  return (uint8_t)(msg->data[6] >> 4);
}

// ------------------------------------------------------------- rx (read) ---

static void changan_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Steering angle (SAS) - GW_180, 0.1 deg/LSB, signed
    if (msg->addr == 0x180U) {
      int angle_meas_new = to_signed((msg->data[1] << 8) | msg->data[0], 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // Driver steering torque - GW_24F, (0.1794, -22.78), signed 8 bit
    if (msg->addr == 0x24FU) {
      int torque = to_signed(msg->data[2], 8);
      update_sample(&torque_meas, torque);
    }
  }

  // Vehicle speed - GW_17A (iDD) / GW_187 (ICE), 0.05625 kph/LSB
  if (msg->addr == 0x17AU || msg->addr == 0x187U) {
    int speed = ((msg->data[5] << 8) | msg->data[4]) >> 3;
    speed &= 0x1FFFU;
    vehicle_moving = speed > 0;
    UPDATE_VEHICLE_SPEED(speed * 0.05625 * KPH_TO_MS);
  }

  // Brake pedal - GW_1A6 (iDD) / GW_196 (ICE), EMS_BrakePedalStatus @35|2
  if (msg->addr == 0x1A6U || msg->addr == 0x196U) {
    brake_pressed = ((msg->data[4] >> 3) & 0x3U) != 0U;
  }

  // Accelerator pedal - GW_1C6 (iDD) / GW_1A6 (ICE), 0.392 %/LSB
  if (msg->addr == 0x1C6U) {
    gas_pressed = msg->data[0] > 0U;
  } else if (msg->addr == 0x1A6U) {
    gas_pressed = msg->data[5] > 0U;
  }

  if (msg->bus == 2U) {
    // Stock ACC state - GW_244, ACC_ACCMode; 0=off, 2=standby, 3=active, 5=override
    if (msg->addr == 0x244U) {
      uint8_t acc_state = (uint8_t)((msg->data[1] >> 1) & 0x7U);
      bool acc_on = (acc_state == 3U) || (acc_state == 5U);
      pcm_cruise_check(acc_on);
    }
  }
}

// ------------------------------------------------------------- tx (write) --

static bool changan_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits CHANGAN_STEERING_LIMITS = {
    .max_angle = 7200,       // 720 deg (EPS_AngleCmd range is -720..720 with 0.1 deg/LSB)
    .angle_deg_to_can = 10,  // 0.1 deg/LSB
    .frequency = 50U,
  };

  const AngleSteeringParams CHANGAN_STEERING_PARAMS = {
    .slip_factor = 0.0,      // Z6 iDD runs an angle command; no torque slip model needed
    .steer_ratio = 14.5,     // matches CarSpecs in values.py
    .wheelbase = 2.795,
  };

  const LongitudinalLimits CHANGAN_LONG_LIMITS = {
    .max_accel = 2000,       // 2.0 m/s^2
    .min_accel = -3500,      // -3.5 m/s^2
    .inactive_accel = 0,
    .max_gas = 0,            // gas is not commanded on this platform
    .min_gas = 0,
    .inactive_gas = 0,
    .max_brake = 0,
  };

  bool tx = true;

  // ---- bus 0: steering ------------------------------------------------
  if (msg->addr == 0x1BAU) {
    int desired_angle = to_signed(((msg->data[3] << 8) | msg->data[2]) >> 1, 15);
    bool steer_req = ((msg->data[4] >> 3) & 0x1U) != 0U;

    if (steer_angle_cmd_checks_vm(desired_angle, steer_req,
                                  CHANGAN_STEERING_LIMITS, CHANGAN_STEERING_PARAMS)) {
      tx = false;
    }

    // Never let the port command a lateral torque beyond the EPS envelope
    int torque_max = to_signed(((msg->data[1] << 8) | msg->data[0]) & 0x7FFU, 11);
    if ((torque_max > 1050) || (torque_max < -1050)) {  // +-21 Nm raw (0.02 Nm/LSB)
      tx = false;
    }
  }

  // EPS lateral status mirror - GW_17E is informational only
  if (msg->addr == 0x17EU) {
    tx = true;
  }

  // ---- bus 2: longitudinal -------------------------------------------
  if (msg->addr == 0x244U) {
    int desired_accel = to_signed(msg->data[0], 8);
    if (longitudinal_accel_checks(desired_accel, CHANGAN_LONG_LIMITS)) {
      tx = false;
    }
  }

  // cruise / IACC status frames are display mirrors, allow them
  if (msg->addr == 0x307U || msg->addr == 0x31AU) {
    tx = true;
  }

  return tx;
}

// ------------------------------------------------------------------ init ---

static safety_config changan_init(uint16_t param) {
  static const CanMsg CHANGAN_TX_MSGS[] = {
    {0x1BA, 0, 32, .check_relay = true},   // GW_1BA  steering command
    {0x17E, 0,  8, .check_relay = true},   // GW_17E  EPS lateral status
    {0x244, 2, 32, .check_relay = true},   // GW_244  ACC target accel
    {0x307, 2,  8, .check_relay = false},  // GW_307  cruise status mirror
    {0x31A, 2,  8, .check_relay = false},  // GW_31A  IACC status mirror
  };

  static RxCheck changan_rx_checks[] = {
    // GW_180 steering angle, 0.1 deg
    {.msg = {{0x180, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // GW_17A vehicle speed (iDD)
    {.msg = {{0x17A, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {0x187, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},
    // GW_24F driver steering torque
    {.msg = {{0x24F, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  safety_config ret;
  SET_RX_CHECKS(changan_rx_checks, ret);
  SET_TX_MSGS(CHANGAN_TX_MSGS, ret);
  return ret;
}

const safety_hooks changan_hooks = {
  .init = changan_init,
  .rx = changan_rx_hook,
  .tx = changan_tx_hook,
  .get_checksum = changan_get_checksum,
  .compute_checksum = changan_compute_checksum,
  .get_counter = changan_get_counter,
};
