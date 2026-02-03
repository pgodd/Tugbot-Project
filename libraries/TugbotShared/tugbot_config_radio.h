// -----------------------------------------------------------------------------
// File:        tugbot_config_radio.h
// Project:     Tugbot
// Role:        SHARED (TX/RX)
// Target:      Common
// -----------------------------------------------------------------------------

#ifndef TUGBOT_CONFIG_RADIO_H
#define TUGBOT_CONFIG_RADIO_H

#include <Arduino.h>
#include <RF24.h>
#include "tugbot_cmd_frames.h"

// RF settings (shared TX/RX)
inline constexpr uint8_t NRF_ADDR_RX_BYTES[5] = {'T','B','O','T','1'};
inline constexpr uint8_t NRF_CHANNEL          = 76;
inline constexpr rf24_datarate_e  NRF_DATARATE = RF24_1MBPS;
inline constexpr rf24_crclength_e NRF_CRC      = RF24_CRC_16;
inline constexpr rf24_pa_dbm_e    NRF_PA_LEVEL = RF24_PA_MIN;

inline constexpr bool    NRF_AUTO_ACK    = true;
inline constexpr uint8_t NRF_RETRY_DELAY = 5;
inline constexpr uint8_t NRF_RETRY_COUNT = 15;

inline constexpr uint32_t NRF_SERIAL_SETTLE_MS = 2000;

// FINAL: fixed payload only (no dynamic payloads, no ack payloads)
inline constexpr bool NRF_USE_DYNAMIC_PAYLOADS = true;
inline constexpr bool NRF_USE_ACK_PAYLOAD      = true;

inline constexpr uint8_t NRF_FIXED_PAYLOAD_BYTES = (uint8_t)sizeof(NrfCmdFrame);

#endif // TUGBOT_CONFIG_RADIO_H
