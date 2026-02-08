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
inline constexpr uint8_t NRF_ADDR_TX_BYTES[5] = {'T','B','O','T','2'};
inline constexpr uint8_t NRF_CHANNEL          = 109;
inline constexpr rf24_datarate_e  NRF_DATARATE = RF24_2MBPS;
inline constexpr uint8_t NRF_BANDWIDTH_MHZ    = 20;
inline constexpr rf24_crclength_e NRF_CRC      = RF24_CRC_16;
inline constexpr rf24_pa_dbm_e    NRF_PA_LEVEL = RF24_PA_MIN;

inline constexpr bool    NRF_AUTO_ACK    = true;
inline constexpr uint8_t NRF_RETRY_DELAY = 5;
inline constexpr uint8_t NRF_RETRY_COUNT = 15;

// FINAL: fixed payload only (no dynamic payloads, no ack payloads)
inline constexpr bool NRF_USE_DYNAMIC_PAYLOADS = true;
inline constexpr bool NRF_USE_ACK_PAYLOAD      = true;

inline constexpr uint8_t NRF_FIXED_PAYLOAD_BYTES = (uint8_t)sizeof(NrfCmdFrame);

static_assert(NRF_DATARATE == RF24_2MBPS, "NRF_DATARATE must be 2MBPS for 20MHz bandwidth");

#endif // TUGBOT_CONFIG_RADIO_H
