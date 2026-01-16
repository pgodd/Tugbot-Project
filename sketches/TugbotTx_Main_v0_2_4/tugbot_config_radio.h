// -----------------------------------------------------------------------------
// File:        tugbot_config_radio.h
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include <RF24.h>

// Canon NRF settings (TX side)
static const uint8_t NRF_ADDR_RX[6] = "TBOT1"; // TX writes to RX pipe
static const uint8_t NRF_ADDR_TX[6] = "TBOT2"; // optional read pipe

static const uint8_t NRF_CHANNEL = 108;
static const rf24_datarate_e NRF_DATARATE = RF24_1MBPS;
static const rf24_crclength_e NRF_CRC = RF24_CRC_16;
static const rf24_pa_dbm_e NRF_PA_LEVEL = RF24_PA_LOW;

static const bool NRF_AUTO_ACK = true;
static const bool NRF_ACK_PAYLOAD = true;
static const uint8_t NRF_RETRY_DELAY = 5;
static const uint8_t NRF_RETRY_COUNT = 15;

// Canon: delay after Serial.begin() so output appears reliably
static const uint32_t NRF_SERIAL_SETTLE_MS = 2000;
