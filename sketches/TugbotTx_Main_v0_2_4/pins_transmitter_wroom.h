// -----------------------------------------------------------------------------
// File:        pins_transmitter_wroom.h
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Tugbot TX (ESP32-WROOM) - CANON PIN MAP
// -----------------------------------------------------------------------------

// NRF24 (VSPI)
static const uint8_t PIN_SPI_SCK  = 18;
static const uint8_t PIN_SPI_MISO = 19;
static const uint8_t PIN_SPI_MOSI = 23;

static const uint8_t PIN_NRF_CE   = 27;
static const uint8_t PIN_NRF_CSN  = 5;

// OLED 0.96" (hardwired I2C)
static const uint8_t PIN_I2C_SDA  = 21;
static const uint8_t PIN_I2C_SCL  = 22;

// Rotary encoders (KEYES/KY-040)
static const uint8_t PIN_ENC1_A   = 32; // CLK
static const uint8_t PIN_ENC1_B   = 33; // DT
static const uint8_t PIN_ENC1_BTN = 25; // SW

static const uint8_t PIN_ENC2_A   = 26; // CLK
static const uint8_t PIN_ENC2_B   = 14; // DT
static const uint8_t PIN_ENC2_BTN = 13; // SW

static const uint8_t PIN_ENC3_A   = 16; // CLK
static const uint8_t PIN_ENC3_B   = 17; // DT
static const uint8_t PIN_ENC3_BTN = 4;  // SW
