#pragma once
#include <Arduino.h>

static const uint8_t PIN_NRF_CE  = 48;
static const uint8_t PIN_NRF_CSN = 49;

// AVR SPI canon: keep SPI master mode
static inline void AvrSpiForceMaster()
{
    pinMode(53, OUTPUT);
    digitalWrite(53, HIGH);
}
