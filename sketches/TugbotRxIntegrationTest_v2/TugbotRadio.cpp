// -----------------------------------------------------------------------------
// File:        TugbotRadio.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      NRF24L01 radio wrapper + self-test
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotRadio.h"
#include <SPI.h>
#include <RF24.h>

// Single global RF24 instance for this translation unit.
// Uses canonical pins from pins_tugbot.h: CE=48, CSN=49
static RF24 radio(PIN_NRF24_CE, PIN_NRF24_CSN);

TugbotRadio::TugbotRadio()
: _initialised(false),
  _present(false)
{
}

void TugbotRadio::begin()
{
    if (_initialised)
    {
        return;
    }

    // SPI is started inside RF24::begin()
    radio.begin();

    // Conservative basic config; we aren't actually sending data yet.
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108); // away from WiFi, arbitrary but sane

    // Avoid auto-ack & payload complexity for now
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();

    _initialised = true;

    // Run an implicit self-test so isPresent() is meaningful immediately.
    selfTest();
}

// -----------------------------------------------------------------------------
// Self-test
// -----------------------------------------------------------------------------

bool TugbotRadio::selfTest()
{
    // Ensure radio is initialised
    if (!_initialised)
    {
        begin();
    }

    // Use the public RF24 API, not read_register()
    bool ok = radio.isChipConnected();

    _present = ok;
    return ok;
}
