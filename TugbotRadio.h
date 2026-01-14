// -----------------------------------------------------------------------------
// File:        TugbotRadio.h
// Project:     Tugbot v1.0.0-beta
// Module:      NRF24L01 radio wrapper + self-test
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// Responsibilities:
//   - Own the RF24 instance on the Tugbot Mega
//   - Perform a power-on self-test to confirm the NRF24 is present
//   - Expose a simple health flag for integration tests
// -----------------------------------------------------------------------------
// Dependencies:
//   - RF24 library (by TMRh20) via Library Manager
//   - pins_tugbot.h for CE / CSN pins
// -----------------------------------------------------------------------------

#ifndef TUGBOT_RADIO_H
#define TUGBOT_RADIO_H

#include <Arduino.h>
#include "pins_tugbot.h"

class TugbotRadio
{
public:
    TugbotRadio();

    // Call once from setup()
    void begin();

    // Run a quick self-test: returns true if the NRF appears present.
    // Safe to call multiple times; caches result.
    bool selfTest();

    // Accessors
    bool isPresent()  const { return _present; }
    bool isInitialised() const { return _initialised; }

private:
    bool _initialised;
    bool _present;
};

#endif // TUGBOT_RADIO_H
