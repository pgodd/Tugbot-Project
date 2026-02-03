// -----------------------------------------------------------------------------
// File:        TugbotRadioLink.h
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include <RF24.h>

#include "pins_transmitter_wroom.h"
#include <tugbot_config_radio.h>
#include <tugbot_cmd_frames.h>

class TugbotRadioLink
{
public:
    struct Stats
    {
        uint32_t ok = 0;
        uint32_t fail = 0;
        uint32_t ackPayload = 0;
        bool chipConnected = false;
    };

public:
    TugbotRadioLink();
    bool Begin();
    bool SendCmd(const NrfCmdFrame& frame);
    const Stats& GetStats() const;

private:
    void ApplyCanonConfig();
    void DrainAckPayloadIfAny();

private:
    RF24  _radio;
    Stats _stats;
};
