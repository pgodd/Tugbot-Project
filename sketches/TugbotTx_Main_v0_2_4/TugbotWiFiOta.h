// -----------------------------------------------------------------------------
// File:        TugbotWiFiOta.h
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

class TugbotWiFiOta
{
public:
    bool Begin();
    void Tick(uint32_t nowMs);

    bool IsStaConnected() const { return _staConnected; }
    bool IsApActive() const { return _apActive; }

private:
    bool BeginWiFi();
    void BeginOta();

private:
    bool _staConnected = false;
    bool _apActive = false;
    uint32_t _lastPrintMs = 0;
};
