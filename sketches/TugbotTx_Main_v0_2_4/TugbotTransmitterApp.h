// -----------------------------------------------------------------------------
// File:        TugbotTransmitterApp.h
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

#include "TugbotRadioLink.h"
#include "TugbotInputModel.h"
#include "TugbotUiOled.h"
#include "TugbotInputEncoder.h"
#include "TugbotWiFiOta.h"

class TugbotTransmitterApp
{
public:
    bool Begin();
    void Tick();

    void SetUiPage(uint8_t page);
    void SetMode(uint8_t mode);

private:
    void TickSerial();
    void HandleLine(String line);
    void PrintHelp() const;
    void PrintStatus() const;
    void SendCmd();

    static int ParseInt(const String& s, bool& ok);

private:
    TugbotWiFiOta _ota;

    TugbotRadioLink _link;
    TugbotInputModel _input;
    TugbotUiOled _ui;

    TugbotInputEncoder _enc1;
    TugbotInputEncoder _enc2;
    TugbotInputEncoder _enc3;

    uint8_t _uiPage = 0;

    uint32_t _seq = 0;
    uint32_t _lastSendMs = 0;
    uint16_t _sendIntervalMs = 100;

    String _rxLine;
};
