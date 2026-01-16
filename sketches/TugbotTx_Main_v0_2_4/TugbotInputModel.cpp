// -----------------------------------------------------------------------------
// File:        TugbotInputModel.cpp
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#include "TugbotInputModel.h"

int TugbotInputModel::ClampPct(int v)
{
    if (v < -100) return -100;
    if (v >  100) return 100;
    return v;
}

void TugbotInputModel::SetThrottlePct(int v) { _thrPct = ClampPct(v); }
void TugbotInputModel::SetRudderPct(int v)   { _rudPct = ClampPct(v); }
void TugbotInputModel::SetArmed(bool v)      { _armed = v; }
void TugbotInputModel::SetMode(uint8_t v)    { _mode = v; }

int TugbotInputModel::GetThrottlePct() const { return _thrPct; }
int TugbotInputModel::GetRudderPct() const   { return _rudPct; }
bool TugbotInputModel::GetArmed() const      { return _armed; }
uint8_t TugbotInputModel::GetMode() const    { return _mode; }
