// -----------------------------------------------------------------------------
// File:        TugbotEngineer.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      Engineer (temps, water ingress, pumps)
// Revision:    1.0.1   // temp pins renamed to PIN_TEMP_MOTOR / PIN_TEMP_SPDCNTRL
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotEngineer.h"

// ADC reference (assume default 5V, 10-bit ADC)
static const float VREF_ADC   = 5.0f;
static const float ADC_COUNTS = 1023.0f;

TugbotEngineer::TugbotEngineer()
: _initialised(false),
  _lastUpdateMs(0),
  _bilgeHoldUntilMs(0),
  _bilgeLastTriggerMs(0),
  _coolManualEnabled(false),
  _coolManualOn(false),
  _bilgeManualEnabled(false),
  _bilgeManualOn(false)
{
    _state.temp1C        = 0.0f;
    _state.temp2C        = 0.0f;
    _state.hottestC      = 0.0f;
    _state.tempClass     = EngineerTempClass::NORMAL;
    _state.waterLevel    = 0.0f;
    _state.waterIngress  = false;
    _state.coolingPumpOn = false;
    _state.bilgePumpOn   = false;
}

void TugbotEngineer::begin(const EngineerConfig &cfg)
{
    _cfg          = cfg;
    _initialised  = true;
    _lastUpdateMs = millis();

    // Pump pins
    pinMode(PIN_PUMP_COOLING, OUTPUT);
    pinMode(PIN_PUMP_BILGE,   OUTPUT);

    digitalWrite(PIN_PUMP_COOLING, LOW);
    digitalWrite(PIN_PUMP_BILGE,   LOW);

    // First readings to seed state
    updateTemps();
    _state.waterLevel   = readWaterLevel();
    _state.waterIngress = (_state.waterLevel >= _cfg.waterWetThreshold);

    applyCoolingPumpToHardware();
    applyBilgePumpToHardware();
}

void TugbotEngineer::update(uint32_t nowMs)
{
    if (!_initialised)
    {
        return;
    }

    uint32_t dtMs = nowMs - _lastUpdateMs;
    if (dtMs < 50U)
    {
        // avoid hammering ADC; caller should schedule at 5–20 Hz
        return;
    }
    _lastUpdateMs = nowMs;

    updateTemps();
    updateWater(nowMs);

    updateCoolingPump();
    updateBilgePump(nowMs);

    applyCoolingPumpToHardware();
    applyBilgePumpToHardware();
}

// ---------------------- Manual overrides ------------------------------------

void TugbotEngineer::setCoolingPumpManual(bool enabled, bool on)
{
    _coolManualEnabled = enabled;
    _coolManualOn      = on;

    if (enabled)
    {
        _state.coolingPumpOn = on;
        applyCoolingPumpToHardware();
    }
}

void TugbotEngineer::setBilgePumpManual(bool enabled, bool on)
{
    _bilgeManualEnabled = enabled;
    _bilgeManualOn      = on;

    if (enabled)
    {
        _state.bilgePumpOn = on;
        applyBilgePumpToHardware();
    }
}

// ---------------------- Helpers: analog reads --------------------------------

float TugbotEngineer::readTempCFromAdc(uint8_t analogPin,
                                       float scale,
                                       float offset) const
{
    int raw = analogRead(analogPin);
    float volts = (raw * VREF_ADC / ADC_COUNTS);
    float tempC = volts * scale + offset;
    return tempC;
}

float TugbotEngineer::readWaterLevel() const
{
    int raw = analogRead(PIN_A_WATER);  // name preserved per your request
    float lv = raw / ADC_COUNTS;
    if (lv < 0.0f) lv = 0.0f;
    if (lv > 1.0f) lv = 1.0f;
    return lv;
}

// ---------------------- Update sub-parts ------------------------------------

void TugbotEngineer::updateTemps()
{
    // NEW NAMES:
    //   Motor temp:        PIN_TEMP_MOTOR
    //   Speed controller:  PIN_TEMP_SPDCNTRL
    _state.temp1C = readTempCFromAdc(PIN_TEMP_MOTOR,
                                     _cfg.temp1Scale,
                                     _cfg.temp1Offset);

    _state.temp2C = readTempCFromAdc(PIN_TEMP_SPDCNTRL,
                                     _cfg.temp2Scale,
                                     _cfg.temp2Offset);

    _state.hottestC = max(_state.temp1C, _state.temp2C);

    // Classification
    EngineerTempClass cls = EngineerTempClass::NORMAL;

    if (_state.hottestC >= _cfg.tempCritC)
    {
        cls = EngineerTempClass::CRIT;
    }
    else if (_state.hottestC >= _cfg.tempWarnC)
    {
        cls = EngineerTempClass::WARN;
    }

    _state.tempClass = cls;
}

void TugbotEngineer::updateWater(uint32_t nowMs)
{
    _state.waterLevel = readWaterLevel();

    bool wasWet = _state.waterIngress;
    bool isWet  = (_state.waterLevel >= _cfg.waterWetThreshold);

    _state.waterIngress = isWet;

    if (isWet && !wasWet)
    {
        // New ingress event -> schedule bilge run
        _bilgeLastTriggerMs = nowMs;
        _bilgeHoldUntilMs   = nowMs + (uint32_t)_cfg.bilgeMinRunMs;
    }
}

// Cooling pump: hysteresis based on hottest temp
void TugbotEngineer::updateCoolingPump()
{
    if (_coolManualEnabled)
    {
        _state.coolingPumpOn = _coolManualOn;
        return;
    }

    if (_state.coolingPumpOn)
    {
        // Currently ON -> wait until well below off threshold
        if (_state.hottestC <= _cfg.coolingOffC)
        {
            _state.coolingPumpOn = false;
        }
    }
    else
    {
        // Currently OFF -> turn on when above on threshold
        if (_state.hottestC >= _cfg.coolingOnC)
        {
            _state.coolingPumpOn = true;
        }
    }
}

// Bilge pump: run whenever ingress, ensure minimum run-time, and
// provide a small retrigger guard.
void TugbotEngineer::updateBilgePump(uint32_t nowMs)
{
    if (_bilgeManualEnabled)
    {
        _state.bilgePumpOn = _bilgeManualOn;
        return;
    }

    bool shouldRun = false;

    if (_state.waterIngress)
    {
        // If still getting wet, keep running and extend hold window a bit
        if (nowMs > _bilgeHoldUntilMs)
        {
            _bilgeHoldUntilMs = nowMs + (uint32_t)_cfg.bilgeMinRunMs;
        }

        shouldRun = true;
    }
    else
    {
        // No new ingress, but if we are within the hold window, keep running
        if (nowMs < _bilgeHoldUntilMs)
        {
            shouldRun = true;
        }
        else
        {
            shouldRun = false;
        }
    }

    _state.bilgePumpOn = shouldRun;
}

// ---------------------- Hardware application --------------------------------

void TugbotEngineer::applyCoolingPumpToHardware()
{
    digitalWrite(PIN_PUMP_COOLING,
                 _state.coolingPumpOn ? HIGH : LOW);
}

void TugbotEngineer::applyBilgePumpToHardware()
{
    digitalWrite(PIN_PUMP_BILGE,
                 _state.bilgePumpOn ? HIGH : LOW);
}
