// -----------------------------------------------------------------------------
// File:        TugbotEnergy.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      Energy management (batteries, currents, limits)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotEnergy.h"

// ADC reference and resolution (assume 5V default, 10-bit ADC)
static const float VREF_ADC   = 5.0f;
static const float ADC_COUNTS = 1023.0f;

TugbotEnergy::TugbotEnergy()
: _initialised(false),
  _lastUpdateMs(0),
  _filterAlpha(0.2f) // modest smoothing
{
    // Initialise state to something sane
    _state.vProp = 0.0f;
    _state.vSys  = 0.0f;
    _state.iProp = 0.0f;
    _state.pProp = 0.0f;

    _state.cls               = EnergyClass::OK;
    _state.throttleLimitNorm = 1.0f;
    _state.throttleLimitPct  = 100;
    _state.effectsAllowed    = true;
}

void TugbotEnergy::begin(const TugbotNvData &nv)
{
    _nv           = nv;    // copy
    _initialised  = true;
    _lastUpdateMs = millis();

    // First reading to seed filters
    float vProp = readVoltageCal(PIN_A_V_PROP, _nv.v_prop_scale, _nv.v_prop_offset);
    float vSys  = readVoltageCal(PIN_A_V_SYS,  _nv.v_sys_scale,  _nv.v_sys_offset);
    float iProp = readCurrentCal(PIN_A_CURRENT_PROP, _nv.i_prop_zero, _nv.i_prop_scale);

    _state.vProp = vProp;
    _state.vSys  = vSys;
    _state.iProp = iProp;
    _state.pProp = vProp * iProp;

    classify(vProp, vSys);
}

// Main update: sample sensors, classify, smooth, compute limits
void TugbotEnergy::update(uint32_t nowMs)
{
    if (!_initialised)
    {
        return;
    }

    uint32_t dtMs = nowMs - _lastUpdateMs;
    if (dtMs < 50U)
    {
        // Avoid hammering ADC and logic; caller should call at ~5–20 Hz anyway
        return;
    }

    _lastUpdateMs = nowMs;

    float vProp = readVoltageCal(PIN_A_V_PROP, _nv.v_prop_scale, _nv.v_prop_offset);
    float vSys  = readVoltageCal(PIN_A_V_SYS,  _nv.v_sys_scale,  _nv.v_sys_offset);
    float iProp = readCurrentCal(PIN_A_CURRENT_PROP, _nv.i_prop_zero, _nv.i_prop_scale);

    applySmoothing(vProp, vSys, iProp);
    _state.pProp = _state.vProp * _state.iProp;

    classify(_state.vProp, _state.vSys);
}

// ----------------------- Helpers --------------------------------------------

float TugbotEnergy::readVoltageCal(uint8_t analogPin,
                                   float scale,
                                   float offset) const
{
    int raw = analogRead(analogPin);
    float v = (raw * VREF_ADC / ADC_COUNTS);
    v = v * scale + offset;
    return v;
}

float TugbotEnergy::readCurrentCal(uint8_t analogPin,
                                   float zero,
                                   float scale) const
{
    int raw = analogRead(analogPin);
    float v = (raw * VREF_ADC / ADC_COUNTS);
    float amps = (v - zero) * scale;
    return amps;
}

void TugbotEnergy::applySmoothing(float vProp, float vSys, float iProp)
{
    float a = _filterAlpha;
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;

    _state.vProp = (1.0f - a) * _state.vProp + a * vProp;
    _state.vSys  = (1.0f - a) * _state.vSys  + a * vSys;
    _state.iProp = (1.0f - a) * _state.iProp + a * iProp;
}

void TugbotEnergy::classify(float vProp, float vSys)
{
    // Baseline: assume OK
    EnergyClass cls = EnergyClass::OK;

    // WARN thresholds
    bool warn =
        (vProp < _nv.v_prop_warn) ||
        (vSys  < _nv.v_sys_warn);

    // CRIT thresholds
    bool crit =
        (vProp < _nv.v_prop_crit) ||
        (vSys  < _nv.v_sys_crit);

    // DEAD (extra conservative): well below CRIT
    float minCrit = min(_nv.v_prop_crit, _nv.v_sys_crit);
    bool dead = (vProp < 0.9f * minCrit) || (vSys < 0.9f * minCrit);

    if (dead)
    {
        cls = EnergyClass::DEAD;
    }
    else if (crit)
    {
        cls = EnergyClass::CRIT;
    }
    else if (warn)
    {
        cls = EnergyClass::WARN;
    }
    else
    {
        cls = EnergyClass::OK;
    }

    _state.cls = cls;

    // Policy: map class -> throttle limit and effects permission
    switch (cls)
    {
        case EnergyClass::OK:
            _state.throttleLimitNorm = 1.0f;
            _state.effectsAllowed    = true;
            break;

        case EnergyClass::WARN:
            _state.throttleLimitNorm = 0.7f;  // gentle limit
            _state.effectsAllowed    = true;  // still allowed, but you may later choose to trim
            break;

        case EnergyClass::CRIT:
            _state.throttleLimitNorm = 0.3f;  // limp mode
            _state.effectsAllowed    = false; // shut off non-essential loads
            break;

        case EnergyClass::DEAD:
        default:
            _state.throttleLimitNorm = 0.0f;  // no propulsion
            _state.effectsAllowed    = false;
            break;
    }

    // Clamp and convert to percentage for convenience / telemetry
    if (_state.throttleLimitNorm < 0.0f)
        _state.throttleLimitNorm = 0.0f;
    if (_state.throttleLimitNorm > 1.0f)
        _state.throttleLimitNorm = 1.0f;

    _state.throttleLimitPct = (uint8_t)(_state.throttleLimitNorm * 100.0f + 0.5f);
}
