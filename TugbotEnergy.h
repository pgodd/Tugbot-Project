// -----------------------------------------------------------------------------
// File:        TugbotEnergy.h
// Project:     Tugbot v1.0.0-beta
// Module:      Energy management (batteries, currents, limits)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// Responsibilities:
//   - Read calibrated PROP & SYS voltages and PROP current
//   - Classify energy state: OK / WARN / CRIT / DEAD
//   - Provide a throttle limit factor for motion control
//   - Indicate whether "fun" effects (smoke, big SFX, etc.) are allowed
//   - Provide a compact EnergyState struct for telemetry/logging
// -----------------------------------------------------------------------------
// Dependencies:
//   - TugbotNvData (from TugbotNv.h) for calibration and thresholds
//   - pins_tugbot.h for analog pin assignments
// -----------------------------------------------------------------------------

#ifndef TUGBOT_ENERGY_H
#define TUGBOT_ENERGY_H

#include <Arduino.h>
#include "TugbotNv.h"
#include "pins_Tugbot.h"

// Energy state classification
enum class EnergyClass : uint8_t
{
    OK    = 0,   // Normal operation
    WARN  = 1,   // Lowish, start backing off
    CRIT  = 2,   // Critical, limp mode
    DEAD  = 3    // Essentially flat, stop propulsion
};

struct EnergyState
{
    // Raw-ish, but calibrated readings
    float vProp;        // V_PROP (propulsion pack)
    float vSys;         // V_SYS  (logic/accessory pack)
    float iProp;        // I_PROP (propulsion current, A)
    float pProp;        // PROP power (approx, W)

    // Classification
    EnergyClass cls;    // OK / WARN / CRIT / DEAD

    // Recommended limits/actions
    float   throttleLimitNorm; // 0..1 (multiply your throttle by this)
    uint8_t throttleLimitPct;  // 0..100
    bool    effectsAllowed;    // true if "fun" loads are allowed
};

class TugbotEnergy
{
public:
    TugbotEnergy();

    // Call once at startup with the loaded NVRAM structure.
    void begin(const TugbotNvData &nv);

    // Call periodically in loop (e.g. 5–10 Hz).
    void update(uint32_t nowMs);

    const EnergyState &getState() const { return _state; }

    // Convenience helpers
    float   getThrottleLimitNorm() const { return _state.throttleLimitNorm; }
    uint8_t getThrottleLimitPct()  const { return _state.throttleLimitPct; }
    bool    areEffectsAllowed()    const { return _state.effectsAllowed; }
    EnergyClass getClass()         const { return _state.cls; }

private:
    TugbotNvData _nv;            // Copy of NVRAM config (cal & thresholds)
    EnergyState  _state;
    bool         _initialised;
    uint32_t     _lastUpdateMs;

    // Simple exponential smoothing factor (0..1)
    float _filterAlpha;

    // Helpers
    float readVoltageCal(uint8_t analogPin,
                         float scale,
                         float offset) const;

    float readCurrentCal(uint8_t analogPin,
                         float zero,
                         float scale) const;

    void  classify(float vProp, float vSys);
    void  applySmoothing(float vProp, float vSys, float iProp);
};

#endif // TUGBOT_ENERGY_H
