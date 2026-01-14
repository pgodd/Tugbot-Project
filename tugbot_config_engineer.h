// -----------------------------------------------------------------------------
// File:        tugbot_config_engineer.h
// Project:     Tugbot v1.0.0-beta
// Module:      Engineer (safety) configuration
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// Responsibilities:
//   - Define thresholds and basic scaling for:
//       * Temperature sensors (TEMP_1, TEMP_2)
//       * Water ingress sensor
//       * Cooling pump and bilge pump behaviour
//   - Provide a canonical default config ("gospel") for the TugbotEngineer
// -----------------------------------------------------------------------------

#ifndef TUGBOT_CONFIG_ENGINEER_H
#define TUGBOT_CONFIG_ENGINEER_H

#include <Arduino.h>
#include "pins_tugbot.h"

// If you want explicit names for pump pins, you can define these earlier.
// Otherwise we alias two accessory PWM outputs here.
#ifndef PIN_PUMP_COOLING
#define PIN_PUMP_COOLING PIN_PWM_ACC_3   // assume MOSFET channel 3
#endif

#ifndef PIN_PUMP_BILGE
#define PIN_PUMP_BILGE   PIN_PWM_ACC_4   // assume MOSFET channel 4
#endif

// Configuration for the engineer / safety subsystem
struct EngineerConfig
{
    // --- Temperature conversion (very simple linear placeholder) -----------
    // tempC = adcVolts * scale + offset
    float temp1Scale;
    float temp1Offset;
    float temp2Scale;
    float temp2Offset;

    // --- Temperature thresholds (deg C) ------------------------------------
    float tempWarnC;         // above this, issue WARN
    float tempCritC;         // above this, issue CRIT

    // Cooling pump hysteresis
    float coolingOnC;        // hottest >= this -> pump ON
    float coolingOffC;       // hottest <= this -> pump OFF

    // --- Water sensor thresholds -------------------------------------------
    // waterLevel is normalised 0..1 from ADC.
    float waterWetThreshold; // >= this -> waterIngress = true

    // Bilge pump behaviour
    float bilgeMinRunMs;     // keep running at least this long once triggered
    float bilgeRetriggerMs;  // minimum time between automatic re-triggers
};

// Canonical default values for the TugbotEngineer
inline void tugbot_getDefaultEngineerConfig(EngineerConfig &cfg)
{
    // These are deliberately conservative placeholders.
    // You can tune them after in-hull testing.

    // If your temp sensor is NTC + divider, these values won't be
    // physically accurate, but they'll still give you a monotonic
    // "higher ADC -> higher C" behaviour for relative safety.
    cfg.temp1Scale  = 100.0f;  // volts * 100 -> ~0..500 C
    cfg.temp1Offset = 0.0f;
    cfg.temp2Scale  = 100.0f;
    cfg.temp2Offset = 0.0f;

    cfg.tempWarnC   = 60.0f;   // WARN above ~60C
    cfg.tempCritC   = 75.0f;   // CRIT above ~75C

    cfg.coolingOnC  = 55.0f;   // turn cooling pump on around 55C
    cfg.coolingOffC = 45.0f;   // turn it off once under ~45C

    cfg.waterWetThreshold = 0.20f; // ~20% of ADC range means "wet"

    cfg.bilgeMinRunMs     = 10000.0f; // 10s minimum run per trigger
    cfg.bilgeRetriggerMs  = 5000.0f;  // no auto retrigger more often than 5s
}

#endif // TUGBOT_CONFIG_ENGINEER_H
