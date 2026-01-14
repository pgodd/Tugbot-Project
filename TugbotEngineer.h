// -----------------------------------------------------------------------------
// File:        TugbotEngineer.h
// Project:     Tugbot v1.0.0-beta
// Module:      Engineer (temps, water ingress, pumps)
// Revision:    1.0.1   // updated pin naming for temp sensors
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------
#ifndef TUGBOT_ENGINEER_H
#define TUGBOT_ENGINEER_H

#include <Arduino.h>
#include "pins_tugbot.h"
#include "tugbot_config_engineer.h"


enum class EngineerTempClass : uint8_t
{
    NORMAL = 0,
    WARN   = 1,
    CRIT   = 2
};

struct EngineerState
{
    // Temperatures
    float temp1C;
    float temp2C;
    float hottestC;
    EngineerTempClass tempClass;  // NORMAL / WARN / CRIT

    // Water ingress
    float waterLevel;   // 0..1 (normalised ADC)
    bool  waterIngress;

    // Pumps
    bool coolingPumpOn; // cooling pump state
    bool bilgePumpOn;   // bilge pump state
};

class TugbotEngineer
{
public:
    TugbotEngineer();

    // Provide configuration (or use gospel defaults if you like).
    void begin(const EngineerConfig &cfg);

    // Periodic update (e.g. 5–20 Hz).
    void update(uint32_t nowMs);

    const EngineerState &getState() const { return _state; }

    // --- Manual overrides ---------------------------------------------------
    // If enabled, the manual state wins over auto logic until disabled again.
    void setCoolingPumpManual(bool enabled, bool on);
    void setBilgePumpManual(bool enabled, bool on);

private:
    EngineerConfig _cfg;
    EngineerState  _state;
    bool           _initialised;

    uint32_t _lastUpdateMs;

    // Bilge auto-control bookkeeping
    uint32_t _bilgeHoldUntilMs;
    uint32_t _bilgeLastTriggerMs;

    // Manual overrides
    bool _coolManualEnabled;
    bool _coolManualOn;
    bool _bilgeManualEnabled;
    bool _bilgeManualOn;

    // Helpers
    float readTempCFromAdc(uint8_t analogPin,
                           float scale,
                           float offset) const;

    float readWaterLevel() const; // 0..1

    void  updateTemps();
    void  updateWater(uint32_t nowMs);
    void  updateCoolingPump();
    void  updateBilgePump(uint32_t nowMs);

    void  applyCoolingPumpToHardware();
    void  applyBilgePumpToHardware();
};

#endif // TUGBOT_ENGINEER_H
