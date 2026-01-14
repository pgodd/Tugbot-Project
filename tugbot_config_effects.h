// -----------------------------------------------------------------------------
// File:        tugbot_config_effects.h
// Project:     Tugbot v1.0.0-beta
// Module:      Effects configuration (lights, SFX, smoke, engine sound)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// This file defines the canonical mapping between logical effects
// (deck lights, cabin lights, etc.) and the physical pins / SFX tracks.
// -----------------------------------------------------------------------------

#ifndef TUGBOT_CONFIG_EFFECTS_H
#define TUGBOT_CONFIG_EFFECTS_H

#include <Arduino.h>
#include "pins_tugbot.h"

// Logical light channels
enum class TugbotLightId : uint8_t
{
    DECK = 0,
    CABIN = 1,
    SPOT = 2,
    PUMP = 3,

    COUNT
};

struct TugbotLightConfig
{
    TugbotLightId id;
    uint8_t       pin;
    bool          activeHigh;
};

// Canonical light mapping
inline void tugbot_getDefaultLightConfig(TugbotLightConfig* outArray,
                                         uint8_t maxCount,
                                         uint8_t& outCount)
{
    if (!outArray || maxCount < (uint8_t)TugbotLightId::COUNT)
    {
        outCount = 0;
        return;
    }

    uint8_t idx = 0;

    outArray[idx++] = { TugbotLightId::DECK,  PIN_PWM_ACC_1, true };
    outArray[idx++] = { TugbotLightId::CABIN, PIN_PWM_ACC_2, true };
    outArray[idx++] = { TugbotLightId::SPOT,  PIN_PWM_ACC_3, true };
    outArray[idx++] = { TugbotLightId::PUMP,  PIN_PWM_ACC_4, true };

    outCount = idx;
}

// --- SFX (MP3) canonical mapping --------------------------------------------

struct TugbotSfxConfig
{
    uint8_t hornTrack;       // short horn blast
    uint8_t engineIdleTrack; // looping engine idle
    uint8_t engineRunTrack;  // looping engine under way
};

inline void tugbot_getDefaultSfxConfig(TugbotSfxConfig& cfg)
{
    // Folder / file numbering follows typical YX5300 convention:
    //   /01/001_xxx.mp3, /01/002_xxx.mp3, etc.
    cfg.hornTrack       = 1; // 001 in folder 01
    cfg.engineIdleTrack = 2; // 002
    cfg.engineRunTrack  = 3; // 003
}

#endif // TUGBOT_CONFIG_EFFECTS_H
