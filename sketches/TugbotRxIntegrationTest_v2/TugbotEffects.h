// -----------------------------------------------------------------------------
// File:        TugbotEffects.h
// Project:     Tugbot v1.0.0-beta
// Module:      Effects (lights, MP3 SFX, smoke, engine sound placeholder)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// Responsibilities:
//   - On/off control for simple LED-based effects (deck/cabin/spot/pump)
//   - Command wrapper for YX5300-style serial MP3 module on Serial3
//   - Placeholders for smoke effect and engine sound PWM shaping
//   - Periodic update() for one-shot / timed effects
// -----------------------------------------------------------------------------

#ifndef TUGBOT_EFFECTS_H
#define TUGBOT_EFFECTS_H

#include <Arduino.h>
#include "tugbot_config_effects.h"

// Simple state for lights (on/off + optional blink later)
struct TugbotLightState
{
    TugbotLightId id;
    bool          enabled;
};

class TugbotEffects
{
public:
    TugbotEffects();

    // --- Setup --------------------------------------------------------------

    // Initialise lights, MP3 module, engine sound, smoke placeholders.
    // sfxSerial is typically Serial3 on the Mega.
    void begin(HardwareSerial& sfxSerial,
               uint32_t sfxBaud = 9600);

    // Call periodically (e.g. 10–50 Hz) for timed effects.
    void update(uint32_t nowMs);

    // --- Lights -------------------------------------------------------------

    void setLight(TugbotLightId id, bool on);
    void toggleLight(TugbotLightId id);
    bool getLight(TugbotLightId id) const;

    // Future: blink patterns, nav lights, etc.

    // --- MP3 SFX (YX5300-style) --------------------------------------------

    // Basic commands. Track numbers follow your SD layout
    // (e.g., folder 01, files 001.mp3, 002.mp3, ...).
    void sfxPlayTrack(uint8_t track);
    void sfxPlayHorn();
    void sfxPlayEngineIdle();
    void sfxPlayEngineRun();
    void sfxStop();
    void sfxSetVolume(uint8_t volume0to30);

    // --- Engine sound placeholder ------------------------------------------

    // Placeholder: you can feed a 0..1 "engine load" or throttle value.
    // Later this can modulate PWM on PIN_ENGINE_SOUND_PWM or crossfade tracks.
    void setEngineSoundLevel(float levelNorm); // 0..1
    float getEngineSoundLevel() const { return _engineSoundLevel; }

    // --- Smoke placeholder --------------------------------------------------

    // Placeholder for smoke intensity; at some point this will drive
    // a MOSFET or pump/heater combination.
    void setSmokeLevel(float levelNorm); // 0..1
    float getSmokeLevel() const { return _smokeLevel; }

private:
    // Lights
    TugbotLightConfig _lightCfg[ (uint8_t)TugbotLightId::COUNT ];
    TugbotLightState  _lightState[ (uint8_t)TugbotLightId::COUNT ];
    uint8_t           _lightCount;

    int8_t findLightIndex(TugbotLightId id) const;
    void   applyLightToHardware(int8_t idx);

    // MP3 SFX
    HardwareSerial*   _sfxSerial;
    bool              _sfxReady;
    TugbotSfxConfig   _sfxCfg;
    uint8_t           _sfxVolume;    // 0..30

    void sfxSendCommand(uint8_t cmd, uint16_t param);

    // Engine / smoke placeholders
    float             _engineSoundLevel;  // 0..1
    float             _smokeLevel;        // 0..1
};

#endif // TUGBOT_EFFECTS_H
