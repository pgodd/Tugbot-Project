// -----------------------------------------------------------------------------
// File:        TugbotEffects.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      Effects (lights, MP3 SFX, smoke, engine sound placeholder)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotEffects.h"
#include "pins_tugbot.h"

// YX5300 / Catalex MP3 command IDs (see datasheet)
static const uint8_t CMD_NEXT        = 0x01;
static const uint8_t CMD_PREV        = 0x02;
static const uint8_t CMD_PLAY_INDEX  = 0x03;
static const uint8_t CMD_SET_VOLUME  = 0x06;
static const uint8_t CMD_SEL_DEV     = 0x09;
static const uint8_t CMD_STOP        = 0x16;
static const uint8_t DEV_TF          = 0x02;  // TF card

TugbotEffects::TugbotEffects()
: _lightCount(0),
  _sfxSerial(nullptr),
  _sfxReady(false),
  _sfxVolume(20),
  _engineSoundLevel(0.0f),
  _smokeLevel(0.0f)
{
}

void TugbotEffects::begin(HardwareSerial& sfxSerial,
                          uint32_t sfxBaud)
{
    // Lights
    tugbot_getDefaultLightConfig(_lightCfg,
                                 (uint8_t)TugbotLightId::COUNT,
                                 _lightCount);

    for (uint8_t i = 0; i < _lightCount; ++i)
    {
        _lightState[i].id      = _lightCfg[i].id;
        _lightState[i].enabled = false;

        pinMode(_lightCfg[i].pin, OUTPUT);
        digitalWrite(_lightCfg[i].pin,
                     _lightCfg[i].activeHigh ? LOW : HIGH);
    }

    // SFX
    tugbot_getDefaultSfxConfig(_sfxCfg);

    _sfxSerial = &sfxSerial;
    _sfxSerial->begin(sfxBaud);
    delay(500); // allow module to boot

    // Select TF card as device
    sfxSendCommand(CMD_SEL_DEV, DEV_TF);
    delay(200);

    sfxSetVolume(_sfxVolume);
    _sfxReady = true;

    // Engine sound PWM placeholder
    pinMode(PIN_ENGINE_SOUND_PWM, OUTPUT);
    analogWrite(PIN_ENGINE_SOUND_PWM, 0);

    // Smoke placeholder: for now we just reserve one ACC line
    pinMode(PIN_PWM_ACC_4, OUTPUT); // if not already used differently
    digitalWrite(PIN_PWM_ACC_4, LOW);
}

void TugbotEffects::update(uint32_t /*nowMs*/)
{
    // Currently nothing time-based; hook for:
    //  - one-shot horn debounce
    //  - nav light patterns
    //  - engine sound PWM shaping
    //  - smoke warmup / cooldown timing

    // Placeholder example: simple linear PWM for engine sound
    uint8_t pwm = (uint8_t)constrain(_engineSoundLevel * 255.0f, 0.0f, 255.0f);
    analogWrite(PIN_ENGINE_SOUND_PWM, pwm);

    // Placeholder: smoke level mapped to ACC_4
    uint8_t smokePwm = (uint8_t)constrain(_smokeLevel * 255.0f, 0.0f, 255.0f);
    analogWrite(PIN_PWM_ACC_4, smokePwm);
}

// ----------------- Lights ---------------------------------------------------

int8_t TugbotEffects::findLightIndex(TugbotLightId id) const
{
    for (uint8_t i = 0; i < _lightCount; ++i)
    {
        if (_lightState[i].id == id)
        {
            return (int8_t)i;
        }
    }
    return -1;
}

void TugbotEffects::applyLightToHardware(int8_t idx)
{
    if (idx < 0 || idx >= (int8_t)_lightCount)
    {
        return;
    }

    const TugbotLightConfig& cfg = _lightCfg[idx];
    const TugbotLightState&  st  = _lightState[idx];

    bool on = st.enabled;
    uint8_t level;

    if (cfg.activeHigh)
    {
        level = on ? HIGH : LOW;
    }
    else
    {
        level = on ? LOW : HIGH;
    }

    digitalWrite(cfg.pin, level);
}

void TugbotEffects::setLight(TugbotLightId id, bool on)
{
    int8_t idx = findLightIndex(id);
    if (idx < 0)
    {
        return;
    }

    _lightState[idx].enabled = on;
    applyLightToHardware(idx);
}

void TugbotEffects::toggleLight(TugbotLightId id)
{
    int8_t idx = findLightIndex(id);
    if (idx < 0)
    {
        return;
    }

    _lightState[idx].enabled = !_lightState[idx].enabled;
    applyLightToHardware(idx);
}

bool TugbotEffects::getLight(TugbotLightId id) const
{
    int8_t idx = findLightIndex(id);
    if (idx < 0)
    {
        return false;
    }

    return _lightState[idx].enabled;
}

// ----------------- MP3 SFX --------------------------------------------------

void TugbotEffects::sfxSendCommand(uint8_t cmd, uint16_t param)
{
    if (!_sfxSerial)
    {
        return;
    }

    // YX5300 8-byte frame:
    // 0: 0x7E
    // 1: 0xFF
    // 2: 0x06
    // 3: CMD
    // 4: 0x00 (no feedback)
    // 5: PARAM_H
    // 6: PARAM_L
    // 7: 0xEF
    uint8_t buf[8];

    buf[0] = 0x7E;
    buf[1] = 0xFF;
    buf[2] = 0x06;
    buf[3] = cmd;
    buf[4] = 0x00;
    buf[5] = (uint8_t)((param >> 8) & 0xFF);
    buf[6] = (uint8_t)(param & 0xFF);
    buf[7] = 0xEF;

    _sfxSerial->write(buf, sizeof(buf));
    _sfxSerial->flush();
    delay(20); // conservative guard between commands
}

void TugbotEffects::sfxPlayTrack(uint8_t track)
{
    if (!_sfxReady)
    {
        return;
    }

    // play index: param = track number (1..255)
    sfxSendCommand(CMD_PLAY_INDEX, track);
}

void TugbotEffects::sfxPlayHorn()
{
    sfxPlayTrack(_sfxCfg.hornTrack);
}

void TugbotEffects::sfxPlayEngineIdle()
{
    sfxPlayTrack(_sfxCfg.engineIdleTrack);
}

void TugbotEffects::sfxPlayEngineRun()
{
    sfxPlayTrack(_sfxCfg.engineRunTrack);
}

void TugbotEffects::sfxStop()
{
    if (!_sfxReady)
    {
        return;
    }

    sfxSendCommand(CMD_STOP, 0x0000);
}

void TugbotEffects::sfxSetVolume(uint8_t volume0to30)
{
    if (!_sfxSerial)
    {
        return;
    }

    if (volume0to30 > 30)
    {
        volume0to30 = 30;
    }

    _sfxVolume = volume0to30;
    sfxSendCommand(CMD_SET_VOLUME, _sfxVolume);
}

// ----------------- Engine / smoke placeholders ------------------------------

void TugbotEffects::setEngineSoundLevel(float levelNorm)
{
    if (levelNorm < 0.0f)
    {
        levelNorm = 0.0f;
    }
    else if (levelNorm > 1.0f)
    {
        levelNorm = 1.0f;
    }

    _engineSoundLevel = levelNorm;

    // Future:
    //   - crossfade idle/run tracks
    //   - or modulate PWM spectrum
}

void TugbotEffects::setSmokeLevel(float levelNorm)
{
    if (levelNorm < 0.0f)
    {
        levelNorm = 0.0f;
    }
    else if (levelNorm > 1.0f)
    {
        levelNorm = 1.0f;
    }

    _smokeLevel = levelNorm;

    // Future:
    //   - apply hysteresis to heaters
    //   - pump pulses vs. steady flow
}
