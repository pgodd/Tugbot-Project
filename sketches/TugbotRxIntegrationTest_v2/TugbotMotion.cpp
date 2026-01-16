// -----------------------------------------------------------------------------
// File:        TugbotMotion.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      Motion & Steering object implementation
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotMotion.h"

// ---------------- CONSTRUCTION ----------------------------------------------

TugbotMotion::TugbotMotion()
: _motionCfg(nullptr),
  _profiles(nullptr),
  _profileCount(0),
  _activeProfile(0),
  _throttleTargetNorm(0.0f),
  _throttleActualNorm(0.0f),
  _rudderTargetDeg(0.0f),
  _rudderActualDeg(0.0f),
  _lastUpdateMs(0),
  _reverseCoastUntilMs(0)
{
}

// ---------------- BASIC SETUP -----------------------------------------------

void TugbotMotion::setMotionConfig(const MotionConfig *cfg)
{
    _motionCfg = cfg;
}

void TugbotMotion::setSteeringProfiles(const SteeringProfileConfig *profiles,
                                       uint8_t profileCount)
{
    _profiles      = profiles;
    _profileCount  = profileCount;

    if (_activeProfile >= _profileCount)
    {
        _activeProfile = 0;
    }
}

void TugbotMotion::setActiveProfile(uint8_t idx)
{
    if (_profileCount == 0)
    {
        return;
    }

    if (idx >= _profileCount)
    {
        idx = 0;
    }

    _activeProfile = idx;
}

uint8_t TugbotMotion::getActiveProfile() const
{
    return _activeProfile;
}

void TugbotMotion::begin(uint32_t nowMs)
{
    _lastUpdateMs        = nowMs;
    _reverseCoastUntilMs = nowMs;
    _throttleTargetNorm  = 0.0f;
    _throttleActualNorm  = 0.0f;
    _rudderTargetDeg     = 0.0f;
    _rudderActualDeg     = 0.0f;
}

const SteeringProfileConfig *TugbotMotion::currentProfile() const
{
    if (_profiles == nullptr || _profileCount == 0)
    {
        return nullptr;
    }

    if (_activeProfile >= _profileCount)
    {
        return &_profiles[0];
    }

    return &_profiles[_activeProfile];
}

// ---------------- HELPERS ---------------------------------------------------

float TugbotMotion::clamp(float v, float lo, float hi) const
{
    if (v < lo)
    {
        return lo;
    }

    if (v > hi)
    {
        return hi;
    }

    return v;
}

// ---------------- MAIN UPDATE ----------------------------------------------

void TugbotMotion::update(float inputThrottleNorm,
                          float inputSteerNorm,
                          uint32_t nowMs)
{
    if (_motionCfg == nullptr || _profiles == nullptr || _profileCount == 0)
    {
        // Unconfigured fallback
        _throttleTargetNorm = clamp(inputThrottleNorm, -1.0f, 1.0f);
        _throttleActualNorm = _throttleTargetNorm;
        _rudderTargetDeg    = 0.0f;
        _rudderActualDeg    = 0.0f;
        _lastUpdateMs       = nowMs;
        return;
    }

    uint32_t dtMs = nowMs - _lastUpdateMs;

    if (dtMs > 1000U)
    {
        dtMs = 1000U; // avoid huge jumps if timing goes odd
    }

    float dtSec = dtMs / 1000.0f;

    updateThrottle(inputThrottleNorm, dtSec, nowMs);
    updateRudder(inputSteerNorm, dtSec);

    _lastUpdateMs = nowMs;
}

// ---------------- THROTTLE LOGIC -------------------------------------------

void TugbotMotion::updateThrottle(float inNorm,
                                  float dtSec,
                                  uint32_t nowMs)
{
    float lim = 1.0f;

    if (_motionCfg != nullptr)
    {
        lim = clamp(_motionCfg->throttle_limit_normal, 0.0f, 1.0f);
    }

    float target = clamp(inNorm, -lim, lim);
    _throttleTargetNorm = target;

    // Reverse coast
    if (_motionCfg != nullptr && _motionCfg->reverse_coast_ms > 0.0f)
    {
        bool signFlip =
            (_throttleActualNorm > 0.0f && target < 0.0f) ||
            (_throttleActualNorm < 0.0f && target > 0.0f);

        if (signFlip)
        {
            _reverseCoastUntilMs = nowMs + (uint32_t)_motionCfg->reverse_coast_ms;
        }

        if (nowMs < _reverseCoastUntilMs)
        {
            target = 0.0f;
        }
    }

    // Ramp
    float rampUp  = 100.0f;
    float rampDown = 100.0f;

    if (_motionCfg != nullptr)
    {
        rampUp   = max(0.0f, _motionCfg->throttle_ramp_up_percent_s);
        rampDown = max(0.0f, _motionCfg->throttle_ramp_down_percent_s);
    }

    float maxUp   = (rampUp / 100.0f) * dtSec;
    float maxDown = (rampDown / 100.0f) * dtSec;

    float diff = target - _throttleActualNorm;

    if (diff > 0.0f)
    {
        if (diff > maxUp)
        {
            diff = maxUp;
        }
    }
    else if (diff < 0.0f)
    {
        if (-diff > maxDown)
        {
            diff = -maxDown;
        }
    }

    _throttleActualNorm = clamp(_throttleActualNorm + diff, -1.0f, 1.0f);
}

// ---------------- STEERING LOGIC -------------------------------------------

float TugbotMotion::applySteerDeadbandExpo(const SteeringProfileConfig &p,
                                           float inputNorm) const
{
    float x = clamp(inputNorm, -1.0f, 1.0f);

    // Deadband around zero
    if (fabs(x) < p.manual_steer_deadband_in)
    {
        x = 0.0f;
    }
    else
    {
        float sign  = (x >= 0.0f) ? 1.0f : -1.0f;
        float mag   = fabs(x);
        float db    = p.manual_steer_deadband_in;
        float scale = 1.0f / (1.0f - db);

        mag = (mag - db) * scale;
        mag = clamp(mag, 0.0f, 1.0f);

        x = sign * mag;
    }

    // Expo (RC-style)
    float e = clamp(p.manual_steer_expo, 0.0f, 1.0f);

    float out = x * (1.0f - e) + x * x * x * e;

    return out;
}

void TugbotMotion::updateRudder(float inNorm,
                                float dtSec)
{
    const SteeringProfileConfig *p = currentProfile();

    if (p == nullptr)
    {
        _rudderTargetDeg = 0.0f;
        _rudderActualDeg = 0.0f;
        return;
    }

    float x = applySteerDeadbandExpo(*p, inNorm);

    float maxCmd = fabs(p->manual_steer_max_deg);

    if (maxCmd <= 0.001f)
    {
        _rudderTargetDeg = 0.0f;
        _rudderActualDeg = 0.0f;
        return;
    }

    float target = p->rudder_center_offset_deg + x * maxCmd;

    if (target < p->rudder_max_port_deg)
    {
        target = p->rudder_max_port_deg;
    }

    if (target > p->rudder_max_starboard_deg)
    {
        target = p->rudder_max_starboard_deg;
    }

    _rudderTargetDeg = target;

    float maxRate = max(0.0f, p->manual_steer_rate_deg_s);
    float maxStep = maxRate * dtSec;

    float diff = _rudderTargetDeg - _rudderActualDeg;

    if (diff > maxStep)
    {
        diff = maxStep;
    }
    else if (diff < -maxStep)
    {
        diff = -maxStep;
    }

    _rudderActualDeg += diff;

    if (fabs(_rudderActualDeg) < p->rudder_mech_deadzone_deg)
    {
        _rudderActualDeg = 0.0f;
    }
}

// ---------------- ACCESSORS -------------------------------------------------

float TugbotMotion::getThrottleTargetNorm() const
{
    return _throttleTargetNorm;
}

float TugbotMotion::getThrottleActualNorm() const
{
    return _throttleActualNorm;
}

float TugbotMotion::getRudderTargetDeg() const
{
    return _rudderTargetDeg;
}

float TugbotMotion::getRudderActualDeg() const
{
    return _rudderActualDeg;
}

uint8_t TugbotMotion::getThrottlePwm255() const
{
    float v = clamp(_throttleActualNorm, -1.0f, 1.0f);
    float pwm = (v + 1.0f) * 0.5f * 255.0f;

    if (pwm < 0.0f)
    {
        pwm = 0.0f;
    }

    if (pwm > 255.0f)
    {
        pwm = 255.0f;
    }

    return (uint8_t)(pwm + 0.5f);
}

float TugbotMotion::getRudderNormFromDeg() const
{
    const SteeringProfileConfig *p = currentProfile();

    if (p == nullptr)
    {
        return 0.0f;
    }

    float maxCmd = fabs(p->manual_steer_max_deg);

    if (maxCmd <= 0.001f)
    {
        return 0.0f;
    }

    float cmd  = _rudderActualDeg - p->rudder_center_offset_deg;
    float norm = cmd / maxCmd;

    return clamp(norm, -1.0f, 1.0f);
}

// ---------------- SELF TESTS ------------------------------------------------

void TugbotMotion::runSelfTests(Stream &out)
{
    out.println(F("=== TugbotMotion Self-Test Start ==="));

    int passCount = 0;
    int failCount = 0;

    auto check =
        [&](const char *name, float got, float expected, float tol)
        {
            float err = fabs(got - expected);

            if (err <= tol)
            {
                out.print(F("[PASS] "));
                passCount++;
            }
            else
            {
                out.print(F("[FAIL] "));
                failCount++;
            }

            out.print(name);
            out.print(F(" got="));
            out.print(got, 4);
            out.print(F(" expected="));
            out.print(expected, 4);
            out.print(F(" tol="));
            out.println(tol, 4);
        };

    // Motion config
    MotionConfig mc;
    mc.throttle_limit_normal        = 1.0f;
    mc.throttle_ramp_up_percent_s   = 20.0f;
    mc.throttle_ramp_down_percent_s = 20.0f;
    mc.reverse_coast_ms             = 0.0f;

    SteeringProfileConfig sp[1];

    sp[0].rudder_center_offset_deg = 0.0f;
    sp[0].rudder_max_port_deg      = -35.0f;
    sp[0].rudder_max_starboard_deg =  35.0f;
    sp[0].rudder_mech_deadzone_deg =  0.5f;
    sp[0].manual_steer_expo        =  0.3f;
    sp[0].manual_steer_rate_deg_s  = 150.0f;
    sp[0].manual_steer_deadband_in = 0.03f;
    sp[0].manual_steer_max_deg     = 30.0f;

    setMotionConfig(&mc);
    setSteeringProfiles(sp, 1);
    setActiveProfile(0);
    begin(0);

    // Throttle ramp test: 0.5s at 20%/s -> 0.1
    _throttleActualNorm = 0.0f;
    updateThrottle(1.0f, 0.5f, 500);
    check("Throttle ramp 0->1 over 0.5s", _throttleActualNorm, 0.1f, 0.02f);

    // Steering expo tests
    float out0 = applySteerDeadbandExpo(sp[0], 0.0f);
    check("Steer expo at 0", out0, 0.0f, 0.001f);

    float outSmall = applySteerDeadbandExpo(sp[0], 0.01f);
    check("Steer small input in deadband", outSmall, 0.0f, 0.05f);

    float outMid = applySteerDeadbandExpo(sp[0], 0.5f);

    if (outMid > 0.2f && outMid < 0.7f)
    {
        out.println(F("[PASS] Steer expo mid-range sensible"));
        passCount++;
    }
    else
    {
        out.print(F("[FAIL] Steer expo mid-range, got="));
        out.println(outMid, 4);
        failCount++;
    }

    // Rudder mapping: 0.5 input with 30deg max should be in that ballpark
    _rudderActualDeg = 0.0f;
    updateRudder(0.5f, 0.1f); // small dt just to get initial target
    check("Rudder target approx 15deg", _rudderTargetDeg, 15.0f, 5.0f);

    out.print(F("=== TugbotMotion Self-Test Complete. PASS="));
    out.print(passCount);
    out.print(F(" FAIL="));
    out.println(failCount);
}
