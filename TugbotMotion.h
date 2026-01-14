// -----------------------------------------------------------------------------
// File:        TugbotMotion.h
// Project:     Tugbot v1.0.0-beta
// Module:      Motion & Steering object
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// Responsibilities:
//   - Throttle shaping & ramping
//   - Reverse coast enforcement
//   - Steering deadband + expo
//   - Steering rate limiting
//   - Mapping to normalised command space (for PWM/servo)
//   - Built-in self-test of core logic (no hardware pins touched)
// -----------------------------------------------------------------------------

#ifndef TUGBOT_MOTION_H
#define TUGBOT_MOTION_H

#include <Arduino.h>

// ---------------- CONFIG STRUCTS --------------------------------------------

struct SteeringProfileConfig
{
    float rudder_center_offset_deg;
    float rudder_max_port_deg;
    float rudder_max_starboard_deg;
    float rudder_mech_deadzone_deg;

    float manual_steer_expo;
    float manual_steer_rate_deg_s;
    float manual_steer_deadband_in;
    float manual_steer_max_deg;
};

struct MotionConfig
{
    float throttle_limit_normal;
    float throttle_ramp_up_percent_s;
    float throttle_ramp_down_percent_s;
    float reverse_coast_ms;
};

// ---------------- MAIN CLASS ------------------------------------------------

class TugbotMotion
{
public:
    TugbotMotion();

    void setMotionConfig(const MotionConfig *cfg);
    void setSteeringProfiles(const SteeringProfileConfig *profiles,
                             uint8_t profileCount);

    void setActiveProfile(uint8_t idx);
    uint8_t getActiveProfile() const;

    void begin(uint32_t nowMs);

    void update(float inputThrottleNorm,
                float inputSteerNorm,
                uint32_t nowMs);

    float getThrottleTargetNorm() const;
    float getThrottleActualNorm() const;

    float getRudderTargetDeg() const;
    float getRudderActualDeg() const;

    // Convenience helpers (optional)
    uint8_t getThrottlePwm255() const;
    float   getRudderNormFromDeg() const;

    // Built-in self tests (pure math, prints to provided stream)
    void runSelfTests(Stream &out);

private:
    const MotionConfig            *_motionCfg;
    const SteeringProfileConfig   *_profiles;
    uint8_t                        _profileCount;
    uint8_t                        _activeProfile;

    float                          _throttleTargetNorm;
    float                          _throttleActualNorm;
    float                          _rudderTargetDeg;
    float                          _rudderActualDeg;

    uint32_t                       _lastUpdateMs;
    uint32_t                       _reverseCoastUntilMs;

    const SteeringProfileConfig   *currentProfile() const;

    float clamp(float v, float lo, float hi) const;
    float applySteerDeadbandExpo(const SteeringProfileConfig &p,
                                 float inputNorm) const;

    void  updateThrottle(float inNorm,
                         float dtSec,
                         uint32_t nowMs);

    void  updateRudder(float inNorm,
                       float dtSec);
};

#endif // TUGBOT_MOTION_H
