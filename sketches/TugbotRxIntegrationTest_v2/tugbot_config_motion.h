// -----------------------------------------------------------------------------
// File:        tugbot_config_motion.h
// Project:     Tugbot v1.0.0-beta
// Module:      Motion & Steering configuration (gospel)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// This file defines the canonical default configuration values for:
//   - Throttle behaviour (ramping, limits, reverse coast)
//   - Steering profiles (SAFE, NORMAL, DEMO)
// These values can be overridden at runtime by NVRAM (EEPROM) in future
// versions, but this file remains the human-readable truth.
// -----------------------------------------------------------------------------

#ifndef TUGBOT_CONFIG_MOTION_H
#define TUGBOT_CONFIG_MOTION_H

#include <Arduino.h>

// Forward declarations from TugbotMotion.h
struct MotionConfig;
struct SteeringProfileConfig;

// --- Motion (throttle) defaults ---------------------------------------------

inline void tugbot_getDefaultMotionConfig(MotionConfig &cfg)
{
    cfg.throttle_limit_normal        = 1.0f;   // full range allowed
    cfg.throttle_ramp_up_percent_s   = 20.0f;  // 20% per second ramp up
    cfg.throttle_ramp_down_percent_s = 20.0f;  // symmetric for now
    cfg.reverse_coast_ms             = 200.0f; // 200ms neutral when reversing
}

// --- Steering profiles ------------------------------------------------------
// Index mapping:
//   0 = SAFE / TRAINING
//   1 = NORMAL
//   2 = DEMO / AGGRESSIVE

inline void tugbot_getDefaultSteeringProfiles(SteeringProfileConfig *profiles,
                                              uint8_t count)
{
    if (count < 3)
    {
        return;
    }

    // Profile 0: SAFE / TRAINING
    {
        SteeringProfileConfig &p = profiles[0];

        p.rudder_center_offset_deg = 0.0f;
        p.rudder_max_port_deg      = -20.0f;
        p.rudder_max_starboard_deg =  20.0f;
        p.rudder_mech_deadzone_deg =  1.0f;

        p.manual_steer_expo        = 0.4f;
        p.manual_steer_rate_deg_s  = 90.0f;
        p.manual_steer_deadband_in = 0.04f;
        p.manual_steer_max_deg     = 20.0f;
    }

    // Profile 1: NORMAL
    {
        SteeringProfileConfig &p = profiles[1];

        p.rudder_center_offset_deg = 0.0f;
        p.rudder_max_port_deg      = -35.0f;
        p.rudder_max_starboard_deg =  35.0f;
        p.rudder_mech_deadzone_deg =  0.5f;

        p.manual_steer_expo        = 0.3f;
        p.manual_steer_rate_deg_s  = 150.0f;
        p.manual_steer_deadband_in = 0.03f;
        p.manual_steer_max_deg     = 30.0f;
    }

    // Profile 2: DEMO / AGGRESSIVE
    {
        SteeringProfileConfig &p = profiles[2];

        p.rudder_center_offset_deg = 0.0f;
        p.rudder_max_port_deg      = -40.0f;
        p.rudder_max_starboard_deg =  40.0f;
        p.rudder_mech_deadzone_deg =  0.5f;

        p.manual_steer_expo        = 0.2f;
        p.manual_steer_rate_deg_s  = 200.0f;
        p.manual_steer_deadband_in = 0.02f;
        p.manual_steer_max_deg     = 35.0f;
    }
}

#endif // TUGBOT_CONFIG_MOTION_H
