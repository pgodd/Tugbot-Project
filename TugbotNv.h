// -----------------------------------------------------------------------------
// File:        TugbotNv.h
// Project:     Tugbot v1.0.0-beta
// Module:      NVRAM (EEPROM) configuration storage
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// Responsibilities:
//   - Define canonical persistent config structure for Tugbot
//   - Load/save from Arduino EEPROM with CRC16 protection
//   - Provide helper to map NVRAM -> motion/steering configs
// -----------------------------------------------------------------------------

#ifndef TUGBOT_NV_H
#define TUGBOT_NV_H

#include <Arduino.h>
#include "TugbotMotion.h"    // for MotionConfig, SteeringProfileConfig

// Steering profile in NVRAM (very close to SteeringProfileConfig)
struct SteeringProfileNv
{
    float rudder_center_offset_deg;
    float rudder_max_port_deg;
    float rudder_max_starboard_deg;
    float rudder_mech_deadzone_deg;

    float manual_steer_expo;
    float manual_steer_rate_deg_s;
    float manual_steer_deadband_in;
    float manual_steer_max_deg;

    float manual_steer_trim_deg;   // reserved for future use

    char  name[8];                 // "SAFE", "NORMAL", "DEMO", etc.
};

struct TugbotNvData
{
    uint16_t magic;        // 'T''B'
    uint8_t  version;      // struct version
    uint8_t  reserved0;

    // --- Steering profiles (3) ---
    uint8_t  activeSteeringProfile;
    uint8_t  reserved1[3]; // alignment
    SteeringProfileNv steering[3];

    // --- Compass calibration ---
    float compass_offset_x;
    float compass_offset_y;
    float compass_offset_z;
    float compass_scale_x;
    float compass_scale_y;
    float compass_scale_z;

    // --- Voltage calibration ---
    float v_prop_scale;
    float v_prop_offset;
    float v_sys_scale;
    float v_sys_offset;

    // --- Current calibration ---
    float i_prop_zero;
    float i_prop_scale;
    float i_sys_zero;
    float i_sys_scale;

    // --- Motion / throttle defaults ---
    float throttle_limit_normal;
    float throttle_ramp_up;
    float throttle_ramp_down;
    float reverse_coast_ms;

    // --- Battery thresholds (V) ---
    float v_prop_warn;
    float v_prop_crit;
    float v_sys_warn;
    float v_sys_crit;

    // --- Counters / misc ---
    uint32_t run_counter;
    uint32_t hours_run;
    uint8_t  last_mode;    // 0=MANUAL, 1=ASSIST, 2=AUTO
    uint8_t  reserved2[7];

    uint16_t crc;          // CRC16 over structure (excluding magic+crc)
};

namespace TugbotNv
{
    const uint16_t NV_MAGIC   = 0x5442; // 'T''B'
    const uint8_t  NV_VERSION = 1;

    bool load(TugbotNvData &out);               // load or set defaults+save
    bool save(const TugbotNvData &data);        // save & update CRC
    void setDefaults(TugbotNvData &nv);         // compiled-in gospel defaults

    // Map NVRAM motion/steering section into live configs.
    // Expects profiles[] with size >= 3.
    void applyToMotion(const TugbotNvData &nv,
                       MotionConfig &motionCfg,
                       SteeringProfileConfig *profiles,
                       uint8_t profileCount);

} // namespace TugbotNv

#endif // TUGBOT_NV_H
