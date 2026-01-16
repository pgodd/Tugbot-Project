// -----------------------------------------------------------------------------
// File:        TugbotNv.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      NVRAM (EEPROM) configuration storage
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotNv.h"
#include <EEPROM.h>
#include "tugbot_config_motion.h"  // for gospel defaults

namespace
{
    const int EEPROM_BASE_ADDR = 0;

    uint16_t crc16_update(uint16_t crc, uint8_t data)
    {
        crc ^= (uint16_t)data << 8;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
        return crc;
    }

    uint16_t computeCrc(const TugbotNvData &d)
    {
        const uint8_t *p = reinterpret_cast<const uint8_t*>(&d);
        size_t len = sizeof(TugbotNvData);

        size_t start = 2;          // skip magic
        size_t end   = len - 2;    // skip crc

        uint16_t crc = 0xFFFF;
        for (size_t i = start; i < end; ++i)
        {
            crc = crc16_update(crc, p[i]);
        }
        return crc;
    }

    void eepromReadBlock(void *dst, int addr, size_t len)
    {
        uint8_t *p = static_cast<uint8_t*>(dst);
        for (size_t i = 0; i < len; ++i)
        {
            p[i] = EEPROM.read(addr + i);
        }
    }

    void eepromWriteBlock(const void *src, int addr, size_t len)
    {
        const uint8_t *p = static_cast<const uint8_t*>(src);
        for (size_t i = 0; i < len; ++i)
        {
            EEPROM.update(addr + i, p[i]);
        }
    }
}

namespace TugbotNv
{
    void setDefaults(TugbotNvData &nv)
    {
        memset(&nv, 0, sizeof(nv));

        nv.magic   = NV_MAGIC;
        nv.version = NV_VERSION;

        // Motion from gospel
        MotionConfig mc;
        tugbot_getDefaultMotionConfig(mc);
        nv.throttle_limit_normal = mc.throttle_limit_normal;
        nv.throttle_ramp_up      = mc.throttle_ramp_up_percent_s;
        nv.throttle_ramp_down    = mc.throttle_ramp_down_percent_s;
        nv.reverse_coast_ms      = mc.reverse_coast_ms;

        // Steering profiles from gospel
        SteeringProfileConfig tmpProfiles[3];
        tugbot_getDefaultSteeringProfiles(tmpProfiles, 3);

        nv.activeSteeringProfile = 1; // NORMAL

        const char *names[3] = { "SAFE", "NORMAL", "DEMO" };

        for (int i = 0; i < 3; ++i)
        {
            SteeringProfileNv &nvp = nv.steering[i];
            const SteeringProfileConfig &sp = tmpProfiles[i];

            nvp.rudder_center_offset_deg = sp.rudder_center_offset_deg;
            nvp.rudder_max_port_deg      = sp.rudder_max_port_deg;
            nvp.rudder_max_starboard_deg = sp.rudder_max_starboard_deg;
            nvp.rudder_mech_deadzone_deg = sp.rudder_mech_deadzone_deg;

            nvp.manual_steer_expo        = sp.manual_steer_expo;
            nvp.manual_steer_rate_deg_s  = sp.manual_steer_rate_deg_s;
            nvp.manual_steer_deadband_in = sp.manual_steer_deadband_in;
            nvp.manual_steer_max_deg     = sp.manual_steer_max_deg;

            nvp.manual_steer_trim_deg    = 0.0f;
            memset(nvp.name, 0, sizeof(nvp.name));
            strncpy(nvp.name, names[i], sizeof(nvp.name) - 1);
        }

        // Compass default calibration: identity
        nv.compass_offset_x = 0.0f;
        nv.compass_offset_y = 0.0f;
        nv.compass_offset_z = 0.0f;
        nv.compass_scale_x  = 1.0f;
        nv.compass_scale_y  = 1.0f;
        nv.compass_scale_z  = 1.0f;

        // Voltage/current calibration defaults: unity/zero
        nv.v_prop_scale = 1.0f;
        nv.v_prop_offset = 0.0f;
        nv.v_sys_scale  = 1.0f;
        nv.v_sys_offset = 0.0f;

        nv.i_prop_zero  = 0.0f;
        nv.i_prop_scale = 1.0f;
        nv.i_sys_zero   = 0.0f;
        nv.i_sys_scale  = 1.0f;

        // Battery thresholds (rough; you can tune later)
        nv.v_prop_warn = 7.0f;
        nv.v_prop_crit = 6.8f;
        nv.v_sys_warn  = 13.8f;
        nv.v_sys_crit  = 13.2f;

        nv.run_counter = 0;
        nv.hours_run   = 0;
        nv.last_mode   = 0;

        nv.crc = computeCrc(nv);
    }

    bool load(TugbotNvData &out)
    {
        TugbotNvData tmp;
        eepromReadBlock(&tmp, EEPROM_BASE_ADDR, sizeof(tmp));

        if (tmp.magic != NV_MAGIC || tmp.version != NV_VERSION)
        {
            setDefaults(out);
            save(out);
            return true;
        }

        uint16_t crcCalc = computeCrc(tmp);
        if (crcCalc != tmp.crc)
        {
            setDefaults(out);
            save(out);
            return true;
        }

        out = tmp;
        return true;
    }

    bool save(const TugbotNvData &data)
    {
        TugbotNvData tmp = data;
        tmp.magic   = NV_MAGIC;
        tmp.version = NV_VERSION;
        tmp.crc     = computeCrc(tmp);

        eepromWriteBlock(&tmp, EEPROM_BASE_ADDR, sizeof(tmp));
        return true;
    }

    void applyToMotion(const TugbotNvData &nv,
                       MotionConfig &motionCfg,
                       SteeringProfileConfig *profiles,
                       uint8_t profileCount)
    {
        // Motion
        motionCfg.throttle_limit_normal        = nv.throttle_limit_normal;
        motionCfg.throttle_ramp_up_percent_s   = nv.throttle_ramp_up;
        motionCfg.throttle_ramp_down_percent_s = nv.throttle_ramp_down;
        motionCfg.reverse_coast_ms             = nv.reverse_coast_ms;

        // Steering – expect at least 3 profiles
        uint8_t n = (profileCount < 3) ? profileCount : 3;
        for (uint8_t i = 0; i < n; ++i)
        {
            const SteeringProfileNv &nvp = nv.steering[i];
            SteeringProfileConfig &sp    = profiles[i];

            sp.rudder_center_offset_deg = nvp.rudder_center_offset_deg;
            sp.rudder_max_port_deg      = nvp.rudder_max_port_deg;
            sp.rudder_max_starboard_deg = nvp.rudder_max_starboard_deg;
            sp.rudder_mech_deadzone_deg = nvp.rudder_mech_deadzone_deg;

            sp.manual_steer_expo        = nvp.manual_steer_expo;
            sp.manual_steer_rate_deg_s  = nvp.manual_steer_rate_deg_s;
            sp.manual_steer_deadband_in = nvp.manual_steer_deadband_in;
            sp.manual_steer_max_deg     = nvp.manual_steer_max_deg;
        }
    }

} // namespace TugbotNv
