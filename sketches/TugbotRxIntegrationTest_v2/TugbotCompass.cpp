// -----------------------------------------------------------------------------
// File:        TugbotCompass.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      Compass wrapper (QMC5883LCompass)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotCompass.h"

#include <Wire.h>
#include <QMC5883LCompass.h>

// -----------------------------------------------------------------------------
// Internal state
// -----------------------------------------------------------------------------

static QMC5883LCompass   s_compass;
static TugbotCompassState s_state;
static bool              s_initialised   = false;
static bool              s_present       = false;

// Approximate local magnetic declination for south-shore Nova Scotia.
// Negative = west of true north.
static const float DECLINATION_DEG      = -17.0f;

// Sensor mounting offset relative to bow (to be refined later and
// ideally loaded from NV).
static const float MOUNT_OFFSET_DEG     = 0.0f;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static float wrap360(float ang)
{
    while (ang < 0.0f)
    {
        ang += 360.0f;
    }
    while (ang >= 360.0f)
    {
        ang -= 360.0f;
    }
    return ang;
}

// Quick I2C probe on default QMC5883L address (0x0D)
static bool probeQmc()
{
    const uint8_t addr = 0x0D;
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    return (err == 0);
}

// -----------------------------------------------------------------------------
// API
// -----------------------------------------------------------------------------

bool TugbotCompass::begin()
{
    s_state.hasHeading   = false;
    s_state.heading_deg  = 0.0f;
    s_state.raw_x        = 0;
    s_state.raw_y        = 0;
    s_state.raw_z        = 0;
    s_state.azimuth_deg  = 0;

    // We assume Wire.begin() has already been called in the .ino.
    s_present = probeQmc();
    if (!s_present)
    {
        s_initialised = false;
        return false;
    }

    s_compass.init();

    // Light smoothing; enough to calm jitter but not kill responsiveness.
    s_compass.setSmoothing(4, false);

    s_initialised = true;
    return true;
}

void TugbotCompass::setCalibration(
    float xMin,
    float xMax,
    float yMin,
    float yMax,
    float zMin,
    float zMax
)
{
    if (!s_initialised)
    {
        return;
    }

    // Library expects min/max integers; we accept floats from NV and
    // cast them down.
    s_compass.setCalibration(
        (int)xMin, (int)xMax,
        (int)yMin, (int)yMax,
        (int)zMin, (int)zMax
    );
}

void TugbotCompass::update()
{
    if (!s_initialised || !s_present)
    {
        s_state.hasHeading = false;
        return;
    }

    // Update sensor
    s_compass.read();

    s_state.raw_x = s_compass.getX();
    s_state.raw_y = s_compass.getY();
    s_state.raw_z = s_compass.getZ();

    int az = s_compass.getAzimuth();   // magnetic north-based
    s_state.azimuth_deg = az;

    float headingTrue = wrap360(
        (float)az + DECLINATION_DEG + MOUNT_OFFSET_DEG
    );

    s_state.heading_deg = headingTrue;
    s_state.hasHeading  = true;
}

const TugbotCompassState & TugbotCompass::get()
{
    return s_state;
}
