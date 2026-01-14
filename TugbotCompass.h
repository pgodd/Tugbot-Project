// -----------------------------------------------------------------------------
// File:        TugbotCompass.h
// Project:     Tugbot v1.0.0-beta
// Module:      Compass wrapper (QMC5883LCompass)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#ifndef TUGBOT_COMPASS_H
#define TUGBOT_COMPASS_H

#include <Arduino.h>

// Forward-declare state structure used by integration test.
struct TugbotCompassState
{
    bool   hasHeading;      // true if heading_deg is valid
    float  heading_deg;     // True heading (after declination + mount offset)

    int16_t raw_x;          // Raw X
    int16_t raw_y;          // Raw Y
    int16_t raw_z;          // Raw Z
    int16_t azimuth_deg;    // Magnetic azimuth (0–359) from library
};

namespace TugbotCompass
{
    // Probe I2C, init QMC5883L, set default mode/smoothing.
    bool begin();

    // Cal data is currently taken from NVRAM; for now we pass it
    // straight through to the underlying library's setCalibration().
    void setCalibration(
        float xMin,
        float xMax,
        float yMin,
        float yMax,
        float zMin,
        float zMax
    );

    // Read sensor, update internal state.
    void update();

    // Return the latest state snapshot (by const reference to avoid copies).
    const TugbotCompassState & get();
}

#endif // TUGBOT_COMPASS_H
