// -----------------------------------------------------------------------------
// File:        TugbotGps.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      GPS wrapper (TinyGPSPlus on Serial1)
// Revision:    1.0.0
// -----------------------------------------------------------------------------

#include "TugbotGps.h"
#include <TinyGPSPlus.h>

namespace
{
    HardwareSerial *gpsPort = nullptr;
    TinyGPSPlus     gps;
    TugbotGpsState  lastState = { false, 0.0f, 0.0f, 0.0f, 0.0f };
}

namespace TugbotGps
{
    void begin(HardwareSerial &port, uint32_t baud)
    {
        gpsPort = &port;
        gpsPort->begin(baud);
        lastState.hasFix = false;
    }

    void update()
    {
        if (!gpsPort)
            return;

        while (gpsPort->available() > 0)
        {
            char c = (char)gpsPort->read();
            gps.encode(c);
        }

        if (gps.location.isUpdated())
        {
            lastState.hasFix   = gps.location.isValid();
            lastState.lat_deg  = gps.location.lat();
            lastState.lon_deg  = gps.location.lng();
            lastState.speed_mps  = gps.speed.mps();
            lastState.course_deg = gps.course.deg();
        }
    }

    TugbotGpsState get()
    {
        return lastState;
    }

} // namespace TugbotGps
