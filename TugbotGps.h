// -----------------------------------------------------------------------------
// File:        TugbotGps.h
// Project:     Tugbot v1.0.0-beta
// Module:      GPS wrapper (TinyGPSPlus on Serial1)
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#ifndef TUGBOT_GPS_H
#define TUGBOT_GPS_H

#include <Arduino.h>

struct TugbotGpsState
{
    bool  hasFix;
    float lat_deg;
    float lon_deg;
    float speed_mps;
    float course_deg;
};

namespace TugbotGps
{
    void begin(HardwareSerial &port, uint32_t baud);
    void update();                 // feed bytes from port, call often
    TugbotGpsState get();          // latest state snapshot

} // namespace TugbotGps

#endif // TUGBOT_GPS_H
