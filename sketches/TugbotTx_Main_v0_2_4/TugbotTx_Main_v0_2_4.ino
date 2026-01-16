// -----------------------------------------------------------------------------
// File:        TugbotTx_Main_v0_2_4.ino
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#include "TugbotTransmitterApp.h"

static TugbotTransmitterApp g_app;

void setup()
{
    (void)g_app.Begin();
}

void loop()
{
    g_app.Tick();
}
