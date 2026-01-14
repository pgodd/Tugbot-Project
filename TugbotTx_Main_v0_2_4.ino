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
