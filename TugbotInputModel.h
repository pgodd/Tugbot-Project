#pragma once
#include <Arduino.h>

class TugbotInputModel
{
public:
    void SetThrottlePct(int v);
    void SetRudderPct(int v);
    void SetArmed(bool v);
    void SetMode(uint8_t v);

    int GetThrottlePct() const;
    int GetRudderPct() const;
    bool GetArmed() const;
    uint8_t GetMode() const;

private:
    static int ClampPct(int v);

private:
    int _thrPct = 0;
    int _rudPct = 0;
    bool _armed = false;
    uint8_t _mode = 0;
};
