#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "OLED096.h"
#include "pins_transmitter_wroom.h"
#include "TugbotRadioLink.h"
#include "TugbotInputModel.h"

class TugbotUiOled
{
public:
    bool Begin();
    void Tick(uint32_t nowMs);

    void NextPage();
    void SetPage(uint8_t page);

    void Bind(const TugbotRadioLink* link, const TugbotInputModel* input);

private:
    static void FmtMode(char* out, size_t n, void*);
    static void FmtArm(char* out, size_t n, void*);
    static void FmtThr(char* out, size_t n, void*);
    static void FmtRud(char* out, size_t n, void*);
    static void FmtLink(char* out, size_t n, void*);
    static void FmtOk(char* out, size_t n, void*);
    static void FmtFail(char* out, size_t n, void*);
    static void FmtApay(char* out, size_t n, void*);

private:
    OLED096 _oled;
    const TugbotRadioLink* _link = nullptr;
    const TugbotInputModel* _input = nullptr;
};
