// -----------------------------------------------------------------------------
// File:        OLED096.h
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include <Wire.h>

class Adafruit_SSD1306;

class OLED096
{
public:
    static constexpr uint8_t kWidth  = 128;
    static constexpr uint8_t kHeight = 64;
    static constexpr uint8_t kRows   = 8;
    static constexpr uint8_t kPages  = 3;

    using FormatFn = void (*)(char*, size_t, void*);

    struct Line
    {
        const char* label = "";
        FormatFn fn = nullptr;
        void* user = nullptr;
    };

    OLED096();
    ~OLED096();

    bool Begin(TwoWire& wire, uint8_t addr = 0x3C);

    void SetPage(uint8_t page);
    uint8_t GetPage() const;
    void NextPage();
    void PrevPage();

    void SetMinRenderIntervalMs(uint32_t ms);
    void Invalidate();                 // force refresh on next Tick()
    void Tick(uint32_t nowMs);
    void Render();

    void SetLine(uint8_t page, uint8_t row, const char* label, FormatFn fn, void* user = nullptr);
    void LoadDefaultTemplate();

private:
    void RenderPage(uint8_t page);
    void DrawLine(uint8_t row, const char* label, const char* value);

private:
    TwoWire* _wire = nullptr;
    Adafruit_SSD1306* _disp = nullptr;
    uint8_t _addr = 0x3C;

    uint8_t _page = 0;

    uint32_t _minIntervalMs = 150;
    uint32_t _lastRenderMs = 0;

    bool _alwaysRefresh = true;        // DEFAULT: live values update
    bool _dirty = true;

    Line _lines[kPages][kRows];
};
