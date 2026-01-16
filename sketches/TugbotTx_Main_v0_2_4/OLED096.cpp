// -----------------------------------------------------------------------------
// File:        OLED096.cpp
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#include "OLED096.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

OLED096::OLED096() {}

OLED096::~OLED096()
{
    if (_disp)
    {
        delete _disp;
        _disp = nullptr;
    }
}

bool OLED096::Begin(TwoWire& wire, uint8_t addr)
{
    _wire = &wire;
    _addr = addr;

    _disp = new Adafruit_SSD1306(kWidth, kHeight, _wire, -1);
    if (!_disp) return false;
    if (!_disp->begin(SSD1306_SWITCHCAPVCC, _addr)) return false;

    _disp->clearDisplay();
    _disp->setTextSize(1);
    _disp->setTextColor(SSD1306_WHITE);
    _disp->setCursor(0, 0);
    _disp->println("OLED096 OK");
    _disp->display();

    LoadDefaultTemplate();
    _dirty = true;
    _lastRenderMs = 0;
    return true;
}

void OLED096::SetPage(uint8_t page)
{
    if (page >= kPages) return;
    if (_page == page) return;
    _page = page;
    _dirty = true;
}

uint8_t OLED096::GetPage() const
{
    return _page;
}

void OLED096::NextPage()
{
    SetPage((_page + 1) % kPages);
}

void OLED096::PrevPage()
{
    SetPage((_page + kPages - 1) % kPages);
}

void OLED096::SetMinRenderIntervalMs(uint32_t ms)
{
    _minIntervalMs = ms;
}

void OLED096::Invalidate()
{
    _dirty = true;
}

void OLED096::SetLine(uint8_t page, uint8_t row, const char* label, FormatFn fn, void* user)
{
    if (page >= kPages || row >= kRows) return;
    _lines[page][row].label = label ? label : "";
    _lines[page][row].fn = fn;
    _lines[page][row].user = user;
    _dirty = true;
}

void OLED096::LoadDefaultTemplate()
{
    for (uint8_t p = 0; p < kPages; p++)
    {
        for (uint8_t r = 0; r < kRows; r++)
        {
            _lines[p][r] = {};
        }
    }
    _dirty = true;
}

void OLED096::Tick(uint32_t nowMs)
{
    if (!_disp) return;

    if (nowMs - _lastRenderMs < _minIntervalMs)
    {
        return;
    }

    // FIX: live values update even if nothing was marked dirty.
    if (_alwaysRefresh || _dirty)
    {
        Render();
        _lastRenderMs = nowMs;
        _dirty = false;
    }
}

void OLED096::Render()
{
    if (!_disp) return;
    RenderPage(_page);
    _disp->display();
}

void OLED096::RenderPage(uint8_t page)
{
    _disp->clearDisplay();
    _disp->setTextSize(1);
    _disp->setTextColor(SSD1306_WHITE);

    for (uint8_t row = 0; row < kRows; row++)
    {
        char value[22] = {0};
        const auto& ln = _lines[page][row];

        if (ln.fn)
        {
            ln.fn(value, sizeof(value), ln.user);
        }
        else
        {
            value[0] = '\0';
        }

        DrawLine(row, ln.label, value);
    }
}

void OLED096::DrawLine(uint8_t row, const char* label, const char* value)
{
    _disp->setCursor(0, row * 8);

    if (label && label[0])
    {
        _disp->print(label);
        _disp->print(": ");
    }

    if (value && value[0])
    {
        _disp->print(value);
    }
}
