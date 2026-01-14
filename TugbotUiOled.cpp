#include "TugbotUiOled.h"
#include "tugbot_config_radio.h"

static TugbotUiOled* g_self = nullptr;

static void FmtCh(char* o, size_t n, void*) { snprintf(o, n, "%u", NRF_CHANNEL); }
static void FmtRate(char* o, size_t n, void*)
{
    snprintf(o, n, "%s",
        (NRF_DATARATE == RF24_1MBPS) ? "1M" :
        (NRF_DATARATE == RF24_250KBPS) ? "250K" : "2M");
}
static void FmtAck(char* o, size_t n, void*) { snprintf(o, n, "%s", NRF_ACK_PAYLOAD ? "ON" : "OFF"); }

bool TugbotUiOled::Begin()
{
    g_self = this;

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    if (!_oled.Begin(Wire, 0x3C))
    {
        return false;
    }

    _oled.SetMinRenderIntervalMs(150);
    _oled.LoadDefaultTemplate();

    _oled.SetLine(0, 1, "MODE", FmtMode, nullptr);
    _oled.SetLine(0, 2, "ARM",  FmtArm,  nullptr);
    _oled.SetLine(0, 3, "THR",  FmtThr,  nullptr);
    _oled.SetLine(0, 4, "RUD",  FmtRud,  nullptr);
    _oled.SetLine(0, 5, "LINK", FmtLink, nullptr);

    _oled.SetLine(1, 1, "CH",   FmtCh, nullptr);
    _oled.SetLine(1, 2, "RATE", FmtRate, nullptr);
    _oled.SetLine(1, 3, "ACK",  FmtAck, nullptr);
    _oled.SetLine(1, 4, "OK",   FmtOk, nullptr);
    _oled.SetLine(1, 5, "FAIL", FmtFail, nullptr);
    _oled.SetLine(1, 6, "APAY", FmtApay, nullptr);

    _oled.Render();
    return true;
}

void TugbotUiOled::Bind(const TugbotRadioLink* link, const TugbotInputModel* input)
{
    _link = link;
    _input = input;
}

void TugbotUiOled::Tick(uint32_t nowMs) { _oled.Tick(nowMs); }

void TugbotUiOled::NextPage() { _oled.NextPage(); _oled.Render(); }
void TugbotUiOled::SetPage(uint8_t page) { _oled.SetPage(page); _oled.Render(); }

void TugbotUiOled::FmtMode(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_input) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%u", g_self->_input->GetMode());
}

void TugbotUiOled::FmtArm(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_input) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%s", g_self->_input->GetArmed() ? "ARM" : "DIS");
}

void TugbotUiOled::FmtThr(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_input) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%+d%%", g_self->_input->GetThrottlePct());
}

void TugbotUiOled::FmtRud(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_input) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%+d", g_self->_input->GetRudderPct());
}

void TugbotUiOled::FmtLink(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_link) { snprintf(out, n, "--"); return; }
    const auto& s = g_self->_link->GetStats();
    snprintf(out, n, "%s", (s.ok > 0) ? "OK" : "--");
}

void TugbotUiOled::FmtOk(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_link) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%lu", (unsigned long)g_self->_link->GetStats().ok);
}

void TugbotUiOled::FmtFail(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_link) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%lu", (unsigned long)g_self->_link->GetStats().fail);
}

void TugbotUiOled::FmtApay(char* out, size_t n, void*)
{
    if (!g_self || !g_self->_link) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%lu", (unsigned long)g_self->_link->GetStats().ackPayload);
}
