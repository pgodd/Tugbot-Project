// -----------------------------------------------------------------------------
// File:        TugbotTransmitterApp.cpp
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#include "TugbotTransmitterApp.h"

#include <tugbot_cmd_frames.h>
#include <tugbot_config_radio.h>
#include "pins_transmitter_wroom.h"

static TugbotTransmitterApp* g_app = nullptr;

static void UiSetPageThunk(uint8_t page)
{
    if (g_app) g_app->SetUiPage(page);
}

static void ModeSetThunk(uint8_t mode)
{
    if (g_app) g_app->SetMode(mode);
}

bool TugbotTransmitterApp::Begin()
{
    g_app = this;

    Serial.begin(115200);

    Serial.println();
    Serial.println("TugbotTx_Main_v0_2_4 (3 encoders + OTA)");
    Serial.println();

    // WiFi + OTA (dev convenience)
    (void)_ota.Begin();

    // Encoder roles:
    _enc1.Begin(PIN_ENC1_A, PIN_ENC1_B, PIN_ENC1_BTN, TugbotInputEncoder::Role::Rudder, 2);
    _enc2.Begin(PIN_ENC2_A, PIN_ENC2_B, PIN_ENC2_BTN, TugbotInputEncoder::Role::Throttle, 2);
    _enc3.Begin(PIN_ENC3_A, PIN_ENC3_B, PIN_ENC3_BTN, TugbotInputEncoder::Role::UiPage, 1);

    _enc1.Bind(&_input);
    _enc2.Bind(&_input);
    _enc3.Bind(&_input);

    _enc3.BindUiPageSetter(&UiSetPageThunk);
    _enc3.BindModeSetter(&ModeSetThunk);

    if (!_ui.Begin())
    {
        Serial.println("OLED BEGIN FAIL");
        return false;
    }

    _ui.Bind(&_link, &_input);
    _ui.SetPage(_uiPage);

    if (!_link.Begin())
    {
        Serial.println("NRF BEGIN FAIL (UI/input/OTA still running)");
    }
    else
    {
        Serial.print("NRF CHIP=");
        Serial.println(_link.GetStats().chipConnected ? "YES" : "NO");
    }

    PrintHelp();
    PrintStatus();
    return true;
}

void TugbotTransmitterApp::SetUiPage(uint8_t page)
{
    _uiPage = page;
    _ui.SetPage(_uiPage);
}

void TugbotTransmitterApp::SetMode(uint8_t mode)
{
    _input.SetMode(mode);
}

void TugbotTransmitterApp::Tick()
{
    uint32_t now = millis();

    // OTA handler must be ticked frequently
    _ota.Tick(now);

    _enc1.Tick(now);
    _enc2.Tick(now);
    _enc3.Tick(now);

    if (now - _lastSendMs >= _sendIntervalMs)
    {
        _lastSendMs = now;
        SendCmd();
    }

    TickSerial();
    _ui.Tick(now);
}

void TugbotTransmitterApp::SendCmd()
{
    NrfCmdFrame c;
    c.magic = NRF_MAGIC_CMD;
    c.seq = _seq++;
    c.thrPct = (int8_t)_input.GetThrottlePct();
    c.rudPct = (int8_t)_input.GetRudderPct();
    c.flags = _input.GetArmed() ? 0x01 : 0x00;
    c.mode = _input.GetMode();
    c.reserved = 0;

    (void)_link.SendCmd(c);
}

void TugbotTransmitterApp::TickSerial()
{
    while (Serial.available() > 0)
    {
        char ch = (char)Serial.read();
        if (ch == '\r') continue;

        if (ch == '\n')
        {
            HandleLine(_rxLine);
            _rxLine = "";
        }
        else
        {
            _rxLine += ch;
            if (_rxLine.length() > 200) _rxLine = "";
        }
    }
}

int TugbotTransmitterApp::ParseInt(const String& s, bool& ok)
{
    char* endp = nullptr;
    long v = strtol(s.c_str(), &endp, 10);
    ok = (endp != s.c_str());
    return (int)v;
}

void TugbotTransmitterApp::HandleLine(String line)
{
    line.trim();
    if (line.length() == 0) return;

    int sp = line.indexOf(' ');
    String cmd = (sp >= 0) ? line.substring(0, sp) : line;
    String args = (sp >= 0) ? line.substring(sp + 1) : "";
    cmd.toUpperCase();
    args.trim();

    if (cmd == "HELP") { PrintHelp(); return; }
    if (cmd == "STATUS") { PrintStatus(); return; }

    if (cmd == "ARM") { _input.SetArmed(true); Serial.println("ARMED"); return; }
    if (cmd == "DISARM") { _input.SetArmed(false); Serial.println("DISARMED"); return; }

    if (cmd == "THR")
    {
        bool ok=false; int v=ParseInt(args,ok); if(!ok){Serial.println("ERR");return;}
        _input.SetThrottlePct(v); Serial.println("OK"); return;
    }

    if (cmd == "RUD")
    {
        bool ok=false; int v=ParseInt(args,ok); if(!ok){Serial.println("ERR");return;}
        _input.SetRudderPct(v); Serial.println("OK"); return;
    }

    if (cmd == "MODE")
    {
        bool ok=false; int v=ParseInt(args,ok); if(!ok){Serial.println("ERR");return;}
        if(v<0)v=0; if(v>255)v=255; _input.SetMode((uint8_t)v); Serial.println("OK"); return;
    }

    if (cmd == "PAGE")
    {
        bool ok=false; int v=ParseInt(args,ok); if(!ok || v<0 || v>2){Serial.println("ERR");return;}
        SetUiPage((uint8_t)v); Serial.println("OK"); return;
    }

    if (cmd == "NEXT") { _ui.NextPage(); Serial.println("OK"); return; }

    Serial.println("ERR");
}

void TugbotTransmitterApp::PrintHelp() const
{
    Serial.println("Commands:");
    Serial.println("  HELP | STATUS");
    Serial.println("  THR <int -100..100>");
    Serial.println("  RUD <int -100..100>");
    Serial.println("  ARM | DISARM");
    Serial.println("  MODE <0..255>");
    Serial.println("  PAGE <0..2> | NEXT");
    Serial.println();
    Serial.println("Encoders:");
    Serial.println("  ENC1: RUD  (press toggles ARM)");
    Serial.println("  ENC2: THR  (press zeros THR)");
    Serial.println("  ENC3: PAGE (press increments MODE)");
    Serial.println();
    Serial.println("OTA:");
    Serial.println("  Configure SSID/PASS in TugbotWiFiOtaConfig.h");
    Serial.println();
}

void TugbotTransmitterApp::PrintStatus() const
{
    const auto& s = _link.GetStats();
    Serial.print("ARM="); Serial.print(_input.GetArmed()?1:0);
    Serial.print(" THR="); Serial.print(_input.GetThrottlePct());
    Serial.print(" RUD="); Serial.print(_input.GetRudderPct());
    Serial.print(" MODE="); Serial.print(_input.GetMode());
    Serial.print(" PAGE="); Serial.print(_uiPage);
    Serial.print(" OK="); Serial.print((unsigned long)s.ok);
    Serial.print(" FAIL="); Serial.print((unsigned long)s.fail);
    Serial.print(" APAY="); Serial.println((unsigned long)s.ackPayload);
}
