// -----------------------------------------------------------------------------
// File:        TugbotWiFiOta.cpp
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#include "TugbotWiFiOta.h"

#include <WiFi.h>
#include <ArduinoOTA.h>

#include "TugbotWiFiOtaConfig.h"

static void PrintIp(const char* label, IPAddress ip)
{
    Serial.print(label);
    Serial.print(ip[0]); Serial.print('.');
    Serial.print(ip[1]); Serial.print('.');
    Serial.print(ip[2]); Serial.print('.');
    Serial.println(ip[3]);
}

bool TugbotWiFiOta::Begin()
{
    if (!WIFI_ENABLE)
    {
        Serial.println("WiFi/OTA disabled.");
        return true;
    }

    bool ok = BeginWiFi();
    BeginOta();
    return ok;
}

bool TugbotWiFiOta::BeginWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    if (!WIFI_SSID || WIFI_SSID[0] == 0 || String(WIFI_SSID) == "YOUR_SSID")
    {
        Serial.println("WiFi SSID not set; starting AP fallback.");
        WiFi.mode(WIFI_AP);
        _apActive = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
        if (_apActive)
        {
            Serial.print("AP SSID: "); Serial.println(WIFI_AP_SSID);
            PrintIp("AP IP: ", WiFi.softAPIP());
        }
        return _apActive;
    }

    Serial.print("WiFi STA connecting to: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS)
    {
        delay(25); // small, bounded
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        _staConnected = true;
        Serial.println("WiFi STA: connected.");
        PrintIp("STA IP: ", WiFi.localIP());
        return true;
    }

    Serial.println("WiFi STA: timeout. Starting AP fallback.");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    _apActive = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    if (_apActive)
    {
        Serial.print("AP SSID: "); Serial.println(WIFI_AP_SSID);
        PrintIp("AP IP: ", WiFi.softAPIP());
    }
    return _apActive;
}

void TugbotWiFiOta::BeginOta()
{
    if (!WIFI_ENABLE)
    {
        return;
    }

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]()
    {
        Serial.println("OTA: start");
    });

    ArduinoOTA.onEnd([]()
    {
        Serial.println("OTA: end");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
    {
        // Keep quiet; print occasionally to avoid serial spam.
        static uint32_t lastMs = 0;
        uint32_t now = millis();
        if (now - lastMs > 500)
        {
            lastMs = now;
            uint32_t pct = (total > 0) ? (progress * 100U) / total : 0;
            Serial.print("OTA: "); Serial.print(pct); Serial.println("%");
        }
    });

    ArduinoOTA.onError([](ota_error_t error)
    {
        Serial.print("OTA error: ");
        Serial.println((int)error);
    });

    ArduinoOTA.begin();
    Serial.print("OTA ready: ");
    Serial.println(OTA_HOSTNAME);
}

void TugbotWiFiOta::Tick(uint32_t nowMs)
{
    if (!WIFI_ENABLE)
    {
        return;
    }

    ArduinoOTA.handle();

    // Optional periodic status line (sparse)
    if (nowMs - _lastPrintMs > 5000)
    {
        _lastPrintMs = nowMs;

        wl_status_t st = WiFi.status();
        if (st == WL_CONNECTED)
        {
            Serial.print("WiFi: STA OK ");
            PrintIp("", WiFi.localIP());
        }
        else if (_apActive)
        {
            Serial.print("WiFi: AP OK ");
            PrintIp("", WiFi.softAPIP());
        }
        else
        {
            Serial.println("WiFi: OFF/FAIL");
        }
    }
}
