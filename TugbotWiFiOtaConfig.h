#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Tugbot TX - WiFi / OTA config (dev only)
// -----------------------------------------------------------------------------
// IMPORTANT:
// - This is for wireless programming convenience.
// - Change these before any public demos.
// - If you don't want WiFi at all, set WIFI_ENABLE = 0.
// -----------------------------------------------------------------------------

static const bool WIFI_ENABLE = true;

// Preferred: STA mode to your router
static const char* WIFI_SSID = "goddard5";
static const char* WIFI_PASS = "norman123";

// Fallback AP if STA fails
static const char* WIFI_AP_SSID = "TugbotTx-Setup";
static const char* WIFI_AP_PASS = "tugbotsetup";

// OTA identity + password
static const char* OTA_HOSTNAME = "TugbotTx-OTA";
static const char* OTA_PASSWORD = "tugbot";

// STA connect timeout (ms)
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
