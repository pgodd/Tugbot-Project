TugbotTx_Main_v0_2_4 (Wireless Programming / OTA)

This adds ArduinoOTA support for ESP32-WROOM.

How to use (Arduino IDE):
1) Install ESP32 board support package.
2) Set your WiFi credentials in TugbotWiFiOtaConfig.h (WIFI_SSID / WIFI_PASS).
3) Flash once over USB (initial seed).
4) Ensure PC + ESP32 are on same WiFi.
5) Arduino IDE -> Tools -> Port -> look for "TugbotTx-OTA".
6) Upload. (Subsequent uploads can be OTA.)

Notes:
- If WiFi STA connect fails, device starts an AP:
    SSID: TugbotTx-Setup
    PASS: tugbotsetup
  Connect your PC to that AP to reflash via USB or to see it alive; OTA over AP can work if your IDE routes correctly.
- OTA password is set in TugbotWiFiOtaConfig.h (default "tugbot").

Security:
- This is dev-mode OTA. Change passwords before public demos.
