/*
  TugBot TX (ESP32-WROOM) — Protocol v2 + OLED + NRF24 + KY-040 (state-table) + Toggleable WiFi/OTA
  ------------------------------------------------------------------------------------------------
  CANON control mapping:
    - Throttle: THROTTLEPOT (A=26 B=14)
    - Rudder:   RUDDERPOT (A=32 B=33)
    - ARM:      THROTTLEPOT button (PIN_THROTTLEPOT_BTN = 13)  [throttle actuator button]
    - Accessory select: NEXT = RUDDERPOT button (25), PREV = MENUPOT button (4)
    - Accessory value: MENUPOT (A=16 B=17)

  Encoders:
    - KY-040 fixed state-table decoder (Buxtronix/ownprox style)
    - Interrupt-driven on A and B (CHANGE)
    - ISR intentionally NOT IRAM_ATTR (uses digitalRead; avoids linker "dangerous relocation")

  NEW (Feb 21 2026): Throttle + Rudder ramps (marine feel)
    - Encoder changes update SETPOINTS immediately
    - Outgoing command to RX is RAMPED toward setpoints at configured rates
    - When DISARMING: throttle/rudder/accessories go to safe values immediately
*/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <stdarg.h>

#include <WiFi.h>
#include <ArduinoOTA.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// CANON Wi-Fi / OTA credentials
// ============================================================================
static const char* WIFI_SSID = "goddard5";
static const char* WIFI_PASS = "norman123";
static const char* OTA_PASS  = "norman123";

// ============================================================================
// CANON TX PINS (ESP32-WROOM)
// ============================================================================
// NRF24 (VSPI pins)
static const uint8_t PIN_SPI_SCK  = 18;
static const uint8_t PIN_SPI_MISO = 19;
static const uint8_t PIN_SPI_MOSI = 23;

static const uint8_t PIN_NRF_CE   = 27;
static const uint8_t PIN_NRF_CSN  = 5;

// OLED I2C
static const uint8_t PIN_I2C_SDA  = 21;
static const uint8_t PIN_I2C_SCL  = 22;

// Rotary encoders (KY-040)
static const uint8_t PIN_RUDDERPOT_A   = 32;
static const uint8_t PIN_RUDDERPOT_B   = 33;
static const uint8_t PIN_RUDDERPOT_BTN = 25;

static const uint8_t PIN_THROTTLEPOT_A   = 26;
static const uint8_t PIN_THROTTLEPOT_B   = 14;
static const uint8_t PIN_THROTTLEPOT_BTN = 13;

static const uint8_t PIN_MENUPOT_A   = 16;
static const uint8_t PIN_MENUPOT_B   = 17;
static const uint8_t PIN_MENUPOT_BTN = 4;

// Dedicated WiFi toggle button (INPUT-ONLY pin; needs external pull-up)
static const uint8_t PIN_WIFI_BTN = 34; // External 10k pull-up to 3.3V, button to GND

// ============================================================================
// RADIO SETTINGS
// ============================================================================
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
static const uint8_t PIPE_ADDR[6] = "tugbt";
static constexpr uint8_t RF_CHANNEL = 124;

// ============================================================================
// OLED
// ============================================================================
static constexpr int OLED_W = 128;
static constexpr int OLED_H = 64;
static constexpr int OLED_RESET = -1;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

// ============================================================================
// PROTOCOL (v2, matches RX)
// ============================================================================
static constexpr uint8_t TB_MAX_AIR = 32;
static constexpr uint8_t TB_VER     = 2;

enum TbMsgType : uint8_t { TB_CMD = 1, TB_PING = 2, TB_ACK = 4 };
enum TbStatus  : uint8_t { TB_S_OK = 0, TB_S_BAD_VER = 1, TB_S_BAD_LEN = 2, TB_S_BAD_CRC = 3, TB_S_BAD_TYPE = 4 };

#pragma pack(push, 1)
struct TbHdr {
  uint8_t ver;
  uint8_t type;
  uint8_t flags;
  uint8_t seq;
  uint8_t len;
};

struct TbCmdV1 {
  int8_t  throttlePct;   // -100..100
  int8_t  rudderPct;     // -100..100
  uint8_t acc[4];        // 0..255
  uint8_t arm;           // 0/1
};

struct TbAckV2 {
  uint8_t  ver;
  uint8_t  type;     // TB_ACK
  uint8_t  seqEcho;
  uint8_t  status;
  uint16_t rxOk;
  uint16_t rxBad;

  uint16_t vSys_mV;
  uint16_t vProp_mV;
  uint16_t iSys_mA;

  int16_t  tMotor_cC;
  int16_t  tEsc_cC;
  uint16_t waterRaw;

  uint16_t crc16;
};
#pragma pack(pop)

static constexpr uint8_t TB_HDR_LEN = sizeof(TbHdr);
static constexpr uint8_t TB_CRC_LEN = 2;
static constexpr uint8_t TB_MAX_PAY = (uint8_t)(TB_MAX_AIR - TB_HDR_LEN - TB_CRC_LEN);

// ============================================================================
// UTIL
// ============================================================================
static int clampi(int v, int lo, int hi) { if (v < lo) return lo; if (v > hi) return hi; return v; }

static void logBoth(const char* s) {
  Serial.println(s);
}

static void logBothf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
}

// CRC16-CCITT (0x1021), init 0xFFFF
static uint16_t TbCrc16Ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static uint16_t TbAckCrc(const TbAckV2& a) {
  return TbCrc16Ccitt((const uint8_t*)&a, sizeof(TbAckV2) - sizeof(uint16_t));
}

static bool TbBuildFrame(uint8_t type, uint8_t flags, uint8_t seq,
                         const uint8_t* payload, uint8_t payLen,
                         uint8_t* outFrame, uint8_t& outLen) {
  if (payLen > TB_MAX_PAY) return false;

  TbHdr h;
  h.ver   = TB_VER;
  h.type  = type;
  h.flags = flags;
  h.seq   = seq;
  h.len   = payLen;

  const uint8_t totalNoCrc = TB_HDR_LEN + payLen;
  const uint8_t total      = totalNoCrc + TB_CRC_LEN;

  memcpy(outFrame, &h, TB_HDR_LEN);
  if (payLen && payload) memcpy(outFrame + TB_HDR_LEN, payload, payLen);

  const uint16_t crc = TbCrc16Ccitt(outFrame, totalNoCrc);
  outFrame[totalNoCrc + 0] = (uint8_t)(crc & 0xFF);
  outFrame[totalNoCrc + 1] = (uint8_t)(crc >> 8);

  outLen = total;
  return true;
}

// ============================================================================
// KY-040 fixed state-table encoder (ownprox/buxtronix style)
// NOTE: ISR intentionally NOT IRAM_ATTR (uses digitalRead; avoids linker issues)
// ============================================================================
#define DIR_NONE 0x00
#define DIR_CW   0x10
#define DIR_CCW  0x20

#define R_START     0x3
#define R_CW_BEGIN  0x1
#define R_CW_NEXT   0x0
#define R_CW_FINAL  0x2
#define R_CCW_BEGIN 0x6
#define R_CCW_NEXT  0x4
#define R_CCW_FINAL 0x5

static const uint8_t kTTable[8][4] = {
  {R_CW_NEXT,   R_CW_BEGIN,  R_CW_FINAL,  R_START},
  {R_CW_NEXT,   R_CW_BEGIN,  R_CW_BEGIN,  R_START},
  {R_CW_NEXT,   R_CW_FINAL,  R_CW_FINAL,  (uint8_t)(R_START | DIR_CW)},
  {R_START,     R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  {R_CCW_NEXT,  R_CCW_FINAL, R_CCW_BEGIN, R_START},
  {R_CCW_NEXT,  R_CCW_FINAL, R_CCW_FINAL, (uint8_t)(R_START | DIR_CCW)},
  {R_CCW_NEXT,  R_CCW_BEGIN, R_CCW_BEGIN, R_START},
  {R_START,     R_START,     R_START,     R_START}
};

class Ky040FixedEncoder {
public:
  void begin(uint8_t pinA, uint8_t pinB, bool usePullups = true) {
    _pinA = pinA;
    _pinB = pinB;

    if (usePullups) {
      pinMode(_pinA, INPUT_PULLUP);
      pinMode(_pinB, INPUT_PULLUP);
    } else {
      pinMode(_pinA, INPUT);
      pinMode(_pinB, INPUT);
    }

    _state = R_START;
    _delta = 0;

    attachInterruptArg(digitalPinToInterrupt(_pinA), isrThunk, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_pinB), isrThunk, this, CHANGE);

    seed();
  }

  int32_t readAndClear() {
    noInterrupts();
    int32_t d = _delta;
    _delta = 0;
    interrupts();
    return d;
  }

private:
  uint8_t _pinA = 0, _pinB = 0;
  volatile uint8_t _state = R_START;
  volatile int32_t _delta = 0;

  static void isrThunk(void* arg) {
    ((Ky040FixedEncoder*)arg)->handleIsr();
  }

  void handleIsr() {
    const uint8_t pinstate = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    _state = kTTable[_state & 0x07][pinstate];

    const uint8_t dir = _state & 0x30;
    if (dir == DIR_CW)  _delta++;
    if (dir == DIR_CCW) _delta--;
  }

  void seed() {
    const uint8_t pinstate = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    _state = kTTable[_state & 0x07][pinstate];
    _state &= 0x07;
  }
};

// ============================================================================
// Debounced button (pressed = LOW when wired with pull-up)
// ============================================================================
class DebouncedButton {
public:
  void begin(uint8_t pin, bool useInternalPullup, uint16_t debounceMs = 25) {
    _pin = pin;
    _debounceMs = debounceMs;
    if (useInternalPullup) pinMode(_pin, INPUT_PULLUP);
    else pinMode(_pin, INPUT);
    const bool now = digitalRead(_pin);
    _stable = now;
    _lastStable = now;
    _lastEdgeMs = millis();
  }

  bool fell() {
    const bool now = digitalRead(_pin);
    const uint32_t ms = millis();

    if (now != _stable) {
      if (ms - _lastEdgeMs >= _debounceMs) {
        _lastStable = _stable;
        _stable = now;
        _lastEdgeMs = ms;
        if (_lastStable == true && _stable == false) return true;
      }
    } else {
      _lastEdgeMs = ms;
    }
    return false;
  }

private:
  uint8_t _pin = 0;
  uint16_t _debounceMs = 25;
  uint32_t _lastEdgeMs = 0;
  bool _stable = true;
  bool _lastStable = true;
};

// ============================================================================
// Slew limiter for "marine feel" (ramps throttle + rudder toward setpoints)
// ============================================================================
class SlewLimiter {
public:
  void reset(float value) { _value = value; }
  float value() const { return _value; }

  float update(float target, float dtSec, float rateUp, float rateDown) {
    if (dtSec <= 0.0f) return _value;

    const float diff = target - _value;
    if (diff == 0.0f) return _value;

    const float maxStep = (diff > 0.0f) ? (rateUp * dtSec) : (rateDown * dtSec);
    if (maxStep <= 0.0f) return _value;

    if (fabsf(diff) <= maxStep) _value = target;
    else _value += (diff > 0.0f ? maxStep : -maxStep);

    return _value;
  }

private:
  float _value = 0.0f;
};

// ============================================================================
// INPUTS => TbCmdV1 setpoints (CANON mapping)
// ============================================================================
class TxInputs {
public:
  enum MenuPage : uint8_t {
    MENU_NONE = 0,
    MENU_ROOT,
    MENU_SUBMENU_1,
    MENU_SUBMENU_2,
    MENU_SUBMENU_3
  };

  enum UiAction : uint8_t {
    ACTION_NONE = 0,
    ACTION_TOGGLE_WIFI,
    ACTION_TOGGLE_OTA
  };

  void begin() {
    _encThr.begin(PIN_THROTTLEPOT_A, PIN_THROTTLEPOT_B, true);
    _encRud.begin(PIN_RUDDERPOT_A, PIN_RUDDERPOT_B, true);
    _encMenu.begin(PIN_MENUPOT_A, PIN_MENUPOT_B, true);

    _btnArm.begin(PIN_THROTTLEPOT_BTN, true);
    _btnRudder.begin(PIN_RUDDERPOT_BTN, true);
    _btnMenu.begin(PIN_MENUPOT_BTN, true);

    memset(&_cmd, 0, sizeof(_cmd));
    _armState = false;
    _accIndex = 0;
    _menuPage = MENU_NONE;
    _menuSelection = 0;
    _menuScroll = 0;
    _pendingAction = ACTION_NONE;
  }

  // Update setpoints from encoders/buttons
  void update() {
    const bool menuPressed = _btnMenu.fell();
    (void)_btnRudder.fell(); // RUDDERPOT button intentionally unused for now.

    if (_menuPage != MENU_NONE) {
      updateMenu(menuPressed);
    } else if (menuPressed) {
      enterMenu(MENU_ROOT, 0);
    }

    if (_btnArm.fell()) {
      _armState = !_armState;
      _cmd.arm = _armState ? 1 : 0;
      if (!_armState) {
        // Setpoints go safe immediately on disarm
        _cmd.throttlePct = 0;
        _cmd.rudderPct   = 0;
        _cmd.acc[0] = _cmd.acc[1] = _cmd.acc[2] = _cmd.acc[3] = 0;
      }
    }

    // Optional: you can keep your detent acceleration for big spins
    const int dT = accel((int)_encThr.readAndClear());
    const int dR = accel((int)_encRud.readAndClear());
    const int dA = (_menuPage == MENU_NONE) ? accel((int)_encMenu.readAndClear()) : 0;

    _cmd.throttlePct = (int8_t)clampi((int)_cmd.throttlePct + dT, -100, 100);
    _cmd.rudderPct   = (int8_t)clampi((int)_cmd.rudderPct   + dR, -100, 100);

    if (dA != 0) {
      int v = (int)_cmd.acc[_accIndex] + dA * ACC_STEP;
      _cmd.acc[_accIndex] = (uint8_t)clampi(v, 0, 255);
    }

    if (!_cmd.arm) _cmd.throttlePct = 0;
  }

  const TbCmdV1& setpointCmd() const { return _cmd; }
  uint8_t accIndex() const { return _accIndex; }
  bool menuActive() const { return _menuPage != MENU_NONE; }
  MenuPage menuPage() const { return _menuPage; }
  uint8_t menuSelection() const { return _menuSelection; }
  uint8_t menuScroll() const { return _menuScroll; }
  UiAction consumeAction() {
    const UiAction action = _pendingAction;
    _pendingAction = ACTION_NONE;
    return action;
  }

  const char* menuTitle() const {
    switch (_menuPage) {
      case MENU_ROOT: return "Main Menu";
      case MENU_SUBMENU_1: return "Submenu 1";
      case MENU_SUBMENU_2: return "Submenu 2";
      case MENU_SUBMENU_3: return "Options";
      default: return "";
    }
  }

  uint8_t menuItemCount() const {
    switch (_menuPage) {
      case MENU_ROOT: return 4;
      case MENU_SUBMENU_1:
      case MENU_SUBMENU_2:
      case MENU_SUBMENU_3:
        return 4;
      default:
        return 0;
    }
  }

  const char* menuItemLabel(uint8_t index) const {
    static const char* const rootItems[] = {
      "Exit",
      "Submenu 1",
      "Submenu 2",
      "Options"
    };
    static const char* const submenu1Items[] = {
      "Back",
      "Placeholder A",
      "Placeholder B",
      "Placeholder C"
    };
    static const char* const submenu2Items[] = {
      "Back",
      "Placeholder D",
      "Placeholder E",
      "Placeholder F"
    };
    static const char* const submenu3Items[] = {
      "Back",
      "WIFI",
      "OTA",
      "Placeholder I"
    };

    const char* const* items = nullptr;
    switch (_menuPage) {
      case MENU_ROOT: items = rootItems; break;
      case MENU_SUBMENU_1: items = submenu1Items; break;
      case MENU_SUBMENU_2: items = submenu2Items; break;
      case MENU_SUBMENU_3: items = submenu3Items; break;
      default: return "";
    }

    return (index < menuItemCount()) ? items[index] : "";
  }

private:
  static constexpr uint8_t MENU_VISIBLE_ROWS = 6;

  Ky040FixedEncoder _encThr, _encRud, _encMenu;
  DebouncedButton _btnArm, _btnRudder, _btnMenu;

  TbCmdV1 _cmd{};
  bool _armState = false;
  uint8_t _accIndex = 0;
  MenuPage _menuPage = MENU_NONE;
  uint8_t _menuSelection = 0;
  uint8_t _menuScroll = 0;
  UiAction _pendingAction = ACTION_NONE;

  static constexpr int ACC_STEP = 5;

  static int accel(int detents) {
    const int a = abs(detents);
    if (a >= 6) return detents * 4;
    if (a >= 3) return detents * 2;
    return detents;
  }

  void enterMenu(MenuPage page, uint8_t selected) {
    _menuPage = page;
    _menuSelection = selected;
    _menuScroll = 0;
    clampMenuWindow();
  }

  void updateMenu(bool menuPressed) {
    const int dMenu = accel((int)_encMenu.readAndClear());
    if (dMenu != 0) {
      const int maxIndex = (int)menuItemCount() - 1;
      _menuSelection = (uint8_t)clampi((int)_menuSelection + dMenu, 0, maxIndex);
      clampMenuWindow();
    }

    if (!menuPressed) return;

    if (_menuPage == MENU_ROOT) {
      switch (_menuSelection) {
        case 0:
          _menuPage = MENU_NONE;
          return;
        case 1: enterMenu(MENU_SUBMENU_1, 0); return;
        case 2: enterMenu(MENU_SUBMENU_2, 0); return;
        case 3: enterMenu(MENU_SUBMENU_3, 0); return;
        default:
          _menuPage = MENU_NONE;
          return;
      }
    }

    if (_menuSelection == 0) {
      enterMenu(MENU_ROOT, 0);
      return;
    }

    if (_menuPage == MENU_SUBMENU_3 && _menuSelection == 1) {
      _pendingAction = ACTION_TOGGLE_WIFI;
      return;
    }

    if (_menuPage == MENU_SUBMENU_3 && _menuSelection == 2) {
      _pendingAction = ACTION_TOGGLE_OTA;
      return;
    }
  }

  void clampMenuWindow() {
    if (_menuSelection < _menuScroll) _menuScroll = _menuSelection;
    const uint8_t bottom = (uint8_t)(_menuScroll + MENU_VISIBLE_ROWS - 1);
    if (_menuSelection > bottom) {
      _menuScroll = (uint8_t)(_menuSelection - MENU_VISIBLE_ROWS + 1);
    }
  }
};

// ============================================================================
// RADIO LINK (send + ack telemetry parse)
// ============================================================================
class TxRadioLink {
public:
  bool begin() {
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    if (!radio.begin()) {
      Serial.println("radio.begin() FAILED");
      return false;
    }

    radio.setChannel(RF_CHANNEL);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(true);

    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    radio.openWritingPipe(PIPE_ADDR);
    radio.stopListening();

    _seq = 1;
    _lastSendOk = false;
    _lastAckUpdated = false;
    _lastAckMs = millis();
    memset(&_lastAck, 0, sizeof(_lastAck));
    return true;
  }

  bool sendCmd(const TbCmdV1& cmd) {
    uint8_t frame[TB_MAX_AIR] = {0};
    uint8_t frameLen = 0;

    if (!TbBuildFrame(TB_CMD, 0, _seq++, (const uint8_t*)&cmd, sizeof(cmd), frame, frameLen)) {
      _lastSendOk = false;
      return false;
    }

    _lastSendOk = radio.write(frame, frameLen);
    _lastAckUpdated = false;

    if (_lastSendOk) {
      TbAckV2 ack {};
      if (readAck(ack)) {
        _lastAck = ack;
        _lastAckMs = millis();
        _lastAckUpdated = true;
      }
    }
    return _lastSendOk;
  }

  bool lastSendOk() const { return _lastSendOk; }
  bool lastAckUpdated() const { return _lastAckUpdated; }
  const TbAckV2& lastAck() const { return _lastAck; }
  uint32_t lastAckMs() const { return _lastAckMs; }

private:
  uint8_t _seq = 1;
  bool _lastSendOk = false;
  bool _lastAckUpdated = false;
  uint32_t _lastAckMs = 0;
  TbAckV2 _lastAck{};

  bool readAck(TbAckV2& outAck) {
    if (!radio.isAckPayloadAvailable()) return false;

    const uint8_t len = radio.getDynamicPayloadSize();
    if (len != sizeof(TbAckV2)) {
      uint8_t junk[32] = {0};
      radio.read(junk, min<uint8_t>(len, 32));
      return false;
    }

    radio.read(&outAck, sizeof(outAck));

    if (outAck.ver != TB_VER) return false;
    if (outAck.type != TB_ACK) return false;

    return (TbAckCrc(outAck) == outAck.crc16);
  }
};

// ============================================================================
// WiFi + OTA manager
// ============================================================================
class WifiWindowManager {
public:
  void begin() {
    _active = false;
    _connected = false;
    _otaStarted = false;
    _otaEnabled = false;
    _lastConnectAttemptMs = 0;
    _printedIpThisWindow = false;

    WiFi.mode(WIFI_OFF);
    delay(10);
  }

  void enable() {
    const uint32_t now = millis();
    _active = true;
    _connected = false;
    _printedIpThisWindow = false;

    WiFi.mode(WIFI_STA);
    // Keep WiFi power modest to reduce 2.4 GHz contention with the nRF24 link.
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    WiFi.disconnect(true);
    delay(50);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    _lastConnectAttemptMs = now;

    Serial.println("WiFi ENABLED (connecting...)");
  }

  void disable() {
    if (!_active && WiFi.getMode() == WIFI_OFF) return;
    shutdown();
  }

  void tick() {
    const uint32_t now = millis();

    if (!_active) return;

    _connected = (WiFi.status() == WL_CONNECTED);

    if (!_connected) {
      if (now - _lastConnectAttemptMs >= CONNECT_RETRY_MS) {
        _lastConnectAttemptMs = now;
        WiFi.disconnect(true);
        delay(20);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        WiFi.setSleep(WIFI_PS_MIN_MODEM);
      }
      return;
    }

    if (!_printedIpThisWindow) {
      _printedIpThisWindow = true;
      Serial.print("WiFi connected. IP=");
      Serial.println(WiFi.localIP());
    }

    if (_otaEnabled && !_otaStarted) {
      ArduinoOTA.begin(WiFi.localIP(), "TugbotTx", OTA_PASS, InternalStorage);
      _otaStarted = true;
      Serial.println("ArduinoOTA ready.");
    }

    if (_otaEnabled && _otaStarted) {
      ArduinoOTA.handle();
    }
  }

  bool isActive() const { return _active; }
  bool isConnected() const { return _connected; }
  bool isOtaActive() const { return _otaEnabled; }
  IPAddress ip() const { return WiFi.localIP(); }

  void setOtaEnabled(bool enabled) {
    if (_otaEnabled == enabled) return;
    _otaEnabled = enabled;

    if (!_otaEnabled && _otaStarted) {
      ArduinoOTA.end();
      _otaStarted = false;
      Serial.println("ArduinoOTA stopped.");
    }
  }

private:
  static constexpr uint32_t CONNECT_RETRY_MS = 5000;

  bool _active = false;
  bool _connected = false;
  bool _otaStarted = false;
  bool _otaEnabled = false;
  bool _printedIpThisWindow = false;
  uint32_t _lastConnectAttemptMs = 0;

  void shutdown() {
    if (_otaStarted) {
      ArduinoOTA.end();
    }
    Serial.println("WiFi OFF");
    _active = false;
    _connected = false;
    _otaStarted = false;
    _printedIpThisWindow = false;

    WiFi.disconnect(true);
    delay(20);
    WiFi.mode(WIFI_OFF);
    delay(10);
  }
};

// ============================================================================
// OLED UI
// ============================================================================
class TxUiOled {
public:
  void begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println("OLED init failed (addr 0x3C?)");
      _ok = false;
      return;
    }
    _ok = true;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("TugBot TX booting...");
    display.display();
  }

  void render(const TbCmdV1& setCmd,
              const TbCmdV1& outCmd,
              uint8_t accIndex,
              const TxInputs& inputs,
              bool linkOk,
              const TbAckV2& ack,
              uint16_t vSysAvg_mV,
              uint16_t vPropAvg_mV,
              uint16_t iSysAvg_mA,
              uint32_t ackAgeMs,
              const WifiWindowManager& wifi) {
    if (!_ok) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    if (inputs.menuActive()) {
      renderMenu(inputs, wifi);
      display.display();
      return;
    }

    String l0 = "ARM:";
    l0 += (outCmd.arm ? "ON " : "OFF");
    l0 += "  LINK:";
    l0 += (linkOk ? "OK" : "FAIL");
    printLine(0, l0);

    String l1 = "THR:";
    l1 += String((int)outCmd.throttlePct);
    l1 += "("; l1 += String((int)setCmd.throttlePct); l1 += ") ";
    l1 += "RUD:";
    l1 += String((int)outCmd.rudderPct);
    l1 += "("; l1 += String((int)setCmd.rudderPct); l1 += ")";
    printLine(1, l1);

    String l2 = "ACC";
    l2 += String(accIndex + 1);
    l2 += ": ";
    l2 += String((int)outCmd.acc[accIndex]);
    l2 += " (RUDDER/MENU btn)";
    printLine(2, l2);

    String l3 = "Vsys:";
    l3 += formatVolts(vSysAvg_mV);
    l3 += "  Vpr:";
    l3 += String(vPropAvg_mV);
    printLine(3, l3);

    char iSysBuf[8];
    snprintf(iSysBuf, sizeof(iSysBuf), "%04u", (unsigned int)iSysAvg_mA);
    String l4 = "Isys:";
    l4 += String(iSysBuf);
    l4 += "mA  W:";
    l4 += String(ack.waterRaw);
    printLine(4, l4);

    String l5 = "ok:";
    l5 += String(ack.rxOk);
    l5 += " bad:";
    l5 += String(ack.rxBad);
    l5 += " age:";
    l5 += String(ackAgeMs);
    printLine(5, l5);

    if (!wifi.isActive()) {
      printLine(6, "WiFi:OFF  Btn34/menu");
      printLine(7, String("OTA:") + (wifi.isOtaActive() ? "ON" : "OFF"));
    } else if (!wifi.isConnected()) {
      printLine(6, "WiFi:CONN");
      printLine(7, String("OTA:") + (wifi.isOtaActive() ? "ON" : "OFF"));
    } else {
      printLine(6, String("WiFi:ON OTA:") + (wifi.isOtaActive() ? "ON" : "OFF"));

      IPAddress ip = wifi.ip();
      String ipLine = "IP:";
      ipLine += String(ip[0]); ipLine += ".";
      ipLine += String(ip[1]); ipLine += ".";
      ipLine += String(ip[2]); ipLine += ".";
      ipLine += String(ip[3]);
      printLine(7, ipLine);
    }

    display.display();
  }

private:
  bool _ok = false;

  static String formatVolts(uint16_t milliVolts) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%u.%03uV",
             (unsigned int)(milliVolts / 1000),
             (unsigned int)(milliVolts % 1000));
    return String(buf);
  }

  static void printLine(int row, const String& s) {
    display.setCursor(0, row * 8);
    display.print(s);
  }

  static void printMenuLine(int row, bool selected, const String& label) {
    display.setCursor(0, row * 8);
    display.print(selected ? ">" : " ");
    display.print(label);
  }

  static void renderMenu(const TxInputs& inputs, const WifiWindowManager& wifi) {
    printLine(0, String(inputs.menuTitle()));
    printLine(1, "Turn=scroll Press=sel");

    const uint8_t total = inputs.menuItemCount();
    const uint8_t scroll = inputs.menuScroll();
    const uint8_t selected = inputs.menuSelection();
    const uint8_t visible = min<uint8_t>(total > scroll ? (uint8_t)(total - scroll) : 0, 6);

    for (uint8_t i = 0; i < visible; ++i) {
      const uint8_t itemIndex = (uint8_t)(scroll + i);
      String label = inputs.menuItemLabel(itemIndex);
      if (inputs.menuPage() == TxInputs::MENU_SUBMENU_3) {
        if (itemIndex == 1 && wifi.isActive()) label += " *";
        if (itemIndex == 2 && wifi.isOtaActive()) label += " *";
      }
      printMenuLine((int)i + 2, itemIndex == selected, label);
    }
  }
};

// ============================================================================
// APP
// ============================================================================
class TugbotTxApp {
public:
  void begin() {
    Serial.begin(115200);
    delay(1000);

    _ui.begin();
    _inputs.begin();
    _wifi.begin();

    _btnWifi.begin(PIN_WIFI_BTN, false); // external pull-up

    const uint32_t now = millis();
    _lastSendMs   = now;
    _lastOledMs   = now;
    _lastSerialMs = now;
    _lastRampMs   = now;
    _avgWindowStartMs = now;

    _thrRamp.reset(0.0f);
    _rudRamp.reset(0.0f);

    memset(&_cmdOut, 0, sizeof(_cmdOut));
    memset(&_lastSetCmd, 0, sizeof(_lastSetCmd));

    _radioReady = _radio.begin();
    _lastRadioRetryMs = now;

    logBoth("TX ready (protocol v2) - KY040 table + WiFi/OTA toggles + THR/RUD ramps.");
    if (!_radioReady) {
      logBoth("NRF24 init failed. OLED will stay alive while radio retries.");
    }
  }

  void tick() {
    const uint32_t now = millis();

    maintainRadioLink(now);

    if (_btnWifi.fell()) {
      if (_wifi.isActive()) _wifi.disable();
      else _wifi.enable();
    }

    _wifi.tick();
    maintainWifiConsole();

    if (now - _lastSendMs >= SEND_PERIOD_MS) {
      _lastSendMs = now;

      _inputs.update();
      handleUiActions();
      const TbCmdV1 setCmd = _inputs.setpointCmd();

      applyRamps(setCmd, now);

      const bool ok = _radioReady ? _radio.sendCmd(_cmdOut) : false;
      updateTelemetryAverage(now, ok);

      if (now - _lastSerialMs >= SERIAL_PERIOD_MS) {
        _lastSerialMs = now;
        logOncePerSecond(ok, setCmd);
      }

      _lastSetCmd = setCmd;
    }

    if (now - _lastOledMs >= OLED_PERIOD_MS) {
      _lastOledMs = now;
      const uint32_t ackAge = now - _radio.lastAckMs();
      const TbAckV2& rawAck = _radio.lastAck();
      uint16_t vSysOut_mV = rawAck.vSys_mV;
      uint16_t vPropOut_mV = rawAck.vProp_mV;
      uint16_t iSysOut_mA = rawAck.iSys_mA;
      getDisplayTelemetry(vSysOut_mV, vPropOut_mV, iSysOut_mA);
      _ui.render(_lastSetCmd,
                 _cmdOut,
                 _inputs.accIndex(),
                 _inputs,
                 _radio.lastSendOk(),
                 rawAck,
                 vSysOut_mV,
                 vPropOut_mV,
                 iSysOut_mA,
                 ackAge,
                 _wifi);
    }
  }

private:
  static constexpr uint32_t SEND_PERIOD_MS   = 50;
  static constexpr uint32_t OLED_PERIOD_MS   = 200;
  static constexpr uint32_t SERIAL_PERIOD_MS = 1000;
  static constexpr uint32_t TELEMETRY_AVG_MS = 10000;
  static constexpr uint16_t CONSOLE_PORT = 23;

  // Ramp tuning defaults (pct per second).
  static constexpr float THR_RATE_UP_PPS   = 35.0f;   // slow acceleration
  static constexpr float THR_RATE_DOWN_PPS = 80.0f;   // faster decel (safety)
  static constexpr float RUD_RATE_PPS      = 220.0f;  // rudder ramp

  TxInputs          _inputs;
  TxRadioLink       _radio;
  TxUiOled          _ui;
  WifiWindowManager _wifi;

  DebouncedButton   _btnWifi;

  SlewLimiter _thrRamp;
  SlewLimiter _rudRamp;

  uint32_t _lastSendMs = 0;
  uint32_t _lastOledMs = 0;
  uint32_t _lastSerialMs = 0;
  uint32_t _lastRampMs = 0;
  uint32_t _avgWindowStartMs = 0;

  TbCmdV1 _cmdOut{};
  TbCmdV1 _lastSetCmd{};

  uint32_t _sumVSys_mV = 0;
  uint32_t _sumVProp_mV = 0;
  uint32_t _sumISys_mA = 0;
  uint16_t _avgSamples = 0;
  bool _hasAveragedTelemetry = false;
  uint16_t _avgVSys_mV = 0;
  uint16_t _avgVProp_mV = 0;
  uint16_t _avgISys_mA = 0;
  bool _radioReady = false;
  uint32_t _lastRadioRetryMs = 0;
  float _thrRateUpPps = THR_RATE_UP_PPS;
  float _thrRateDownPps = THR_RATE_DOWN_PPS;
  float _rudRatePps = RUD_RATE_PPS;
  WiFiServer _consoleServer{CONSOLE_PORT};
  WiFiClient _consoleClient{};
  bool _consoleServerStarted = false;
  char _consoleLineBuf[128] = {0};
  uint8_t _consoleLineLen = 0;
  uint8_t _telnetSkipBytes = 0;
  bool _consoleTelemetryEnabled = true;

  void handleUiActions() {
    switch (_inputs.consumeAction()) {
      case TxInputs::ACTION_TOGGLE_WIFI:
        if (_wifi.isActive()) _wifi.disable();
        else _wifi.enable();
        break;
      case TxInputs::ACTION_TOGGLE_OTA:
        _wifi.setOtaEnabled(!_wifi.isOtaActive());
        break;
      case TxInputs::ACTION_NONE:
      default:
        break;
    }
  }

  void applyRamps(const TbCmdV1& setCmd, uint32_t nowMs) {
    _cmdOut.arm = setCmd.arm;
    _cmdOut.acc[0] = setCmd.acc[0];
    _cmdOut.acc[1] = setCmd.acc[1];
    _cmdOut.acc[2] = setCmd.acc[2];
    _cmdOut.acc[3] = setCmd.acc[3];

    if (!setCmd.arm) {
      _thrRamp.reset(0.0f);
      _rudRamp.reset(0.0f);
      _cmdOut.throttlePct = 0;
      _cmdOut.rudderPct   = 0;
      return;
    }

    uint32_t dtMs = nowMs - _lastRampMs;
    _lastRampMs = nowMs;
    if (dtMs > 250) dtMs = 250;

    const float dtSec = (float)dtMs / 1000.0f;

    const float thr = _thrRamp.update((float)setCmd.throttlePct, dtSec, _thrRateUpPps, _thrRateDownPps);
    const float rud = _rudRamp.update((float)setCmd.rudderPct,   dtSec, _rudRatePps,   _rudRatePps);

    _cmdOut.throttlePct = (int8_t)clampi((int)lroundf(thr), -100, 100);
    _cmdOut.rudderPct   = (int8_t)clampi((int)lroundf(rud), -100, 100);
  }

  void logOncePerSecond(bool lastSendOk, const TbCmdV1& setCmd) {
    const TbAckV2& ack = _radio.lastAck();
    uint16_t vSysOut_mV = ack.vSys_mV;
    uint16_t vPropOut_mV = ack.vProp_mV;
    uint16_t iSysOut_mA = ack.iSys_mA;
    getDisplayTelemetry(vSysOut_mV, vPropOut_mV, iSysOut_mA);
    char line[256];
    snprintf(line, sizeof(line),
             "TX %s thr=%d(%d) rud=%d(%d) arm=%d acc%u=%d",
             lastSendOk ? "OK" : "FAIL",
             (int)_cmdOut.throttlePct, (int)setCmd.throttlePct,
             (int)_cmdOut.rudderPct,   (int)setCmd.rudderPct,
             (int)_cmdOut.arm,
             (unsigned int)(_inputs.accIndex() + 1),
             (int)_cmdOut.acc[_inputs.accIndex()]);

    String msg(line);

    if (lastSendOk) {
      char ackBuf[128];
      snprintf(ackBuf, sizeof(ackBuf),
               " | ACK st=%d ok=%u bad=%u Vsys(V)=%.3f Isys(mA)=%04u",
               (int)ack.status,
               (unsigned int)ack.rxOk,
               (unsigned int)ack.rxBad,
               vSysOut_mV / 1000.0f,
               (unsigned int)iSysOut_mA);
      msg += ackBuf;
    }

    if (_wifi.isActive()) {
      char wifiBuf[96];
      snprintf(wifiBuf, sizeof(wifiBuf), " | WiFi %s OTA %s",
               _wifi.isConnected() ? "ON" : "CONN",
               _wifi.isOtaActive() ? "ON" : "OFF");
      msg += wifiBuf;
      if (_wifi.isConnected()) {
        msg += " IP=";
        msg += _wifi.ip().toString();
      }
    }

    logBoth(msg.c_str());
    if (_consoleTelemetryEnabled) {
      consolePrintLine(msg.c_str());
    }
  }

  void updateTelemetryAverage(uint32_t now, bool sendOk) {
    if (sendOk && _radio.lastAckUpdated()) {
      const TbAckV2& ack = _radio.lastAck();
      _sumVSys_mV += ack.vSys_mV;
      _sumVProp_mV += ack.vProp_mV;
      _sumISys_mA += ack.iSys_mA;
      _avgSamples++;
    }

    while ((now - _avgWindowStartMs) >= TELEMETRY_AVG_MS) {
      if (_avgSamples > 0) {
        _avgVSys_mV = (uint16_t)(_sumVSys_mV / _avgSamples);
        _avgVProp_mV = (uint16_t)(_sumVProp_mV / _avgSamples);
        _avgISys_mA = (uint16_t)(_sumISys_mA / _avgSamples);
        _hasAveragedTelemetry = true;
      }
      _sumVSys_mV = 0;
      _sumVProp_mV = 0;
      _sumISys_mA = 0;
      _avgSamples = 0;
      _avgWindowStartMs += TELEMETRY_AVG_MS;
    }
  }

  void getDisplayTelemetry(uint16_t& vSys_mV, uint16_t& vProp_mV, uint16_t& iSys_mA) const {
    if (!_hasAveragedTelemetry) return;
    vSys_mV = _avgVSys_mV;
    vProp_mV = _avgVProp_mV;
    iSys_mA = _avgISys_mA;
  }

  void maintainRadioLink(uint32_t now) {
    static constexpr uint32_t RADIO_RETRY_MS = 2000;
    if (_radioReady) return;
    if (now - _lastRadioRetryMs < RADIO_RETRY_MS) return;

    _lastRadioRetryMs = now;
    _radioReady = _radio.begin();
    if (_radioReady) {
      logBoth("NRF24 init recovered.");
    } else {
      Serial.println("NRF24 retry failed.");
    }
  }

  void maintainWifiConsole() {
    if (!_wifi.isConnected()) {
      stopConsoleServer();
      return;
    }

    if (!_consoleServerStarted) {
      _consoleServer.begin();
      _consoleServer.setNoDelay(true);
      _consoleServerStarted = true;
      Serial.printf("WiFi terminal listening on port %u\r\n", (unsigned int)CONSOLE_PORT);
    }

    WiFiClient candidate = _consoleServer.available();
    if (candidate) {
      if (_consoleClient && _consoleClient.connected()) {
        _consoleClient.println("Another client connected. Closing this session.");
        _consoleClient.stop();
      }
      _consoleClient = candidate;
      _consoleClient.setNoDelay(true);
      _consoleLineLen = 0;
      _telnetSkipBytes = 0;
      printConsoleHelp();
      printConsoleStatus();
    }

    if (!_consoleClient || !_consoleClient.connected()) return;

    while (_consoleClient.available()) {
      const uint8_t raw = (uint8_t)_consoleClient.read();

      if (_telnetSkipBytes > 0) {
        _telnetSkipBytes--;
        continue;
      }

      // Ignore Telnet IAC negotiation bytes so terminal clients can talk plain text.
      if (raw == 255U) {
        _telnetSkipBytes = 2;
        continue;
      }

      if (raw == 0U) continue;

      const char ch = (char)raw;
      if (ch == '\r' || ch == '\n') {
        _consoleLineBuf[_consoleLineLen] = '\0';
        if (_consoleLineLen > 0) processConsoleCommand(_consoleLineBuf);
        _consoleLineLen = 0;
        continue;
      }
      if (ch < 32 || ch > 126) continue;

      if (_consoleLineLen + 1 < sizeof(_consoleLineBuf)) {
        _consoleLineBuf[_consoleLineLen++] = ch;
      }
    }
  }

  void stopConsoleServer() {
    if (_consoleClient) _consoleClient.stop();
    if (_consoleServerStarted) {
      _consoleServer.stop();
      _consoleServerStarted = false;
    }
    _consoleLineLen = 0;
    _telnetSkipBytes = 0;
  }

  void consolePrintf(const char* fmt, ...) {
    if (!_consoleClient || !_consoleClient.connected()) return;
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    _consoleClient.print(buf);
  }

  void consolePrintLine(const char* s) {
    if (!_consoleClient || !_consoleClient.connected()) return;
    _consoleClient.println(s);
  }

  void printConsoleHelp() {
    consolePrintLine("");
    consolePrintLine("TugBot TX WiFi terminal");
    consolePrintLine("Commands: help, status, vars, get <name>, set <name> <value>");
    consolePrintLine("          wifi on|off, ota on|off, telemetry on|off, reboot");
    consolePrintLine("Vars: thr_rate_up, thr_rate_down, rud_rate");
  }

  void printConsoleStatus() {
    const TbAckV2& ack = _radio.lastAck();
    const uint32_t ackAge = millis() - _radio.lastAckMs();
    consolePrintf("link=%s ackAge=%lums radioReady=%u wifi=%s ota=%s\r\n",
                  _radio.lastSendOk() ? "OK" : "FAIL",
                  (unsigned long)ackAge,
                  (unsigned int)_radioReady,
                  _wifi.isActive() ? (_wifi.isConnected() ? "connected" : "starting") : "off",
                  _wifi.isOtaActive() ? "on" : "off");
    consolePrintf("thr_out=%d thr_set=%d rud_out=%d rud_set=%d arm=%u acc1=%u\r\n",
                  (int)_cmdOut.throttlePct,
                  (int)_lastSetCmd.throttlePct,
                  (int)_cmdOut.rudderPct,
                  (int)_lastSetCmd.rudderPct,
                  (unsigned int)_cmdOut.arm,
                  (unsigned int)_cmdOut.acc[0]);
    consolePrintf("rx_ok=%u rx_bad=%u vsys=%umV vprop=%umV isys=%umA water=%u\r\n",
                  (unsigned int)ack.rxOk,
                  (unsigned int)ack.rxBad,
                  (unsigned int)ack.vSys_mV,
                  (unsigned int)ack.vProp_mV,
                  (unsigned int)ack.iSys_mA,
                  (unsigned int)ack.waterRaw);
    consolePrintf("telemetry=%s\r\n", _consoleTelemetryEnabled ? "on" : "off");
    printConsoleVars();
  }

  void printConsoleVars() {
    consolePrintf("thr_rate_up=%.2f\r\n", _thrRateUpPps);
    consolePrintf("thr_rate_down=%.2f\r\n", _thrRateDownPps);
    consolePrintf("rud_rate=%.2f\r\n", _rudRatePps);
  }

  bool printVarValue(const char* name) {
    if (strcmp(name, "thr_rate_up") == 0) {
      consolePrintf("thr_rate_up=%.2f\r\n", _thrRateUpPps);
      return true;
    }
    if (strcmp(name, "thr_rate_down") == 0) {
      consolePrintf("thr_rate_down=%.2f\r\n", _thrRateDownPps);
      return true;
    }
    if (strcmp(name, "rud_rate") == 0) {
      consolePrintf("rud_rate=%.2f\r\n", _rudRatePps);
      return true;
    }
    return false;
  }

  bool setVarValue(const char* name, const char* value) {
    if (strcmp(name, "thr_rate_up") == 0) {
      const float v = atof(value);
      if (v <= 0.0f) return false;
      _thrRateUpPps = v;
      return true;
    }
    if (strcmp(name, "thr_rate_down") == 0) {
      const float v = atof(value);
      if (v <= 0.0f) return false;
      _thrRateDownPps = v;
      return true;
    }
    if (strcmp(name, "rud_rate") == 0) {
      const float v = atof(value);
      if (v <= 0.0f) return false;
      _rudRatePps = v;
      return true;
    }
    return false;
  }

  void processConsoleCommand(char* line) {
    while (*line == ' ' || *line == '\t') line++;
    char* end = line + strlen(line);
    while (end > line && (end[-1] == ' ' || end[-1] == '\t')) --end;
    *end = '\0';

    for (char* p = line; *p; ++p) *p = (char)tolower((unsigned char)*p);
    if (*line == '\0') return;

    consolePrintf("> %s\r\n", line);

    char* save = nullptr;
    char* cmd = strtok_r(line, " \t", &save);
    if (cmd == nullptr) return;

    if (strcmp(cmd, "help") == 0) {
      printConsoleHelp();
      return;
    }
    if (strcmp(cmd, "status") == 0) {
      printConsoleStatus();
      return;
    }
    if (strcmp(cmd, "vars") == 0) {
      printConsoleVars();
      return;
    }
    if (strcmp(cmd, "get") == 0) {
      char* name = strtok_r(nullptr, " \t", &save);
      if (name == nullptr || !printVarValue(name)) {
        consolePrintLine("Unknown variable.");
      }
      return;
    }
    if (strcmp(cmd, "set") == 0) {
      char* name = strtok_r(nullptr, " \t", &save);
      char* value = strtok_r(nullptr, " \t", &save);
      if (name == nullptr || value == nullptr || !setVarValue(name, value)) {
        consolePrintLine("Set failed. Usage: set <name> <value>");
        return;
      }
      printVarValue(name);
      return;
    }
    if (strcmp(cmd, "wifi") == 0) {
      char* state = strtok_r(nullptr, " \t", &save);
      if (state == nullptr) {
        consolePrintLine("Usage: wifi on|off");
        return;
      }
      if (strcmp(state, "on") == 0) {
        _wifi.enable();
        consolePrintLine("WiFi enabling.");
        return;
      }
      if (strcmp(state, "off") == 0) {
        _wifi.disable();
        consolePrintLine("WiFi disabled.");
        return;
      }
      consolePrintLine("Usage: wifi on|off");
      return;
    }
    if (strcmp(cmd, "ota") == 0) {
      char* state = strtok_r(nullptr, " \t", &save);
      if (state == nullptr) {
        consolePrintLine("Usage: ota on|off");
        return;
      }
      if (strcmp(state, "on") == 0) {
        _wifi.setOtaEnabled(true);
        consolePrintLine("OTA enabled.");
        return;
      }
      if (strcmp(state, "off") == 0) {
        _wifi.setOtaEnabled(false);
        consolePrintLine("OTA disabled.");
        return;
      }
      consolePrintLine("Usage: ota on|off");
      return;
    }
    if (strcmp(cmd, "telemetry") == 0) {
      char* state = strtok_r(nullptr, " \t", &save);
      if (state == nullptr) {
        consolePrintLine("Usage: telemetry on|off");
        return;
      }
      if (strcmp(state, "on") == 0) {
        _consoleTelemetryEnabled = true;
        consolePrintLine("Telemetry output enabled.");
        return;
      }
      if (strcmp(state, "off") == 0) {
        _consoleTelemetryEnabled = false;
        consolePrintLine("Telemetry output ignored.");
        return;
      }
      consolePrintLine("Usage: telemetry on|off");
      return;
    }
    if (strcmp(cmd, "reboot") == 0) {
      consolePrintLine("Rebooting transmitter...");
      delay(50);
      ESP.restart();
      return;
    }

    consolePrintLine("Unknown command. Type: help");
  }
};

// ============================================================================
// Arduino entrypoints
// ============================================================================
static TugbotTxApp g_app;
void setup() { g_app.begin(); }
void loop()  { g_app.tick(); }
