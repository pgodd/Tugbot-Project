/*
  TugBot TX (ESP32-WROOM) — Protocol v2 + OLED + NRF24 + KY-040 (state-table) + On-demand WiFi/OTA
  ------------------------------------------------------------------------------------------------
  CANON control mapping:
    - Throttle: Encoder 2 (ENC2 A=26 B=14)
    - Rudder:   Encoder 1 (ENC1 A=32 B=33)
    - ARM:      ENC2 button (PIN_ENC2_BTN = 13)  [throttle actuator button]
    - Accessory select: NEXT = ENC1 button (25), PREV = ENC3 button (4)
    - Accessory value: Encoder 3 (ENC3 A=16 B=17)

  WiFi/OTA:
    - Dedicated WiFi button on GPIO34 (external 10k pull-up to 3.3V, button to GND)
    - WiFi/OTA OFF by default; press WiFi button to open OTA window (3 minutes)
    - WiFi TX power reduced during window to help nRF coexistence

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
static const uint8_t PIN_ENC1_A   = 32;
static const uint8_t PIN_ENC1_B   = 33;
static const uint8_t PIN_ENC1_BTN = 25;

static const uint8_t PIN_ENC2_A   = 26;
static const uint8_t PIN_ENC2_B   = 14;
static const uint8_t PIN_ENC2_BTN = 13;

static const uint8_t PIN_ENC3_A   = 16;
static const uint8_t PIN_ENC3_B   = 17;
static const uint8_t PIN_ENC3_BTN = 4;

// Dedicated WiFi window button (INPUT-ONLY pin; needs external pull-up)
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
    _stable = true;
    _lastStable = true;
    _lastEdgeMs = 0;
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
  void begin() {
    _encThr.begin(PIN_ENC2_A, PIN_ENC2_B, true);
    _encRud.begin(PIN_ENC1_A, PIN_ENC1_B, true);
    _encAcc.begin(PIN_ENC3_A, PIN_ENC3_B, true);

    _btnArm.begin(PIN_ENC2_BTN, true);
    _btnAccNext.begin(PIN_ENC1_BTN, true);
    _btnAccPrev.begin(PIN_ENC3_BTN, true);

    memset(&_cmd, 0, sizeof(_cmd));
    _armState = false;
    _accIndex = 0;
  }

  // Update setpoints from encoders/buttons
  void update() {
    if (_btnAccPrev.fell()) _accIndex = (_accIndex == 0) ? 3 : (uint8_t)(_accIndex - 1);
    if (_btnAccNext.fell()) _accIndex = (uint8_t)((_accIndex + 1) & 0x03);

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
    const int dA = accel((int)_encAcc.readAndClear());

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

private:
  Ky040FixedEncoder _encThr, _encRud, _encAcc;
  DebouncedButton _btnArm, _btnAccNext, _btnAccPrev;

  TbCmdV1 _cmd{};
  bool _armState = false;
  uint8_t _accIndex = 0;

  static constexpr int ACC_STEP = 5;

  static int accel(int detents) {
    const int a = abs(detents);
    if (a >= 6) return detents * 4;
    if (a >= 3) return detents * 2;
    return detents;
  }
};

// ============================================================================
// RADIO LINK (send + ack telemetry parse)
// ============================================================================
class TxRadioLink {
public:
  void begin() {
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    if (!radio.begin()) {
      Serial.println("radio.begin() FAILED");
      while (true) { delay(1000); }
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
// WiFi/OTA window manager
// ============================================================================
class WifiWindowManager {
public:
  void begin() {
    _active = false;
    _connected = false;
    _otaStarted = false;
    _windowEndMs = 0;
    _lastConnectAttemptMs = 0;
    _printedIpThisWindow = false;

    WiFi.mode(WIFI_OFF);
    delay(10);
  }

  void enableFor(uint32_t durationMs) {
    const uint32_t now = millis();
    _windowEndMs = now + durationMs;
    _active = true;
    _connected = false;
    _otaStarted = false;
    _printedIpThisWindow = false;

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    WiFi.disconnect(true);
    delay(50);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    _lastConnectAttemptMs = now;

    Serial.println("WiFi window ENABLED (connecting...)");
  }

  void tick() {
    const uint32_t now = millis();

    if (!_active) return;

    if ((int32_t)(now - _windowEndMs) >= 0) {
      shutdown();
      return;
    }

    _connected = (WiFi.status() == WL_CONNECTED);

    if (!_connected) {
      if (now - _lastConnectAttemptMs >= CONNECT_RETRY_MS) {
        _lastConnectAttemptMs = now;
        WiFi.disconnect(true);
        delay(20);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
      return;
    }

    if (!_printedIpThisWindow) {
      _printedIpThisWindow = true;
      Serial.print("WiFi connected. IP=");
      Serial.println(WiFi.localIP());
    }

    if (!_otaStarted) {
      ArduinoOTA.begin(WiFi.localIP(), "TugbotTx", OTA_PASS, InternalStorage);
      _otaStarted = true;
      Serial.println("ArduinoOTA ready.");
    }

    ArduinoOTA.handle();
  }

  bool isActive() const { return _active; }
  bool isConnected() const { return _connected; }
  uint32_t remainingMs() const {
    if (!_active) return 0;
    const uint32_t now = millis();
    if ((int32_t)(now - _windowEndMs) >= 0) return 0;
    return _windowEndMs - now;
  }

  IPAddress ip() const { return WiFi.localIP(); }

private:
  static constexpr uint32_t CONNECT_RETRY_MS = 5000;

  bool _active = false;
  bool _connected = false;
  bool _otaStarted = false;
  bool _printedIpThisWindow = false;

  uint32_t _windowEndMs = 0;
  uint32_t _lastConnectAttemptMs = 0;

  void shutdown() {
    Serial.println("WiFi window EXPIRED -> WiFi OFF");
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
    l2 += " (BTN1/3 sel)";
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
      printLine(6, "WiFi:OFF  (BTN WiFi)");
      printLine(7, "OTA:OFF  Btn34->ON");
    } else {
      uint32_t remS = wifi.remainingMs() / 1000;
      if (!wifi.isConnected()) {
        String w = "WiFi:CONN ";
        w += String(remS);
        w += "s";
        printLine(6, w);
        printLine(7, "OTA:WAIT  Btn34=+3m");
      } else {
        String w = "WiFi:ON ";
        w += String(remS);
        w += "s";
        printLine(6, w);

        IPAddress ip = wifi.ip();
        String ipLine = "IP:";
        ipLine += String(ip[0]); ipLine += ".";
        ipLine += String(ip[1]); ipLine += ".";
        ipLine += String(ip[2]); ipLine += ".";
        ipLine += String(ip[3]);
        printLine(7, ipLine);
      }
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
    _radio.begin();
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

    Serial.println("TX ready (protocol v2) — KY040 table + WiFi window + THR/RUD ramps.");
  }

  void tick() {
    const uint32_t now = millis();

    if (_btnWifi.fell()) {
      _wifi.enableFor(WIFI_WINDOW_MS);
    }

    _wifi.tick();

    if (now - _lastSendMs >= SEND_PERIOD_MS) {
      _lastSendMs = now;

      _inputs.update();
      const TbCmdV1 setCmd = _inputs.setpointCmd();

      applyRamps(setCmd, now);

      const bool ok = _radio.sendCmd(_cmdOut);
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
  static constexpr uint32_t WIFI_WINDOW_MS   = 180000; // 3 minutes

  // Ramp tuning (pct per second). Tweak these to taste.
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

    const float thr = _thrRamp.update((float)setCmd.throttlePct, dtSec, THR_RATE_UP_PPS, THR_RATE_DOWN_PPS);
    const float rud = _rudRamp.update((float)setCmd.rudderPct,   dtSec, RUD_RATE_PPS,    RUD_RATE_PPS);

    _cmdOut.throttlePct = (int8_t)clampi((int)lroundf(thr), -100, 100);
    _cmdOut.rudderPct   = (int8_t)clampi((int)lroundf(rud), -100, 100);
  }

  void logOncePerSecond(bool lastSendOk, const TbCmdV1& setCmd) {
    const TbAckV2& ack = _radio.lastAck();
    uint16_t vSysOut_mV = ack.vSys_mV;
    uint16_t vPropOut_mV = ack.vProp_mV;
    uint16_t iSysOut_mA = ack.iSys_mA;
    getDisplayTelemetry(vSysOut_mV, vPropOut_mV, iSysOut_mA);

    Serial.print("TX ");
    Serial.print(lastSendOk ? "OK " : "FAIL ");

    Serial.print("thr="); Serial.print((int)_cmdOut.throttlePct);
    Serial.print("(");    Serial.print((int)setCmd.throttlePct); Serial.print(")");

    Serial.print(" rud="); Serial.print((int)_cmdOut.rudderPct);
    Serial.print("(");     Serial.print((int)setCmd.rudderPct); Serial.print(")");

    Serial.print(" arm="); Serial.print((int)_cmdOut.arm);
    Serial.print(" acc"); Serial.print(_inputs.accIndex() + 1);
    Serial.print("="); Serial.print((int)_cmdOut.acc[_inputs.accIndex()]);

    if (lastSendOk) {
      Serial.print(" | ACK st="); Serial.print((int)ack.status);
      Serial.print(" ok=");       Serial.print(ack.rxOk);
      Serial.print(" bad=");      Serial.print(ack.rxBad);
      Serial.print(" Vsys(V)=");  Serial.print(vSysOut_mV / 1000.0f, 3);
      Serial.print(" Isys(mA)=");
      if (iSysOut_mA < 1000) Serial.print('0');
      if (iSysOut_mA < 100)  Serial.print('0');
      if (iSysOut_mA < 10)   Serial.print('0');
      Serial.print(iSysOut_mA);
    }

    if (_wifi.isActive()) {
      Serial.print(" | WiFiWin ");
      Serial.print(_wifi.isConnected() ? "ON " : "CONN ");
      Serial.print((_wifi.remainingMs() / 1000));
      Serial.print("s");
      if (_wifi.isConnected()) {
        Serial.print(" IP=");
        Serial.print(_wifi.ip());
      }
    }

    Serial.println();
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
};

// ============================================================================
// Arduino entrypoints
// ============================================================================
static TugbotTxApp g_app;
void setup() { g_app.begin(); }
void loop()  { g_app.tick(); }
