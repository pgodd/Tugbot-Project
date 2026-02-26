/*
  TugBot RX (Arduino Mega) — Protocol v2 + Actuation + Telemetry (OOP refactor)
  -----------------------------------------------------------------------------
  Goal: behaviour-preserving refactor into classes, Arduino-IDE friendly (single .ino)

  Notes:
  - This keeps your on-air protocol and telemetry exactly the same.
  - One small behavioural/accounting change is DEFAULT ON:
      rxBad is counted once per received packet (not double-incremented in some branches).
    If you want the *exact old counter semantics*, set TB_COUNT_BAD_ONCE to 0 below.
*/

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <math.h>

// =============================================================================
// CONFIG SWITCHES
// =============================================================================
#define TB_COUNT_BAD_ONCE  1   // 1 = count g_rxBad once per received packet (recommended)
#define TB_SERIAL_WAIT     1   // 1 = while(!Serial) {} (your current behaviour)
#define TB_DEBUG_PRINTS    1   // 1 = print ACS Vzero calibration

// =============================================================================
// CANON RX PINS (Mega)
// =============================================================================
static const uint8_t PIN_NRF24_CE   = 48;
static const uint8_t PIN_NRF24_CSN  = 49;

// BTS7960
static const uint8_t PIN_BTS_LEN    = 23;
static const uint8_t PIN_BTS_REN    = 25;
static const uint8_t PIN_BTS_LPWM   = 6;
static const uint8_t PIN_BTS_RPWM   = 4;

// Accessories
static const uint8_t PIN_PWM_ACC_1  = 2;
static const uint8_t PIN_PWM_ACC_2  = 5;
static const uint8_t PIN_PWM_ACC_3  = 7;
static const uint8_t PIN_PWM_ACC_4  = 10;

// Rudder
static const uint8_t PIN_RUDDER_SERVO = 3;

// Analog sensors (CANON)
static const uint8_t PIN_A_CURRENT_SYS = A0; // system current (ACS712-05B)
static const uint8_t PIN_A_V_SYS       = A1;
static const uint8_t PIN_A_WATER       = A2;
static const uint8_t PIN_A_V_PROP      = A3;
static const uint8_t PIN_TEMP_MOTOR    = A8;
static const uint8_t PIN_TEMP_SPDCNTRL = A9;

// =============================================================================
// RADIO
// =============================================================================
RF24 radio(PIN_NRF24_CE, PIN_NRF24_CSN);
static const uint8_t PIPE_ADDR[6] = "tugbt"; // 5-byte on-air pipe
static constexpr uint8_t RF_CHANNEL = 124;

// =============================================================================
// PROTOCOL v2
// =============================================================================
static constexpr uint8_t TB_MAX_AIR = 32;
static constexpr uint8_t TB_VER     = 2;

enum TbMsgType : uint8_t {
  TB_CMD  = 1,
  TB_PING = 2,
  TB_ACK  = 4
};

enum TbStatus : uint8_t {
  TB_S_OK       = 0,
  TB_S_BAD_VER  = 1,
  TB_S_BAD_LEN  = 2,
  TB_S_BAD_CRC  = 3,
  TB_S_BAD_TYPE = 4
};

#pragma pack(push, 1)
struct TbHdr {
  uint8_t ver;
  uint8_t type;
  uint8_t flags;
  uint8_t seq;
  uint8_t len;   // payload length
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

  uint16_t iSys_mA;  // system current

  int16_t  tMotor_cC;
  int16_t  tEsc_cC;
  uint16_t waterRaw;

  uint16_t crc16;
};
#pragma pack(pop)

static constexpr uint8_t TB_HDR_LEN = sizeof(TbHdr);
static constexpr uint8_t TB_CRC_LEN = 2;
static constexpr uint8_t TB_MAX_PAY = (uint8_t)(TB_MAX_AIR - TB_HDR_LEN - TB_CRC_LEN);
static constexpr uint8_t TB_CMD_LEN = sizeof(TbCmdV1);

// =============================================================================
// UTIL
// =============================================================================
static int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
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

static TbStatus TbParseFrame(const uint8_t* frame, uint8_t frameLen,
                             TbHdr& outHdr, const uint8_t*& outPayload, uint8_t& outPayLen) {
  if (frameLen < TB_HDR_LEN + TB_CRC_LEN) return TB_S_BAD_LEN;

  memcpy(&outHdr, frame, TB_HDR_LEN);
  if (outHdr.ver != TB_VER) return TB_S_BAD_VER;

  const uint8_t expected = (uint8_t)(TB_HDR_LEN + outHdr.len + TB_CRC_LEN);
  if (expected != frameLen) return TB_S_BAD_LEN;
  if (outHdr.len > TB_MAX_PAY) return TB_S_BAD_LEN;

  const uint8_t totalNoCrc = (uint8_t)(TB_HDR_LEN + outHdr.len);
  const uint16_t got = (uint16_t)frame[totalNoCrc + 0] | ((uint16_t)frame[totalNoCrc + 1] << 8);
  const uint16_t cal = TbCrc16Ccitt(frame, totalNoCrc);
  if (got != cal) return TB_S_BAD_CRC;

  outPayload = frame + TB_HDR_LEN;
  outPayLen  = outHdr.len;
  return TB_S_OK;
}

// =============================================================================
// TELEMETRY SAMPLER (CANON)
// =============================================================================
struct Telemetry {
  uint16_t vSys_mV = 0;
  uint16_t vProp_mV = 0;
  uint16_t iSys_mA = 0;
  int16_t  tMotor_cC = 0;
  int16_t  tEsc_cC = 0;
  uint16_t waterRaw = 0;
};

class TelemetrySampler {
public:
  void begin() {
    calibrateAcsZero(2000);
  }

  Telemetry read() {
    const uint16_t adcVsys   = analogRead(PIN_A_V_SYS);
    const uint16_t adcVprop  = analogRead(PIN_A_V_PROP);
    const uint16_t adcIsys   = analogRead(PIN_A_CURRENT_SYS);
    const uint16_t adcWater  = analogRead(PIN_A_WATER);
    const uint16_t adcTMotor = analogRead(PIN_TEMP_MOTOR);
    const uint16_t adcTEsc   = analogRead(PIN_TEMP_SPDCNTRL);

    Telemetry t;
    t.vSys_mV  = adcToBatteryMilliVolts(adcVsys);
    t.vProp_mV = adcToBatteryMilliVolts(adcVprop);
    t.iSys_mA  = adcToSystemMilliAmpsAcs712(adcIsys);

    const float motC = adcToTempC_NoOffset(adcTMotor) + T_MOTOR_OFFSET_C;
    const float escC = adcToTempC_NoOffset(adcTEsc)   + T_ESC_OFFSET_C;
    t.tMotor_cC = tempCToCentiDegC(motC);
    t.tEsc_cC   = tempCToCentiDegC(escC);

    t.waterRaw = adcWater;
    return t;
  }

private:
  // --- CANON conversion constants
  static constexpr float VREF_CAL = 5.136f;

  static constexpr float VDIV_R1   = 100000.0f;
  static constexpr float VDIV_R2   = 10000.0f;
  static constexpr float VDIV_GAIN = (VDIV_R1 + VDIV_R2) / VDIV_R2;

  static constexpr float ACS_MV_PER_A = 185.0f; // ACS712-05B
  float _acsVzero = 2.50f;

  static constexpr float T_RSERIES = 10000.0f;
  static constexpr float T_R0      = 10000.0f;
  static constexpr float T_BETA    = 3950.0f;
  static constexpr float T0_K      = 298.15f;

  static constexpr float T_MOTOR_OFFSET_C = -2.754f;
  static constexpr float T_ESC_OFFSET_C   = -3.615f;

  static float adcToVolts(uint16_t adc) {
    return (float)adc * (VREF_CAL / 1023.0f);
  }

  // Keep your preferred model form (uses 1024 in the multiplier)
  static uint16_t adcToBatteryMilliVolts(uint16_t adc) {
    const float vBat = (float)adc * (VREF_CAL / 1024.0f) * VDIV_GAIN;
    long mv = (long)(vBat * 1000.0f + 0.5f);
    if (mv < 0) mv = 0;
    if (mv > 65535) mv = 65535;
    return (uint16_t)mv;
  }

  uint16_t adcToSystemMilliAmpsAcs712(uint16_t adc) {
    const float v = adcToVolts(adc);
    const float sens = ACS_MV_PER_A / 1000.0f; // V/A
    const float amps = (v - _acsVzero) / sens;

    // ACK field is unsigned: clamp negative to 0 for now
    long mA = (long)(amps * 1000.0f + (amps >= 0 ? 0.5f : -0.5f));
    if (mA < 0) mA = 0;
    if (mA > 65535) mA = 65535;
    return (uint16_t)mA;
  }

  static float adcToTempC_NoOffset(uint16_t adc) {
    if (adc == 0 || adc >= 1023) return NAN;

    const float v = adcToVolts(adc);
    const float r_ntc = T_RSERIES * (VREF_CAL / v - 1.0f); // NTC on top

    const float invT = (1.0f / T0_K) + (1.0f / T_BETA) * logf(r_ntc / T_R0);
    const float T = 1.0f / invT;
    return T - 273.15f;
  }

  static int16_t tempCToCentiDegC(float tC) {
    if (isnan(tC)) return 0;
    long cC = (long)(tC * 100.0f + (tC >= 0 ? 0.5f : -0.5f));
    if (cC < -32768) cC = -32768;
    if (cC >  32767) cC =  32767;
    return (int16_t)cC;
  }

  void calibrateAcsZero(uint16_t ms) {
    uint32_t t0 = millis();
    uint32_t acc = 0;
    uint32_t n = 0;

    while (millis() - t0 < ms) {
      acc += (uint16_t)analogRead(PIN_A_CURRENT_SYS);
      n++;
      delay(2);
    }

    const uint16_t adcAvg = (n ? (uint16_t)(acc / n) : 512);
    _acsVzero = adcToVolts(adcAvg);

#if TB_DEBUG_PRINTS
    Serial.print(F("ACS712-05B Vzero calibrated: adc="));
    Serial.print(adcAvg);
    Serial.print(F("  Vzero="));
    Serial.println(_acsVzero, 4);
#endif
  }
};

// =============================================================================
// ACTUATORS
// =============================================================================
class Actuators {
public:
  void begin() {
    // Outputs
    pinMode(PIN_BTS_LEN, OUTPUT);
    pinMode(PIN_BTS_REN, OUTPUT);
    pinMode(PIN_BTS_LPWM, OUTPUT);
    pinMode(PIN_BTS_RPWM, OUTPUT);

    digitalWrite(PIN_BTS_LEN, LOW);
    digitalWrite(PIN_BTS_REN, LOW);
    analogWrite(PIN_BTS_LPWM, 0);
    analogWrite(PIN_BTS_RPWM, 0);

    pinMode(PIN_PWM_ACC_1, OUTPUT);
    pinMode(PIN_PWM_ACC_2, OUTPUT);
    pinMode(PIN_PWM_ACC_3, OUTPUT);
    pinMode(PIN_PWM_ACC_4, OUTPUT);

    analogWrite(PIN_PWM_ACC_1, 0);
    analogWrite(PIN_PWM_ACC_2, 0);
    analogWrite(PIN_PWM_ACC_3, 0);
    analogWrite(PIN_PWM_ACC_4, 0);

    _rudder.attach(PIN_RUDDER_SERVO);
    _rudder.writeMicroseconds(SERVO_US_CENTER);
  }

  void apply(const TbCmdV1& cmd, bool armed) {
    drivePropulsion(cmd.throttlePct, armed);
    driveRudder(cmd.rudderPct, armed);
    driveAccessories(cmd.acc, armed);
  }

private:
  Servo _rudder;

  static constexpr int SERVO_US_CENTER = 1500;
  static constexpr int SERVO_US_RANGE  = 400;

  static int rudderPctToUs(int8_t rudderPct) {
    const int r = clampi((int)rudderPct, -100, 100);
    return SERVO_US_CENTER + (r * SERVO_US_RANGE) / 100;
  }

  static void drivePropulsion(int8_t throttlePct, bool armed) {
    if (!armed) {
      digitalWrite(PIN_BTS_LEN, LOW);
      digitalWrite(PIN_BTS_REN, LOW);
      analogWrite(PIN_BTS_LPWM, 0);
      analogWrite(PIN_BTS_RPWM, 0);
      return;
    }

    int t = clampi((int)throttlePct, -100, 100);

    digitalWrite(PIN_BTS_LEN, HIGH);
    digitalWrite(PIN_BTS_REN, HIGH);

    if (t == 0) {
      analogWrite(PIN_BTS_LPWM, 0);
      analogWrite(PIN_BTS_RPWM, 0);
      return;
    }

    const uint8_t pwm = (uint8_t)((abs(t) * 255) / 100);
    if (t > 0) {
      analogWrite(PIN_BTS_LPWM, pwm);
      analogWrite(PIN_BTS_RPWM, 0);
    } else {
      analogWrite(PIN_BTS_LPWM, 0);
      analogWrite(PIN_BTS_RPWM, pwm);
    }
  }

  static void driveAccessories(const uint8_t acc[4], bool armed) {
    analogWrite(PIN_PWM_ACC_1, armed ? acc[0] : 0);
    analogWrite(PIN_PWM_ACC_2, armed ? acc[1] : 0);
    analogWrite(PIN_PWM_ACC_3, armed ? acc[2] : 0);
    analogWrite(PIN_PWM_ACC_4, armed ? acc[3] : 0);
  }

  void driveRudder(int8_t rudderPct, bool armed) {
    const int us = armed ? rudderPctToUs(rudderPct) : SERVO_US_CENTER;
    _rudder.writeMicroseconds(us);
  }
};

// =============================================================================
// FAILSAFE
// =============================================================================
class Failsafe {
public:
  void begin(uint32_t failsafeMs) {
    _failsafeMs = failsafeMs;
    memset(&_lastCmd, 0, sizeof(_lastCmd));
    _lastCmdMs = millis();
  }

  void noteCommand(const TbCmdV1& cmd, uint32_t nowMs) {
    _lastCmd = cmd;
    _lastCmdMs = nowMs;
  }

  TbCmdV1 commandToApply(uint32_t nowMs) const {
    TbCmdV1 out = _lastCmd;
    const bool fresh = (nowMs - _lastCmdMs) <= _failsafeMs;
    if (!fresh) {
      out.arm = 0;
      out.throttlePct = 0;
      out.rudderPct = 0;
      out.acc[0] = out.acc[1] = out.acc[2] = out.acc[3] = 0;
    }
    return out;
  }

private:
  uint32_t _failsafeMs = 500;
  TbCmdV1  _lastCmd {};
  uint32_t _lastCmdMs = 0;
};

// =============================================================================
// RX RADIO LINK + ACK BUILDER
// =============================================================================
class RxRadioLink {
public:
  bool begin() {
    if (!radio.begin()) return false;

    radio.setChannel(RF_CHANNEL);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    radio.openReadingPipe(1, PIPE_ADDR);
    radio.startListening();
    return true;
  }

  // Polls radio; returns true if a packet was read (good or bad)
  bool poll(TbHdr& outHdr, TbStatus& outStatus, TbCmdV1& outCmd, bool& outHasCmd) {
    outHasCmd = false;

    uint8_t pipe = 0;
    if (!radio.available(&pipe)) return false;

    const uint8_t len = radio.getDynamicPayloadSize();
    if (len == 0 || len > TB_MAX_AIR) {
      radio.flush_rx();
      outStatus = TB_S_BAD_LEN;

      // Use last hdr values as "unknown"
      memset(&outHdr, 0, sizeof(outHdr));
      outHdr.seq = 0;
      _lastPipe = pipe ? pipe : 1;
      bumpBadOnce();
      return true;
    }

    uint8_t frame[TB_MAX_AIR] = {0};
    radio.read(frame, len);

    const uint8_t* payload = nullptr;
    uint8_t payloadLen = 0;

    TbStatus st = TbParseFrame(frame, len, outHdr, payload, payloadLen);

    // packet-level counters
    if (st == TB_S_OK) {
      _rxOk++;
    } else {
      bumpBadOnce();
    }

    // semantic checks
    if (st == TB_S_OK) {
      if (outHdr.type == TB_CMD) {
        if (payloadLen != TB_CMD_LEN) {
          st = TB_S_BAD_LEN;
          bumpBadMaybe(); // may be no-op if TB_COUNT_BAD_ONCE==1
        } else {
          TbCmdV1 cmd {};
          memcpy(&cmd, payload, sizeof(cmd));
          cmd.throttlePct = (int8_t)clampi((int)cmd.throttlePct, -100, 100);
          cmd.rudderPct   = (int8_t)clampi((int)cmd.rudderPct,   -100, 100);
          outCmd = cmd;
          outHasCmd = true;
        }
      } else if (outHdr.type == TB_PING) {
        // OK; telemetry still returned
      } else {
        st = TB_S_BAD_TYPE;
        bumpBadMaybe(); // may be no-op if TB_COUNT_BAD_ONCE==1
      }
    }

    outStatus = st;
    _lastPipe = (pipe == 0) ? 1 : pipe;
    return true;
  }

  void queueAck(uint8_t seqEcho, TbStatus status, const Telemetry& tel) {
    TbAckV2 ack {};
    ack.ver     = TB_VER;
    ack.type    = TB_ACK;
    ack.seqEcho = seqEcho;
    ack.status  = (uint8_t)status;
    ack.rxOk    = _rxOk;
    ack.rxBad   = _rxBad;

    ack.vSys_mV  = tel.vSys_mV;
    ack.vProp_mV = tel.vProp_mV;
    ack.iSys_mA  = tel.iSys_mA;
    ack.tMotor_cC = tel.tMotor_cC;
    ack.tEsc_cC   = tel.tEsc_cC;
    ack.waterRaw  = tel.waterRaw;

    ack.crc16 = TbAckCrc(ack);
    radio.writeAckPayload(_lastPipe, &ack, sizeof(ack));
  }

  uint16_t rxOk() const { return _rxOk; }
  uint16_t rxBad() const { return _rxBad; }

private:
  uint16_t _rxOk = 0;
  uint16_t _rxBad = 0;
  uint8_t  _lastPipe = 1;

  // Count bad exactly once per packet when enabled
  void bumpBadOnce() {
    _rxBad++;
  }

  void bumpBadMaybe() {
#if TB_COUNT_BAD_ONCE
    // no-op (already counted at packet level)
#else
    _rxBad++;
#endif
  }
};

// =============================================================================
// APP (wires everything together)
// =============================================================================
class TugbotRxApp {
public:
  void begin() {
    Serial.begin(115200);
#if TB_SERIAL_WAIT
    while (!Serial) {}
#endif

    Serial.println();
    Serial.println(F("TugBot RX — Protocol v2 (iSys_mA) — OOP"));

    _act.begin();
    _tel.begin();

    if (!_link.begin()) {
      Serial.println(F("radio.begin() FAILED"));
      while (1) {}
    }

    _failsafe.begin(500);

    // Initial ACK payload present (optional but handy)
    const Telemetry t = _tel.read();
    _link.queueAck(0, TB_S_OK, t);

    Serial.println(F("Listening..."));
  }

  void tick() {
    const uint32_t now = millis();

    // Failsafe apply
    const TbCmdV1 cmdToApply = _failsafe.commandToApply(now);
    const bool armed = (cmdToApply.arm != 0);
    _act.apply(cmdToApply, armed);

    // Receive one packet max per loop
    TbHdr hdr {};
    TbStatus st = TB_S_OK;
    TbCmdV1 cmd {};
    bool hasCmd = false;

    if (!_link.poll(hdr, st, cmd, hasCmd)) {
      return;
    }

    if (st == TB_S_OK && hasCmd) {
      _failsafe.noteCommand(cmd, now);
    }

    // Always queue telemetry ACK (good or bad)
    const Telemetry t = _tel.read();
    _link.queueAck(hdr.seq, st, t);
  }

private:
    TelemetrySampler _tel;
    Actuators        _act;
    Failsafe         _failsafe;
    RxRadioLink      _link;
};

// =============================================================================
// GLOBAL APP + ARDUINO ENTRYPOINTS
// =============================================================================
static TugbotRxApp g_app;

void setup() {
  g_app.begin();
}

void loop() {
  g_app.tick();
}