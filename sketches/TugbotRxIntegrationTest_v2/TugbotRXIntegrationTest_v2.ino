// -----------------------------------------------------------------------------
// File:        TugbotIntegrationTest_v2.ino
// Project:     Tugbot
// Role:        UNCLASSIFIED
// Target:      UNKNOWN
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// File:        TugbotIntegrationTest_v2_breadcrumb.ino
// Project:     Tugbot v1.0.0-beta
// Module:      System integration test with EVT & STATE telemetry + init breadcrumbs
// Revision:    2.1.1-breadcrumb
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// License:     MIT (typical liberal rights)
// -----------------------------------------------------------------------------
//
// WHAT THIS VERSION DOES
// ----------------------
// Same behaviour as your v2 sketch, BUT it prints "STEP x" lines before and
// after every init stage in setup(). If the sketch "stalls", the last printed
// STEP line tells you the exact call that blocked.
//
// IMPORTANT
// ---------
// This does NOT magically fix the stall; it pinpoints it reliably in one run.
// After you tell me the last step printed, we’ll harden that library call
// (typically: add a timeout, or skip the subsystem if device absent).
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

#include "pins_tugbot.h"
#include "TugbotMotion.h"
#include "tugbot_config_motion.h"
#include "TugbotNv.h"
#include "TugbotCompass.h"
#include "TugbotGps.h"
#include "TugbotEffects.h"
#include "TugbotEnergy.h"
#include "TugbotEngineer.h"
#include "tugbot_config_engineer.h"
#include "TugbotRadio.h"
#include "tugbot_config_radio.h"

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

TugbotNvData          g_nv;

TugbotMotion          g_motion;
MotionConfig          g_motionCfg;
SteeringProfileConfig g_steerProfiles[3];

TugbotEffects         g_effects;
TugbotEnergy          g_energy;
TugbotEngineer        g_engineer;
EngineerConfig        g_engCfg;
TugbotRadio           g_radio;

Servo                 g_rudderServo;

uint32_t g_lastUpdateMs = 0;
uint32_t g_serialStartMs = 0;
uint32_t g_radioInitAfterMs = 0;
bool     g_radioReady = false;

static const uint32_t kRadioCmdTimeoutMs = 600;

NrfCmdFrame g_lastCmd{};
bool        g_hasCmd = false;
uint32_t    g_lastCmdMs = 0;
uint32_t    g_lastCmdSeq = 0;

// --- EVT change tracking ----------------------------------------------------
EnergyClass        g_lastEnergyClass    = EnergyClass::OK;
EngineerTempClass  g_lastTempClass      = EngineerTempClass::NORMAL;
bool               g_lastWaterIngress   = false;
bool               g_lastRadioPresent   = false;
bool               g_lastBilgePumpOn    = false;
bool               g_lastLinkOk         = false;

// -----------------------------------------------------------------------------
// Small helpers for breadcrumb printing
// -----------------------------------------------------------------------------

static void stepPrint(const __FlashStringHelper* msg)
{
    Serial.println(msg);
    // Force serial buffer out so we see the last line *before* a stall.
    Serial.flush();
}

static void stepPrint2(const __FlashStringHelper* msg, bool ok)
{
    Serial.print(msg);
    Serial.println(ok ? F("OK") : F("FAIL"));
    Serial.flush();
}

static float pctToNorm(int8_t pct)
{
    return constrain(pct / 100.0f, -1.0f, 1.0f);
}

static bool linkOk(uint32_t now)
{
    return g_hasCmd && (now - g_lastCmdMs) <= kRadioCmdTimeoutMs;
}

static void initRadioIfReady(uint32_t now)
{
    if (g_radioReady || now < g_radioInitAfterMs)
    {
        return;
    }

    stepPrint(F("STEP 10: g_radio.begin()..."));
    g_radio.begin();
    stepPrint(F("STEP 10: g_radio.begin() returned"));

    stepPrint(F("STEP 10b: g_radio.selfTest()..."));
    bool radioOk = g_radio.selfTest();
    stepPrint2(F("STEP 10b: selfTest -> "), radioOk);

    g_lastRadioPresent = radioOk;

    Serial.print(F("NRF24 present: "));
    Serial.println(radioOk ? F("YES") : F("NO"));

    Serial.print(F("EVT,"));
    Serial.print(now);
    Serial.print(F(",RADIO,"));
    Serial.println(radioOk ? F("1") : F("0"));

    g_radioReady = true;
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    g_serialStartMs = millis();
    g_radioInitAfterMs = g_serialStartMs + NRF_SERIAL_SETTLE_MS;

    Serial.println();
    Serial.println(F("TugbotIntegrationTest_v2 starting..."));

    // STEP A: I2C
    stepPrint(F("STEP A: Wire.begin()..."));
    Wire.begin();
    #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(2000 /* microseconds */, true /* reset on timeout */);
    #endif
    stepPrint(F("STEP A: Wire.begin() OK"));

    // STEP B: NVRAM load/increment/save
    stepPrint(F("STEP B: TugbotNv::load()..."));
    TugbotNv::load(g_nv);
    stepPrint(F("STEP B: TugbotNv::load() OK"));

    g_nv.run_counter++;
    stepPrint(F("STEP B2: TugbotNv::save()..."));
    TugbotNv::save(g_nv);
    stepPrint(F("STEP B2: TugbotNv::save() OK"));

    Serial.print(F("NVRAM run_counter = "));
    Serial.println(g_nv.run_counter);
    Serial.flush();

    // STEP 1: Energy manager
    stepPrint(F("STEP 1: g_energy.begin()..."));
    g_energy.begin(g_nv);
    stepPrint(F("STEP 1: g_energy.begin() OK"));

    // STEP 2: Motion config from NVRAM
    stepPrint(F("STEP 2: TugbotNv::applyToMotion()..."));
    TugbotNv::applyToMotion(g_nv, g_motionCfg, g_steerProfiles, 3);
    stepPrint(F("STEP 2: TugbotNv::applyToMotion() OK"));

    // STEP 3: Motion begin
    stepPrint(F("STEP 3: g_motion.begin()..."));
    g_motion.setMotionConfig(&g_motionCfg);
    g_motion.setSteeringProfiles(g_steerProfiles, 3);
    g_motion.setActiveProfile(g_nv.activeSteeringProfile);
    g_motion.begin(millis());
    stepPrint(F("STEP 3: g_motion.begin() OK"));

    // STEP 4: Engineer begin
    stepPrint(F("STEP 4: engineer config defaults..."));
    tugbot_getDefaultEngineerConfig(g_engCfg);
    stepPrint(F("STEP 4: g_engineer.begin()..."));
    g_engineer.begin(g_engCfg);
    stepPrint(F("STEP 4: g_engineer.begin() OK"));

    // STEP 5: Effects begin (common stall point if MP3 module missing/wired wrong)
    stepPrint(F("STEP 5: g_effects.begin(Serial3,9600)..."));
    g_effects.begin(Serial3, 9600);
    stepPrint(F("STEP 5: g_effects.begin() returned"));

    // Safe initial effects commands (should not block; if they do, also informative)
    stepPrint(F("STEP 5b: initial lights..."));
    g_effects.setLight(TugbotLightId::DECK,  true);
    g_effects.setLight(TugbotLightId::CABIN, true);
    stepPrint(F("STEP 5b: initial lights OK"));

    // STEP 6: Compass begin + calibration
    stepPrint(F("STEP 6: TugbotCompass::begin()..."));
    bool compassOk = TugbotCompass::begin();
    stepPrint2(F("STEP 6: Compass begin -> "), compassOk);

    if (compassOk)
    {
        stepPrint(F("STEP 6b: Compass setCalibration()..."));
        TugbotCompass::setCalibration(
            g_nv.compass_offset_x,
            g_nv.compass_offset_y,
            g_nv.compass_offset_z,
            g_nv.compass_scale_x,
            g_nv.compass_scale_y,
            g_nv.compass_scale_z
        );
        stepPrint(F("STEP 6b: Compass calibration OK"));
    }

    // STEP 7: GPS begin
    stepPrint(F("STEP 7: TugbotGps::begin(Serial1,9600)..."));
    TugbotGps::begin(Serial1, 9600);
    stepPrint(F("STEP 7: GPS initialised on Serial1 @ 9600."));

    // STEP 8: Rudder servo attach
    stepPrint(F("STEP 8: rudder servo attach..."));
    g_rudderServo.attach(PIN_RUDDER_SERVO);
    stepPrint(F("STEP 8: rudder servo attach OK"));

    // STEP 9: BTS7960 pins
    stepPrint(F("STEP 9: BTS7960 pinMode/enable..."));
    pinMode(PIN_BTS_LEN, OUTPUT);
    pinMode(PIN_BTS_REN, OUTPUT);
    pinMode(PIN_BTS_LPWM, OUTPUT);
    pinMode(PIN_BTS_RPWM, OUTPUT);
    digitalWrite(PIN_BTS_LEN, HIGH);
    digitalWrite(PIN_BTS_REN, HIGH);
    stepPrint(F("STEP 9: BTS7960 enabled OK"));

    // STEP 10: Radio begin is deferred until Serial settle timeout
    stepPrint(F("STEP 10: radio init deferred for serial settle."));

    Serial.println(F("Entering integration test loop..."));
    Serial.flush();
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------

void loop()
{
    uint32_t now = millis();

    // Fast sensor/service updates (as often as possible)
    initRadioIfReady(now);

    if (g_radioReady)
    {
        NrfCmdFrame cmd{};
        if (g_radio.readLatest(cmd))
        {
            g_lastCmd = cmd;
            g_hasCmd = true;
            g_lastCmdMs = now;
            g_lastCmdSeq = cmd.seq;
        }
    }

    TugbotGps::update();
    TugbotCompass::update();
    g_energy.update(now);
    g_engineer.update(now);

    // ~10 Hz integration tick (actuation + prints)
    if (now - g_lastUpdateMs < 100U)
    {
        return;
    }
    g_lastUpdateMs = now;

    const EnergyState   &es  = g_energy.getState();
    const EngineerState &eng = g_engineer.getState();

    // ---------------- EVT: state change detection ---------------------------

    // Energy class change
    if (es.cls != g_lastEnergyClass)
    {
        Serial.print(F("EVT,"));
        Serial.print(now);
        Serial.print(F(",ENERGY,"));
        Serial.print((int)g_lastEnergyClass); // old
        Serial.print(F(","));
        Serial.print((int)es.cls);            // new
        Serial.print(F(","));
        Serial.print(es.vProp, 2);
        Serial.print(F(","));
        Serial.print(es.vSys, 2);
        Serial.println();
        g_lastEnergyClass = es.cls;
    }

    // Temperature class change
    if (eng.tempClass != g_lastTempClass)
    {
        Serial.print(F("EVT,"));
        Serial.print(now);
        Serial.print(F(",TEMP,"));
        Serial.print((int)g_lastTempClass); // old
        Serial.print(F(","));
        Serial.print((int)eng.tempClass);   // new
        Serial.print(F(","));
        Serial.print(eng.hottestC, 1);
        Serial.println();
        g_lastTempClass = eng.tempClass;
    }

    // Water ingress change
    if (eng.waterIngress != g_lastWaterIngress)
    {
        Serial.print(F("EVT,"));
        Serial.print(now);
        Serial.print(F(",WATER,"));
        Serial.print(g_lastWaterIngress ? F("1") : F("0")); // old
        Serial.print(F(","));
        Serial.print(eng.waterIngress ? F("1") : F("0"));   // new
        Serial.print(F(","));
        Serial.print(eng.waterLevel, 3);
        Serial.println();
        g_lastWaterIngress = eng.waterIngress;
    }

    bool radioPresent = g_radioReady && g_radio.isPresent();
    if (radioPresent != g_lastRadioPresent)
    {
        Serial.print(F("EVT,"));
        Serial.print(now);
        Serial.print(F(",RADIO,"));
        Serial.print(g_lastRadioPresent ? F("1") : F("0")); // old
        Serial.print(F(","));
        Serial.print(radioPresent ? F("1") : F("0"));       // new
        Serial.println();
        g_lastRadioPresent = radioPresent;
    }

    // Radio link change (valid command stream)
    bool linkOkNow = linkOk(now);
    if (linkOkNow != g_lastLinkOk)
    {
        Serial.print(F("EVT,"));
        Serial.print(now);
        Serial.print(F(",LINK,"));
        Serial.print(g_lastLinkOk ? F("1") : F("0")); // old
        Serial.print(F(","));
        Serial.print(linkOkNow ? F("1") : F("0"));    // new
        Serial.print(F(","));
        Serial.print(g_lastCmdSeq);
        Serial.println();
        g_lastLinkOk = linkOkNow;
    }

    // Bilge pump change
    if (eng.bilgePumpOn != g_lastBilgePumpOn)
    {
        Serial.print(F("EVT,"));
        Serial.print(now);
        Serial.print(F(",BILGE,"));
        Serial.print(g_lastBilgePumpOn ? F("1") : F("0"));  // old
        Serial.print(F(","));
        Serial.print(eng.bilgePumpOn ? F("1") : F("0"));    // new
        Serial.print(F(","));
        Serial.print(eng.waterLevel, 3);
        Serial.println();
        g_lastBilgePumpOn = eng.bilgePumpOn;
    }

    // ---------------- Synthetic control inputs -----------------------------

    float throttleIn = 0.0f;
    float steerIn = 0.0f;
    bool armed = false;

    if (linkOkNow)
    {
        armed = (g_lastCmd.flags & 0x01U) != 0U;
        if (armed)
        {
            throttleIn = pctToNorm(g_lastCmd.thrPct);
            steerIn = pctToNorm(g_lastCmd.rudPct);
        }
    }

    // Apply energy-based throttle limit
    float thLimited = throttleIn * es.throttleLimitNorm;

    // Motion update
    g_motion.update(thLimited, steerIn, now);

    // Motor driver output (BTS7960)
    uint8_t pwm = g_motion.getThrottlePwm255();
    int centered = (int)pwm - 127;

    if (centered > 5)
    {
        analogWrite(PIN_BTS_LPWM, (uint8_t)min(centered * 2, 255));
        analogWrite(PIN_BTS_RPWM, 0);
    }
    else if (centered < -5)
    {
        analogWrite(PIN_BTS_LPWM, 0);
        analogWrite(PIN_BTS_RPWM, (uint8_t)min(-centered * 2, 255));
    }
    else
    {
        analogWrite(PIN_BTS_LPWM, 0);
        analogWrite(PIN_BTS_RPWM, 0);
    }

    // Rudder servo output
    float rudderNorm = g_motion.getRudderNormFromDeg();
    float pulseUs    = 1500.0f + rudderNorm * 400.0f;
    pulseUs          = constrain(pulseUs, 1000.0f, 2000.0f);
    g_rudderServo.writeMicroseconds((int)pulseUs);

    // Effects gating
    float thActual = g_motion.getThrottleActualNorm();
    float thMag    = fabs(thActual);

    g_effects.setEngineSoundLevel(armed ? thMag : 0.0f);

    float smokeLevel = 0.0f;
    if (armed && es.effectsAllowed && eng.tempClass != EngineerTempClass::CRIT && thMag > 0.1f)
    {
        smokeLevel = (thMag - 0.1f) / 0.9f;
    }
    g_effects.setSmokeLevel(smokeLevel);

    g_effects.setLight(TugbotLightId::PUMP, eng.bilgePumpOn);
    g_effects.update(now);

    // Navigation state
    TugbotGpsState     gpsState  = TugbotGps::get();
    TugbotCompassState compState = TugbotCompass::get();

    float lat     = gpsState.hasFix ? gpsState.lat_deg     : 0.0f;
    float lon     = gpsState.hasFix ? gpsState.lon_deg     : 0.0f;
    float speedM  = gpsState.hasFix ? gpsState.speed_mps   : 0.0f;
    float heading = compState.hasHeading ? compState.heading_deg : 0.0f;

    uint8_t mode      = linkOkNow ? g_lastCmd.mode : 0;
    uint8_t profile   = g_motion.getActiveProfile();
    uint8_t battState = (uint8_t)es.cls;
    uint8_t radio_ok  = linkOkNow ? 1 : 0;

    // ---------------- TELEMETRY: STATE LINE ---------------------------------
    Serial.print(F("STATE,"));
    Serial.print(now);                        // time_ms
    Serial.print(F(","));
    Serial.print(mode);                       // mode
    Serial.print(F(","));
    Serial.print(lat, 6);                     // lat
    Serial.print(F(","));
    Serial.print(lon, 6);                     // lon
    Serial.print(F(","));
    Serial.print(speedM, 2);                  // speed_mps
    Serial.print(F(","));
    Serial.print(heading, 1);                 // heading_deg
    Serial.print(F(","));
    Serial.print(g_motion.getRudderTargetDeg(), 2); // rudder_cmd_deg
    Serial.print(F(","));
    Serial.print(throttleIn, 3);              // throttle_input_norm
    Serial.print(F(","));
    Serial.print(steerIn, 3);                 // steer_input_norm
    Serial.print(F(","));
    Serial.print(es.vProp, 2);                // V_PROP
    Serial.print(F(","));
    Serial.print(es.vSys, 2);                 // V_SYS
    Serial.print(F(","));
    Serial.print(es.iProp, 2);                // I_PROP
    Serial.print(F(","));
    Serial.print(battState);                  // batt_state
    Serial.print(F(","));
    Serial.print(profile);                    // steering_profile
    Serial.print(F(","));
    Serial.print(eng.hottestC, 1);            // temp_hottest_C
    Serial.print(F(","));
    Serial.print(eng.waterLevel, 3);          // water_level_norm
    Serial.print(F(","));
    Serial.print((int)eng.coolingPumpOn);     // cooling_pump_on
    Serial.print(F(","));
    Serial.print((int)eng.bilgePumpOn);       // bilge_pump_on
    Serial.print(F(","));
    Serial.print(radio_ok);                   // radio_ok 0/1
    Serial.println();
}
