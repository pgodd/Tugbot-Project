// -----------------------------------------------------------------------------
// File:        pins_tugbot.h
// Project:     Tugbot v1.0.0-beta
// Module:      Global pin map (canonical)
// Revision:    1.0.1   // temp pins renamed to PIN_TEMP_MOTOR / PIN_TEMP_SPDCNTRL
// Author:      Peter B. Goddard (Tugbot Project)
// License:     MIT-style (free use with attribution)
// -----------------------------------------------------------------------------
// NOTE:
//   These assignments reflect our current canon for the Mega-based Tugbot.
//   If you change hardware wiring, update this file and treat it as GOSPEL.
// -----------------------------------------------------------------------------

#ifndef PINS_TUGBOT_H
#define PINS_TUGBOT_H

// --- I2C bus (LCD, IMU/compass, etc.) ---------------------------------------
#define PIN_I2C_SDA            20
#define PIN_I2C_SCL            21

// --- BTS7960 motor driver (propulsion) --------------------------------------
// Canon from project notes:
//   LEN = 23, REN = 25, LPWM = 6, RPWM = 4
#define PIN_BTS_LEN            23
#define PIN_BTS_REN            25
#define PIN_BTS_LPWM           6
#define PIN_BTS_RPWM           4

// --- Engine sound PWM (reserved) --------------------------------------------
#define PIN_ENGINE_SOUND_PWM   9

// --- NRF24L01 (SPI plus CE / CSN) -------------------------------------------
// SPI pins are hardware (50–53 on Mega)
#define PIN_NRF24_CE           48
#define PIN_NRF24_CSN          49

// --- Serial devices ---------------------------------------------------------
// GPS on Serial1 (TX18, RX19)
#define PIN_GPS_RX             19
#define PIN_GPS_TX             18

// SFX HW-311 MP3 on Serial3 (TX14, RX15)
#define PIN_SFX_RX             15
#define PIN_SFX_TX             14

// --- Analog sensors ---------------------------------------------------------
// Canon from notes:
//   A0 = current sensor (propulsion)
//   A1 = SYS battery voltage
//   A2 = water level / detector
//   A3 = PROP (motor) battery voltage
//   A8, A9 = temperature sensors (renamed for clarity)

// Propulsion current sensor
#define PIN_A_CURRENT_PROP     A0

// System battery voltage
#define PIN_A_V_SYS            A1

// Water ingress sensor (name preserved per request)
#define PIN_A_WATER            A2

// Propulsion / motor battery voltage
#define PIN_A_V_PROP           A3

// Motor temperature sensor 
#define PIN_TEMP_MOTOR         A8

// Speed controller temperature sensor 
#define PIN_TEMP_SPDCNTRL      A9

// --- MOSFET accessory PWM outputs ------------------------------------------
// D2, D5, D7, D8 used for lights / pumps / smoke, etc.
#define PIN_PWM_ACC_1          2
#define PIN_PWM_ACC_2          5
#define PIN_PWM_ACC_3          7
#define PIN_PWM_ACC_4          10 //bilge pump

// --- Steering servo (rudder) -----------------------------------------------
// NOTE: This is a working assumption. Confirm actual wiring on your hull
//       and then treat this as canon.
#define PIN_RUDDER_SERVO       3

#endif // PINS_TUGBOT_H
