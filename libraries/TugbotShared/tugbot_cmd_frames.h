// -----------------------------------------------------------------------------
// File:        tugbot_cmd_frames.h
// Project:     Tugbot
// Role:        SHARED (TX/RX)
// Target:      Common
// -----------------------------------------------------------------------------

#ifndef TUGBOT_CMD_FRAMES_H
#define TUGBOT_CMD_FRAMES_H

#include <Arduino.h>

// Header-safe FOURCC helper (little-endian)
inline constexpr uint32_t TugbotFourCC(char a, char b, char c, char d)
{
    return (uint32_t)(uint8_t)a
        | ((uint32_t)(uint8_t)b << 8)
        | ((uint32_t)(uint8_t)c << 16)
        | ((uint32_t)(uint8_t)d << 24);
}

// Canon command-frame magic: "CMDO"
inline constexpr uint32_t NRF_MAGIC_CMD = TugbotFourCC('C','M','D','O');

#if defined(__GNUC__)
  #define TUGBOT_PACKED __attribute__((packed))
#else
  #define TUGBOT_PACKED
#endif

// Stable on-air layout across AVR/ESP32/etc.
struct TUGBOT_PACKED NrfCmdFrame
{
    uint32_t magic;   // NRF_MAGIC_CMD
    uint32_t seq;     // increments every send
    int8_t   thrPct;  // -100..100
    int8_t   rudPct;  // -100..100
    uint8_t  flags;   // bit0 = armed
    uint8_t  mode;    // 0..255
    uint16_t reserved;
};

static_assert(sizeof(NrfCmdFrame) == 14, "NrfCmdFrame must be 14 bytes");

#undef TUGBOT_PACKED

#endif // TUGBOT_CMD_FRAMES_H
