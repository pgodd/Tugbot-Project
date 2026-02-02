// -----------------------------------------------------------------------------
// File:        tugbot_cmd_frames.h
// Project:     Tugbot
// Role:        RECEIVER (RX)
// Target:      Arduino Mega-class
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

// Must match TX magic value.
static const uint32_t NRF_MAGIC_CMD = 0x434D444FULL; // 'CMDO'

struct NrfCmdFrame
{
    uint32_t magic;
    uint32_t seq;
    int8_t   thrPct;
    int8_t   rudPct;
    uint8_t  flags;     // bit0 = armed
    uint8_t  mode;
    uint16_t reserved;
};
