// -----------------------------------------------------------------------------
// File:        TugbotRadio.cpp
// Project:     Tugbot v1.0.0-beta
// Module:      NRF24L01 radio wrapper + self-test
// Revision:    1.0.0
// Author:      Peter B. Goddard (Tugbot Project)
// Implementation Assistance: ChatGPT
// -----------------------------------------------------------------------------

#include "TugbotRadio.h"
#include <SPI.h>
#include <RF24.h>
#include <string.h>
#include "pins_radio_rx_mega.h"
#include <tugbot_config_radio.h>

// Single global RF24 instance for this translation unit.
// Uses canonical pins from pins_tugbot.h: CE=48, CSN=49
static RF24 radio(PIN_NRF24_CE, PIN_NRF24_CSN);

TugbotRadio::TugbotRadio()
: _initialised(false),
  _present(false),
  _lastRxMs(0),
  _stats{0, 0, 0, 0}
{
}

void TugbotRadio::begin()
{
    if (_initialised)
    {
        return;
    }

    AvrSpiForceMaster();

    // SPI is started inside RF24::begin()
    radio.begin();

    // Canon radio config (mirror TX settings)
    radio.setChannel(NRF_CHANNEL);
    radio.setDataRate(NRF_DATARATE);
    radio.setCRCLength(NRF_CRC);
    radio.setPALevel(NRF_PA_LEVEL);

    radio.setAutoAck(NRF_AUTO_ACK);
    if (NRF_USE_ACK_PAYLOAD)
    {
        radio.enableAckPayload();
    }
    radio.setRetries(NRF_RETRY_DELAY, NRF_RETRY_COUNT);

    if (NRF_USE_DYNAMIC_PAYLOADS)
    {
        radio.enableDynamicPayloads();
    }
    else
    {
        radio.disableDynamicPayloads();
        radio.setPayloadSize(NRF_FIXED_PAYLOAD_BYTES);
    }

    radio.openReadingPipe(1, NRF_ADDR_RX_BYTES);
    radio.startListening();

    _initialised = true;

    // Run an implicit self-test so isPresent() is meaningful immediately.
    selfTest();
}

// -----------------------------------------------------------------------------
// Self-test
// -----------------------------------------------------------------------------

bool TugbotRadio::selfTest()
{
    // Ensure radio is initialised
    if (!_initialised)
    {
        begin();
    }

    // Use the public RF24 API, not read_register()
    bool ok = radio.isChipConnected();

    _present = ok;
    return ok;
}

bool TugbotRadio::readLatest(NrfCmdFrame &frameOut)
{
    if (!_initialised)
    {
        return false;
    }

    bool gotFrame = false;
    bool gotValid = false;
    while (radio.available())
    {
        uint8_t payloadSize = NRF_FIXED_PAYLOAD_BYTES;
        if (NRF_USE_DYNAMIC_PAYLOADS)
        {
            payloadSize = radio.getDynamicPayloadSize();
            if (payloadSize == 0 || payloadSize > 32)
            {
                payloadSize = 32;
            }
        }

        uint8_t buffer[32];
        radio.read(buffer, payloadSize);
        gotFrame = true;

        if (payloadSize != sizeof(NrfCmdFrame))
        {
            _stats.framesBadSize++;
            continue;
        }

        memcpy(&frameOut, buffer, sizeof(NrfCmdFrame));

        if (frameOut.magic != NRF_MAGIC_CMD)
        {
            _stats.framesBadMagic++;
            continue;
        }

        _stats.framesOk++;
        _stats.lastSeq = frameOut.seq;
        _lastRxMs = millis();
        gotValid = true;
    }

    return gotFrame && gotValid;
}
