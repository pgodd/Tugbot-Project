// -----------------------------------------------------------------------------
// File:        TugbotRadioLink.cpp
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#include "TugbotRadioLink.h"
#include <SPI.h>

TugbotRadioLink::TugbotRadioLink() : _radio(PIN_NRF_CE, PIN_NRF_CSN) {}

bool TugbotRadioLink::Begin()
{
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_NRF_CSN);

    if (!_radio.begin())
    {
        _stats.chipConnected = false;
        return false;
    }

    _stats.chipConnected = _radio.isChipConnected();
    ApplyCanonConfig();
    return true;
}

void TugbotRadioLink::ApplyCanonConfig()
{
    _radio.setChannel(NRF_CHANNEL);
    _radio.setDataRate(NRF_DATARATE);
    _radio.setCRCLength(NRF_CRC);
    _radio.setPALevel(NRF_PA_LEVEL);

    _radio.setAutoAck(NRF_AUTO_ACK);
    if (NRF_USE_ACK_PAYLOAD) { _radio.enableAckPayload(); }
    _radio.setRetries(NRF_RETRY_DELAY, NRF_RETRY_COUNT);

    if (NRF_USE_DYNAMIC_PAYLOADS)
    {
        _radio.enableDynamicPayloads();
    }
    else
    {
        _radio.disableDynamicPayloads();
        _radio.setPayloadSize(NRF_FIXED_PAYLOAD_BYTES);
    }

    _radio.openWritingPipe(NRF_ADDR_RX_BYTES);
    _radio.openReadingPipe(1, NRF_ADDR_RX_BYTES);
    _radio.stopListening();
}

bool TugbotRadioLink::SendCmd(const NrfCmdFrame& frame)
{
    if (!NRF_USE_DYNAMIC_PAYLOADS)
    {
        _radio.setPayloadSize(NRF_FIXED_PAYLOAD_BYTES);
    }

    bool ok = _radio.write(&frame, NRF_FIXED_PAYLOAD_BYTES);
    if (ok)
    {
        _stats.ok++;
        DrainAckPayloadIfAny();
    }
    else
    {
        _stats.fail++;
    }
    return ok;
}

void TugbotRadioLink::DrainAckPayloadIfAny()
{
    if (!_radio.isAckPayloadAvailable()) { return; }
    uint8_t buf[32];
    _radio.read(&buf[0], sizeof(buf));
    _stats.ackPayload++;
}

const TugbotRadioLink::Stats& TugbotRadioLink::GetStats() const { return _stats; }
