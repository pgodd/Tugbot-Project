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
    if (NRF_ACK_PAYLOAD) { _radio.enableAckPayload(); }
    _radio.setRetries(NRF_RETRY_DELAY, NRF_RETRY_COUNT);

    _radio.setPayloadSize(sizeof(NrfCmdFrame));
    _radio.openWritingPipe(NRF_ADDR_RX);
    _radio.openReadingPipe(1, NRF_ADDR_TX);
    _radio.stopListening();
}

bool TugbotRadioLink::SendCmd(const NrfCmdFrame& frame)
{
    _radio.setPayloadSize(sizeof(NrfCmdFrame));

    bool ok = _radio.write(&frame, sizeof(frame));
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
