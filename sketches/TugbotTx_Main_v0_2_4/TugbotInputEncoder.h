// -----------------------------------------------------------------------------
// File:        TugbotInputEncoder.h
// Project:     Tugbot
// Role:        TRANSMITTER (TX)
// Target:      ESP32-WROOM
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include "TugbotInputModel.h"

class TugbotInputEncoder
{
public:
    enum class Role : uint8_t { Rudder=0, Throttle=1, UiPage=2, Mode=3, None=255 };

    void Begin(uint8_t pinA, uint8_t pinB, uint8_t pinBtn, Role role, int stepPerDetent);
    void Bind(TugbotInputModel* model);

    void BindUiPageSetter(void (*setPageFn)(uint8_t page));
    void BindModeSetter(void (*setModeFn)(uint8_t mode));

    void Tick(uint32_t nowMs);
    int32_t GetDetents() const { return _detents; }

private:
    void TickButton(uint32_t nowMs);
    void ApplyDetents(int8_t detentDelta);
    void OnButtonClick();

private:
    uint8_t _pinA=0,_pinB=0,_pinBtn=0xFF;
    Role _role = Role::None;
    int _stepPerDetent = 1;

    TugbotInputModel* _model=nullptr;
    void (*_setPageFn)(uint8_t)=nullptr;
    void (*_setModeFn)(uint8_t)=nullptr;

    uint8_t _prevAB=0;
    int32_t _transitions=0;
    int32_t _detents=0;

    bool _btnStable=true,_btnLastStable=true;
    uint32_t _btnLastChangeMs=0;
    bool _clickLatch=false;

    uint8_t _transitionsPerDetent=4;
};
