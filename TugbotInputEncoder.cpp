#include "TugbotInputEncoder.h"

static const int8_t kQuadTable[16] =
{
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

void TugbotInputEncoder::Begin(uint8_t pinA, uint8_t pinB, uint8_t pinBtn, Role role, int stepPerDetent)
{
    _pinA=pinA; _pinB=pinB; _pinBtn=pinBtn; _role=role; _stepPerDetent=stepPerDetent;

    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    if (_pinBtn != 0xFF)
    {
        pinMode(_pinBtn, INPUT_PULLUP);
        _btnStable = true; _btnLastStable = true; _btnLastChangeMs = 0; _clickLatch = false;
    }

    uint8_t a=(uint8_t)digitalRead(_pinA);
    uint8_t b=(uint8_t)digitalRead(_pinB);
    _prevAB=(a<<1)|b;
    _transitions=0; _detents=0;
}

void TugbotInputEncoder::Bind(TugbotInputModel* model) { _model=model; }
void TugbotInputEncoder::BindUiPageSetter(void (*fn)(uint8_t)) { _setPageFn=fn; }
void TugbotInputEncoder::BindModeSetter(void (*fn)(uint8_t)) { _setModeFn=fn; }

void TugbotInputEncoder::Tick(uint32_t nowMs)
{
    uint8_t a=(uint8_t)digitalRead(_pinA);
    uint8_t b=(uint8_t)digitalRead(_pinB);
    uint8_t currAB=(a<<1)|b;

    if (currAB != _prevAB)
    {
        uint8_t idx = (_prevAB<<2) | currAB;
        int8_t delta = kQuadTable[idx & 0x0F];
        _transitions += delta;
        _prevAB = currAB;

        while (_transitions >= (int)_transitionsPerDetent)
        {
            _transitions -= _transitionsPerDetent;
            _detents++;
            ApplyDetents(+1);
        }
        while (_transitions <= -(int)_transitionsPerDetent)
        {
            _transitions += _transitionsPerDetent;
            _detents--;
            ApplyDetents(-1);
        }
    }

    if (_pinBtn != 0xFF) { TickButton(nowMs); }
}

void TugbotInputEncoder::ApplyDetents(int8_t detentDelta)
{
    if (_role == Role::None) return;

    if (_role == Role::UiPage)
    {
        if (!_setPageFn) return;
        static uint8_t s_page = 0;
        int v = (int)s_page + (detentDelta * _stepPerDetent);
        while (v < 0) v += 3;
        while (v > 2) v -= 3;
        s_page = (uint8_t)v;
        _setPageFn(s_page);
        return;
    }

    if (_role == Role::Mode)
    {
        if (!_setModeFn) return;
        static uint8_t s_mode = 0;
        s_mode = (uint8_t)(s_mode + (detentDelta * _stepPerDetent));
        _setModeFn(s_mode);
        return;
    }

    if (!_model) return;

    if (_role == Role::Rudder)
    {
        int rud = _model->GetRudderPct();
        rud += detentDelta * _stepPerDetent;
        _model->SetRudderPct(rud);
        return;
    }

    if (_role == Role::Throttle)
    {
        int thr = _model->GetThrottlePct();
        thr += detentDelta * _stepPerDetent;
        _model->SetThrottlePct(thr);
        return;
    }
}

void TugbotInputEncoder::OnButtonClick()
{
    if (_role == Role::Rudder)
    {
        if (_model) { _model->SetArmed(!_model->GetArmed()); }
        return;
    }
    if (_role == Role::Throttle)
    {
        if (_model) { _model->SetThrottlePct(0); }
        return;
    }
    if (_role == Role::UiPage)
    {
        if (_setPageFn)
        {
            static uint8_t s_page = 0;
            s_page = (uint8_t)((s_page + 1) % 3);
            _setPageFn(s_page);
        }
        return;
    }
    if (_role == Role::Mode)
    {
        if (_setModeFn)
        {
            static uint8_t s_mode = 0;
            s_mode++;
            _setModeFn(s_mode);
        }
        return;
    }
}

void TugbotInputEncoder::TickButton(uint32_t nowMs)
{
    bool reading = (digitalRead(_pinBtn) != LOW); // true=released

    if (reading != _btnStable)
    {
        _btnStable = reading;
        _btnLastChangeMs = nowMs;
    }

    if (nowMs - _btnLastChangeMs < 30) return;

    if (_btnStable != _btnLastStable)
    {
        _btnLastStable = _btnStable;

        if (_btnLastStable == false) { _clickLatch = true; }

        if (_btnLastStable == true && _clickLatch)
        {
            _clickLatch = false;
            OnButtonClick();
        }
    }
}
