TugbotTx_KY040_Table_WifiWindow_Ramps

Adds slew-rate limiting (ramps) for throttle + rudder.

OLED + Serial display shows:
  THR:<out>(<set>) and RUD:<out>(<set>)

Tuning constants near TugbotTxApp:
  THR_RATE_UP_PPS
  THR_RATE_DOWN_PPS
  RUD_RATE_PPS

Safety:
- Disarm forces OUT to 0 immediately (no ramp lag).
