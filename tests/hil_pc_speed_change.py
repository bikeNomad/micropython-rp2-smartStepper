"""HIL script: PulseCounter accuracy through multiple mid-move speed replans.

Performs moveTo(100) at maxSpeed=50 (9600 steps total).  Three maxSpeed changes
are issued during the cruise phase to trigger _replan().  Because replans only
alter velocity — not target position — the counter must still reach exactly 9600
at the end.

Prints:
  done steps=<n>    PulseCounter value after motion completes (expect ~9600)
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50      # 4800 steps/s
s.acceleration = 300

# Accel from 5→50 u/s at 300 u/s^2 takes ~0.15 s; cruise ≈1.84 s; total ~2.1 s.
# Replan triggers are spaced 0.4 s apart, all within the cruise window.
s.moveTo(100)
time.sleep_ms(400)
s.maxSpeed = 15      # drop to 1440 steps/s; triggers _replan()
time.sleep_ms(400)
s.maxSpeed = 40      # raise to 3840 steps/s; triggers _replan()
time.sleep_ms(400)
s.maxSpeed = 50      # restore to 4800 steps/s; triggers _replan()
s.waitEndOfMove()

print(f'done steps={s._pulseCounter.value}')
