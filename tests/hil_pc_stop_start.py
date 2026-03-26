"""HIL script: PulseCounter accuracy through multiple stop/restart cycles.

Phase 1: moveTo(50) — stop() after 500 ms mid-cruise.
Phase 2: moveTo(100) — continue forward to 100 units; wait for completion.
Phase 3: moveTo(0)   — return to origin; wait for completion.

After phase 3 the net displacement is zero, so both Saleae (direction-aware
edge count) and the PulseCounter must read ~0.

Prints:
  stop1 steps=<n1>   PulseCounter after phase-1 stop   (0 < n < 4800)
  stop2 steps=<n2>   PulseCounter after phase-2 finish (~9600)
  done  steps=<n3>   PulseCounter after phase-3 finish (~0)
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50      # 4800 steps/s
s.acceleration = 300

# Phase 1: start a 50-unit move (4800 steps), stop partway through.
s.moveTo(50)
time.sleep_ms(500)   # well into cruise at ~4800 steps/s
s.stop()
while s.moving:
    time.sleep_ms(5)
print(f'stop1 steps={s._pulseCounter.value}')

# Phase 2: continue forward to 100 units.
s.moveTo(100)
s.waitEndOfMove()
print(f'stop2 steps={s._pulseCounter.value}')

# Phase 3: return to origin.
s.moveTo(0)
s.waitEndOfMove()
print(f'done steps={s._pulseCounter.value}')
