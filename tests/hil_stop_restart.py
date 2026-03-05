"""HIL script: smooth-stop a move mid-flight, then start a new move.

Phase 1: moveTo(500) at maxSpeed=50 (~10 s total); stop() after 1000 ms.
  stop() uses interrupt_with() which zeroes the PIO Y register so the current
  segment ends after one half-period, then plays a decel ramp to minSpeed.
Phase 2: moveTo(0) — move back to origin.

Prints:
  stopped steps=<n>       PulseCounter value after phase 1 halts
  done steps=<n>          PulseCounter value after phase 2 completes
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50      # 4800 steps/s
s.acceleration = 300

start_ms = time.ticks_ms()

# Phase 1: start a very long move, smooth-stop partway through
s.moveTo(500)
time.sleep_ms(1000)  # ~10 s total move; 1 s puts us well into the cruise phase
s.stop()             # smooth decel via interrupt_with()
while s.moving:
    time.sleep_ms(5)

stopped_steps = s._pulseCounter.value
print(f'stopped steps={stopped_steps}')

# Phase 2: move back to origin
s.moveTo(0)
s.waitEndOfMove()
print(f'done steps={s._pulseCounter.value}')

end_ms = time.ticks_ms()
print(f"Total time: {time.ticks_diff(end_ms, start_ms)/1000:.3f}")
