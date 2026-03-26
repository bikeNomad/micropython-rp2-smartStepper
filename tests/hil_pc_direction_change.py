"""HIL script: PulseCounter accuracy through three direction-changing moves.

Move 1 (forward):  moveTo(30)  →  +2880 steps.
Move 2 (reverse):  moveTo(0)   →  -2880 steps back to origin.
Move 3 (forward):  moveTo(50)  →  +4800 steps.

Net signed displacement = +4800 steps.  Both the Saleae direction-aware edge
count and the PulseCounter must agree at each waypoint and at the final
position.

Prints:
  fwd  steps=<n1>   PulseCounter after move 1 (~+2880)
  rev  steps=<n2>   PulseCounter after move 2 (~0)
  done steps=<n3>   PulseCounter after move 3 (~+4800)
"""

from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50      # 4800 steps/s
s.acceleration = 300

# Move 1: forward to 30 units.
s.moveTo(30)
s.waitEndOfMove()
print(f'fwd steps={s._pulseCounter.value}')

# Move 2: reverse to origin.
s.moveTo(0)
s.waitEndOfMove()
print(f'rev steps={s._pulseCounter.value}')

# Move 3: forward to 50 units.
s.moveTo(50)
s.waitEndOfMove()
print(f'done steps={s._pulseCounter.value}')
