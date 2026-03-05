"""HIL on-Pico script: moveTo(5, triangular=True) with known parameters.

Parameters: stepsPerUnit=96, minSpeed=5, maxSpeed=50, acceleration=300.

Natural triangular peak = sqrt((5*2*300 + 5^2 + 5^2) / 2) ≈ 39.1 units/s,
which is below maxSpeed=50, so acceleration is unchanged.
accelDist = decelDist ≈ 2.5 units each; no constant-velocity section.
Expected: ~480 step pulses (5 units * 96 steps/unit).

Prints "done steps=<n>" on completion for host-side verification.
"""

import time
from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50
s.acceleration = 300

s.moveTo(5, triangular=True)
s.waitEndOfMove()
print("done steps={}".format(s._pulseCounter.value))
