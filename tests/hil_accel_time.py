"""HIL on-Pico script: moveTo(50, accel_time=0.1) with known parameters.

Parameters: stepsPerUnit=96, minSpeed=5, maxSpeed=50, acceleration=300.

forced_peak = minSpeed + acceleration * accel_time = 5 + 300*0.1 = 35 units/s.
35 < maxSpeed=50, so the cruise speed is distinctly below the speed ceiling.
accelDist = decelDist = (35^2 - 5^2) / (2*300) = 2 units each.
constDist = 50 - 2 - 2 = 46 units → most of the move is constant-speed.
Expected: ~4800 step pulses (50 units * 96 steps/unit).

Prints "done steps=<n>" on completion for host-side verification.
"""

from smartstepper import smartStepper
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN

s = smartStepper.SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s.stepsPerUnit = 96
s.minSpeed = 5
s.maxSpeed = 50
s.acceleration = 300

s.moveTo(50, accel_time=0.1)
s.waitEndOfMove()
print("done steps={}".format(s._pulseCounter.value))
