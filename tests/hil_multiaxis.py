"""HIL on-Pico script: MultiAxis.move() with two steppers.

Parameters (chosen so axes have different natural total times):
  Axis 1: 40 units * 96 steps/unit = 3840 steps
    natural peak = 50 u/s (capped); T_total ~= 0.934 s (dominant)
  Axis 2: 30 units * 96 steps/unit = 2880 steps
    natural peak ~= 95 u/s -> capped to 50; T_natural ~= 0.733 s
    MultiAxis slows axis 2 to v_peak ~= 35 u/s -> T_total ~= 0.934 s

Both axes start simultaneously (DMA_MULTI_CHAN_TRIGGER) and finish together.
Axis 2's accel ramp (5 -> 35 u/s) is visible on the logic analyser.

Expected output line: done x_steps=<n> y_steps=<m>
"""

import asyncio
from smartstepper import SmartStepper, Axis, MultiAxis
from test_config import STEP_PIN, DIR_PIN, ENABLE_PIN, STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2

s1 = SmartStepper(STEP_PIN, DIR_PIN, ENABLE_PIN)
s1.stepsPerUnit = 96
s1.minSpeed = 5
s1.maxSpeed = 50
s1.acceleration = 300

s2 = SmartStepper(STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2)
s2.stepsPerUnit = 96
s2.minSpeed = 5
s2.maxSpeed = 50
s2.acceleration = 300

x = Axis(s1, hard_max_speed=50, hard_max_accel=300)
y = Axis(s2, hard_max_speed=50, hard_max_accel=300)
ma = MultiAxis([x, y])


async def main():
    ma.move({x: 40, y: 30})
    await ma.wait_done()
    print("done x_steps={} y_steps={}".format(
        s1._pulseCounter.value, s2._pulseCounter.value))


asyncio.run(main())
