# -*- coding: utf-8 -*-

"""MultiAxis — synchronized multi-axis motion controller.

Uses the RP2040/RP2350 DMA_MULTI_CHAN_TRIGGER register to fire all DMA
channels in a single AHB bus write, guaranteeing hardware-simultaneous start
across all axes.

Synchronization model (CNC linear-interpolation style):
  All axes start at the same hardware instant AND finish at the same time.
  The dominant axis (longest actual total-move time) runs at its natural peak
  speed.  Every other axis has its peak speed reduced so its actual move time
  equals the dominant axis's actual time.  The result is straight-line motion
  in N-dimensional position space.

  Timing uses SmartStepper._profile_time() — the sum of real PIO-segment
  durations — rather than a continuous-time approximation.  This accounts for
  integer-step rounding in _accelPoints so the hardware finish times match.
"""

import asyncio
import math
import machine

# RP2040/RP2350: single write triggers all DMA channels whose bits are set.
_DMA_MULTI_CHAN_TRIGGER = 0x50000430


class MultiAxis:
    """Synchronized multi-axis motion controller.

    Example usage::

        x_axis = Axis(stepper_x, hard_max_speed=200, hard_max_accel=500)
        y_axis = Axis(stepper_y, hard_max_speed=150, hard_max_accel=400)
        ma = MultiAxis([x_axis, y_axis])

        # Synchronised move: all axes start and finish at the same time.
        ma.move({x_axis: 100, y_axis: 50})
        await ma.wait_done()
    """

    def __init__(self, axes):
        """axes: list of Axis objects (each wrapping a SmartStepper)."""
        self._axes = list(axes)

    def move(self, targets):
        """Synchronized multi-axis move (CNC linear-interpolation style).

        targets: dict {axis: target_position} or list of target positions
                 parallel to the axes list given at construction.

        Algorithm:
          1. For each axis compute its actual total move time at full speed
             using SmartStepper._profile_time() (accounts for step rounding).
          2. T_dominant = max of all per-axis actual times.
          3. For each subordinate axis, binary-search for the peak speed whose
             _profile_time equals T_dominant, slowing it down to match.
          4. Call axis.prepare_move(target, forced_peak=v_peak) for each axis.
          5. Fire all DMA channels simultaneously via DMA_MULTI_CHAN_TRIGGER.

        Raises ValueError if an axis cannot be slowed to match T_dominant
        (i.e. even at minSpeed the axis finishes before T_dominant).
        """
        if isinstance(targets, dict):
            axis_targets = [(ax, targets[ax]) for ax in self._axes if ax in targets]
        else:
            axis_targets = list(zip(self._axes, targets))

        # 1. Compute actual total time for each axis
        total_times = []
        distances = []
        for ax, tgt in axis_targets:
            d = abs(tgt - ax.position)
            distances.append(d)
            total_times.append(ax.stepper._profile_time(d))

        # 2. Dominant total time
        T_dominant = max(total_times) if total_times else 0.0

        # 3 & 4. Prepare each axis
        bitmask = 0
        for (ax, tgt), T_i, d in zip(axis_targets, total_times, distances):
            if d < 1e-9 or T_i >= T_dominant - 1e-9:
                # Dominant axis (or zero-distance): natural profile
                v_peak = None
            else:
                # Subordinate axis: solve for peak speed that gives T_dominant
                v_peak = self._find_vpeak_for_time(ax, d, T_dominant)
            ch = ax.prepare_move(tgt, forced_peak=v_peak)
            bitmask |= (1 << ch)

        # 5. Simultaneous hardware start
        if bitmask:
            machine.mem32[_DMA_MULTI_CHAN_TRIGGER] = bitmask

    # ------------------------------------------------------------------ #
    # Internal helpers                                                     #
    # ------------------------------------------------------------------ #

    def _find_vpeak_for_time(self, axis, distance, T_target):
        """Binary-search for the peak speed whose _profile_time == T_target.

        Searches v_peak in [v_min, min(v_max, v_nat)].  _profile_time is
        monotonically decreasing in v_peak over this interval:
          - At v_min: profile_time = distance / v_min  (slowest)
          - At v_nat or v_max: profile_time is minimum

        Using _profile_time (actual segment sums) rather than a continuous-
        time formula ensures the binary search target accounts for the same
        integer-step rounding as the dominant axis, so hardware finish times
        align closely.

        Raises ValueError if T_target > distance / v_min (impossible to slow
        the axis enough with the given minSpeed constraint).
        """
        stepper = axis.stepper
        accel = stepper.acceleration
        v_min = stepper.minSpeed
        v_nat = math.sqrt(distance * accel + v_min ** 2)
        v_hi = min(v_nat, axis.hard_max_speed)
        v_lo = v_min

        T_max = stepper._profile_time(distance, forced_peak=v_min)
        if T_target > T_max + 1e-3:
            raise ValueError(
                f"Cannot synchronize: axis distance={distance:.3f} needs "
                f"{T_target:.4f}s but maximum achievable time at minSpeed is "
                f"{T_max:.4f}s"
            )

        # 30 iterations -> precision < 2^-30 of the initial interval
        for _ in range(30):
            v_mid = (v_lo + v_hi) / 2
            T_mid = stepper._profile_time(distance, forced_peak=v_mid)
            if T_mid > T_target:
                v_lo = v_mid   # too slow -> increase v_peak
            else:
                v_hi = v_mid   # too fast -> decrease v_peak

        return (v_lo + v_hi) / 2

    async def wait_done(self):
        """Async wait until all axes finish moving."""
        while any(ax.moving for ax in self._axes):
            await asyncio.sleep_ms(1)

    def stop(self, emergency=False):
        """Stop all moving axes."""
        for ax in self._axes:
            if ax.moving:
                ax.stop(emergency)
