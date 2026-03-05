# -*- coding: utf-8 -*-

"""MultiAxis — synchronized multi-axis motion controller.

Uses the RP2040/RP2350 DMA_MULTI_CHAN_TRIGGER register to fire all DMA
channels in a single AHB bus write, guaranteeing hardware-simultaneous start
across all axes.

Synchronization model (CNC linear-interpolation style):
  All axes start at the same hardware instant AND finish at the same time.
  The dominant axis (longest natural total-move time) runs at its natural
  peak speed. Every other axis has its peak speed reduced so its total move
  time equals the dominant axis's total time.  The result is straight-line
  motion in N-dimensional position space.
"""

import asyncio
import math
import machine

from .pulseGenerator import PulseGenerator

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
          1. For each axis compute its natural total move time at full speed.
          2. T_dominant = max of all per-axis total times.
          3. For each subordinate axis, binary-search for the peak speed that
             makes its total move time equal T_dominant (slowing it down).
          4. Call axis.prepare_move(target, forced_peak=v_peak) for each axis.
          5. Fire all DMA channels simultaneously via DMA_MULTI_CHAN_TRIGGER.

        Raises ValueError if an axis cannot be slowed to match T_dominant
        (i.e. even at minSpeed the axis finishes before T_dominant).
        """
        if isinstance(targets, dict):
            axis_targets = [(ax, targets[ax]) for ax in self._axes if ax in targets]
        else:
            axis_targets = list(zip(self._axes, targets))

        # 1. Compute natural total time for each axis
        total_times = []
        distances = []
        for ax, tgt in axis_targets:
            d = abs(tgt - ax.position)
            distances.append(d)
            total_times.append(self._compute_total_time(ax, d))

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

    def _compute_total_time(self, axis, distance):
        """Return the total move duration at the axis's natural (maximum) speed.

        Computes the natural triangular peak (starting from minSpeed).  If
        the natural peak exceeds hard_max_speed the profile is trapezoidal
        at hard_max_speed.
        """
        stepper = axis.stepper
        accel = stepper.acceleration
        v_min = stepper.minSpeed
        v_max = axis.hard_max_speed

        if distance < 1e-9:
            return 0.0

        v_nat = math.sqrt(distance * accel + v_min ** 2)
        v_peak = min(v_nat, v_max)
        return self._total_time_for_vpeak(accel, v_min, distance, v_peak)

    @staticmethod
    def _total_time_for_vpeak(accel, v_min, distance, v_peak):
        """Total move time for a symmetric trapezoidal/triangular profile.

        Assumes accel == decel and the profile goes v_min -> v_peak -> v_min.
        """
        accel_dist = (v_peak ** 2 - v_min ** 2) / (2 * accel)
        const_dist = distance - 2 * accel_dist
        t_accel = (v_peak - v_min) / accel
        t_const = const_dist / v_peak if const_dist > 0.0 else 0.0
        return 2 * t_accel + t_const

    def _find_vpeak_for_time(self, axis, distance, T_target):
        """Binary-search for the peak speed that makes total_time == T_target.

        Searches v_peak in [v_min, min(v_max, v_nat)].  total_time is
        monotonically decreasing in v_peak over this interval:
          - At v_min: total_time = distance / v_min  (slowest)
          - At v_nat (or v_max): total_time is minimum

        Raises ValueError if T_target > distance / v_min (impossible to slow
        the axis enough with the given minSpeed constraint).
        """
        stepper = axis.stepper
        accel = stepper.acceleration
        v_min = stepper.minSpeed
        v_nat = math.sqrt(distance * accel + v_min ** 2)
        v_hi = min(v_nat, axis.hard_max_speed)
        v_lo = v_min

        T_max = distance / v_min
        if T_target > T_max + 1e-9:
            raise ValueError(
                f"Cannot synchronize: axis distance={distance:.3f} needs "
                f"{T_target:.4f}s but maximum achievable time at minSpeed is "
                f"{T_max:.4f}s"
            )

        # 48 iterations -> precision < 2^-48 of the initial interval
        for _ in range(48):
            v_mid = (v_lo + v_hi) / 2
            T_mid = self._total_time_for_vpeak(accel, v_min, distance, v_mid)
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
