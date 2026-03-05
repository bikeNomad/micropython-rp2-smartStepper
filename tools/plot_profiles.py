#!/usr/bin/env python3
"""
plot_profiles.py - visualize SmartStepper motion profiles.

Ports the exact _accelPoints / _buildProfile logic from smartStepper.py
to standard Python, simulates discrete step generation, and plots speed,
position, and instantaneous acceleration vs. time for each of the four
curve types.

Requirements: matplotlib  (pip install matplotlib)

Usage:
    python tools/plot_profiles.py [options]

Options:
    --distance F        Move distance in units (default: 100)
    --min-speed F       Min speed in units/s (default: 5)
    --max-speed F       Max speed in units/s (default: 50)
    --acceleration F    Acceleration in units/s2 (default: 200)
    --steps-per-unit F  Steps per unit (default: 96)
    --triangular        Triangular profile (no constant-velocity phase)
    --accel-time F      Fixed acceleration phase duration in seconds
    --output FILE       Save to FILE (default: profiles.png)
    --show              Display interactively instead of saving to file
"""

import math
import argparse
import sys

try:
    import matplotlib
    from matplotlib import gridspec
    import matplotlib.pyplot as plt
except ImportError:
    print(
        "matplotlib is required.  Install with:  pip install matplotlib",
        file=sys.stderr,
    )
    sys.exit(1)

# ---------------------------------------------------------------------------
# Curve definitions - identical to SmartStepper._accel()
# ---------------------------------------------------------------------------

NB_ACCEL_PTS = 100

_CURVE_FNS = {
    'linear': lambda x: x,
    'smooth1': lambda x: x * x * (3 - 2 * x),
    'smooth2': lambda x: x * x * x * (x * (x * 6 - 15) + 10),
    'sine': lambda x: (math.cos((x + 1) * math.pi) + 1) / 2,
}

_COLORS = {
    'linear': '#e74c3c',
    'smooth1': '#3498db',
    'smooth2': '#27ae60',
    'sine': '#e67e22',
}

_LABELS = {
    'linear': 'linear',
    'smooth1': 'smooth1 (smoothstep)',
    'smooth2': 'smooth2 (smootherstep)',
    'sine': 'sine',
}

CURVE_ORDER = ('linear', 'smooth1', 'smooth2', 'sine')


def _make_accel_table(curve_name, nb_pts=NB_ACCEL_PTS):
    f = _CURVE_FNS[curve_name]
    return [f(i / nb_pts) for i in range(nb_pts + 1)]


# ---------------------------------------------------------------------------
# Profile simulator - mirrors SmartStepper logic exactly
# ---------------------------------------------------------------------------

class ProfileSimulator:
    """
    Mirrors SmartStepper._accelPoints and ._buildProfile in standard Python.
    No hardware or MicroPython dependencies.
    """

    def __init__(self, min_speed, max_speed, acceleration,
                 steps_per_unit=96, nb_accel_pts=NB_ACCEL_PTS,
                 curve='smooth2'):
        """Initialise with the same parameters as SmartStepper."""
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.steps_per_unit = steps_per_unit
        self.nb_accel_pts = nb_accel_pts
        self.accel_table = _make_accel_table(curve, nb_accel_pts)

    def accel_points(self, from_speed, to_speed, accel=None):
        """Port of SmartStepper._accelPoints."""
        if accel is None:
            accel = self.acceleration

        if from_speed > to_speed:
            pts = self.accel_points(to_speed, from_speed, accel=accel)
            pts.reverse()
            return pts

        accel_time = (to_speed - from_speed) / accel
        accel_dist = (to_speed**2 - from_speed**2) / (2 * accel)
        accel_steps = round(accel_dist * self.steps_per_unit)

        points = []
        real_steps = 0
        dt = accel_time / self.nb_accel_pts

        for i in range(self.nb_accel_pts):
            y = self.accel_table[i]
            speed = to_speed * y + from_speed * (1 - y)
            pulses = round(speed * self.steps_per_unit * dt)
            if pulses:
                points.append((speed * self.steps_per_unit, pulses))
            real_steps += pulses

        if real_steps < accel_steps:
            points.append(
                (to_speed * self.steps_per_unit, accel_steps - real_steps)
            )
        elif real_steps > accel_steps:
            excess = real_steps - accel_steps
            while excess > 0 and points:
                freq, n = points[-1]
                trim = min(n, excess)
                if n - trim > 0:
                    points[-1] = (freq, n - trim)
                else:
                    points.pop()
                excess -= trim

        return points

    def build_profile(self, from_speed, remaining,
                      triangular=False, forced_peak=None):
        """Port of SmartStepper._buildProfile."""
        if abs(remaining) < 1.0 / self.steps_per_unit:
            return []

        accel = self.acceleration

        if forced_peak is not None:
            peak_speed = min(forced_peak, self.max_speed)
        else:
            peak = math.sqrt(
                (abs(remaining) * 2 * accel
                 + from_speed**2 + self.min_speed**2) / 2
            )
            if triangular and peak > self.max_speed:
                accel = (
                    (2 * self.max_speed**2
                     - from_speed**2 - self.min_speed**2)
                    / (2 * abs(remaining))
                )
                peak_speed = self.max_speed
            else:
                peak_speed = min(peak, self.max_speed)

        points = []

        if from_speed < peak_speed - 1e-6:
            points.extend(
                self.accel_points(from_speed, peak_speed, accel=accel)
            )

        if not triangular and peak_speed >= self.max_speed - 1e-6:
            accel_up_dist = (peak_speed**2 - from_speed**2) / (2 * accel)
            decel_dist = (peak_speed**2 - self.min_speed**2) / (2 * accel)
            const_dist = abs(remaining) - accel_up_dist - decel_dist
            if const_dist > 0:
                points.append((
                    peak_speed * self.steps_per_unit,
                    round(const_dist * self.steps_per_unit),
                ))

        points.extend(
            self.accel_points(peak_speed, self.min_speed, accel=accel)
        )
        return points

    def simulate(self, profile):
        """
        Expand a (freq_steps_per_s, n_pulses) profile into time-series data.

        Returns:
            times     - N+1 boundary times [s]
            speeds    - N+1 speeds [units/s]; speeds[i] is held during
                        segment i; speeds[-1] = min_speed (motor stopped)
            positions - N+1 cumulative positions [units] at each boundary
        """
        times = [0.0]
        speeds = []
        positions = [0.0]
        t = 0.0
        pos = 0.0

        for freq_steps, n_pulses in profile:
            speed = freq_steps / self.steps_per_unit
            duration = n_pulses / freq_steps
            speeds.append(speed)
            t += duration
            pos += n_pulses / self.steps_per_unit
            times.append(t)
            positions.append(pos)

        speeds.append(self.min_speed)  # speed after last segment
        return times, speeds, positions


# ---------------------------------------------------------------------------
# Acceleration and jerk - numerical derivatives of the speed step function
# ---------------------------------------------------------------------------

def _compute_accel(times, speeds):
    """
    Compute effective acceleration at each segment boundary.

    In the step-function model speed is constant within each segment and
    changes abruptly at boundaries.  The rate of change across each
    boundary approximates the derivative of the acceleration curve.

    Returns (accel_times, accel_vals), each of length N-1.
    """
    accel_times = []
    accel_vals = []
    for i in range(len(times) - 2):
        dt = times[i + 1] - times[i]
        if dt > 0:
            accel_times.append(times[i + 1])
            accel_vals.append((speeds[i + 1] - speeds[i]) / dt)
    return accel_times, accel_vals


def _compute_jerk(accel_times, accel_vals):
    """
    Compute effective jerk (d/dt of acceleration) at interior accel boundaries.

    Returns (jerk_times, jerk_vals), each of length len(accel_vals)-1.
    """
    jerk_times = []
    jerk_vals = []
    for i in range(len(accel_times) - 1):
        dt = accel_times[i + 1] - accel_times[i]
        if dt > 0:
            jerk_times.append((accel_times[i] + accel_times[i + 1]) / 2)
            jerk_vals.append(
                (accel_vals[i + 1] - accel_vals[i]) / dt
            )
    return jerk_times, jerk_vals


# ---------------------------------------------------------------------------
# Main plotting function
# ---------------------------------------------------------------------------

def plot_all_curves(args):
    """Build and return the figure with all four curve profiles."""
    title_line2 = (
        f"distance={args.distance} units, "
        f"min_speed={args.min_speed} u/s, "
        f"max_speed={args.max_speed} u/s, "
        f"acceleration={args.acceleration} u/s\u00b2"
    )
    if args.triangular:
        title_line2 += " [triangular]"
    elif args.accel_time is not None:
        title_line2 += f" [accel_time={args.accel_time}s]"

    fig = plt.figure(figsize=(11, 10))
    fig.suptitle(
        "SmartStepper motion profiles\n" + title_line2,
        fontsize=11,
    )

    gs = gridspec.GridSpec(4, 1, hspace=0.55)
    ax_speed = fig.add_subplot(gs[0])
    ax_pos = fig.add_subplot(gs[1])
    ax_accel = fig.add_subplot(gs[2])
    ax_jerk = fig.add_subplot(gs[3])

    ax_speed.set_ylabel("Speed (units/s)")
    ax_pos.set_ylabel("Position (units)")
    ax_accel.set_ylabel("Accel (units/s\u00b2)")
    ax_jerk.set_ylabel("Jerk (units/s\u00b3)")
    ax_jerk.set_xlabel("Time (s)")

    ax_speed.set_title("Speed vs. time")
    ax_pos.set_title("Position vs. time")
    ax_accel.set_title("Effective acceleration vs. time")
    ax_jerk.set_title("Effective jerk vs. time")

    forced_peak = None
    if args.accel_time is not None:
        forced_peak = args.min_speed + args.acceleration * args.accel_time

    for curve in CURVE_ORDER:
        sim = ProfileSimulator(
            min_speed=args.min_speed,
            max_speed=args.max_speed,
            acceleration=args.acceleration,
            steps_per_unit=args.steps_per_unit,
            nb_accel_pts=NB_ACCEL_PTS,
            curve=curve,
        )
        profile = sim.build_profile(
            from_speed=args.min_speed,
            remaining=args.distance,
            triangular=args.triangular,
            forced_peak=forced_peak,
        )
        if not profile:
            print(
                f"Warning: {curve} produced an empty profile"
                " - distance too small?",
                file=sys.stderr,
            )
            continue

        times, speeds, positions = sim.simulate(profile)
        color = _COLORS[curve]
        label = _LABELS[curve]

        # Speed - step plot: speeds[i] is held from times[i] to times[i+1]
        ax_speed.step(times, speeds, where='post',
                      color=color, label=label, linewidth=1.5)

        # Position - piecewise linear (integral of the speed step function)
        ax_pos.plot(times, positions,
                    color=color, label=label, linewidth=1.5)

        # Acceleration - numerical derivative at segment boundaries
        at, av = _compute_accel(times, speeds)
        ax_accel.plot(at, av, color=color, label=label, linewidth=1.5)

        # Jerk - numerical derivative of acceleration
        jt, jv = _compute_jerk(at, av)
        ax_jerk.plot(jt, jv, color=color, label=label, linewidth=1.5)

    ax_accel.axhline(0, color='black', linewidth=0.6, linestyle='--')
    ax_jerk.axhline(0, color='black', linewidth=0.6, linestyle='--')
    # Symlog scale keeps zero visible while compressing large linear spikes
    # so smooth-curve jerk remains readable alongside linear's step-changes.
    ax_jerk.set_yscale('symlog')

    for ax in (ax_speed, ax_pos, ax_accel, ax_jerk):
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)

    return fig


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    """Parse command-line arguments."""
    p = argparse.ArgumentParser(
        description="Plot SmartStepper motion profiles for all curves.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--distance', type=float, default=100, metavar='F',
                   help="Move distance in units")
    p.add_argument('--min-speed', type=float, default=5, metavar='F',
                   help="Min speed in units/s")
    p.add_argument('--max-speed', type=float, default=50, metavar='F',
                   help="Max speed in units/s")
    p.add_argument('--acceleration', type=float, default=200, metavar='F',
                   help="Acceleration in units/s2")
    p.add_argument('--steps-per-unit', type=float, default=96, metavar='F',
                   help="Steps per unit")
    p.add_argument('--triangular', action='store_true',
                   help="Triangular profile (no constant-velocity phase)")
    p.add_argument('--accel-time', type=float, default=None, metavar='F',
                   help="Fixed acceleration phase duration in seconds")
    p.add_argument('--output', default='profiles.png', metavar='FILE',
                   help="Output filename (ignored when --show is used)")
    p.add_argument('--show', action='store_true',
                   help="Display interactively instead of saving")
    return p.parse_args()


def main():
    """Entry point."""
    args = parse_args()

    if args.triangular and args.accel_time is not None:
        print(
            "Error: --triangular and --accel-time are mutually exclusive.",
            file=sys.stderr,
        )
        sys.exit(1)

    if not args.show:
        matplotlib.use('Agg')

    fig = plot_all_curves(args)

    if args.show:
        plt.show()
    else:
        fig.savefig(args.output, dpi=150, bbox_inches='tight')
        print(f"Saved {args.output}")


if __name__ == '__main__':
    main()
