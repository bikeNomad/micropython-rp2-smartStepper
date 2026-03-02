"""Host-side hardware-in-the-loop tests for stepper-lib.

Prerequisites:
  - Logic 2 open with Automation enabled (Settings → Automation, port 10430)
  - Saleae wired to Pico per hil_config.py
  - Pico connected via USB

Run:
  python test_hil.py
"""

import csv
import subprocess
import sys
import tempfile
from pathlib import Path

import saleae.automation as saleae

import hil_config

# ---------------------------------------------------------------------------
# Infrastructure
# ---------------------------------------------------------------------------

PICO_FILES = [
    'smartStepper.py',
    'pulseGenerator.py',
    'pulseCounter.py',
    'test_config.py',
    'hil_moveto.py',
]


def _mpremote(*args):
    """Run an mpremote command; return (stdout, returncode)."""
    cmd = ['mpremote', 'connect', hil_config.PICO_PORT] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.stdout + result.stderr, result.returncode


def deploy():
    """Copy library and HIL scripts to the Pico."""
    print('  Deploying files to Pico...')
    stdout, rc = _mpremote('cp', *PICO_FILES, ':')
    if rc != 0:
        raise RuntimeError(f'deploy failed:\n{stdout}')


def parse_rising_edges(digital_csv: Path, channel: int) -> list:
    """Return list of timestamps (s) for rising edges on the given Saleae channel."""
    col = f'Channel {channel}'
    edges = []
    prev = None
    with open(digital_csv, newline='') as f:
        for row in csv.DictReader(f):
            val = int(row[col])
            if prev == 0 and val == 1:
                edges.append(float(row['Time [s]']))
            prev = val
    return edges


def run_capture(manager: saleae.Manager, script: str, channels: list):
    """
    Start a manual Saleae capture, run a Pico script synchronously, then stop.

    Returns (tmpdir Path, mpremote stdout string).
    Raises RuntimeError if the Pico script exits non-zero.
    """
    tmpdir = Path(tempfile.mkdtemp())
    dev_cfg = saleae.LogicDeviceConfiguration(
        enabled_digital_channels=channels,
        digital_sample_rate=hil_config.DIGITAL_SAMPLE_RATE,
    )
    cap_cfg = saleae.CaptureConfiguration(
        capture_mode=saleae.ManualCaptureMode()
    )

    cap = manager.start_capture(device_configuration=dev_cfg,
                                capture_configuration=cap_cfg)
    try:
        stdout, rc = _mpremote('run', script)
        cap.stop()
        if rc != 0:
            raise RuntimeError(f'mpremote run {script} failed:\n{stdout}')
        cap.export_raw_data_csv(str(tmpdir), digital_channels=channels)
    finally:
        cap.close()

    return tmpdir, stdout


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_pulse_generator(manager: saleae.Manager):
    """
    PulseGenerator points=((1,3),(5,5)) → 8 pulses (±1 for profile rounding).
    First 2 inter-pulse gaps ~1 s (1 Hz); last 5 gaps ~0.2 s (5 Hz).
    """
    channels = [hil_config.STEP_CHANNEL]
    tmpdir, _ = run_capture(manager, 'test_pulseGenerator.py', channels)

    edges = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert abs(len(edges) - 8) <= 1, f'Expected 8 pulses (±1), got {len(edges)}'

    # Use the last 8 edges (skip any leading spurious edge from PIO init)
    edges = edges[-8:]
    gaps = [edges[i + 1] - edges[i] for i in range(len(edges) - 1)]
    # Gaps 0-1: ~1 Hz
    for i in range(2):
        assert 0.8 < gaps[i] < 1.25, f'Gap {i} expected ~1 s, got {gaps[i]:.3f} s'
    # Gaps 2-6: ~5 Hz (0.2 s)
    for i in range(2, 7):
        assert 0.15 < gaps[i] < 0.28, f'Gap {i} expected ~0.2 s, got {gaps[i]:.3f} s'

    print(f'  PASS  test_pulse_generator  ({len(edges)} pulses, timing OK)')


def test_moveto_pulse_count(manager: saleae.Manager):
    """
    moveTo(50) with stepsPerUnit=96 → ~4800 step pulses.
    Saleae and PulseCounter must agree exactly; count must be within 1 of 4800.
    """
    channels = [hil_config.STEP_CHANNEL, hil_config.DIR_CHANNEL]
    tmpdir, stdout = run_capture(manager, 'hil_moveto.py', channels)

    saleae_count = len(parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL))

    # Cross-check with PulseCounter value printed by the Pico script
    pico_count = None
    for line in stdout.splitlines():
        if line.startswith('done steps='):
            pico_count = int(line.split('=')[1])
            break

    assert pico_count is not None, f'Pico script did not print "done steps=..."\nstdout: {stdout}'
    assert saleae_count == pico_count, \
        f'Saleae ({saleae_count}) and PulseCounter ({pico_count}) disagree'
    assert abs(saleae_count - 4800) <= 1, \
        f'Expected ~4800 pulses, got {saleae_count}'

    print(f'  PASS  test_moveto_pulse_count  ({saleae_count} pulses, matches PulseCounter)')


def test_accel_profile(manager: saleae.Manager):
    """
    moveTo should accel then cruise then decel.
    Checks that instantaneous frequency is strictly increasing at start
    and strictly decreasing at end of the move.
    """
    channels = [hil_config.STEP_CHANNEL]
    tmpdir, _ = run_capture(manager, 'hil_moveto.py', channels)

    edges = parse_rising_edges(tmpdir / 'digital.csv', hil_config.STEP_CHANNEL)
    assert len(edges) > 50, f'Too few pulses to check profile ({len(edges)})'

    # Instantaneous frequency between successive pulses
    freqs = [1.0 / (edges[i + 1] - edges[i]) for i in range(len(edges) - 1)]

    # Accel: first N frequency samples must be monotonically increasing
    n = min(20, len(freqs) // 4)
    accel = freqs[:n]
    assert all(accel[i] < accel[i + 1] for i in range(len(accel) - 1)), \
        f'Accel ramp not monotonically increasing: {[f"{f:.1f}" for f in accel]}'

    # Decel: last N frequency samples must be monotonically decreasing
    decel = freqs[-n:]
    assert all(decel[i] > decel[i + 1] for i in range(len(decel) - 1)), \
        f'Decel ramp not monotonically decreasing: {[f"{f:.1f}" for f in decel]}'

    peak_hz = max(freqs)
    print(f'  PASS  test_accel_profile  (peak {peak_hz:.0f} Hz, accel/decel ramps OK)')


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

TESTS = [
    test_pulse_generator,
    test_moveto_pulse_count,
    test_accel_profile,
]


def main():
    deploy()

    print('Connecting to Logic 2...')
    with saleae.Manager.connect(port=hil_config.LOGIC2_PORT) as manager:
        passed = failed = 0
        for test in TESTS:
            print(f'\n{test.__name__}')
            try:
                test(manager)
                passed += 1
            except AssertionError as e:
                print(f'  FAIL  {e}')
                failed += 1
            except Exception as e:
                print(f'  ERROR  {e}')
                failed += 1

    print(f'\n{passed} passed, {failed} failed')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
