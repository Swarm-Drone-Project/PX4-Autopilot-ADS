#!/usr/bin/env python3
"""Step-response metrics for a step_rig_att run, from its .ulg.

Reads the logged attitude setpoint and attitude estimate, finds the commanded
step on each axis independently, and reports rise time, peak time, percent
overshoot and settling time. Roll and pitch are treated separately so a
single-axis run reports one axis and skips the flat one.

The log, not the live stream, is the measurement: it is logged at full rate on
the vehicle with no link losses in between.

Usage:
    ./analyze_step_response.py log.ulg
    ./analyze_step_response.py log.ulg --settle-band 5 --plot
"""

import argparse
import math
import sys

import numpy as np
from pyulog import ULog

# A commanded step shows up as a single-sample discontinuity. Tracking wander
# (this module mirrors the live attitude before OFFBOARD) moves the setpoint
# just as far, but gradually - so detection keys on the jump between
# consecutive samples, never on displacement from the start of the log.
STEP_JUMP_DEG = 5.0
# After a real step the setpoint HOLDS. Loss of control, a crash or a mode
# change also produce large jumps, but the value thrashes rather than settling,
# so a candidate must stay put for this long, within this spread, to count.
STEP_HOLD_S = 1.0
STEP_HOLD_SPREAD_DEG = 5.0
# A commanded move smaller than this has no meaningful rise time to report.
MIN_STEP_DEG = 1.0


def quaternion_to_roll_pitch(q):
    """Roll/pitch in degrees from (w, x, y, z) arrays, intrinsic Z-Y-X as PX4."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return np.degrees(roll), np.degrees(pitch)


def load_series(ulog, dataset, quat_field):
    data = ulog.get_dataset(dataset)
    t = np.array(data.data["timestamp"], dtype=np.float64) * 1e-6
    q = np.column_stack([np.array(data.data[f"{quat_field}[{i}]"], dtype=np.float64)
                         for i in range(4)])
    roll, pitch = quaternion_to_roll_pitch(q)
    return t, roll, pitch


def find_step(t_sp, sp):
    """Index, time and held value of the commanded step, or None.

    Returns the first discontinuity that is both large and followed by a hold.
    Both conditions are needed on a real flight log: the "large" test alone
    also fires on the thrashing setpoint of a crash or a mode change, and a
    displacement test fires on ordinary pre-OFFBOARD attitude tracking.
    """
    if len(sp) < 3:
        return None

    candidates = []
    for j in np.flatnonzero(np.abs(np.diff(sp)) > STEP_JUMP_DEG):
        idx = int(j) + 1
        t0 = t_sp[idx]
        held = sp[(t_sp >= t0) & (t_sp <= t0 + STEP_HOLD_S)]
        if len(held) < 2 or float(held.max() - held.min()) > STEP_HOLD_SPREAD_DEG:
            continue
        # Median over the hold rather than the single post-jump sample, so one
        # noisy sample cannot set the target the whole run is scored against.
        target = float(np.median(held))
        candidates.append((idx, float(t0), target, abs(target - float(sp[idx - 1]))))

    if not candidates:
        return None

    # Earliest of the near-largest, which is not the same as either "first" or
    # "largest" alone. A run holds the commanded attitude and later releases it
    # on disarm or a crash, and that release is a discontinuity of the SAME
    # magnitude in the opposite direction - so "largest" picks a coin flip
    # between the step and its mirror image, while "first" picks up any small
    # jump that happens to precede the command.
    biggest = max(c[3] for c in candidates)
    for idx, t0, target, amplitude in candidates:
        if amplitude >= 0.9 * biggest:
            return idx, t0, target

    return None


def crossing_time(t, y, level, rising):
    """Linearly interpolated first time y crosses level."""
    hit = (y >= level) if rising else (y <= level)
    if not hit.any():
        return None
    i = int(np.argmax(hit))
    if i == 0:
        return t[0]
    # Interpolate between the straddling samples so the result is not quantized
    # to the logging interval - at 200+ Hz that is small, but rise times on a
    # well-tuned axis are only a few hundred ms.
    y0, y1, t0, t1 = y[i - 1], y[i], t[i - 1], t[i]
    if y1 == y0:
        return t1
    return t0 + (level - y0) * (t1 - t0) / (y1 - y0)


def first_peak(t, y, target, rising, amplitude):
    """Time and value of the FIRST peak past the target - classical peak time.

    Deliberately not the global maximum of the window: on a descending vehicle
    the response keeps wandering long after the transient is over, and the
    largest excursion in the log is frequently late drift that has nothing to
    do with the step. A response that never passes the target has no overshoot,
    and no peak time to report.
    """
    # Smoothed only for locating the turning point; the value reported is the
    # raw sample there, so the printed overshoot is a real measurement.
    kernel = min(5, len(y))
    smooth = np.convolve(y, np.ones(kernel) / kernel, mode="same") if kernel > 1 else y

    past_target = (smooth > target) if rising else (smooth < target)
    if not past_target.any():
        return None, None, 0.0, 0.0

    start = int(np.argmax(past_target))
    for i in range(start + 1, len(smooth) - 1):
        turning = (smooth[i] >= smooth[i + 1]) if rising else (smooth[i] <= smooth[i + 1])
        if turning and past_target[i]:
            overshoot_deg = (y[i] - target) if rising else (target - y[i])
            overshoot_deg = max(overshoot_deg, 0.0)
            return float(t[i]), float(y[i]), overshoot_deg, 100.0 * overshoot_deg / abs(amplitude)

    # Crossed the target but never turned back inside the window.
    i = int(np.argmax(y) if rising else np.argmin(y))
    overshoot_deg = max((y[i] - target) if rising else (target - y[i]), 0.0)
    return float(t[i]), float(y[i]), overshoot_deg, 100.0 * overshoot_deg / abs(amplitude)


def analyze_axis(name, t_sp, sp, t_att, att, settle_band_pct, window_s):
    step = find_step(t_sp, sp)
    if step is None:
        return None, f"{name}: no commanded step found (setpoint never steps and holds)"

    _, t0, target = step

    # Response from the step onward, with the pre-step value as the baseline -
    # a step from 0 and a step from an already-tilted attitude are both handled.
    # Bounded window. The vehicle is descending throughout a large-angle hold
    # (fixed thrust, no altitude loop), so the tail of the log is the fall, not
    # the transient - left unbounded, a late drift becomes "the peak" and the
    # settling time becomes the length of the log.
    mask = (t_att >= t0) & (t_att <= t0 + window_s)
    if mask.sum() < 10:
        return None, f"{name}: too few samples after the step"
    t = t_att[mask] - t0
    y = att[mask]
    y0 = float(y[0])

    amplitude = target - y0
    if abs(amplitude) < MIN_STEP_DEG:
        return None, f"{name}: step of {amplitude:.2f} deg is too small to characterize"

    rising = amplitude > 0
    # Percent-of-amplitude levels, so a negative step works without sign games.
    def level(frac):
        return y0 + frac * amplitude

    t10 = crossing_time(t, y, level(0.10), rising)
    t90 = crossing_time(t, y, level(0.90), rising)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    # Dead time is reported separately from rise time because the two have
    # different causes and the 10-90% definition hides it entirely. A large
    # value here usually means the airframe could not respond yet rather than
    # that the loop is slow - a vehicle still on the ground cannot roll, which
    # is why the SITL procedure starts the test airborne.

    peak_time, peak_value, overshoot_deg, overshoot_pct = first_peak(
        t, y, target, rising, amplitude)

    # Settling: scan backwards for the last exit from the band, which is robust
    # to noise dipping back in and out near the end of the window.
    band = abs(amplitude) * settle_band_pct / 100.0
    outside = np.abs(y - target) > band
    if outside.any():
        last_out = int(np.max(np.flatnonzero(outside)))
        settle = float(t[last_out]) if last_out < len(t) - 1 else None
    else:
        settle = 0.0

    return {
        "axis": name,
        "window": window_s,
        "t0": t0,
        "delay": t10,
        "from": y0,
        "target": target,
        "amplitude": amplitude,
        "rise": rise,
        "peak_time": peak_time,
        "peak_value": peak_value,
        "overshoot_deg": overshoot_deg,
        "overshoot_pct": overshoot_pct,
        "settle": settle,
        "band_deg": band,
        "final": float(y[-1]),
        "t": t,
        "y": y,
    }, None


def format_result(r, settle_band_pct):
    def fmt(v, unit, nd=3):
        return "n/a" if v is None else f"{v:.{nd}f} {unit}"

    print(f"\n  {r['axis'].upper()}")
    print(f"    step          : {r['from']:.2f} -> {r['target']:.2f} deg "
          f"(amplitude {r['amplitude']:+.2f} deg) at t = {r['t0']:.3f} s")
    print(f"    dead time     : {fmt(r['delay'], 's')}   (step -> 10% of amplitude)")
    print(f"    rise time     : {fmt(r['rise'], 's')}   (10% -> 90% of amplitude)")
    if r["peak_time"] is None:
        print("    peak time     : n/a   (response never passed the target - no overshoot)")
    else:
        print(f"    peak time     : {fmt(r['peak_time'], 's')}   (peak {r['peak_value']:.2f} deg)")
    print(f"    overshoot     : {r['overshoot_pct']:.1f} %  ({r['overshoot_deg']:+.2f} deg)")
    settle_txt = (f"{r['settle']:.3f} s" if r["settle"] is not None
                  else f"not within {r['window']:.0f} s")
    print(f"    settling time : {settle_txt}   "
          f"(+/-{settle_band_pct:.0f}% = +/-{r['band_deg']:.2f} deg)")
    print(f"    final value   : {r['final']:.2f} deg "
          f"(steady-state error {r['final'] - r['target']:+.2f} deg)")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ulog", help="path to the .ulg file")
    parser.add_argument("--settle-band", type=float, default=2.0,
                        help="settling band as %% of step amplitude (default: 2)")
    parser.add_argument("--window", type=float, default=10.0,
                        help="seconds after the step to analyze (default: 10)")
    parser.add_argument("--plot", action="store_true", help="overlay setpoint and response")
    args = parser.parse_args()

    try:
        ulog = ULog(args.ulog)
    except Exception as exc:
        print(f"failed to read {args.ulog}: {exc}", file=sys.stderr)
        return 1

    try:
        t_sp, roll_sp, pitch_sp = load_series(ulog, "vehicle_attitude_setpoint", "q_d")
        t_att, roll, pitch = load_series(ulog, "vehicle_attitude", "q")
    except (KeyError, IndexError) as exc:
        print(f"missing required topic/field in log: {exc}", file=sys.stderr)
        print("vehicle_attitude_setpoint and vehicle_attitude must both be logged.", file=sys.stderr)
        return 1

    commanded = {}
    for name in ("STEPATT_ROLL", "STEPATT_PITCH", "STEPATT_THRUST"):
        if name in ulog.initial_parameters:
            commanded[name] = ulog.initial_parameters[name]

    print(f"log: {args.ulog}")
    if commanded:
        print("logged parameters: " + ", ".join(f"{k}={v}" for k, v in commanded.items()))
    print(f"samples: {len(t_att)} attitude, {len(t_sp)} setpoint")

    results = []
    for name, sp, att in (("roll", roll_sp, roll), ("pitch", pitch_sp, pitch)):
        result, skip_reason = analyze_axis(name, t_sp, sp, t_att, att, args.settle_band, args.window)
        if result is None:
            print(f"\n  {skip_reason}")
        else:
            results.append(result)
            format_result(result, args.settle_band)

    if not results:
        print("\nNo step found on either axis - was OFFBOARD engaged during this log?")
        return 1

    if args.plot:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(len(results), 1, figsize=(10, 4 * len(results)), squeeze=False)
        for ax, r in zip(axes[:, 0], results):
            ax.axhline(r["target"], color="k", linestyle="--", linewidth=1, label="target")
            ax.axhspan(r["target"] - r["band_deg"], r["target"] + r["band_deg"],
                       color="0.85", label=f"+/-{args.settle_band:.0f}% band")
            ax.plot(r["t"], r["y"], linewidth=1.4, label="response")
            if r["peak_time"] is not None:
                ax.axvline(r["peak_time"], color="r", linestyle=":", linewidth=1, label="peak")
            if r["settle"]:
                ax.axvline(r["settle"], color="g", linestyle=":", linewidth=1, label="settled")
            ax.set_title(f"{r['axis']} step response "
                         f"({r['from']:.1f} -> {r['target']:.1f} deg)")
            ax.set_xlabel("time since step (s)")
            ax.set_ylabel("deg")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="lower right", fontsize=8)
        plt.tight_layout()
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
