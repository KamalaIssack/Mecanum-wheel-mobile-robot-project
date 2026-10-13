#!/usr/bin/env python3
"""Plot step_logger.py CSVs and print a summary table.

    python3 plot_step.py logs/step_fl_kp*.csv          # Kp sweep, one wheel
    python3 plot_step.py logs/dir_*.csv                # forward and backward

Runs are overlaid with t = 0 at each run's own acknowledged step edge.

Table columns:
    mean    average speed over the last half of the hold (signed)
    error   how much slower than commanded the wheel ran (+ = too slow,
            in either direction)
    jitter  standard deviation over the same window
    over    overshoot in whole encoder counts: 1 = quantization, 2+ = real
    t63     time to 63% of the final mean (first-order time constant)
    gaps    host-clock gaps over 3 ticks (dropped telemetry lines)
"""

import argparse
import csv
import math
import os
import statistics
import sys

import matplotlib.pyplot as plt

TICK_S = 0.02                 # one V line per TIM6 tick
COUNT_STEP = 0.335            # rad/s per encoder count per tick


def load(path):
    """Return (metadata dict from the '# key=value' header, row dicts)."""
    meta = {}
    rows = []
    with open(path, newline="") as f:
        first = f.readline()
        if first.startswith("#"):
            for item in first[1:].split():
                if "=" in item:
                    k, v = item.split("=", 1)
                    meta[k] = v
        else:
            f.seek(0)
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("%s has no data rows" % path)
    if "wheel" not in meta:
        sys.exit("%s has no '# wheel=...' line; was it made by step_logger.py?" % path)
    return meta, rows


def count_overshoot(peak_along, target_mag):
    """Overshoot in encoder counts above the highest reading the target can
    produce. Readings are whole counts per tick, so a wheel tracking 4.0 rad/s
    (11.94 counts) reads 11 or 12, and a brief 13 needs only 0.5% excess
    speed, which load ripple supplies. One count is therefore not evidence of
    overshoot; two or more is."""
    peak_counts = round(peak_along / COUNT_STEP)          # readings are exact
    ceiling = math.ceil(target_mag / COUNT_STEP - 1e-9)   # float guard
    return max(0, peak_counts - ceiling)


def analyse(meta, rows):
    wheel = meta["wheel"]
    target_cmd = float(meta["target"])
    if target_cmd == 0.0:
        sys.exit("target is 0 in the metadata; nothing to analyse")

    # Metrics are computed on speed along the commanded direction, so one set
    # of formulas, written for a positive step, serves backward runs too.
    s = 1.0 if target_cmd > 0 else -1.0
    target_mag = abs(target_cmd)

    n = [int(r["n"]) for r in rows]
    tgt = [float(r["target"]) for r in rows]
    meas = [float(r[wheel]) for r in rows]
    phase = [r["phase"] for r in rows]

    # Time from the board's tick count (exact) rather than the host clock
    # (USB-jittered); t = 0 where the board first acknowledged the step.
    edge = next((i for i, v in enumerate(tgt) if v != 0.0), None)
    if edge is None:
        sys.exit("no acknowledged step found (target column is all zero)")
    t = [(k - n[edge]) * TICK_S for k in n]

    along = [s * v for v in meas]
    hold_idx = [i for i, p in enumerate(phase) if p == "step"]
    window = hold_idx[len(hold_idx) // 2:]            # skip the spin-up half
    w_along = [along[i] for i in window]
    mean_along = statistics.fmean(w_along)
    peak_along = max(along[i] for i in hold_idx)

    goal = 0.63 * mean_along
    t63 = next((t[i] for i in hold_idx if along[i] >= goal), None)

    th = [float(r["t_host"]) for r in rows]
    gaps = sum(1 for a, b in zip(th, th[1:]) if b - a > 3 * TICK_S)

    return {
        "t": t, "tgt": tgt, "meas": meas,
        "wheel": wheel, "kp": float(meta["kp"]), "target": target_cmd,
        "mean": s * mean_along,
        "error": target_mag - mean_along,
        "jitter": statistics.pstdev(w_along),
        "over": count_overshoot(peak_along, target_mag),
        "t63": t63, "gaps": gaps,
    }


def run_label(r):
    direction = " back" if r["target"] < 0 else ""
    return "%s Kp %g%s (err %.2f)" % (r["wheel"].upper(), r["kp"], direction,
                                      r["error"])


def default_png_name(paths, runs):
    """One CSV: named after it (already unique). Several: named after what is
    plotted, e.g. fl_kp0-10-20-40.png, so a sweep plot can never overwrite a
    single-run plot."""
    if len(paths) == 1:
        return os.path.splitext(paths[0])[0] + ".png"
    parts = []
    for wheel in dict.fromkeys(r["wheel"] for r in runs):
        mine = [r for r in runs if r["wheel"] == wheel]
        kps = "-".join("%g" % k for k in dict.fromkeys(r["kp"] for r in mine))
        fwd = any(r["target"] > 0 for r in mine)
        back = any(r["target"] < 0 for r in mine)
        tag = "_fwdback" if (fwd and back) else ("_back" if back else "")
        parts.append("%s_kp%s%s" % (wheel, kps, tag))
    return os.path.join(os.path.dirname(paths[0]), "_".join(parts) + ".png")


def main():
    ap = argparse.ArgumentParser(description="Plot step_logger.py CSVs.")
    ap.add_argument("files", nargs="+")
    ap.add_argument("--out", default=None,
                    help="PNG path (default: built from the runs plotted)")
    ap.add_argument("--no-show", action="store_true",
                    help="save the PNG without opening a window")
    args = ap.parse_args()

    runs = [analyse(*load(p)) for p in args.files]

    print("%-5s %6s %7s %7s %7s %7s %7s %6s %5s"
          % ("wheel", "Kp", "target", "mean", "error", "jitter", "over", "t63", "gaps"))
    for r in runs:
        print("%-5s %6.1f %7.2f %7.2f %7.2f %7.3f %7d %6s %5d"
              % (r["wheel"].upper(), r["kp"], r["target"], r["mean"], r["error"],
                 r["jitter"], r["over"],
                 "%.2f" % r["t63"] if r["t63"] is not None else "n/a",
                 r["gaps"]))
    print("over: counts above the highest reading the target allows; "
          "1 = quantization, 2+ = real overshoot")

    fig, ax = plt.subplots(figsize=(10, 5.5))

    # One target staircase and +/- 1 count band per distinct target, so mixed
    # forward/backward plots show both.
    first_of_target = {}
    for r in runs:
        first_of_target.setdefault(r["target"], r)
    for i, (target, ref) in enumerate(first_of_target.items()):
        ax.step(ref["t"], ref["tgt"], where="post", color="black",
                linewidth=1.2, label="target (board ack)" if i == 0 else None)
        ax.axhspan(target - COUNT_STEP, target + COUNT_STEP, color="grey",
                   alpha=0.15,
                   label="target +/- 1 count (0.335)" if i == 0 else None)

    for r in runs:
        # Steps, not lines: each reading is the average over the previous
        # 20 ms, so interpolating between readings would invent detail.
        ax.plot(r["t"], r["meas"], drawstyle="steps-post", linewidth=1.0,
                label=run_label(r))

    ax.axvline(0.0, color="black", linewidth=0.6, linestyle=":")
    ax.set_xlabel("time from step edge (s), board clock = sample x 20 ms")
    ax.set_ylabel("wheel speed (rad/s)")
    ax.set_title("Velocity step response, wheels up")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()

    out = args.out or default_png_name(args.files, runs)
    fig.savefig(out, dpi=120)
    print("Saved " + out)
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
