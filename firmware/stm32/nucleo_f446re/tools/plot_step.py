#!/usr/bin/env python3
"""Plot step_logger.py CSVs and print a summary table.

    python3 plot_step.py logs/step_fl_kp*.csv                # Kp sweep, P only
    python3 plot_step.py logs/step_fl_kp10_ki*.csv           # Ki sweep
    python3 plot_step.py logs/step_fl_kp10_ki100_lead15_*.csv   # windup test
    python3 plot_step.py logs/dir_*.csv                      # forward and backward

Runs are overlaid with t = 0 at each run's own acknowledged step edge. When
the CSVs carry integral columns, a second panel shows each run's integral
against the +/-60 clamp.

Table columns:
    Kp, Ki  the gains the board acknowledged (Ki 0 for 2026-10-12 CSVs)
    from    the speed commanded before the step: 0, or the lead target of
            a windup run
    mean    average speed over the last half of the hold (signed)
    error   how much slower than commanded the wheel ran (+ = too slow,
            in either direction)
    jitter  standard deviation over the same window
    over    overshoot in whole encoder counts: 1 = quantization, 2+ = real
    t63     time to 63% of the change (first-order time constant)
    t_in    time until the 0.2 s average enters +/-0.1 rad/s of the target
            and STAYS there for the rest of the hold (n/a = never settled)
    I_ss    average integral over the same window as mean (duty counts),
            n/a for CSVs without integral columns
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
I_LIMIT = 60.0                # the firmware's integral clamp
T_IN_BAND = 0.1               # rad/s either side of the target
T_IN_AVG = 10                 # samples in the t_in average (0.2 s)
START_SAMPLES = 5             # samples averaged for the start speed


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


def count_overshoot(peak_along, goal_along):
    """Overshoot in encoder counts past the reading nearest the goal on the
    far side. Readings are whole counts per tick, so a wheel tracking 4.0
    rad/s (11.94 counts) reads 11 or 12, and a brief 13 needs only 0.5%
    excess speed, which load ripple supplies. One count is therefore not
    evidence of overshoot; two or more is.

    Both arguments are measured along the direction of change, so for a
    step down they are negated speeds; ceil() of a negated goal is then the
    reading just below the goal, the ceiling seen from that side."""
    peak_counts = round(peak_along / COUNT_STEP)          # readings are exact
    ceiling = math.ceil(goal_along / COUNT_STEP - 1e-9)   # float guard
    return max(0, peak_counts - ceiling)


def column(rows, name):
    """The column as floats, or None if absent or entirely blank. Blank
    cells (a V row with no D line) become None. CSVs from before integral
    telemetry have no such column."""
    if name not in rows[0]:
        return None
    vals = []
    for r in rows:
        cell = r[name]
        vals.append(float(cell) if cell not in ("", None) else None)
    return vals if any(v is not None for v in vals) else None


def time_in_band(t, meas, idx, target, edge):
    """First time after which the trailing T_IN_AVG-sample average of `meas`
    stays within T_IN_BAND of `target` for every remaining sample in `idx`
    (the step phase). None if it is not inside at the end.

    An average because no single reading can sit within 0.1 of 4.0: the
    nearest readings are 3.685 and 4.020 (11 and 12 counts). The trailing
    average makes t_in up to 0.2 s pessimistic, the safe direction for a
    pass mark."""
    inside_from = None
    for i in idx:
        if i - T_IN_AVG + 1 < edge:
            continue                       # window would reach before the step
        avg = statistics.fmean(meas[i - T_IN_AVG + 1:i + 1])
        if abs(avg - target) <= T_IN_BAND:
            if inside_from is None:
                inside_from = t[i]
        else:
            inside_from = None             # left the band: start again
    return inside_from


def analyse(meta, rows):
    wheel = meta["wheel"]
    target_cmd = float(meta["target"])
    if target_cmd == 0.0:
        sys.exit("target is 0 in the metadata; nothing to analyse")
    ki = float(meta.get("ki", 0.0))
    lead_cmd = float(meta.get("lead_target", 0.0))
    if lead_cmd == target_cmd:
        sys.exit("lead target equals target; there is no step to analyse")

    # Mean and error are computed along the commanded direction, so one set
    # of formulas, written for a positive step, serves backward runs too.
    s = 1.0 if target_cmd > 0 else -1.0
    target_mag = abs(target_cmd)

    n = [int(r["n"]) for r in rows]
    tgt = [float(r["target"]) for r in rows]
    meas = [float(r[wheel]) for r in rows]
    phase = [r["phase"] for r in rows]
    integ = column(rows, "i_" + wheel)

    # Time from the board's tick count (exact) rather than the host clock
    # (USB-jittered); t = 0 where the board first acknowledged the step.
    # "The step" is the edge into --target, not into a lead phase.
    edge = next((i for i, v in enumerate(tgt)
                 if phase[i] == "step" and abs(v - target_cmd) < 0.006), None)
    if edge is None:
        sys.exit("no acknowledged step found (target never reached %g)" % target_cmd)
    t = [(k - n[edge]) * TICK_S for k in n]

    along = [s * v for v in meas]
    # Every step-phase row, including the one or two logged before the
    # board's ack arrived, so the 2026-10-12 numbers reproduce exactly.
    hold_idx = [i for i, p in enumerate(phase) if p == "step"]
    window = hold_idx[len(hold_idx) // 2:]            # skip the spin-up half
    w_along = [along[i] for i in window]
    mean_along = statistics.fmean(w_along)

    # Where the wheel actually was just before the edge (0 for an ordinary
    # step, about 12.4 for a windup run that could not reach 15), and which
    # way it was asked to go from there.
    before = meas[max(0, edge - START_SAMPLES):edge]
    start = statistics.fmean(before) if before else 0.0
    d = 1.0 if target_cmd > lead_cmd else -1.0
    final = s * mean_along                            # signed final speed
    peak_d = max(d * meas[i] for i in hold_idx)
    goal_63 = d * (start + 0.63 * (final - start))
    t63 = next((t[i] for i in hold_idx if d * meas[i] >= goal_63), None)

    th = [float(r["t_host"]) for r in rows]
    gaps = sum(1 for a, b in zip(th, th[1:]) if b - a > 3 * TICK_S)

    i_ss = None
    if integ is not None:
        w_i = [integ[i] for i in window if integ[i] is not None]
        i_ss = statistics.fmean(w_i) if w_i else None

    return {
        "t": t, "tgt": tgt, "meas": meas, "integ": integ,
        "wheel": wheel, "kp": float(meta["kp"]), "ki": ki,
        "target": target_cmd, "lead": lead_cmd,
        "mean": final,
        "error": target_mag - mean_along,
        "jitter": statistics.pstdev(w_along),
        "over": count_overshoot(peak_d, d * target_cmd),
        "t63": t63,
        "t_in": time_in_band(t, meas, hold_idx, target_cmd, edge),
        "i_ss": i_ss,
        "gaps": gaps,
    }


def run_label(r, show_ki):
    """Ki is shown whenever any run in the plot uses it, so a Ki 0 run beside
    PI runs reads 'Ki 0' rather than looking like a different test."""
    ki = (" Ki %g" % r["ki"]) if show_ki else ""
    lead = (" from %g" % r["lead"]) if r["lead"] else ""
    direction = " back" if r["target"] < 0 else ""
    return "%s Kp %g%s%s%s (err %.2f)" % (r["wheel"].upper(), r["kp"], ki,
                                          lead, direction,
                                          no_neg_zero(r["error"]))


def default_png_name(paths, runs):
    """One CSV: named after it (already unique). Several: named after what is
    plotted, e.g. fl_kp0-10-20-40.png or fl_kp10_ki0-50-100-200.png, so a
    sweep plot can never overwrite a single-run plot."""
    if len(paths) == 1:
        return os.path.splitext(paths[0])[0] + ".png"

    def joined(values):
        return "-".join("%g" % v for v in dict.fromkeys(values))

    parts = []
    for wheel in dict.fromkeys(r["wheel"] for r in runs):
        mine = [r for r in runs if r["wheel"] == wheel]
        name = "%s_kp%s" % (wheel, joined(r["kp"] for r in mine))
        if any(r["ki"] for r in mine):
            name += "_ki" + joined(r["ki"] for r in mine)
        if any(r["lead"] for r in mine):
            name += "_lead" + joined(r["lead"] for r in mine if r["lead"])
        fwd = any(r["target"] > 0 for r in mine)
        back = any(r["target"] < 0 for r in mine)
        name += "_fwdback" if (fwd and back) else ("_back" if back else "")
        parts.append(name)
    return os.path.join(os.path.dirname(paths[0]), "_".join(parts) + ".png")


def no_neg_zero(x):
    """-0.004 prints as '-0.00', which reads like a tiny slow-down that is
    not there. Round first, then add 0.0, which turns -0.0 into 0.0."""
    return round(x, 2) + 0.0


def fmt_opt(value, spec):
    return spec % value if value is not None else "n/a"


def main():
    ap = argparse.ArgumentParser(description="Plot step_logger.py CSVs.")
    ap.add_argument("files", nargs="+")
    ap.add_argument("--out", default=None,
                    help="PNG path (default: built from the runs plotted)")
    ap.add_argument("--no-show", action="store_true",
                    help="save the PNG without opening a window")
    args = ap.parse_args()

    runs = [analyse(*load(p)) for p in args.files]
    # Sorted by the run's parameters, not its file name: the shell sorts
    # ki100 before ki50, and a sweep should read 0, 50, 100, 200.
    runs.sort(key=lambda r: (r["wheel"], r["kp"], r["ki"], r["lead"], r["target"]))

    print("%-5s %5s %5s %5s %7s %7s %7s %7s %5s %6s %6s %7s %5s"
          % ("wheel", "Kp", "Ki", "from", "target", "mean", "error", "jitter",
             "over", "t63", "t_in", "I_ss", "gaps"))
    for r in runs:
        print("%-5s %5.1f %5.0f %5g %7.2f %7.2f %7.2f %7.3f %5d %6s %6s %7s %5d"
              % (r["wheel"].upper(), r["kp"], r["ki"], r["lead"], r["target"],
                 no_neg_zero(r["mean"]), no_neg_zero(r["error"]),
                 r["jitter"], r["over"],
                 fmt_opt(r["t63"], "%.2f"), fmt_opt(r["t_in"], "%.2f"),
                 fmt_opt(r["i_ss"], "%+.1f"), r["gaps"]))
    print("over: counts past the target's nearest reading; "
          "1 = quantization, 2+ = real overshoot")
    print("t_in: 0.2 s average inside +/-%.1f rad/s for the rest of the hold"
          % T_IN_BAND)

    # The integral panel only when there is data for it, so P-only CSVs
    # still make a single-panel figure.
    has_integ = any(r["integ"] is not None for r in runs)
    if has_integ:
        fig, (ax, ax_i) = plt.subplots(
            2, 1, figsize=(10, 7.5), sharex=True,
            gridspec_kw={"height_ratios": [2, 1]})
    else:
        fig, ax = plt.subplots(figsize=(10, 5.5))
        ax_i = None
    show_ki = any(r["ki"] for r in runs)

    # One target staircase per distinct (lead, target) pair, so a windup run
    # draws its own 15 -> 4 staircase, and one +/- 1 count band per distinct
    # target, so mixed forward/backward plots show both.
    first_of_shape = {}
    for r in runs:
        first_of_shape.setdefault((r["lead"], r["target"]), r)
    for i, ref in enumerate(first_of_shape.values()):
        ax.step(ref["t"], ref["tgt"], where="post", color="black",
                linewidth=1.2, label="target (board ack)" if i == 0 else None)
    for i, target in enumerate(dict.fromkeys(r["target"] for r in runs)):
        ax.axhspan(target - COUNT_STEP, target + COUNT_STEP, color="grey",
                   alpha=0.15,
                   label="target +/- 1 count (0.335)" if i == 0 else None)

    for r in runs:
        # Steps, not lines: each reading is the average over the previous
        # 20 ms, so interpolating between readings would invent detail.
        line, = ax.plot(r["t"], r["meas"], drawstyle="steps-post",
                        linewidth=1.0, label=run_label(r, show_ki))
        if ax_i is not None and r["integ"] is not None:
            # Same colour in both panels. Blank cells (None) leave gaps.
            vals = [v if v is not None else float("nan") for v in r["integ"]]
            ax_i.plot(r["t"], vals, drawstyle="steps-post", linewidth=1.0,
                      color=line.get_color())

    ax.axvline(0.0, color="black", linewidth=0.6, linestyle=":")
    ax.set_ylabel("wheel speed (rad/s)")
    ax.set_title("Velocity step response, wheels up")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    xlabel = "time from step edge (s), board clock = sample x 20 ms"
    if ax_i is not None:
        for y in (I_LIMIT, -I_LIMIT):
            ax_i.axhline(y, color="red", linewidth=0.8, linestyle="--",
                         label="clamp +/-%g" % I_LIMIT if y > 0 else None)
        ax_i.axhline(0.0, color="black", linewidth=0.5)
        ax_i.axvline(0.0, color="black", linewidth=0.6, linestyle=":")
        ax_i.set_ylabel("integral (duty counts)")
        ax_i.set_xlabel(xlabel)
        ax_i.grid(True, alpha=0.3)
        ax_i.legend(loc="upper right", fontsize=9)
    else:
        ax.set_xlabel(xlabel)
    fig.tight_layout()

    out = args.out or default_png_name(args.files, runs)
    fig.savefig(out, dpi=120)
    print("Saved " + out)
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
