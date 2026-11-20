#!/usr/bin/env python3
"""Regenerate every plot in ../plots from the files in ../data.

Usage (from the repo root, ROS 2 Jazzy and the workspace overlay sourced):
    python3 ros2_ws/bench/2026-11-18_hardware_launch/tools/make_plots.py

Style follows firmware/stm32/nucleo_f446re/tools/plot_step.py: 10 inch
wide figures, steps-post lines (each reading is an average over the
previous tick), black for commands, light grid, legend font 9, 120 dpi.
Wheels keep one colour in every plot: FL C0, FR C1, RL C2, RR C3.
The plots show recorded data only.

Time axes are seconds from the first message of each file or slice, except
the take plot, which uses the Foxglove export's own elapsed time. The local
clock time of t = 0 is in each title, marked (UTC+3).
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyse_bag  # noqa: E402
import analyse_csv  # noqa: E402

DATA = os.path.join(HERE, '..', 'data')
PLOTS = os.path.join(HERE, '..', 'plots')

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
SHORT = ('FL', 'FR', 'RL', 'RR')
COLOURS = ('C0', 'C1', 'C2', 'C3')
EXPECTED = 0.1 / 0.0483          # rad/s, 2.0704, same for all three motions
CMD_TIMEOUT = 0.5                # s, nucleo_bridge cmd_timeout
LINE_END = 0.1                   # s, a step line runs this far past its last message
DPI = 120

# Motion name for each wheel sign pattern (FL, FR, RL, RR).
PATTERNS = {
    (1, 1, 1, 1): 'forward',
    (-1, 1, 1, -1): 'strafe left',
    (-1, 1, -1, 1): 'rotate CCW',
}

# Watchdog slices and the start of their launch, as found by analyse_bag.py.
TRIPS = (
    ('watchdog_trip_1813', '18:08:57'),
    ('watchdog_trip_1815', '18:14:24'),
)


def clock(t):
    return analyse_bag.clock(t)[:8]


def sign_text(signs):
    return ' '.join('+' if s > 0 else '-' for s in signs)


def save(fig, name):
    path = os.path.join(PLOTS, name)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    print(f'Saved {os.path.relpath(path)} ({os.path.getsize(path) // 1024} KB)')


def broken(t, v, gap):
    """Insert NaN after any gap over `gap` s, so a step line stops when the
    publisher stops instead of holding the last value forever."""
    t = np.asarray(t, float)
    v = np.asarray(v, float)
    if len(t) == 0:
        return t, v
    tt, vv = [t[0]], [v[0]]
    for i in range(1, len(t)):
        if t[i] - t[i - 1] > gap:
            tt.append(t[i - 1] + LINE_END)
            vv.append(np.nan)
        tt.append(t[i])
        vv.append(v[i])
    tt.append(t[-1] + LINE_END)
    vv.append(v[-1])
    return np.array(tt), np.array(vv)


def load_slice(name):
    cmd_vel, ws_cmd, ws, odom = analyse_bag.read_bag(os.path.join(DATA, name))
    t0 = min(a[0, 0] for a in (cmd_vel, ws_cmd, ws, odom) if len(a))
    for a in (cmd_vel, ws_cmd, ws, odom):
        a[:, 0] -= t0
    return t0, cmd_vel, ws_cmd, ws, odom


def plot_take1():
    data = analyse_csv.load(os.path.join(DATA, 'take1_wheel_speeds_foxglove.csv'))
    fl = data['front_left']
    t0 = fl[0, 1] - fl[0, 0]   # Unix time of elapsed = 0
    segs = []
    for i, j in analyse_csv.segments(fl):
        us, ue = fl[i, 1], fl[j, 1]
        signs = []
        for w in WHEELS:
            a = data[w]
            sel = ((a[:, 1] >= us + analyse_csv.SKIP_START)
                   & (a[:, 1] <= ue - analyse_csv.SKIP_END))
            signs.append(int(np.sign(a[sel, 2].mean())))
        segs.append((fl[i, 0], fl[j, 0], tuple(signs)))

    fig, ax = plt.subplots(figsize=(10, 5.5))
    for s, e, signs in segs:
        ax.axvspan(s, e, color='grey', alpha=0.15)
        name = PATTERNS.get(signs, 'unknown pattern')
        ax.text((s + e) / 2, 2.75, f'{name}\n{sign_text(signs)}',
                ha='center', va='bottom', fontsize=9)
    for y in (EXPECTED, -EXPECTED):
        ax.axhline(y, color='black', linewidth=0.8, linestyle='--',
                   label='expected +/-%.4f rad/s' % EXPECTED if y > 0 else None)
    ax.axhline(0.0, color='black', linewidth=0.5)
    for w, name, c in zip(WHEELS, SHORT, COLOURS):
        d = data[w]
        ax.plot(d[:, 0], d[:, 2], drawstyle='steps-post', linewidth=1.0,
                color=c, label=name)
    ax.set_xlim(segs[0][0] - 4.0, segs[-1][1] + 4.0)
    ax.set_ylim(-3.0, 3.6)
    ax.set_ylabel('wheel speed (rad/s)')
    ax.set_xlabel('time (s), Foxglove elapsed time')
    ax.set_title(f'Filmed take, /wheel_speeds from the Foxglove export, t = 0 at {clock(t0)} (UTC+3)')
    ax.grid(True, alpha=0.3)
    # Between the forward and strafe blocks, below zero, nothing is drawn.
    ax.legend(loc='lower left', fontsize=9)
    fig.tight_layout()
    save(fig, 'take1_wheel_speeds.png')


def plot_run(name):
    t0, cmd_vel, ws_cmd, ws, odom = load_slice(name)

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for k, (ax, label, c) in enumerate(zip(axes, SHORT, COLOURS)):
        tc, vc = broken(ws_cmd[:, 0], ws_cmd[:, k + 1], CMD_TIMEOUT)
        ax.step(tc, vc, where='post', color='black', linewidth=1.2,
                label='/wheel_speeds_cmd (kinematics output)')
        ax.plot(ws[:, 0], ws[:, k + 1], drawstyle='steps-post', linewidth=1.0,
                color=c, label=f'{label} /wheel_speeds (measured)')
        ax.axhline(0.0, color='black', linewidth=0.5)
        ax.set_ylabel(f'{label} (rad/s)')
        ax.set_ylim(-2.8, 2.8)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='lower left', fontsize=8)
    axes[0].set_title(f'Commanded and measured wheel speeds, unfilmed run, t = 0 at {clock(t0)} (UTC+3)')
    axes[-1].set_xlabel('time from slice start (s)')
    fig.tight_layout()
    save(fig, f'{name}_cmd_vs_measured.png')

    fig, axes = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    rows = (('vx (m/s)', 'linear.x', 'center right'),
            ('vy (m/s)', 'linear.y', 'center left'),
            ('wz (rad/s)', 'angular.z', 'center left'))
    for k, (ax, (ylabel, field, loc)) in enumerate(zip(axes, rows)):
        tc, vc = broken(cmd_vel[:, 0], cmd_vel[:, k + 1], CMD_TIMEOUT)
        ax.step(tc, vc, where='post', color='black', linewidth=1.2, zorder=3,
                label=f'/cmd_vel {field}')
        ax.plot(odom[:, 0], odom[:, k + 1], drawstyle='steps-post', linewidth=1.0,
                color='C4', label=f'/odom twist {field}')
        ax.axhline(0.0, color='black', linewidth=0.5)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc=loc, fontsize=9)
    axes[0].set_title(f'/odom twist against /cmd_vel, unfilmed run, t = 0 at {clock(t0)} (UTC+3)')
    axes[-1].set_xlabel('time from slice start (s)')
    fig.tight_layout()
    save(fig, f'{name}_odom.png')


def plot_trips():
    fig, axes = plt.subplots(len(TRIPS), 1, figsize=(10, 7.5))
    for ax, (name, launch_start) in zip(axes, TRIPS):
        t0, cmd_vel, ws_cmd, ws, odom = load_slice(name)
        cmd = ws_cmd[:, 1:]
        # All four wheels get the same magnitude here (forward), so one line
        # stands for all of them. Stop if that ever stops being true.
        if not np.allclose(cmd, cmd[:, :1]):
            sys.exit(f'{name}: wheels are not all commanded the same speed')
        tc, vc = broken(ws_cmd[:, 0], cmd[:, 0], CMD_TIMEOUT)
        ax.step(tc, vc, where='post', color='black', linewidth=1.2,
                label='/wheel_speeds_cmd, all four wheels (%.4f rad/s)' % cmd[:, 0].mean())
        for k, (label, c) in enumerate(zip(SHORT, COLOURS)):
            ax.plot(ws[:, 0], ws[:, k + 1], drawstyle='steps-post', linewidth=1.0,
                    color=c, label=f'{label} /wheel_speeds (measured)')
        ax.axvline(cmd_vel[0, 0], color='black', linewidth=0.6, linestyle=':',
                   label='/cmd_vel starts')
        ax.axhline(0.0, color='black', linewidth=0.5)
        ax.set_ylim(-0.5, 2.8)
        ax.set_ylabel('wheel speed (rad/s)')
        ax.set_xlabel('time from slice start (s)')
        ax.set_title(f'Launch at {launch_start}: commands ignored after a watchdog trip, '
                     f't = 0 at {clock(t0)} (UTC+3)', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='center right', fontsize=8)
    fig.tight_layout()
    save(fig, 'watchdog_trips.png')


def main():
    os.makedirs(PLOTS, exist_ok=True)
    plot_take1()
    plot_run('run_1751')
    plot_trips()


if __name__ == '__main__':
    main()
