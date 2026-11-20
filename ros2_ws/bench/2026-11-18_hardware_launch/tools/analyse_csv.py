#!/usr/bin/env python3
"""Find the motion segments in a Foxglove Plot-panel CSV export.

Usage:
    python3 analyse_csv.py <csv_file>

The CSV is in long format: columns `elapsed time`, `timestamp` (Unix
seconds), `topic` (for example `/wheel_speeds.front_left`) and `value`.

A motion segment is a stretch where |front_left| is above 1 rad/s for more
than 1 s. Per-wheel means use the steady part, from start + 0.8 s to
end - 0.3 s. Clock times are printed in local time (UTC+3).

The timestamps are receive times in Foxglove, in whole milliseconds.
Messages often arrive in bursts, so several rows can share one timestamp.
Each row is a separate message, so every row is kept.
"""
import csv
import sys
from datetime import datetime, timedelta, timezone

import numpy as np

LOCAL_TZ = timezone(timedelta(hours=3))
WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
THRESH = 1.0
MIN_SEGMENT = 1.0
SKIP_START = 0.8
SKIP_END = 0.3


def clock(t):
    return datetime.fromtimestamp(t, LOCAL_TZ).strftime('%H:%M:%S.%f')[:-3]


def load(path):
    rows = {w: [] for w in WHEELS}
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            w = r['topic'].rsplit('.', 1)[-1]
            if w in rows:
                rows[w].append((float(r['elapsed time']), float(r['timestamp']),
                                float(r['value'])))
    return {w: np.array(v) for w, v in rows.items()}


def segments(a):
    """a: rows of (elapsed, unix, value). Returns list of (i_start, i_end)."""
    mask = np.abs(a[:, 2]) > THRESH
    out, i, n = [], 0, len(a)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if a[j, 0] - a[i, 0] > MIN_SEGMENT:
                out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    data = load(sys.argv[1])
    fl = data['front_left']

    print('## Sample rate per wheel\n')
    print('| wheel | rows | span (s) | rate (Hz) | rows sharing a timestamp | '
          'largest gap (ms) |')
    print('|---|---|---|---|---|---|')
    for w in WHEELS:
        a = data[w]
        t = np.sort(a[:, 1])
        span = t[-1] - t[0]
        dt = np.diff(t)
        shared = len(t) - len(np.unique(t))
        print(f'| {w} | {len(a)} | {span:.3f} | {(len(a) - 1) / span:.2f} | {shared} | '
              f'{dt.max() * 1e3:.0f} |')
    print(f'\nFile start: elapsed {fl[0, 0]:.3f} s at {clock(fl[0, 1])}; '
          f'end: elapsed {fl[-1, 0]:.3f} s at {clock(fl[-1, 1])}')

    print('\n## Motion segments (|front_left| > 1 rad/s for > 1 s)\n')
    print('| # | elapsed start (s) | elapsed end (s) | dur (s) | start | end | '
          'FL | FR | RL | RR | n (FL) |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for k, (i, j) in enumerate(segments(fl), 1):
        es, ee = fl[i, 0], fl[j, 0]
        us, ue = fl[i, 1], fl[j, 1]
        means = []
        for w in WHEELS:
            a = data[w]
            sel = (a[:, 1] >= us + SKIP_START) & (a[:, 1] <= ue - SKIP_END)
            v = a[sel, 2]
            means.append(v.mean())
            if w == 'front_left':
                n_fl = len(v)
        cells = ' | '.join(f'{m:+.4f}' for m in means)
        print(f'| {k} | {es:.3f} | {ee:.3f} | {ee - es:.2f} | {clock(us)} | {clock(ue)} | '
              f'{cells} | {n_fl} |')


if __name__ == '__main__':
    main()
