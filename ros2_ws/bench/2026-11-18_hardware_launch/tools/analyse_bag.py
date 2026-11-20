#!/usr/bin/env python3
"""Find launches, motion segments and watchdog trips in a rosbag2 bag.

Usage:
    python3 analyse_bag.py <bag_dir>

Needs a sourced ROS 2 Jazzy environment and the workspace overlay
(for mecanum_interfaces/msg/WheelSpeeds).

Times are bag receive times, printed in local time (UTC+3).

Definitions:
- launch: a stretch of /wheel_speeds messages with no gap over 1 s.
- motion segment: consecutive non-zero /cmd_vel messages with the same
  value and no gap over 0.5 s (the bridge's cmd_timeout), lasting over 1 s.
  Wheel means use /wheel_speeds from start + 0.8 s to end - 0.3 s, where
  end is the receive time of the last /cmd_vel message of the segment.
  The lag column is the time from the first /cmd_vel of the segment to the
  first /wheel_speeds_cmd after it.
- watchdog-trip signature: a /wheel_speeds_cmd value with any wheel above
  0.5 rad/s in magnitude (held for 0.5 s after each message, the bridge's
  cmd_timeout) while all four /wheel_speeds stay below 0.1 rad/s, for over
  1 s.
"""
import sys
from datetime import datetime, timedelta, timezone

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LOCAL_TZ = timezone(timedelta(hours=3))
WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
LAUNCH_GAP = 1.0
CMD_TIMEOUT = 0.5
MIN_SEGMENT = 1.0
SKIP_START = 0.8
SKIP_END = 0.3
TRIP_CMD = 0.5
TRIP_MEAS = 0.1
TRIP_MIN = 1.0


def clock(t):
    return datetime.fromtimestamp(t, LOCAL_TZ).strftime('%H:%M:%S.%f')[:-3]


def read_bag(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    cmd_vel, ws_cmd, ws, odom = [], [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1e-9
        msg = deserialize_message(data, types[topic])
        if topic == '/cmd_vel':
            cmd_vel.append((t, msg.linear.x, msg.linear.y, msg.angular.z))
        elif topic == '/wheel_speeds_cmd':
            ws_cmd.append((t,) + tuple(getattr(msg, w) for w in WHEELS))
        elif topic == '/wheel_speeds':
            ws.append((t,) + tuple(getattr(msg, w) for w in WHEELS))
        elif topic == '/odom':
            tw = msg.twist.twist
            odom.append((t, tw.linear.x, tw.linear.y, tw.angular.z))

    def arr(rows, n):
        a = np.array(rows, dtype=float).reshape(-1, n)
        return a[np.argsort(a[:, 0], kind='stable')]
    return arr(cmd_vel, 4), arr(ws_cmd, 5), arr(ws, 5), arr(odom, 4)


def stretches(mask, t, min_len):
    """Return (t_start, t_end) of runs where mask is True and last > min_len."""
    out = []
    i, n = 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if t[j] - t[i] > min_len:
                out.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    return out


def find_launches(ws):
    t = ws[:, 0]
    breaks = np.where(np.diff(t) > LAUNCH_GAP)[0]
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(t) - 1]
    return [(t[s], t[e], e - s + 1) for s, e in zip(starts, ends)]


def find_segments(cmd_vel):
    segs = []
    cur = None
    for t, vx, vy, wz in cmd_vel:
        val = (round(vx, 4), round(vy, 4), round(wz, 4))
        nonzero = any(v != 0.0 for v in val)
        if cur and (not nonzero or val != cur['val'] or t - cur['end'] > CMD_TIMEOUT):
            segs.append(cur)
            cur = None
        if nonzero:
            if cur is None:
                cur = {'start': t, 'end': t, 'val': val, 'n': 0}
            cur['end'] = t
            cur['n'] += 1
    if cur:
        segs.append(cur)
    return [s for s in segs if s['end'] - s['start'] > MIN_SEGMENT]


def window_mean(a, t0, t1):
    sel = (a[:, 0] >= t0) & (a[:, 0] <= t1)
    return a[sel, 1:].mean(axis=0), int(sel.sum())


def find_trips(ws_cmd, ws):
    t = ws[:, 0]
    # latest command at or before each /wheel_speeds sample
    idx = np.searchsorted(ws_cmd[:, 0], t, side='right') - 1
    valid = idx >= 0
    age = np.where(valid, t - ws_cmd[np.clip(idx, 0, None), 0], np.inf)
    cmd = ws_cmd[np.clip(idx, 0, None), 1:]
    cmd_active = valid & (age <= CMD_TIMEOUT) & (np.abs(cmd).max(axis=1) > TRIP_CMD)
    still = np.abs(ws[:, 1:]).max(axis=1) < TRIP_MEAS
    return stretches(cmd_active & still, t, TRIP_MIN)


def launch_of(launches, t):
    for k, (s, e, _) in enumerate(launches, 1):
        if s - 1e-6 <= t <= e + 1e-6:
            return k, t - s
    return None, None


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    cmd_vel, ws_cmd, ws, odom = read_bag(sys.argv[1])
    print(f'Messages: /cmd_vel {len(cmd_vel)}, /wheel_speeds_cmd {len(ws_cmd)}, '
          f'/wheel_speeds {len(ws)}, /odom {len(odom)}')

    launches = find_launches(ws)
    print('\n## Launches (stretches of /wheel_speeds, gap > 1 s)\n')
    print('| # | start | end | duration (s) | messages | rate (Hz) |')
    print('|---|---|---|---|---|---|')
    for k, (s, e, n) in enumerate(launches, 1):
        rate = (n - 1) / (e - s) if e > s else float('nan')
        print(f'| {k} | {clock(s)} | {clock(e)} | {e - s:.1f} | {n} | {rate:.2f} |')

    segs = find_segments(cmd_vel)
    print('\n## Motion segments (/cmd_vel non-zero for > 1 s)\n')
    print('| launch | start | end | dur (s) | vx | vy | wz | cmd lag (s) | '
          'FL | FR | RL | RR | n | odom vx | odom vy | odom wz |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for s in segs:
        t0, t1 = s['start'] + SKIP_START, s['end'] - SKIP_END
        k, _ = launch_of(launches, s['start'])
        if t1 > t0:
            m, n = window_mean(ws, t0, t1)
            o, _ = window_mean(odom, t0, t1)
            wheels = ' | '.join(f'{v:+.4f}' for v in m)
            od = ' | '.join(f'{v:+.5f}' for v in o)
        else:
            n, wheels, od = 0, ' | '.join(['n/a'] * 4), ' | '.join(['n/a'] * 3)
        vx, vy, wz = s['val']
        later = ws_cmd[ws_cmd[:, 0] >= s['start'], 0]
        lag = f'{later[0] - s["start"]:.2f}' if len(later) and later[0] <= s['end'] else 'none'
        print(f'| {k} | {clock(s["start"])} | {clock(s["end"])} | {s["end"] - s["start"]:.1f} '
              f'| {vx:g} | {vy:g} | {wz:g} | {lag} | {wheels} | {n} | {od} |')

    trips = find_trips(ws_cmd, ws)
    print('\n## Watchdog-trip signatures (cmd > 0.5 rad/s, all measured < 0.1 rad/s, > 1 s)\n')
    print('| launch | launch start | signature start | signature end '
          '| dur (s) | start after launch start (s) |')
    print('|---|---|---|---|---|---|')
    for s, e in trips:
        k, dt = launch_of(launches, s)
        ls = clock(launches[k - 1][0]) if k else 'n/a'
        print(f'| {k} | {ls} | {clock(s)} | {clock(e)} | {e - s:.1f} | {dt:.1f} |')


if __name__ == '__main__':
    main()
