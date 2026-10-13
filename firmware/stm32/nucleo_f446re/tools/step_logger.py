#!/usr/bin/env python3
"""Log one velocity step response on one wheel over the Nucleo serial link.

Sequence: E,0 -> K,<kp> (ack required) -> E,1 -> 0 for --settle s ->
--target for --hold s -> 0 for --post s -> E,0 (always, via finally).
Each V,... telemetry line becomes one CSV row; the first line of the CSV is a
'# key=value' metadata header read by plot_step.py.

    python3 step_logger.py --wheel fl --kp 20
    python3 step_logger.py --wheel rr --kp 0 --target -4

One wheel per run: wheels up, the four are independent, so each log describes
one plant at one gain. The other three are commanded 0 (held, at Kp > 0).
"""

import argparse
import csv
import glob
import math
import os
import sys
import time

import serial

BAUD = 115200

# Firmware watchdog is 300 ms; 100 ms survives two consecutive lost commands.
HEARTBEAT_S = 0.1

# Mirrors of firmware limits, so a bad argument fails here with a message
# instead of as a silent firmware rejection.
KP_MAX = 80.0
MAX_CMD_RAD_S = 42.0

WHEELS = ("fl", "fr", "rl", "rr")   # field order of C,... and V,...

REPLY_TIMEOUT_S = 1.0


def find_port():
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports:
        sys.exit("No ST-Link VCP found. Is the Nucleo plugged into USB?")
    return ports[0]


class LineReader:
    """Hands out only complete lines. readline() with a short timeout can
    return half a line ("V,  +3.6"), which would parse as a wrong number."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def poll(self):
        waiting = self.ser.in_waiting
        # When idle, block for at most ser.timeout on one byte so the main
        # loop does not spin at 100% CPU.
        self.buf += self.ser.read(waiting if waiting else 1)
        lines = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            lines.append(raw.decode("ascii", errors="replace").strip())
        return lines


def wait_for(reader, predicate, timeout_s):
    """Return the first line matching predicate, or None on timeout."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for line in reader.poll():
            if predicate(line):
                return line
    return None


def command_line(wheel_index, value):
    v = [0.0, 0.0, 0.0, 0.0]
    v[wheel_index] = value
    return "C,%.2f,%.2f,%.2f,%.2f\n" % tuple(v)


def parse_args():
    ap = argparse.ArgumentParser(description="Log a velocity step response.")
    ap.add_argument("--wheel", required=True, choices=WHEELS)
    ap.add_argument("--kp", required=True, type=float)
    ap.add_argument("--target", type=float, default=4.0,
                    help="step size in rad/s, negative for backward (default 4.0)")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds at 0 before the step (default 1.0)")
    ap.add_argument("--hold", type=float, default=3.0,
                    help="seconds at target (default 3.0)")
    ap.add_argument("--post", type=float, default=1.5,
                    help="seconds at 0 after the step (default 1.5)")
    ap.add_argument("--port", default=None,
                    help="serial port (default: auto-detect)")
    ap.add_argument("--out", default=None,
                    help="CSV path (default: logs/step_<wheel>_kp<kp>_<time>.csv)")
    args = ap.parse_args()

    # argparse's float() accepts "nan", and NaN passes no comparison, so a
    # range check alone would let it through. Check finiteness first.
    if not math.isfinite(args.kp) or not math.isfinite(args.target):
        ap.error("--kp and --target must be finite numbers")
    if not (0.0 <= args.kp <= KP_MAX):
        ap.error("--kp must be between 0 and %.0f" % KP_MAX)
    if not (abs(args.target) <= MAX_CMD_RAD_S):
        ap.error("--target must be within +/-%.0f rad/s" % MAX_CMD_RAD_S)
    for name in ("settle", "hold", "post"):
        if getattr(args, name) <= 0:
            ap.error("--%s must be positive" % name)
    return args


def default_out_path(args):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return os.path.join("logs", "step_%s_kp%g_%s.csv" % (args.wheel, args.kp, stamp))


def main():
    args = parse_args()
    wheel_index = WHEELS.index(args.wheel)
    out_path = args.out or default_out_path(args)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    port = args.port or find_port()
    # 10 ms read timeout bounds how late a heartbeat or phase change can be.
    ser = serial.Serial(port, BAUD, timeout=0.01)
    ser.reset_input_buffer()
    reader = LineReader(ser)
    print("Connected %s. Wheel %s, Kp %g, step 0 -> %g -> 0 rad/s."
          % (port, args.wheel.upper(), args.kp, args.target))

    rows = 0
    try:
        # Disarm first: K is accepted only while disarmed, and it makes the
        # starting state certain whatever ran before.
        ser.write(b"E,0\n")
        if wait_for(reader, lambda l: l == "S,DISARMED", REPLY_TIMEOUT_S) is None:
            raise RuntimeError("no S,DISARMED reply. Is piece 2 firmware flashed?")

        # Require the ack: a rejected gain would otherwise leave the old one
        # running under a CSV labelled with the new one.
        ser.write(("K,%g\n" % args.kp).encode("ascii"))
        ack = wait_for(reader, lambda l: l.startswith("A,K,"), REPLY_TIMEOUT_S)
        if ack is None:
            raise RuntimeError("no A,K ack. Firmware rejected the gain or is "
                               "an older build without the K command.")
        acked_kp = float(ack.split(",")[2])
        if abs(acked_kp - args.kp) > 0.005:
            raise RuntimeError("board acked Kp %g, asked for %g" % (acked_kp, args.kp))
        print("  gain confirmed: " + ack)

        ser.write(b"E,1\n")
        if wait_for(reader, lambda l: l == "S,ARMED", REPLY_TIMEOUT_S) is None:
            raise RuntimeError("no S,ARMED reply")
        print("  armed, logging...")

        with open(out_path, "w", newline="") as f:
            f.write("# wheel=%s kp=%g target=%g settle=%g hold=%g post=%g "
                    "port=%s date=%s\n"
                    % (args.wheel, acked_kp, args.target, args.settle,
                       args.hold, args.post, port,
                       time.strftime("%Y-%m-%d %H:%M:%S")))
            w = csv.writer(f)
            # n: board tick index, so n * 20 ms is the exact time axis.
            # t_host: Mac clock (USB-jittered), kept only to detect dropped lines.
            # target: the value the board ACKNOWLEDGED for this wheel, so the
            #         step edge in the data is when the board accepted it.
            w.writerow(["n", "t_host", "phase", "target", "fl", "fr", "rl", "rr"])

            t_start = time.monotonic()
            t_step = t_start + args.settle
            t_post = t_step + args.hold
            t_end = t_post + args.post

            acked_target = 0.0
            next_send = t_start
            last_phase = None
            n = 0

            while True:
                now = time.monotonic()
                if now >= t_end:
                    break
                if now < t_step:
                    phase, value = "pre", 0.0
                elif now < t_post:
                    phase, value = "step", args.target
                else:
                    phase, value = "post", 0.0

                # Send on a phase change too, not only on the heartbeat, so
                # the step edge is not delayed by up to 100 ms.
                if now >= next_send or phase != last_phase:
                    ser.write(command_line(wheel_index, value).encode("ascii"))
                    next_send = now + HEARTBEAT_S
                    last_phase = phase

                for line in reader.poll():
                    if line.startswith("V,"):
                        parts = line.split(",")
                        if len(parts) != 5:
                            continue
                        try:
                            vals = [float(p) for p in parts[1:]]
                        except ValueError:
                            continue
                        w.writerow([n, "%.4f" % (time.monotonic() - t_step),
                                    phase, "%.2f" % acked_target]
                                   + ["%.2f" % v for v in vals])
                        n += 1
                        rows = n
                    elif line.startswith("A,") and not line.startswith("A,K,"):
                        try:
                            acked_target = float(line.split(",")[1 + wheel_index])
                        except (ValueError, IndexError):
                            pass
                    elif line.startswith("S,"):
                        # Any status line mid-run (S,TIMEOUT) means the board
                        # stopped; later data would be meaningless.
                        raise RuntimeError("board reported %s mid-run" % line)
    except KeyboardInterrupt:
        print("\n  interrupted")
    except RuntimeError as e:
        print("  ERROR: %s" % e)
    finally:
        # Explicit disarm on every exit path; the firmware watchdog is only
        # the backstop.
        ser.write(b"E,0\n")
        time.sleep(0.05)              # let the bytes leave before closing
        ser.close()
        print("  disarmed.")

    if rows:
        expected = int((args.settle + args.hold + args.post) / 0.02)
        print("Wrote %d rows to %s (about %d expected for a full run)."
              % (rows, out_path, expected))


if __name__ == "__main__":
    main()
