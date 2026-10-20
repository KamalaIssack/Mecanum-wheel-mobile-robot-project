#!/usr/bin/env python3
"""Log one velocity step response on one wheel over the Nucleo serial link.

Sequence:

    1. E,0, wait for S,DISARMED              known starting state (retried once)
    2. K,<kp> then I,<ki>, each ack required proves the board runs these gains
    3. E,1, wait for S,ARMED
    4. target 0 for --settle s               baseline before the step
       (optional) --lead-target for --lead s windup test
    5. --target for --hold s                 the step response
    6. target 0 for --post s                 braking and holding at 0
    7. E,0, always, even on error or Ctrl-C

Each V,... telemetry line becomes one CSV row; the D,... line that follows it
adds that tick's four integrals to the row. The first line of the CSV is a
'# key=value' metadata header read by plot_step.py. A run that does not reach
its natural end is moved to a failed/ folder beside the CSV, and the exit
status is 0 only for a complete run.

    python3 step_logger.py --wheel fl --kp 0
    python3 step_logger.py --wheel fl --kp 10 --ki 100      # PI
    python3 step_logger.py --wheel fl --kp 10 --ki 100 --lead-target 15 --lead 2
    python3 step_logger.py --wheel fl --kp 20 --target 4.0
    python3 step_logger.py --wheel fr --kp 40 --hold 4
    python3 step_logger.py --wheel rr --kp 0 --target -4      # backward
    python3 step_logger.py --wheel fl --kp 0 --allow-no-motion  # USB-only dry run

One wheel per run: wheels up, the four are independent, so each log describes
one plant at one gain set. The other three are commanded 0 (held, at Kp > 0).
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
KI_MAX = 400.0
NO_INTEGRALS = ["", "", "", ""]   # integral columns for a V row with no D line
MAX_CMD_RAD_S = 42.0

WHEELS = ("fl", "fr", "rl", "rr")   # field order of C,... and V,...

REPLY_TIMEOUT_S = 1.0

# Pause between opening the port and the first write. Bytes written in the
# first instant after opening appear to be lost between macOS, the ST-Link
# USB bridge and USART2 (suspected cause of the occasional missing
# S,DISARMED on 2026-10-12). The E,0 retry below covers the case where this
# hypothesis is wrong.
PORT_SETTLE_S = 0.2

# E,0 is idempotent, so a duplicate that does arrive is harmless. E,1 is
# never retried: arming should always be a single deliberate act. If two
# E,0 attempts both go unanswered the problem is real (wrong firmware, wrong
# port, board fault) and a third would only hide it.
DISARM_ATTEMPTS = 2

# How long the stepped wheel may read exactly 0 after it is commanded to
# move before the run is declared dead. On 2026-10-12 every wheel, cold FR
# included, read nonzero within 3 ticks (60 ms); 0.5 s is about 8 times that.
NO_MOTION_S = 0.5


class NoMotionError(RuntimeError):
    """The stepped wheel never moved (motor power missing). A RuntimeError,
    so it is reported like any other run failure."""


def find_port():
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports:
        sys.exit("No ST-Link VCP found. Is the Nucleo plugged into USB?")
    return ports[0]


def open_port(port):
    """Open the port, wait PORT_SETTLE_S, then discard anything buffered.
    Flushing after the pause also drops any half-line the board was sending
    while the port opened, so LineReader starts on a line boundary."""
    # timeout=0.01: poll() blocks at most 10 ms when idle, so heartbeats and
    # phase changes are never more than about 10 ms late.
    ser = serial.Serial(port, BAUD, timeout=0.01)
    time.sleep(PORT_SETTLE_S)
    ser.reset_input_buffer()
    return ser


class LineReader:
    """Hands out only complete lines. readline() with a short timeout can
    return half a line ("V,  +3.6"), which would parse as a wrong number."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def poll(self):
        """Return the complete lines received so far (maybe none)."""
        waiting = self.ser.in_waiting
        # When idle, block for at most ser.timeout on one byte so the main
        # loop does not spin at 100% CPU.
        chunk = self.ser.read(waiting if waiting else 1)
        self.buf += chunk
        lines = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            lines.append(raw.decode("ascii", errors="replace").strip())
        return lines


def wait_for(reader, predicate, timeout_s):
    """Return the first line satisfying predicate, or None on timeout.
    Telemetry streaming meanwhile is skipped."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for line in reader.poll():
            if predicate(line):
                return line
    return None


def disarm_handshake(ser, reader):
    """Send E,0 up to DISARM_ATTEMPTS times. Return the attempt number that
    got S,DISARMED, or None. main() reports any attempt other than 1, which
    keeps collecting evidence on the lost-first-bytes hypothesis."""
    for attempt in range(1, DISARM_ATTEMPTS + 1):
        ser.write(b"E,0\n")
        if wait_for(reader, lambda l: l == "S,DISARMED", REPLY_TIMEOUT_S):
            return attempt
    return None


def set_gain(ser, reader, tag, value):
    """Send "<tag>,<value>" and require the board's "A,<tag>,<value>" ack.
    Returns the acknowledged value.

    The ack is required because a rejected or lost gain leaves the previous
    gain running under a CSV labelled with the new one. Not retried: the
    board has just answered E,0, so a missing ack most likely means a
    rejection (out of range, or firmware without the command), and a retry
    would only hide it."""
    ser.write(("%s,%g\n" % (tag, value)).encode("ascii"))
    prefix = "A,%s," % tag
    ack = wait_for(reader, lambda l: l.startswith(prefix), REPLY_TIMEOUT_S)
    if ack is None:
        raise RuntimeError("no %s ack. The firmware rejected the gain or is "
                           "an older build without the %s command."
                           % (prefix, tag))
    acked = float(ack.split(",")[2])
    if abs(acked - value) > 0.005:
        raise RuntimeError("board acked %s %g, asked for %g" % (tag, acked, value))
    return acked


def command_line(wheel_index, value):
    """C,... with `value` on the chosen wheel and 0 on the other three."""
    v = [0.0, 0.0, 0.0, 0.0]
    v[wheel_index] = value
    return "C,%.2f,%.2f,%.2f,%.2f\n" % tuple(v)


def parse_args():
    ap = argparse.ArgumentParser(description="Log a velocity step response.")
    ap.add_argument("--wheel", required=True, choices=WHEELS)
    ap.add_argument("--kp", required=True, type=float)
    # Default 0 keeps P-only command lines meaning what they meant.
    ap.add_argument("--ki", type=float, default=0.0,
                    help="integral gain, 0 to %.0f (default 0: P only)" % KI_MAX)
    ap.add_argument("--target", type=float, default=4.0,
                    help="step size in rad/s, negative for backward "
                         "(default 4.0, the piece 1 value)")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds at 0 before the step (default 1.0)")
    ap.add_argument("--hold", type=float, default=3.0,
                    help="seconds at target (default 3.0)")
    # Both default 0: no lead phase.
    ap.add_argument("--lead-target", type=float, default=0.0,
                    help="rad/s to hold before the step (windup test; "
                         "default 0, no lead phase)")
    ap.add_argument("--lead", type=float, default=0.0,
                    help="seconds at --lead-target (default 0)")
    ap.add_argument("--post", type=float, default=1.5,
                    help="seconds at 0 after the step (default 1.5)")
    ap.add_argument("--port", default=None,
                    help="serial port (default: auto-detect)")
    ap.add_argument("--out", default=None,
                    help="CSV path (default: logs/step_<wheel>_kp<kp>_<time>.csv)")
    # For USB-only handshake tests, which run without motor power on purpose.
    ap.add_argument("--allow-no-motion", action="store_true",
                    help="skip the no-motion check (USB-only dry runs)")
    args = ap.parse_args()

    # float() accepts "nan" and "inf", and NaN passes no comparison, so the
    # range checks below would not catch it. Test finiteness first.
    if not all(math.isfinite(x) for x in
               (args.kp, args.ki, args.target, args.lead_target, args.lead)):
        ap.error("--kp, --ki, --target, --lead-target and --lead must be "
                 "finite numbers")
    if not (0.0 <= args.kp <= KP_MAX):
        ap.error("--kp must be between 0 and %.0f" % KP_MAX)
    if not (0.0 <= args.ki <= KI_MAX):
        ap.error("--ki must be between 0 and %.0f" % KI_MAX)
    if not (abs(args.target) <= MAX_CMD_RAD_S):
        ap.error("--target must be within +/-%.0f rad/s" % MAX_CMD_RAD_S)
    for name in ("settle", "hold", "post"):
        if getattr(args, name) <= 0:
            ap.error("--%s must be positive" % name)
    # A lead time without a lead target is just a longer settle, and a lead
    # target without a time is never sent: refuse half a lead.
    if (args.lead > 0) != (args.lead_target != 0.0):
        ap.error("--lead-target and --lead go together (both set, or neither)")
    if args.lead < 0:
        ap.error("--lead must not be negative")
    if not (abs(args.lead_target) <= MAX_CMD_RAD_S):
        ap.error("--lead-target must be within +/-%.0f rad/s" % MAX_CMD_RAD_S)
    return args


def default_out_path(args):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    # Ki in the name only when it is used, so P-only names stay as before.
    gains = "kp%g" % args.kp + ("_ki%g" % args.ki if args.ki else "")
    if args.lead > 0:                      # e.g. ..._lead15_...
        gains += "_lead%g" % args.lead_target
    name = "step_%s_%s_%s.csv" % (args.wheel, gains, stamp)
    return os.path.join("logs", name)


def move_to_failed(out_path):
    """Move an incomplete CSV into a failed/ folder beside it, keeping its
    name, and return the new path. Plot globs such as logs/step_fl_kp*.csv
    then cannot pick up a partial run."""
    failed_dir = os.path.join(os.path.dirname(out_path) or ".", "failed")
    os.makedirs(failed_dir, exist_ok=True)
    new_path = os.path.join(failed_dir, os.path.basename(out_path))
    os.replace(out_path, new_path)
    return new_path


def main():
    args = parse_args()
    wheel_index = WHEELS.index(args.wheel)
    out_path = args.out or default_out_path(args)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    port = args.port or find_port()
    ser = open_port(port)
    reader = LineReader(ser)
    lead = ("%g -> " % args.lead_target) if args.lead > 0 else ""
    print("Connected %s. Wheel %s, Kp %g, Ki %g, step 0 -> %s%g -> 0 rad/s."
          % (port, args.wheel.upper(), args.kp, args.ki, lead, args.target))

    # Whatever happens below (exception, unexpected status, Ctrl-C), the
    # finally block sends E,0. The firmware watchdog is only the backstop.
    rows = 0
    csv_opened = False   # only a file that exists can be moved to failed/
    complete = False     # True only if the run reaches t_end
    try:
        # Disarm first: gains are accepted only while disarmed, and this
        # makes the starting state certain.
        attempt = disarm_handshake(ser, reader)
        if attempt is None:
            raise RuntimeError("no S,DISARMED reply after %d attempts. Is "
                               "piece 2 firmware flashed, and is this the "
                               "right port?" % DISARM_ATTEMPTS)
        if attempt > 1:
            print("  note: board answered E,0 on attempt %d (first E,0 lost)"
                  % attempt)

        # I is sent every run, even at Ki 0: the board keeps ki across
        # disarm, so skipping it could silently inherit the previous Ki.
        acked_kp = set_gain(ser, reader, "K", args.kp)
        acked_ki = set_gain(ser, reader, "I", args.ki)
        print("  gains confirmed: Kp %.2f, Ki %.2f" % (acked_kp, acked_ki))

        ser.write(b"E,1\n")
        if wait_for(reader, lambda l: l == "S,ARMED", REPLY_TIMEOUT_S) is None:
            raise RuntimeError("no S,ARMED reply")
        print("  armed, logging...")

        with open(out_path, "w", newline="") as f:
            csv_opened = True
            # Metadata line for plot_step.py. A missing ki or lead_target is
            # read as 0, so older CSVs still load.
            f.write("# wheel=%s kp=%g ki=%g target=%g settle=%g "
                    "lead_target=%g lead=%g hold=%g post=%g port=%s date=%s\n"
                    % (args.wheel, acked_kp, acked_ki, args.target, args.settle,
                       args.lead_target, args.lead, args.hold, args.post, port,
                       time.strftime("%Y-%m-%d %H:%M:%S")))
            w = csv.writer(f)
            # n          sample index, one per TIM6 tick: n * 0.02 s is the
            #            board clock. Use this as the time axis.
            # t_host     seconds since the step, Mac clock. USB-jittered;
            #            kept only to spot dropped lines.
            # phase      pre / lead / step / post (lead only in windup runs)
            # target     the value the board acknowledged for this wheel, not
            #            what the Mac sent: the true step edge
            # fl..rr     measured speeds, rad/s, from V,...
            # i_fl..i_rr each wheel's integral term in duty counts, from the
            #            D,... line of the same tick. Empty if the firmware
            #            sends no D lines.
            w.writerow(["n", "t_host", "phase", "target", "fl", "fr", "rl", "rr",
                        "i_fl", "i_fr", "i_rl", "i_rr"])

            t_start = time.monotonic()
            t_lead = t_start + args.settle      # == t_step if no lead
            t_step = t_lead + args.lead
            t_post = t_step + args.hold
            t_end = t_post + args.post

            check_motion = args.target != 0.0 and not args.allow_no_motion
            moved = False              # any nonzero reading while moving?
            # The no-motion clock starts when the wheel is first asked to
            # move: the lead phase if there is one, otherwise the step.
            t_motion_deadline = t_lead + NO_MOTION_S

            acked_target = 0.0
            next_send = t_start       # send immediately, then every 100 ms
            last_phase = None
            n = 0

            # A V row is held here until its D line arrives.
            pending = None
            try:
                while True:
                    now = time.monotonic()
                    if now >= t_end:
                        complete = True
                        break
                    if now < t_lead:
                        phase, value = "pre", 0.0
                    elif now < t_step:              # empty if no lead
                        phase, value = "lead", args.lead_target
                    elif now < t_post:
                        phase, value = "step", args.target
                    else:
                        phase, value = "post", 0.0

                    # Only while the wheel is commanded to move: before that
                    # it should be still, and after it a stop is the goal.
                    if (check_motion and phase in ("lead", "step") and not moved
                            and now >= t_motion_deadline):
                        raise NoMotionError(
                            "no motion: %s read 0 for %.1f s after it was "
                            "commanded to %g. "
                            "Is the XT60 connected? (For a deliberate USB-only "
                            "dry run, add --allow-no-motion.)"
                            % (args.wheel.upper(), NO_MOTION_S,
                               args.lead_target if args.lead > 0 else args.target))

                    # Send on every heartbeat and immediately on a phase
                    # change, so the step edge is not delayed by up to 100 ms.
                    if now >= next_send or phase != last_phase:
                        ser.write(command_line(wheel_index, value).encode("ascii"))
                        next_send = now + HEARTBEAT_S
                        last_phase = phase

                    for line in reader.poll():
                        if line.startswith("V,"):
                            parts = line.split(",")
                            if len(parts) != 5:
                                continue        # corrupted line: skip, keep going
                            try:
                                vals = [float(p) for p in parts[1:]]
                            except ValueError:
                                continue
                            # The D line for this tick comes next. If the
                            # previous V never got its D (old firmware or a
                            # dropped line), write it now with empty integrals.
                            if pending is not None:
                                w.writerow(pending + NO_INTEGRALS)
                            pending = [n, "%.4f" % (time.monotonic() - t_step),
                                       phase, "%.2f" % acked_target] \
                                      + ["%.2f" % v for v in vals]
                            n += 1
                            rows = n    # kept current so a partial log is reported
                            # abs() so a backward step counts as motion too.
                            if (phase in ("lead", "step")
                                    and abs(vals[wheel_index]) > 0.0):
                                moved = True
                        elif line.startswith("D,"):
                            # D,<i_fl>,<i_fr>,<i_rl>,<i_rr>, sent right after
                            # its V line. A D with nothing pending is skipped.
                            parts = line.split(",")
                            if pending is None or len(parts) != 5:
                                continue
                            try:
                                ivals = [float(p) for p in parts[1:]]
                            except ValueError:
                                continue
                            w.writerow(pending + ["%.2f" % v for v in ivals])
                            pending = None
                        elif line.startswith("A,") and not line.startswith(("A,K,", "A,I,")):
                            # Velocity ack: A,<fl>,<fr>,<rl>,<rr>
                            try:
                                acked_target = float(line.split(",")[1 + wheel_index])
                            except (ValueError, IndexError):
                                pass
                        elif line.startswith("S,"):
                            # Any status line mid-run means the board stopped
                            # (S,TIMEOUT is the watchdog); later data is void.
                            raise RuntimeError("board reported %s mid-run" % line)
            finally:
                # However the loop ends, the last V row may still be waiting
                # for its D. Write it so no measured sample is lost.
                if pending is not None:
                    w.writerow(pending + NO_INTEGRALS)
    except KeyboardInterrupt:
        print("\n  interrupted")
    except RuntimeError as e:          # also catches NoMotionError
        print("  ERROR: %s" % e)
    finally:
        ser.write(b"E,0\n")
        time.sleep(0.05)              # let the bytes leave before closing
        ser.close()
        print("  disarmed.")

    expected = int((args.settle + args.lead + args.hold + args.post) / 0.02)
    if complete:
        print("Wrote %d rows to %s (about %d expected for a full run)."
              % (rows, out_path, expected))
        return 0

    # A CSV exists only if the run got as far as arming; a handshake failure
    # leaves nothing to move.
    if csv_opened:
        new_path = move_to_failed(out_path)
        print("Incomplete run: %d rows moved to %s" % (rows, new_path))
    return 1


if __name__ == "__main__":
    sys.exit(main())
