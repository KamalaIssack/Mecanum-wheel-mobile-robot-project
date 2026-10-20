#!/usr/bin/env python3
"""USB-only check of the PI firmware protocol (run with the XT60 unplugged).

Every command keeps the wheels at 0, so no motor power is needed. Checks:

    1. E,0 is answered with S,DISARMED          the board is alive
    2. I,50  -> A,I,50.00                       Ki accepted
    3. I,500 -> silence                         above KI_MAX 400, refused
    4. I,-1  -> silence                         negative, refused
    5. I,abc -> silence                         not a number, refused
    6. K,10  -> A,K,10.00                       Kp still works (regression)
    7. E,1   -> S,ARMED, then 1 s of C,0,0,0,0  D lines arrive, one after
                                                each V, all integrals 0.00
    8. I,50 while armed -> silence              gains locked while armed
    9. E,0   -> S,DISARMED
   10. I,0   -> A,I,0.00                        leave the board at Ki 0

Prints PASS or FAIL for each, and a summary at the end. Exit status 0 only
if every check passes.

    python3 i_check.py                     # auto-detect the ST-Link port
    python3 i_check.py /dev/cu.usbmodemXXXX
"""

import glob
import sys
import time

import serial

BAUD = 115200


def find_port():
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports:
        sys.exit("No /dev/cu.usbmodem* port. Is the Nucleo plugged in?")
    return ports[0]


class Lines:
    """Complete lines from the serial stream (partial text stays buffered)."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def poll(self):
        self.buf += self.ser.read(self.ser.in_waiting or 1)
        out = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            out.append(raw.decode("ascii", "replace").strip())
        return out

    def collect(self, seconds, heartbeat=None):
        """All lines for `seconds`, sending `heartbeat` every 100 ms."""
        lines = []
        t_end = time.monotonic() + seconds
        next_hb = 0.0
        while time.monotonic() < t_end:
            if heartbeat and time.monotonic() >= next_hb:
                self.ser.write(heartbeat)
                next_hb = time.monotonic() + 0.1
            lines += self.poll()
        return lines


results = []


def check(name, ok, detail=""):
    results.append(ok)
    print("%s  %-38s %s" % ("PASS" if ok else "FAIL", name, detail))


def replies(lines):
    """Everything except telemetry, which streams continuously."""
    return [l for l in lines if l and not l.startswith(("V,", "D,"))]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = serial.Serial(port, BAUD, timeout=0.01)
    # Same port settle as step_logger.py before the first write.
    time.sleep(0.2)
    ser.reset_input_buffer()
    rd = Lines(ser)
    print("Port %s\n" % port)

    def send(cmd, seconds=0.3, heartbeat=None):
        ser.write((cmd + "\n").encode("ascii"))
        return rd.collect(seconds, heartbeat)

    try:
        r = replies(send("E,0"))
        if "S,DISARMED" not in r:
            r = replies(send("E,0"))          # same one retry as the logger
        check("E,0 -> S,DISARMED", "S,DISARMED" in r, str(r))

        r = replies(send("I,50"))
        check("I,50 -> A,I,50.00", r == ["A,I,50.00"], str(r))
        for bad in ("I,500", "I,-1", "I,abc"):
            r = replies(send(bad))
            check("%s -> silence" % bad, r == [], str(r))
        r = replies(send("K,10"))
        check("K,10 -> A,K,10.00", r == ["A,K,10.00"], str(r))

        lines = send("E,1", 1.0, heartbeat=b"C,0,0,0,0\n")
        r = [l for l in replies(lines) if not l.startswith("A,")]
        check("E,1 -> S,ARMED", r[:1] == ["S,ARMED"], str(r))
        tele = [l for l in lines if l.startswith(("V,", "D,"))]
        v = sum(1 for l in tele if l.startswith("V,"))
        d = sum(1 for l in tele if l.startswith("D,"))
        check("V and D lines both arrive (~50 each)",
              v >= 40 and d >= 40, "V %d, D %d" % (v, d))
        paired = all(b.startswith("D,") for a, b in zip(tele, tele[1:])
                     if a.startswith("V,"))
        check("every V is followed by a D", paired)
        zeros = all(all(float(x) == 0.0 for x in l.split(",")[1:])
                    for l in tele if l.startswith("D,"))
        check("integrals all 0.00 at target 0", zeros)

        # Keep only gain acks: the C heartbeat earns velocity acks
        # (A,  +0.00,...), which are expected and not what is tested here.
        r = [l for l in replies(send("I,50", 0.3, heartbeat=b"C,0,0,0,0\n"))
             if l.startswith(("A,I,", "A,K,"))]
        check("I,50 while armed -> silence", r == [], str(r))

        r = replies(send("E,0"))
        check("E,0 -> S,DISARMED", "S,DISARMED" in r, str(r))
        r = replies(send("I,0"))
        check("I,0 -> A,I,0.00", r == ["A,I,0.00"], str(r))
    finally:
        ser.write(b"E,0\n")
        time.sleep(0.05)
        ser.close()

    print("\n%d of %d checks passed." % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
