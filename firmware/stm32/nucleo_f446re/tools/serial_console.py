#!/usr/bin/env python3
"""Two-way console for the Nucleo link."""
import sys, glob, threading, serial

BAUD = 115200

def find_port():
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports:
        sys.exit("No ST-Link VCP found. Is the Nucleo plugged into USB?")
    return ports[0]

def reader(ser):
    while True:
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
        except serial.SerialException:
            return
        if not line:
            continue
        if line.startswith("A,"):
            print("  [ACK] " + line)
        elif line.startswith("V,"):
            parts = line.split(",")
            if len(parts) == 5:
                try:
                    vals = [float(p) for p in parts[1:]]
                except ValueError:
                    continue
                if any(abs(v) > 0.001 for v in vals):
                    fl, fr, rl, rr = vals
                    print("  V  FL %+7.2f  FR %+7.2f  RL %+7.2f  RR %+7.2f" % (fl, fr, rl, rr))

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = serial.Serial(port, BAUD, timeout=1)
    ser.reset_input_buffer()
    print("Connected " + port + " at 115200 8N1.")
    print("Type: fl fr rl rr  (four numbers)  |  z = stop  |  Ctrl-C = quit")
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()
    try:
        while True:
            text = input().strip()
            if not text:
                continue
            if text.lower() == "z":
                nums = [0.0, 0.0, 0.0, 0.0]
            else:
                try:
                    nums = [float(x) for x in text.split()]
                except ValueError:
                    print("  (need four numbers, or z)")
                    continue
                if len(nums) != 4:
                    print("  (need exactly four numbers: fl fr rl rr)")
                    continue
            cmd = "C,%.2f,%.2f,%.2f,%.2f\n" % tuple(nums)
            ser.write(cmd.encode("ascii"))
            print("  [sent] " + cmd.strip())
    except (KeyboardInterrupt, EOFError):
        print("")
        ser.close()

if __name__ == "__main__":
    main()
