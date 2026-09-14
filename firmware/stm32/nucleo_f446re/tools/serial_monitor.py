#!/usr/bin/env python3
"""Read and pretty-print the Nucleo's V,... wheel-velocity telemetry."""
import sys
import glob
import serial

BAUD = 115200

def find_port():
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports:
        sys.exit("No ST-Link VCP found. Is the Nucleo plugged into USB?")
    return ports[0]

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"Opening {port} at {BAUD} 8N1. Ctrl-C to stop.\n")
    with serial.Serial(port, BAUD, timeout=1) as ser:
        ser.reset_input_buffer()
        while True:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            if not line.startswith("V,"):
                continue
            parts = line.split(",")
            if len(parts) != 5:
                print(f"malformed: {line}")
                continue
            try:
                fl, fr, rl, rr = (float(p) for p in parts[1:])
            except ValueError:
                print(f"parse error: {line}")
                continue
            print(f"FL {fl:+7.2f}   FR {fr:+7.2f}   RL {rl:+7.2f}   RR {rr:+7.2f}   rad/s")

if __name__ == "__main__":
    main()

