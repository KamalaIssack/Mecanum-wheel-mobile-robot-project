#!/usr/bin/env python3
"""Two-way console with watchdog heartbeat for the Nucleo link."""
import sys, glob, threading, time, serial

BAUD = 115200
HEARTBEAT_S = 0.2

state = {"cmd": None, "beat": False}
lock = threading.Lock()

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
        if line.startswith("S,"):
            print("  [STATUS] " + line)
        elif line.startswith("A,"):
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

def beater(ser):
    while True:
        time.sleep(HEARTBEAT_S)
        with lock:
            if state["beat"] and state["cmd"]:
                ser.write(state["cmd"].encode("ascii"))

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = serial.Serial(port, BAUD, timeout=1)
    ser.reset_input_buffer()
    print("Connected " + port + " at 115200 8N1.")
    print("Commands:  arm | disarm | z | q | four numbers (fl fr rl rr)")
    threading.Thread(target=reader, args=(ser,), daemon=True).start()
    threading.Thread(target=beater, args=(ser,), daemon=True).start()
    try:
        while True:
            text = input().strip().lower()
            if not text:
                continue
            if text == "q":
                break
            if text == "arm":
                ser.write(b"E,1\n"); print("  [sent] E,1"); continue
            if text == "disarm":
                with lock: state["beat"] = False
                ser.write(b"E,0\n"); print("  [sent] E,0"); continue
            if text == "z":
                cmd = "C,0.00,0.00,0.00,0.00\n"
                with lock: state["cmd"] = cmd; state["beat"] = True
                ser.write(cmd.encode("ascii")); print("  [sent] " + cmd.strip()); continue
            try:
                nums = [float(x) for x in text.split()]
            except ValueError:
                print("  (arm | disarm | z | q | four numbers)"); continue
            if len(nums) != 4:
                print("  (need four numbers: fl fr rl rr)"); continue
            cmd = "C,%.2f,%.2f,%.2f,%.2f\n" % tuple(nums)
            with lock: state["cmd"] = cmd; state["beat"] = True
            ser.write(cmd.encode("ascii")); print("  [sent] " + cmd.strip())
    except (KeyboardInterrupt, EOFError):
        pass
    ser.close(); print("")

if __name__ == "__main__":
    main()
