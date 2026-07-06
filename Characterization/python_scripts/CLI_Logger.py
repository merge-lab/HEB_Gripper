#!/usr/bin/env python3
"""
serial_monitor_logger.py

A minimal serial-terminal UI that behaves like the PlatformIO/Arduino serial
monitor for the Gripper ESP32 CLI, but ALSO tees structured status lines from
the ESP32 into a CSV file (useful for torque/encoder characterization and
import into Google Sheets).

  - Type CLI commands exactly as you would in the serial monitor
    (s <k>, c <t>, f <hz>, a <amp>, v <hz> <amp>, sweep, rampup, stop, ...).
    Each line is sent to the ESP32 terminated with '\\n' (the firmware uses
    Serial.readStringUntil('\\n')).
  - Structured ESP32 status lines are parsed and written as CSV rows.
    Non-status lines (boot messages, etc.) are printed to console only.
  - Type ':quit' (or ':q', or Ctrl-C) to exit.

The command input stays pinned on its own line at the bottom of the terminal
while received data scrolls above it, so the fast status stream never eats your
keystrokes (needs prompt_toolkit; falls back to plain input() without it or with
--simple).

Requires pyserial (and prompt_toolkit for the pinned input line):
    pip install pyserial prompt_toolkit

Examples:
  python serial_monitor_logger.py                 # auto-pick a port, default log
  python serial_monitor_logger.py --port COM7
  python serial_monitor_logger.py --port COM7 --log runs/spring_test.csv
  python serial_monitor_logger.py --list          # list serial ports and exit
  python serial_monitor_logger.py --raw            # log without host timestamps
"""

import argparse
import csv
import os
import re
import sys
import threading
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required. Install it with:  pip install pyserial")

# Optional: prompt_toolkit gives a fixed input line at the bottom of the screen
# while a background thread prints above it (patch_stdout). Fall back gracefully.
try:
    from prompt_toolkit import PromptSession
    from prompt_toolkit.patch_stdout import patch_stdout
    _HAVE_PTK = True
except ImportError:
    _HAVE_PTK = False


# --- Defaults ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOG = os.path.join(SCRIPT_DIR, "..", "logs", "esp32_session.csv")
DEFAULT_BAUD = 115200

# Substrings that hint a port is the ESP32-S3 (native USB CDC / common bridges).
PORT_HINTS = ("usb", "cdc", "cp210", "ch340", "ch910", "esp", "espressif", "serial")

# Matches the structured ESP32 status line, e.g.:
#   [RELAX | spr 0.00 | vib 0Hz 0.00] set=+0.000 pos=+7.60 enc=173 cmd=-0.000 motorT=-0.018 userT=+0.018
STATUS_RE = re.compile(
    r"\[(?P<mode>\w+)\s*\|\s*spr\s*(?P<spr_k>[+-]?\d+\.\d+)\s*\|"
    r"\s*vib\s*(?P<vib_hz>\d+)Hz\s*(?P<vib_amp>[+-]?\d+\.\d+)\]"
    r"\s*set=(?P<setpoint>[+-]?\d+\.\d+)"
    r"\s*pos=(?P<position_deg>[+-]?\d+\.\d+)"
    r"\s*enc=(?P<encoder>[+-]?\d+)"
    r"\s*cmd=(?P<cmd_torque>[+-]?\d+\.\d+)"
    r"\s*motorT=(?P<motor_torque>[+-]?\d+\.\d+)"
    r"\s*userT=(?P<user_torque>[+-]?\d+\.\d+)"
)

CSV_HEADER = [
    "timestamp", "mode", "spr_k", "vib_hz", "vib_amp",
    "setpoint_A", "position_deg", "encoder",
    "cmd_torque_A", "motor_torque_A", "user_torque_A",
]


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return ports
    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device:12s}  {p.description}")
    return ports


def pick_port():
    """Auto-select a likely ESP32 port, else prompt the user to choose."""
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Plug in the ESP32 or pass --port.")

    # Prefer a port whose description looks like a USB-serial / ESP device.
    candidates = [
        p for p in ports
        if any(h in (p.description or "").lower() for h in PORT_HINTS)
    ]
    if len(candidates) == 1:
        print(f"Auto-selected port: {candidates[0].device} "
              f"({candidates[0].description})")
        return candidates[0].device

    # Otherwise, present a numbered menu.
    print("Multiple/ambiguous serial ports found:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device:12s}  {p.description}")
    while True:
        choice = input("Select a port by number: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(ports):
            return ports[int(choice)].device
        print("Invalid selection.")


def reader_loop(ser, log_fh, csv_writer, add_timestamps, stop_event):
    """Read lines from the serial port; print them and write parsed rows to CSV."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()  # blocks up to ser.timeout
        except serial.SerialException as e:
            print(f"\n[serial read error: {e}]")
            stop_event.set()
            break
        if not raw:
            continue  # timeout with no data

        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        print(line)

        m = STATUS_RE.search(line)
        if not m:
            continue  # boot messages, echo lines, etc. — skip from CSV

        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] if add_timestamps else ""
        g = m.groupdict()
        csv_writer.writerow([
            ts,
            g["mode"], g["spr_k"], g["vib_hz"], g["vib_amp"],
            g["setpoint"], g["position_deg"], g["encoder"],
            g["cmd_torque"], g["motor_torque"], g["user_torque"],
        ])
        log_fh.flush()


def main():
    ap = argparse.ArgumentParser(
        description="Serial terminal for the Gripper ESP32 CLI that logs all "
                    "received data to a file.")
    ap.add_argument("--port", help="Serial port (e.g. COM7 or /dev/ttyUSB0). "
                                    "Auto-detected if omitted.")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help=f"Baud rate (default {DEFAULT_BAUD}).")
    ap.add_argument("--log", default=DEFAULT_LOG,
                    help=f"Log file path (default: {DEFAULT_LOG}).")
    ap.add_argument("--raw", action="store_true",
                    help="Log lines exactly as received (no host timestamps).")
    ap.add_argument("--list", action="store_true",
                    help="List available serial ports and exit.")
    ap.add_argument("--simple", action="store_true",
                    help="Force plain input() mode (no pinned input line).")
    args = ap.parse_args()

    if args.list:
        list_serial_ports()
        return

    port = args.port or pick_port()
    log_path = os.path.abspath(args.log)
    os.makedirs(os.path.dirname(log_path), exist_ok=True)

    # Open the port. A short read timeout lets the reader thread poll stop_event.
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port} @ {args.baud}: {e}")

    # Append so multiple sessions accumulate in one file.
    # Write the CSV header only when starting a new (empty) file.
    is_new_file = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
    log_fh = open(log_path, "a", newline="", encoding="utf-8")
    csv_writer = csv.writer(log_fh)
    if is_new_file:
        csv_writer.writerow(CSV_HEADER)
        log_fh.flush()

    use_ptk = _HAVE_PTK and not args.simple

    print(f"Connected to {port} @ {args.baud}.")
    print(f"Logging CSV data to: {log_path}"
          f"{'' if args.raw else '  (host-timestamped)'}")
    print("Type CLI commands and press Enter. ':quit' or Ctrl-C to exit.")
    if not use_ptk and not args.simple:
        print("(Install prompt_toolkit for a fixed input line: pip install prompt_toolkit)")
    print()

    stop_event = threading.Event()
    reader = threading.Thread(
        target=reader_loop,
        args=(ser, log_fh, csv_writer, not args.raw, stop_event),
        daemon=True,
    )
    reader.start()

    def send(cmd):
        # Send one line to the ESP32 CLI (terminated with '\n').
        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()

    try:
        if use_ptk:
            # patch_stdout() pins the prompt at the bottom; the reader thread's
            # print() output scrolls above it without clobbering what you type.
            session = PromptSession()
            with patch_stdout():
                while not stop_event.is_set():
                    try:
                        cmd = session.prompt("cmd> ")
                    except (EOFError, KeyboardInterrupt):
                        break
                    if cmd.strip().lower() in (":quit", ":q", ":exit"):
                        break
                    send(cmd)
        else:
            while not stop_event.is_set():
                try:
                    cmd = input()
                except EOFError:
                    break  # stdin closed (e.g. piped input finished)
                if cmd.strip().lower() in (":quit", ":q", ":exit"):
                    break
                send(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        reader.join(timeout=1.0)
        ser.close()
        log_fh.close()
        print("\nDisconnected. CSV saved to:", log_path)


if __name__ == "__main__":
    main()
