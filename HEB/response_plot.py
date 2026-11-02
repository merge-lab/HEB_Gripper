#!/usr/bin/env python3
"""
response_plot.py

Connects to the Gripper ESP32 CLI over USB serial, triggers "tml" (trigger
mapping + log) mode, and logs every streamed [TM] line - expected vs.
actual position for both Dynamixel servos - to a CSV file until the user
quits. Then plots expected vs. actual position for each servo with
matplotlib.

  - Sends "tml" to start streaming.
  - Type 'q' + Enter (or Ctrl-C) to stop: sends "e" to the firmware so it
    exits tml mode cleanly (relaxes the haptic motor, stops the gripper)
    before the port is closed.

Requires pyserial and matplotlib:
    pip install pyserial matplotlib

Examples:
  python response_plot.py                       # auto-pick a port
  python response_plot.py --port COM7 -o run1.csv
  python response_plot.py --list                # list serial ports and exit
"""

import argparse
import csv
import re
import sys
import threading
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required. Install it with:  pip install pyserial")


DEFAULT_BAUD = 115200

# Substrings that hint a port is the ESP32-S3 (native USB CDC / common bridges).
PORT_HINTS = ("usb", "cdc", "cp210", "ch340", "ch910", "esp", "espressif", "serial")

# Matches a streamed trigger-mapping line, e.g.:
#   [TM] pos=+12.34 t=0.56 L=45.0(act=44.8) R=135.0(act=135.2)
TM_RE = re.compile(
    r"\[TM\]\s*pos=(?P<pos>[+-]?\d+\.\d+)\s*t=(?P<t>[+-]?\d+\.\d+)\s*"
    r"L=(?P<left_exp>[+-]?\d+\.\d+)\(act=(?P<left_act>[+-]?\d+\.\d+)\)\s*"
    r"R=(?P<right_exp>[+-]?\d+\.\d+)\(act=(?P<right_act>[+-]?\d+\.\d+)\)"
)

CSV_HEADER = [
    "elapsed_s", "pos_deg", "t",
    "left_expected_deg", "left_actual_deg",
    "right_expected_deg", "right_actual_deg",
]

# Print one console line per PRINT_EVERY logged rows so the terminal doesn't
# get flooded by the ~66 Hz [TM] stream while still logging every row to CSV.
PRINT_EVERY = 10


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

    candidates = [
        p for p in ports
        if any(h in (p.description or "").lower() for h in PORT_HINTS)
    ]
    if len(candidates) == 1:
        print(f"Auto-selected port: {candidates[0].device} "
              f"({candidates[0].description})")
        return candidates[0].device

    print("Multiple/ambiguous serial ports found:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device:12s}  {p.description}")
    while True:
        choice = input("Select a port by number: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(ports):
            return ports[int(choice)].device
        print("Invalid selection.")


def reader_loop(ser, csv_writer, csv_fh, rows, t0, stop_event):
    """Read lines from the serial port; log [TM] rows, echo everything else."""
    buf = b""
    tm_count = 0
    while not stop_event.is_set():
        try:
            chunk = ser.read(max(1, ser.in_waiting))
        except serial.SerialException as e:
            print(f"\n[serial read error: {e}]")
            stop_event.set()
            break
        if not chunk:
            continue

        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").rstrip("\r")
            if not line:
                continue

            m = TM_RE.search(line)
            if m:
                elapsed = time.monotonic() - t0
                g = m.groupdict()
                row = [
                    f"{elapsed:.4f}", g["pos"], g["t"],
                    g["left_exp"], g["left_act"],
                    g["right_exp"], g["right_act"],
                ]
                csv_writer.writerow(row)
                csv_fh.flush()
                rows.append([elapsed] + [float(g[k]) for k in
                             ("pos", "t", "left_exp", "left_act", "right_exp", "right_act")])
                tm_count += 1
                if tm_count % PRINT_EVERY == 0:
                    print(line)
            else:
                print(line)


def plot_response(rows):
    import matplotlib.pyplot as plt

    t = [r[0] for r in rows]
    left_exp = [r[3] for r in rows]
    left_act = [r[4] for r in rows]
    right_exp = [r[5] for r in rows]
    right_act = [r[6] for r in rows]

    EXPECTED_COLOR = "#2a78d6"  # categorical slot 1 (blue)
    ACTUAL_COLOR = "#eb6834"    # categorical slot 2 (orange)
    SURFACE = "#fcfcfb"
    GRID = "#e1e0d9"
    INK = "#0b0b0b"
    MUTED = "#898781"

    fig, (ax_left, ax_right) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.patch.set_facecolor(SURFACE)

    for ax, exp, act, title in (
        (ax_left, left_exp, left_act, "Left servo"),
        (ax_right, right_exp, right_act, "Right servo"),
    ):
        ax.set_facecolor(SURFACE)
        ax.plot(t, exp, color=EXPECTED_COLOR, linewidth=2, label="Expected")
        ax.plot(t, act, color=ACTUAL_COLOR, linewidth=2, label="Actual")
        ax.set_title(title, color=INK, fontsize=11)
        ax.set_ylabel("Position (deg)", color=INK)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.tick_params(colors=MUTED)
        for spine in ax.spines.values():
            spine.set_color(GRID)
        ax.legend(frameon=False, labelcolor=INK)

    ax_right.set_xlabel("Elapsed time (s)", color=INK)
    fig.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(
        description="Log and plot expected vs. actual Dynamixel positions "
                    "during the Gripper ESP32's 'tml' (trigger mapping + log) mode.")
    ap.add_argument("--port", help="Serial port (e.g. COM7 or /dev/ttyUSB0). "
                                    "Auto-detected if omitted.")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help=f"Baud rate (default {DEFAULT_BAUD}).")
    ap.add_argument("-o", "--output",
                    default=datetime.now().strftime("tm_response_%Y%m%d_%H%M%S.csv"),
                    help="CSV output path (default: tm_response_<timestamp>.csv).")
    ap.add_argument("--list", action="store_true",
                    help="List available serial ports and exit.")
    args = ap.parse_args()

    if args.list:
        list_serial_ports()
        return

    port = args.port or pick_port()

    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port} @ {args.baud}: {e}")

    time.sleep(2.0)
    ser.reset_input_buffer()

    csv_fh = open(args.output, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_fh)
    csv_writer.writerow(CSV_HEADER)
    csv_fh.flush()

    print(f"Connected to {port} @ {args.baud}.")
    print(f"Logging CSV data to: {args.output}")
    print("Sending 'tml' to start trigger mapping (logged)...")

    rows = []
    stop_event = threading.Event()
    t0 = time.monotonic()

    reader = threading.Thread(
        target=reader_loop,
        args=(ser, csv_writer, csv_fh, rows, t0, stop_event),
        daemon=True,
    )
    reader.start()

    ser.write(b"tml\n")
    ser.flush()

    print("Type 'q' + Enter (or Ctrl-C) to stop logging.\n")
    try:
        while not stop_event.is_set():
            try:
                cmd = input()
            except EOFError:
                break
            if cmd.strip().lower() in ("q", "quit"):
                break
    except KeyboardInterrupt:
        print()

    print("Stopping trigger mapping...")
    ser.write(b"e\n")
    ser.flush()
    time.sleep(0.3)  # let the reader thread catch "Trigger mapping OFF."

    stop_event.set()
    reader.join(timeout=1.0)
    ser.close()
    csv_fh.close()

    print(f"\nDisconnected. Logged {len(rows)} rows to: {args.output}")

    if not rows:
        print("No [TM] rows were captured - nothing to plot.")
        return

    plot_response(rows)


if __name__ == "__main__":
    main()
