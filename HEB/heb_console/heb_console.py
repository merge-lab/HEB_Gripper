#!/usr/bin/env python3
"""HEB Console - serial monitor + live plots for the gripper ESP32.

Replaces the PlatformIO serial monitor for the demo: one window with
  * a serial console (ESP32 output, command box with history, quick buttons)
  * live plots that light up automatically:
      - 'tac' mode: tactile T1..T6 (tared), 100 Hz
      - 'tmt' mode: tactile T1..T6, side sums, haptic force c, trigger
        encoder position, and gripper ID0/ID1 commanded positions
    with a sample-rate readout and CSV recording.

Only one program can hold the serial port: close the PlatformIO monitor
before connecting here.

Run:  ./run.sh            (creates .venv on first run)
      ./run.sh --port /dev/ttyACM0
"""
import argparse
import collections
import csv
import datetime
import os
import re
import sys
import time

import numpy as np
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------
N_TAC = 6                      # T1..T6, matches tactile_sensors.h
MAX_HISTORY_S = 300            # seconds of data kept for plotting
MAX_RATE_HZ = 100              # sizing for the history buffers
DEFAULT_WINDOW_S = 10.0        # visible time window
PLOT_REFRESH_MS = 33           # ~30 fps redraw
CONSOLE_MAX_LINES = 5000
QUICK_COMMANDS = [
    ("tac", "tac", "Stream tactile only (100 Hz)"),
    ("tmt", "tmt", "Trigger -> gripper mapping with tactile-driven force"),
    ("Tare", "tare", "Re-zero the tactile sensors (keep them unloaded)"),
    ("Exit mode", "e", "Exit the current mode (tac, tmt, tm, gpos, ...)"),
    ("gpos", "gpos", "Torque off + stream servo positions"),
    ("gping", "gping", "Ping the gripper board"),
    ("gstop", "gstop", "Gripper torque off"),
    ("relax", "stop", "Relax the haptic motor"),
]

# ESP32 lines that mean something went wrong; shown in the red alert banner.
ALERT_PREFIXES = (
    "[DXL GUARD]",          # gripper move rejected / servos relaxed
    "[MW] aborted", "[MW] REJECTED", "[MW] move failed", "[MW] lost contact",
    "move REJECTED",
    "[TM] no ACK", "[TMT] no ACK", "[HU] no ACK",
    "no ACK",
    "gripper no PONG", "haptic no PONG",
    "Calibration FAILED",
)

# Once-per-second timing line from tm/tmt (see trigger_mapping()).
STATS_RE = re.compile(
    r"^\[(?:TMT|TM)\] loop (?P<hz>\d+) Hz, haptic exchange avg (?P<hap>[\d.]+) ms / max (?P<hapmax>[\d.]+) ms, "
    r"gripper feedback latency (?P<lat>\d+) ms, age (?P<age>-?\d+) ms(?:, mega bad replies (?P<bad>\d+))?")

# Haptic trigger force: c (command units) x 3.18 = Newtons at the trigger.
NEWTONS_PER_C = 3.18

# Fields of a "TMT,..." line, in order (see trigger_mapping() in CLI_Control.cpp).
TMT_FIELDS = ["pos", "t", "id0", "id1", "sumL", "sumR", "c"] + [f"T{i + 1}" for i in range(N_TAC)]
CSV_FIELDS = ["t_s", "mode"] + TMT_FIELDS

# Plot panels: (key, title, [(series, label)], fixed y-range or None for auto)
PANELS = [
    ("tactile", "Tactile (tared)", [(f"T{i + 1}", f"T{i + 1}") for i in range(N_TAC)], None),
    ("force", "Haptic force c (right axis: N = c x 3.18)", [("c", "c")], (0.25, 1.05)),
    ("trigger", "Trigger encoder", [("pos", "pos (deg)")], None),
    ("gripper", "Gripper commanded", [("id1", "ID1 (left)"), ("id0", "ID0 (right)")], (-5, 185)),
]

# One colour family per finger, dark -> light within a finger:
#   left finger  (T1..T3, servo ID 1): blues
#   right finger (T4..T6, servo ID 0): oranges
LEFT_COLORS = ["#2f6fdf", "#5b9cf6", "#9cc4fb"]
RIGHT_COLORS = ["#e2590f", "#f98b3c", "#fcbc85"]
SERIES_COLORS = {
    **{f"T{i + 1}": c for i, c in enumerate(LEFT_COLORS)},
    **{f"T{i + 4}": c for i, c in enumerate(RIGHT_COLORS)},
    "id1": LEFT_COLORS[1],     # left finger's servo
    "id0": RIGHT_COLORS[1],    # right finger's servo
    "c": "#4cc38a",
    "pos": "#d9d9d9",
}


class NewtonAxis(pg.AxisItem):
    """Right-hand axis for the force plot: same view, ticks shown in N."""

    def tickStrings(self, values, scale, spacing):
        return [f"{v * NEWTONS_PER_C:.2f}" for v in values]


def parse_tactile(line):
    """'T1:0.1,T2:0.2,...,T6:0.6' -> [0.1, ..., 0.6], else None."""
    if not line.startswith("T1:"):
        return None
    parts = line.split(",")
    if len(parts) != N_TAC:
        return None
    vals = []
    for i, part in enumerate(parts):
        key, sep, val = part.partition(":")
        if not sep or key.strip() != f"T{i + 1}":
            return None
        try:
            vals.append(float(val))
        except ValueError:
            return None
    return vals


def parse_tmt(line):
    """'TMT,pos,t,id0,...' -> dict of floats, else None (also for the header)."""
    if not line.startswith("TMT,"):
        return None
    parts = line.split(",")[1:]
    if len(parts) != len(TMT_FIELDS):
        return None
    try:
        return dict(zip(TMT_FIELDS, (float(p) for p in parts)))
    except ValueError:
        return None   # the column-header line


# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------
class SerialReader(QtCore.QThread):
    """Reads lines off the port and hands them to the GUI in batches, so a
    100 Hz stream doesn't flood the Qt event loop one signal per line."""

    batch = QtCore.pyqtSignal(list)    # [(host_time_s, line_str), ...]
    failed = QtCore.pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        buf = b""
        pending = []
        last_emit = time.perf_counter()
        while not self._stop:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError) as e:
                if not self._stop:
                    self.failed.emit(str(e))
                return
            if chunk:
                buf += chunk
                now = time.perf_counter()
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    pending.append((now, raw.decode("utf-8", errors="replace").rstrip("\r")))
            now = time.perf_counter()
            if pending and (now - last_emit >= 0.02 or len(pending) >= 200):
                self.batch.emit(pending)
                pending = []
                last_emit = now
        if pending:
            self.batch.emit(pending)


# ---------------------------------------------------------------------------
# Command box with up/down history
# ---------------------------------------------------------------------------
class CommandLine(QtWidgets.QLineEdit):
    def __init__(self):
        super().__init__()
        self.history = []
        self.hist_idx = 0
        self.setPlaceholderText("Command (Enter to send, Up/Down for history)")

    def remember(self, text):
        if text and (not self.history or self.history[-1] != text):
            self.history.append(text)
        self.hist_idx = len(self.history)

    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_Up and self.history:
            self.hist_idx = max(0, self.hist_idx - 1)
            self.setText(self.history[self.hist_idx])
        elif e.key() == QtCore.Qt.Key_Down and self.history:
            self.hist_idx = min(len(self.history), self.hist_idx + 1)
            self.setText(self.history[self.hist_idx] if self.hist_idx < len(self.history) else "")
        else:
            super().keyPressEvent(e)


# ---------------------------------------------------------------------------
# Time-series store: one time base per stream, series appended together
# ---------------------------------------------------------------------------
class Stream:
    def __init__(self, names):
        maxlen = MAX_HISTORY_S * MAX_RATE_HZ
        self.t = collections.deque(maxlen=maxlen)
        self.series = {n: collections.deque(maxlen=maxlen) for n in names}

    def clear(self):
        self.t.clear()
        for d in self.series.values():
            d.clear()

    def append(self, t, values):
        self.t.append(t)
        for n, d in self.series.items():
            d.append(values[n])


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------
class HebConsole(QtWidgets.QMainWindow):
    def __init__(self, port=None, baud=115200):
        super().__init__()
        self.setWindowTitle("HEB Console")
        self.resize(1500, 950)

        self.ser = None
        self.reader = None

        self.tac = Stream([f"T{i + 1}" for i in range(N_TAC)])
        self.tmt = Stream(TMT_FIELDS[:7])          # pos, t, id0, id1, sumL, sumR, c
        self.t0 = None
        self.new_data = False
        self.rate_times = collections.deque(maxlen=MAX_RATE_HZ * 3)
        self.last_sample_time = 0.0
        self.mode = None                           # None, "tac", "tmt"

        self.rec_file = None
        self.rec_writer = None
        self.rec_count = 0
        self.rec_t0 = 0.0

        self._build_ui()
        self.refresh_ports(select=port)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.redraw)
        self.plot_timer.start(PLOT_REFRESH_MS)

        self.baud_box.setCurrentText(str(baud))
        if port:
            QtCore.QTimer.singleShot(0, self.connect_port)

    # ---- UI ----------------------------------------------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        # Connection bar
        bar = QtWidgets.QHBoxLayout()
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(320)
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(lambda: self.refresh_ports())
        self.baud_box = QtWidgets.QComboBox()
        self.baud_box.setEditable(True)
        self.baud_box.addItems(["115200", "921600", "1000000", "2000000"])
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.status = QtWidgets.QLabel("Disconnected")
        bar.addWidget(QtWidgets.QLabel("Port"))
        bar.addWidget(self.port_box)
        bar.addWidget(refresh)
        bar.addWidget(QtWidgets.QLabel("Baud"))
        bar.addWidget(self.baud_box)
        bar.addWidget(self.connect_btn)
        bar.addSpacing(16)
        bar.addWidget(self.status, 1)
        root.addLayout(bar)

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(split, 1)

        # ---- Left: console
        left = QtWidgets.QWidget()
        ll = QtWidgets.QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)

        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(CONSOLE_MAX_LINES)
        self.console.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.console.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        ll.addWidget(self.console, 1)

        opts = QtWidgets.QHBoxLayout()
        self.show_data_cb = QtWidgets.QCheckBox("Show data lines in console")
        self.show_data_cb.setToolTip("T-/TMT-lines arrive at up to 100 Hz; hide them to keep the console readable")
        self.autoscroll_cb = QtWidgets.QCheckBox("Autoscroll")
        self.autoscroll_cb.setChecked(True)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self.console.clear)
        opts.addWidget(self.show_data_cb)
        opts.addWidget(self.autoscroll_cb)
        opts.addStretch(1)
        opts.addWidget(clear)
        ll.addLayout(opts)

        quick = QtWidgets.QGridLayout()
        for k, (label, cmd, tip) in enumerate(QUICK_COMMANDS):
            b = QtWidgets.QPushButton(label)
            b.setToolTip(tip)
            b.clicked.connect(lambda _=False, c=cmd: self.send(c))
            if cmd == "tare":
                b.setStyleSheet("font-weight: bold;")
            quick.addWidget(b, k // 4, k % 4)
        ll.addLayout(quick)

        cmd_row = QtWidgets.QHBoxLayout()
        self.cmd = CommandLine()
        self.cmd.returnPressed.connect(self.send_from_box)
        send = QtWidgets.QPushButton("Send")
        send.clicked.connect(self.send_from_box)
        cmd_row.addWidget(self.cmd, 1)
        cmd_row.addWidget(send)
        ll.addLayout(cmd_row)
        split.addWidget(left)

        # ---- Right: plots
        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)

        ctl = QtWidgets.QHBoxLayout()
        self.mode_label = QtWidgets.QLabel("Mode: idle")
        self.mode_label.setStyleSheet("font-weight: bold;")
        self.hz_label = QtWidgets.QLabel("-- Hz")
        self.c_label = QtWidgets.QLabel("c = --")
        self.c_label.setStyleSheet("font-weight: bold;")
        self.window_box = QtWidgets.QDoubleSpinBox()
        self.window_box.setRange(0.5, MAX_HISTORY_S)
        self.window_box.setValue(DEFAULT_WINDOW_S)
        self.window_box.setSuffix(" s")
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.record_btn = QtWidgets.QPushButton("Record CSV")
        self.record_btn.setCheckable(True)
        self.record_btn.toggled.connect(self.toggle_record)
        self.esp_label = QtWidgets.QLabel("")
        self.esp_label.setToolTip("From the ESP32's once-per-second timing line (tm/tmt)")
        for w in (self.mode_label, self.hz_label, self.c_label, self.esp_label):
            ctl.addWidget(w)
            ctl.addSpacing(12)
        ctl.addStretch(1)
        ctl.addWidget(QtWidgets.QLabel("Window"))
        ctl.addWidget(self.window_box)
        ctl.addWidget(self.pause_btn)
        ctl.addWidget(self.record_btn)
        rl.addLayout(ctl)

        # Alert banner: latest error line from the ESP32 (or the app).
        self.alert_box = QtWidgets.QFrame()
        self.alert_box.setStyleSheet("QFrame { background: #b3261e; border-radius: 4px; }"
                                     "QLabel { color: white; font-weight: bold; }")
        ab = QtWidgets.QHBoxLayout(self.alert_box)
        ab.setContentsMargins(8, 4, 4, 4)
        self.alert_label = QtWidgets.QLabel("")
        self.alert_label.setWordWrap(True)
        self.alert_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        dismiss = QtWidgets.QPushButton("Dismiss")
        dismiss.clicked.connect(self.clear_alert)
        ab.addWidget(self.alert_label, 1)
        ab.addWidget(dismiss)
        self.alert_box.hide()
        self.alert_text = None
        self.alert_count = 0
        rl.addWidget(self.alert_box)

        # Panel visibility + tactile channel toggles + live values
        vis = QtWidgets.QHBoxLayout()
        vis.addWidget(QtWidgets.QLabel("Show:"))
        self.panel_cbs = {}
        for key, title, _, _ in PANELS:
            cb = QtWidgets.QCheckBox(title.split(" (")[0])
            cb.setChecked(True)
            cb.toggled.connect(lambda on, k=key: self.set_panel_visible(k, on))
            self.panel_cbs[key] = cb
            vis.addWidget(cb)
        vis.addStretch(1)
        rl.addLayout(vis)

        self.colors = [pg.mkColor(SERIES_COLORS[f"T{i + 1}"]) for i in range(N_TAC)]
        chan_row = QtWidgets.QHBoxLayout()
        self.value_labels = []
        self.chan_cbs = []
        for i in range(N_TAC):
            cb = QtWidgets.QCheckBox(f"T{i + 1}")
            cb.setChecked(True)
            cb.setStyleSheet(f"color: {self.colors[i].name()}; font-weight: bold;")
            cb.toggled.connect(lambda on, k=i: self.curves[f"T{k + 1}"].setVisible(on))
            self.chan_cbs.append(cb)
            val = QtWidgets.QLabel("  --  ")
            val.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
            self.value_labels.append(val)
            chan_row.addWidget(cb)
            chan_row.addWidget(val)
            chan_row.addSpacing(6)
        chan_row.addStretch(1)
        rl.addLayout(chan_row)

        # Stacked plots, x-axes linked
        self.glw = pg.GraphicsLayoutWidget()
        rl.addWidget(self.glw, 1)
        self.plots = {}
        self.curves = {}
        first = None
        for row, (key, title, series, yr) in enumerate(PANELS):
            axes = {"right": NewtonAxis(orientation="right")} if key == "force" else None
            p = self.glw.addPlot(row=row, col=0, title=title, axisItems=axes)
            if key == "force":
                p.showAxis("right")
                p.setLabel("right", "N")
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(5, 5))
            if yr:
                p.setYRange(*yr, padding=0)
                p.enableAutoRange(axis="y", enable=False)
            else:
                p.enableAutoRange(axis="y", enable=True)
            if first is None:
                first = p
            else:
                p.setXLink(first)
            for j, (name, label) in enumerate(series):
                color = SERIES_COLORS[name]
                c = p.plot(pen=pg.mkPen(color, width=2), name=label)
                c.setClipToView(True)
                c.setDownsampling(auto=True, method="peak")
                self.curves[name] = c
            self.plots[key] = p
        self.plots[PANELS[-1][0]].setLabel("bottom", "Time", units="s")

        # Bar view of current tactile values
        self.bar_plot = pg.PlotWidget()
        self.bar_plot.setMaximumHeight(140)
        self.bar_plot.showGrid(y=True, alpha=0.3)
        self.bar_plot.setLabel("left", "Now")
        self.bar_plot.setXRange(0.4, N_TAC + 0.6)
        self.bar_plot.getAxis("bottom").setTicks([[(i + 1, f"T{i + 1}") for i in range(N_TAC)]])
        self.bar_plot.setMouseEnabled(x=False, y=False)
        self.bars = pg.BarGraphItem(x=np.arange(1, N_TAC + 1), height=np.zeros(N_TAC),
                                    width=0.6, brushes=self.colors)
        self.bar_plot.addItem(self.bars)
        rl.addWidget(self.bar_plot)

        split.addWidget(right)
        split.setSizes([560, 940])

    def set_panel_visible(self, key, on):
        self.plots[key].setVisible(on)

    # ---- Connection ---------------------------------------------------------
    def refresh_ports(self, select=None):
        current = select or self.port_box.currentData()
        self.port_box.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        # Real USB devices first; skip the 32 legacy /dev/ttyS* stubs.
        ports = [p for p in ports if not p.device.startswith("/dev/ttyS")] or ports
        for p in ports:
            self.port_box.addItem(f"{p.device}  |  {p.description}", p.device)
        if current:
            idx = self.port_box.findData(current)
            if idx < 0:
                self.port_box.addItem(current, current)
                idx = self.port_box.count() - 1
            self.port_box.setCurrentIndex(idx)

    def toggle_connection(self):
        if self.ser:
            self.disconnect_port()
        else:
            self.connect_port()

    def connect_port(self):
        port = self.port_box.currentData()
        if not port:
            self.log_local("No port selected.")
            return
        try:
            baud = int(self.baud_box.currentText())
        except ValueError:
            self.log_local("Invalid baud rate.")
            return
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baud
            ser.timeout = 0.01
            # Keep DTR/RTS low so opening the port doesn't reset the ESP32-S3.
            ser.dtr = False
            ser.rts = False
            ser.open()
        except (serial.SerialException, OSError) as e:
            self.log_local(f"Could not open {port}: {e}")
            if "busy" in str(e).lower() or "permission" in str(e).lower():
                self.log_local("Is the PlatformIO serial monitor still open? Close it first.")
            return
        self.ser = ser
        self.reader = SerialReader(ser)
        self.reader.batch.connect(self.on_lines)
        self.reader.failed.connect(self.on_serial_error)
        self.reader.start()
        self.connect_btn.setText("Disconnect")
        self.status.setText(f"Connected: {port} @ {baud}")
        self.log_local(f"Connected to {port}.")
        self.cmd.setFocus()

    def disconnect_port(self):
        if self.reader:
            self.reader.stop()
            self.reader.wait(500)
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.connect_btn.setText("Connect")
        self.status.setText("Disconnected")
        self.set_mode(None)

    def on_serial_error(self, msg):
        self.log_local(f"Serial error: {msg} (board unplugged or reset?)")
        self.show_alert(f"Serial error: {msg} (board unplugged or reset?)")
        self.disconnect_port()

    # ---- Sending --------------------------------------------------------------
    def send_from_box(self):
        text = self.cmd.text().strip()
        if not text:
            return
        self.cmd.remember(text)
        self.cmd.clear()
        self.send(text)

    def send(self, text):
        if not self.ser:
            self.log_local("Not connected.")
            return
        try:
            self.ser.write((text + "\n").encode("utf-8"))
        except (serial.SerialException, OSError) as e:
            self.on_serial_error(str(e))
            return
        self.append_console([f">> {text}"])

    # ---- Receiving ------------------------------------------------------------
    def on_lines(self, items):
        text_lines = []
        show_data = self.show_data_cb.isChecked()
        for t_host, line in items:
            tmt = parse_tmt(line)
            if tmt is not None:
                self.add_tmt(t_host, tmt)
                if show_data:
                    text_lines.append(line)
                continue
            vals = parse_tactile(line)
            if vals is not None:
                self.add_tac(t_host, vals)
                if show_data:
                    text_lines.append(line)
                continue
            if line.startswith("TMT,pos"):
                continue                     # column header, not useful in the console
            m = STATS_RE.match(line)
            if m:
                self.show_esp_stats(m)
                if show_data:
                    text_lines.append(line)
                continue
            text_lines.append(line)
            if line.startswith(ALERT_PREFIXES):
                self.show_alert(line)
            if line.startswith("Tactile mode ON"):
                self.set_mode("tac")
            elif line.startswith("Tactile mode OFF") or line.startswith("Trigger mapping OFF"):
                self.set_mode(None)
        if text_lines:
            self.append_console(text_lines)

    def _stamp(self, t_host):
        if self.t0 is None:
            self.t0 = t_host
        self.rate_times.append(t_host)
        self.last_sample_time = t_host
        self.new_data = True
        return t_host - self.t0

    def add_tac(self, t_host, vals):
        if self.mode != "tac":
            self.set_mode("tac")             # also catches connecting mid-stream
        t = self._stamp(t_host)
        self.tac.append(t, {f"T{i + 1}": vals[i] for i in range(N_TAC)})
        if self.rec_writer:
            row = {f"T{i + 1}": vals[i] for i in range(N_TAC)}
            self._record(t_host, "tac", row)

    def add_tmt(self, t_host, d):
        if self.mode != "tmt":
            self.set_mode("tmt")
        t = self._stamp(t_host)
        self.tac.append(t, d)
        self.tmt.append(t, d)
        if self.rec_writer:
            self._record(t_host, "tmt", d)

    def _record(self, t_host, mode, row):
        self.rec_writer.writerow([f"{t_host - self.rec_t0:.4f}", mode] +
                                 [("%.4g" % row[f]) if f in row else "" for f in TMT_FIELDS])
        self.rec_count += 1

    def append_console(self, lines):
        sb = self.console.verticalScrollBar()
        at_bottom = sb.value() >= sb.maximum() - 4
        self.console.appendPlainText("\n".join(lines))
        if self.autoscroll_cb.isChecked() and at_bottom:
            sb.setValue(sb.maximum())

    def log_local(self, msg):
        self.append_console([f"-- {msg}"])

    def show_esp_stats(self, m):
        bad = int(m.group("bad") or 0)
        age = int(m.group("age"))
        txt = (f"ESP {m.group('hz')} Hz | haptic {float(m.group('hap')):.1f} ms | "
               f"gripper fb {m.group('lat')} ms, age {age} ms | Mega bad {bad}/s")
        warn = bad > 0 or age < 0 or age > 300
        self.esp_label.setText(txt)
        self.esp_label.setStyleSheet("color: #e2590f; font-weight: bold;" if warn else "color: #888;")

    def show_alert(self, text):
        # Same message repeating (e.g. the guard at 1 Hz): count, don't spam.
        if text == self.alert_text:
            self.alert_count += 1
        else:
            self.alert_text = text
            self.alert_count = 1
        stamp = datetime.datetime.now().strftime("%H:%M:%S")
        suffix = f"   (x{self.alert_count})" if self.alert_count > 1 else ""
        self.alert_label.setText(f"{stamp}  {text}{suffix}")
        self.alert_box.show()
        QtWidgets.QApplication.beep()

    def clear_alert(self):
        self.alert_box.hide()
        self.alert_text = None
        self.alert_count = 0

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        if mode:
            # Fresh traces each time a streaming mode starts.
            self.tac.clear()
            self.tmt.clear()
            self.t0 = None
            self.rate_times.clear()
            name = {"tac": "TACTILE", "tmt": "TRIGGER MAP + TACTILE FORCE"}[mode]
            self.mode_label.setText(f"Mode: {name}")
            self.mode_label.setStyleSheet("font-weight: bold; color: #2a9d3a;")
        else:
            self.mode_label.setText("Mode: idle")
            self.mode_label.setStyleSheet("font-weight: bold;")
            self.hz_label.setText("-- Hz")
            self.c_label.setText("c = --")
            self.esp_label.setText("")

    # ---- Plotting -------------------------------------------------------------
    def redraw(self):
        now = time.perf_counter()
        if len(self.rate_times) >= 2 and now - self.last_sample_time < 1.0:
            span = self.rate_times[-1] - self.rate_times[0]
            if span > 0:
                self.hz_label.setText(f"{(len(self.rate_times) - 1) / span:5.1f} Hz")
        elif self.mode and now - self.last_sample_time > 1.0:
            self.hz_label.setText("no data")

        if self.rec_writer:
            self.record_btn.setText(f"Stop ({self.rec_count})")

        if not self.new_data or self.pause_btn.isChecked():
            return
        self.new_data = False

        win = self.window_box.value()
        t_end = None
        for stream in (self.tac, self.tmt):
            if not stream.t:
                for name in stream.series:
                    if name in self.curves:
                        self.curves[name].setData([], [])
                continue
            t = np.fromiter(stream.t, dtype=float, count=len(stream.t))
            start = np.searchsorted(t, t[-1] - win)
            t_end = t[-1] if t_end is None else max(t_end, t[-1])
            for name, d in stream.series.items():
                if name not in self.curves:      # e.g. 't' (trigger fraction): recorded, not plotted
                    continue
                y = np.fromiter(d, dtype=float, count=len(d))
                self.curves[name].setData(t[start:], y[start:])

        if t_end is not None:
            self.plots[PANELS[0][0]].setXRange(max(0.0, t_end - win), max(win, t_end), padding=0)

        if self.tac.t:
            latest = [self.tac.series[f"T{i + 1}"][-1] for i in range(N_TAC)]
            for i, v in enumerate(latest):
                self.value_labels[i].setText(f"{v:+.3f}")
            self.bars.setOpts(height=np.array(latest))
            lo, hi = min(-1.0, min(latest)), max(1.0, max(latest))
            self.bar_plot.setYRange(lo - 0.1, hi + 0.1, padding=0)
        if self.tmt.t:
            c = self.tmt.series['c'][-1]
            self.c_label.setText(f"c = {c:.3f}  ({c * NEWTONS_PER_C:.2f} N)")

    # ---- Recording ------------------------------------------------------------
    def toggle_record(self, on):
        if on:
            default = os.path.abspath(datetime.datetime.now().strftime("heb_%Y%m%d_%H%M%S.csv"))
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Record data", default, "CSV (*.csv)")
            if not path:
                self.record_btn.setChecked(False)
                return
            try:
                self.rec_file = open(path, "w", newline="")
            except OSError as e:
                self.log_local(f"Could not open {path}: {e}")
                self.record_btn.setChecked(False)
                return
            self.rec_writer = csv.writer(self.rec_file)
            self.rec_writer.writerow(CSV_FIELDS)
            self.rec_count = 0
            self.rec_t0 = time.perf_counter()
            self.log_local(f"Recording to {path}")
        else:
            if self.rec_file:
                self.rec_file.close()
                self.log_local(f"Recording stopped: {self.rec_count} samples saved.")
            self.rec_file = None
            self.rec_writer = None
            self.record_btn.setText("Record CSV")

    def closeEvent(self, e):
        if self.record_btn.isChecked():
            self.record_btn.setChecked(False)
        self.disconnect_port()
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port to open on start, e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud rate (the ESP32-S3 USB port ignores it; default 115200)")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = HebConsole(port=args.port, baud=args.baud)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
