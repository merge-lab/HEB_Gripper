import sys
import os
import time
import struct
import datetime
import queue
import multiprocessing as mp
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


# =========================
# User settings
# =========================
DEFAULT_BAUDRATE = "1000000"

BATCH_SIZE = 5
BATCH_MAX_DELAY = 0.02
GUI_REFRESH_MS = 10

SAMPLE_RATE_HZ = 100
MAX_PLOT_HOURS = 100
MAX_PLOT_SECONDS = MAX_PLOT_HOURS * 3600
MAX_STORED_POINTS = int(MAX_PLOT_SECONDS * SAMPLE_RATE_HZ)
MAX_DISPLAY_POINTS = 6000

STABLE_LINE_COUNT = 3
DEFAULT_SAVE_HOURS = 10 * 24
SAVE_FLUSH_INTERVAL = 1.0
SAVE_DTYPE = np.float32

HEADER_MAGIC = b"SPDT"
HEADER_VERSION = 1

pg.setConfigOptions(antialias=False)
pg.setConfigOptions(useOpenGL=False)


# =========================
# Parser
# =========================
def parse_line(line: str):
    vals = []
    for part in line.replace("\t", ",").split(","):
        part = part.strip()
        if not part:
            continue

        if ":" in part:
            _, part = part.split(":", 1)

        try:
            vals.append(float(part))
        except ValueError:
            return None

    return vals if vals else None


def default_save_path():
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.abspath(f"serial_data_{ts}.bin")


# =========================
# Serial worker
# =========================
def serial_worker(port, baudrate, data_queue, stop_event, save_path=None, max_run_secs=None):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.001)
        time.sleep(2.0)
        ser.reset_input_buffer()
    except Exception as e:
        data_queue.put(("error", f"Serial open failed: {e}"))
        return

    save_file = None
    save_sample_count = 0
    save_header_written = False
    save_start_ns = 0

    if save_path:
        try:
            save_file = open(save_path, "wb")
            save_file.write(struct.pack("<4sBBHQQ", HEADER_MAGIC, HEADER_VERSION, 1, 0, 0, 0))
        except Exception as e:
            data_queue.put(("error", f"Save file open failed: {e}"))
            save_file = None

    t0 = time.perf_counter()
    last_flush = time.perf_counter()

    batch_t = []
    batch_y = []

    good = 0
    bad = 0
    dropped = 0

    stable_n = None
    candidate_n = None
    candidate_count = 0

    last_line = ""

    while not stop_event.is_set():
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        vals = parse_line(line)
        if vals is None:
            bad += 1
            continue

        n = len(vals)

        if stable_n is None:
            if candidate_n == n:
                candidate_count += 1
            else:
                candidate_n = n
                candidate_count = 1

            if candidate_count >= STABLE_LINE_COUNT:
                stable_n = candidate_n
                data_queue.put(("info", f"Detected {stable_n} channels"))
            continue

        if n != stable_n:
            bad += 1
            continue

        if max_run_secs and (time.perf_counter() - t0) >= max_run_secs:
            data_queue.put(("info", "Max run time reached"))
            break

        t = time.perf_counter() - t0

        batch_t.append(t)
        batch_y.append(vals)

        good += 1
        last_line = line

        now = time.perf_counter()
        if len(batch_t) >= BATCH_SIZE or (now - last_flush) >= BATCH_MAX_DELAY:

            try:
                data_queue.put_nowait(("data", batch_t, batch_y, good, bad, dropped, last_line))
            except queue.Full:
                dropped += len(batch_t)

            batch_t = []
            batch_y = []
            last_flush = now

    ser.close()


# =========================
# Port dialog
# =========================
class PortDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Serial Port")

        layout = QtWidgets.QVBoxLayout(self)

        self.port_box = QtWidgets.QComboBox()
        self.baud_box = QtWidgets.QComboBox()
        self.save_edit = QtWidgets.QLineEdit(default_save_path())
        self.save_button = QtWidgets.QPushButton("Browse...")
        self.duration_box = QtWidgets.QDoubleSpinBox()

        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.port_box.addItem(f"{p.device} | {p.description}", p.device)

        self.baud_box.addItems(["9600","115200","460800","921600","1000000","2000000"])
        self.baud_box.setCurrentText(DEFAULT_BAUDRATE)

        self.duration_box.setRange(0.1, DEFAULT_SAVE_HOURS)
        self.duration_box.setValue(DEFAULT_SAVE_HOURS)
        self.duration_box.setSuffix(" h")

        layout.addWidget(QtWidgets.QLabel("Port"))
        layout.addWidget(self.port_box)
        layout.addWidget(QtWidgets.QLabel("Baud"))
        layout.addWidget(self.baud_box)
        layout.addWidget(QtWidgets.QLabel("Save"))
        layout.addWidget(self.save_edit)
        layout.addWidget(self.save_button)

        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def selected(self):
        return (
            self.port_box.currentData(),
            int(self.baud_box.currentText()),
            self.save_edit.text(),
            float(self.duration_box.value())
        )


# =========================
# Main plotter
# =========================
class SerialPlotter(QtWidgets.QMainWindow):
    def __init__(self, port, baudrate, save_path=None, max_run_secs=None):
        super().__init__()

        self.setWindowTitle(f"Serial Plotter - {port}")

        self.save_path = save_path
        self.max_run_secs = max_run_secs
        self.start_time = time.perf_counter()

        self.times = deque(maxlen=MAX_STORED_POINTS)
        self.channels = []
        self.curves = []
        self.checkboxes = []

        self.has_force = False
        self.force = deque(maxlen=MAX_STORED_POINTS)
        self.force_curve = None

        self.last_hz_time = time.perf_counter()
        self.last_sample_count = 0
        self.current_hz = 0.0

        self.data_queue = mp.Queue(maxsize=500)
        self.stop_event = mp.Event()

        self.proc = mp.Process(
            target=serial_worker,
            args=(port, baudrate, self.data_queue, self.stop_event, save_path, max_run_secs),
            daemon=True
        )
        self.proc.start()

        self._build_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_from_queue)
        self.timer.start(GUI_REFRESH_MS)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)

        self.horizon_box = QtWidgets.QDoubleSpinBox()
        self.horizon_box.setRange(0.2, MAX_PLOT_SECONDS)
        self.horizon_box.setValue(20.0)

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.setCheckable(True)

        self.status_label = QtWidgets.QLabel("Waiting...")
        self.hz_label = QtWidgets.QLabel("Hz: --")

        layout.addWidget(self.horizon_box)
        layout.addWidget(self.pause_button)
        layout.addWidget(self.status_label)
        layout.addWidget(self.hz_label)

        self.channel_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(self.channel_layout)

        self.plot_top = pg.PlotWidget()
        self.plot_top.showGrid(x=True, y=True)
        self.plot_top.addLegend()
        self.plot_top.setLabel("bottom", "Time")
        self.plot_top.setLabel("left", "Sensors")

        self.plot_bottom = pg.PlotWidget()
        self.plot_bottom.showGrid(x=True, y=True)
        self.plot_bottom.addLegend()
        self.plot_bottom.setLabel("bottom", "Time")
        self.plot_bottom.setLabel("left", "Force")

        layout.addWidget(self.plot_top)
        layout.addWidget(self.plot_bottom)

        self.setCentralWidget(central)

    def init_channels(self, n):
        self.channels = [deque(maxlen=MAX_STORED_POINTS) for _ in range(n)]

        for i in range(n):
            cb = QtWidgets.QCheckBox(f"T{i}")
            cb.setChecked(True)
            self.channel_layout.addWidget(cb)
            self.checkboxes.append(cb)

            # ONLY CHANGE: assign unique color per sensor
            pen = pg.intColor(i, hues=n)
            curve = self.plot_top.plot(name=f"T{i}", pen=pen)

            self.curves.append(curve)

    def update_from_queue(self):
        if self.pause_button.isChecked():
            return

        while True:
            try:
                msg = self.data_queue.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "error":
                self.status_label.setText(msg[1])
                return

            if msg[0] == "info":
                self.status_label.setText(msg[1])
                continue

            _, batch_t, batch_y, good, bad, dropped, line = msg

            if not batch_y:
                continue

            if not self.channels:
                if "Force" in line:
                    self.has_force = True
                    self.init_channels(len(batch_y[0]) - 1)
                    self.force_curve = self.plot_bottom.plot(name="Force")
                else:
                    self.init_channels(len(batch_y[0]))

            for t, vals in zip(batch_t, batch_y):
                self.times.append(t)

                if self.has_force:
                    self.force.append(vals[-1])
                    vals = vals[:-1]

                for i, v in enumerate(vals):
                    self.channels[i].append(v)

        now = time.perf_counter()
        dt = now - self.last_hz_time

        if dt >= 0.5:
            sample_count = len(self.times)
            self.current_hz = (sample_count - self.last_sample_count) / dt
            self.last_sample_count = sample_count
            self.last_hz_time = now

            self.hz_label.setText(f"Hz: {self.current_hz:.1f}")

        self.redraw()

    def redraw(self):
        if len(self.times) < 2:
            return

        t = np.array(self.times)

        t0 = t[-1] - self.horizon_box.value()
        mask = t >= t0

        t_view = t[mask]

        step = max(1, len(t_view) // MAX_DISPLAY_POINTS) if len(t_view) > MAX_DISPLAY_POINTS else 1
        t_view = t_view[::step]

        for i, c in enumerate(self.curves):
            y = np.array(self.channels[i])
            c.setData(t_view, y[mask][::step])

        self.plot_top.setXRange(t0, t[-1])

        if self.has_force and self.force_curve:
            f = np.array(self.force)
            self.force_curve.setData(t_view, f[mask][::step])
            self.plot_bottom.setXRange(t0, t[-1])

    def closeEvent(self, event):
        self.stop_event.set()
        if self.proc.is_alive():
            self.proc.terminate()
        event.accept()


# =========================
# Main
# =========================
def main():
    mp.set_start_method("spawn", force=True)

    app = QtWidgets.QApplication(sys.argv)

    dlg = PortDialog()
    if dlg.exec_() != QtWidgets.QDialog.Accepted:
        return

    port, baud, save, hrs = dlg.selected()

    win = SerialPlotter(port, baud, save_path=save, max_run_secs=hrs * 3600)
    win.resize(1400, 800)
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()