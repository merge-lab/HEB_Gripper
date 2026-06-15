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

PRESSURE_BAUD = 1000000
LOADCELL_BAUD = 115200

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
# Serial workers
# =========================
def pressure_worker(port, baudrate, data_queue, stop_event):
    ser = serial.Serial(port, baudrate, timeout=0.001)
    time.sleep(2.0)
    ser.reset_input_buffer()

    t0 = time.perf_counter()
    batch_t, batch_y = [], []

    while not stop_event.is_set():
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        if line.startswith("LC:"):
            continue  # ignore wrong stream

        vals = parse_line(line)
        if vals is None:
            continue

        t = time.perf_counter() - t0

        batch_t.append(t)
        batch_y.append(vals)

        if len(batch_t) >= BATCH_SIZE:
            data_queue.put(("pressure", batch_t, batch_y, line))
            batch_t, batch_y = [], []

    ser.close()


def loadcell_worker(port, baudrate, data_queue, stop_event):
    ser = serial.Serial(port, baudrate, timeout=0.001)
    time.sleep(2.0)
    ser.reset_input_buffer()

    t0 = time.perf_counter()
    batch_t, batch_y = [], []

    while not stop_event.is_set():
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        if not line.startswith("LC:"):
            continue

        try:
            val = float(line.split(":")[1])
        except:
            continue

        t = time.perf_counter() - t0

        batch_t.append(t)
        batch_y.append(val)

        if len(batch_t) >= BATCH_SIZE:
            data_queue.put(("loadcell", batch_t, batch_y))
            batch_t, batch_y = [], []

    ser.close()


# =========================
# Port dialog
# =========================
class PortDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Devices")

        layout = QtWidgets.QVBoxLayout(self)

        ports = list(serial.tools.list_ports.comports())

        self.port1 = QtWidgets.QComboBox()
        self.port2 = QtWidgets.QComboBox()

        for p in ports:
            self.port1.addItem(f"{p.device} | {p.description}", p.device)
            self.port2.addItem(f"{p.device} | {p.description}", p.device)

        layout.addWidget(QtWidgets.QLabel("Pressure Sensor Port"))
        layout.addWidget(self.port1)

        layout.addWidget(QtWidgets.QLabel("Load Cell Port"))
        layout.addWidget(self.port2)

        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def selected(self):
        return (
            self.port1.currentData(),
            self.port2.currentData()
        )


# =========================
# Main plotter
# =========================
class SerialPlotter(QtWidgets.QMainWindow):
    def __init__(self, port_pressure, port_loadcell):
        super().__init__()

        self.setWindowTitle("Dual Serial Plotter")

        self.times = deque(maxlen=MAX_STORED_POINTS)
        self.channels = []
        self.curves = []

        self.force = deque(maxlen=MAX_STORED_POINTS)
        self.force_curve = None

        self.load_times = deque(maxlen=MAX_STORED_POINTS)
        self.load_vals = deque(maxlen=MAX_STORED_POINTS)
        self.load_curve = None

        self.queue = mp.Queue()
        self.stop_event = mp.Event()

        self.proc1 = mp.Process(
            target=pressure_worker,
            args=(port_pressure, PRESSURE_BAUD, self.queue, self.stop_event),
            daemon=True
        )

        self.proc2 = mp.Process(
            target=loadcell_worker,
            args=(port_loadcell, LOADCELL_BAUD, self.queue, self.stop_event),
            daemon=True
        )

        self.proc1.start()
        self.proc2.start()

        self._build_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(GUI_REFRESH_MS)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)

        self.plot_top = pg.PlotWidget()
        self.plot_top.setLabel("bottom", "Time")
        self.plot_top.setLabel("left", "Sensors")

        self.plot_force = pg.PlotWidget()
        self.plot_force.setLabel("bottom", "Time")
        self.plot_force.setLabel("left", "Force")

        self.plot_load = pg.PlotWidget()
        self.plot_load.setLabel("bottom", "Time")
        self.plot_load.setLabel("left", "Load Cell")

        layout.addWidget(self.plot_top)
        layout.addWidget(self.plot_force)
        layout.addWidget(self.plot_load)

        self.setCentralWidget(central)

    def init_channels(self, n):
        self.channels = [deque(maxlen=MAX_STORED_POINTS) for _ in range(n)]

        for i in range(n):
            curve = self.plot_top.plot(pen=pg.intColor(i, hues=n))
            self.curves.append(curve)

        self.force_curve = self.plot_force.plot(pen="y")
        self.load_curve = self.plot_load.plot(pen="g")

    def update(self):
        while True:
            try:
                msg = self.queue.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "pressure":
                _, t, y, line = msg

                if not self.channels:
                    self.init_channels(6)

                for ti, vals in zip(t, y):
                    self.times.append(ti)

                    # always assume format: T1..T6 + Force
                    if len(vals) < 7:
                        continue  # skip bad packet entirely

                    sensor_vals = vals[:6]
                    force_val = vals[6]

                    self.force.append(force_val)
                    
                    for i, v in enumerate(sensor_vals):
                        if i < len(self.channels):
                            self.channels[i].append(v)

            elif msg[0] == "loadcell":
                _, t, y = msg
                for ti, v in zip(t, y):
                    self.load_times.append(ti)
                    self.load_vals.append(v)

        self.redraw()

    def redraw(self):
        if len(self.times) < 2:
            return

        t = np.array(self.times)
        t0 = t[-1] - 20
        mask = t >= t0

        for i, c in enumerate(self.curves):
            y = np.array(self.channels[i])
            c.setData(t[mask], y[mask])

        self.force_curve.setData(t[mask], np.array(self.force)[mask])

        if len(self.load_times) > 2:
            lt = np.array(self.load_times)
            lv = np.array(self.load_vals)
            m2 = lt >= t0
            self.load_curve.setData(lt[m2], lv[m2])

    def closeEvent(self, event):
        self.stop_event.set()
        self.proc1.terminate()
        self.proc2.terminate()
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

    port1, port2 = dlg.selected()

    win = SerialPlotter(port1, port2)
    win.resize(1400, 800)
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


"""
IndexError: boolean index did not match indexed array along axis 0; size of axis is 35373 but size of corresponding boolean axis is 35375
Traceback (most recent call last):
  File "/home/manavkaronde/projects/HEB_Gripper/python_scripts/simultaneous_plotter.py", line 298, in update
    self.redraw()
  File "/home/manavkaronde/projects/HEB_Gripper/python_scripts/simultaneous_plotter.py", line 312, in redraw
    self.force_curve.setData(t[mask], np.array(self.force)[mask])
                                      ~~~~~~~~~~~~~~~~~~~~^^^^^^
IndexError: boolean index did not match indexed array along axis 0; size of axis is 35373 but size of corresponding boolean axis is 35375
Traceback (most recent call last):
  File "/home/manavkaronde/projects/HEB_Gripper/python_scripts/simultaneous_plotter.py", line 298, in update
    self.redraw()
  File "/home/manavkaronde/projects/HEB_Gripper/python_scripts/simultaneous_plotter.py", line 312, in redraw
    self.force_curve.setData(t[mask], np.array(self.force)[mask])
                                      ~~~~~~~~~~~~~~~~~~~~^^^^^^
"""