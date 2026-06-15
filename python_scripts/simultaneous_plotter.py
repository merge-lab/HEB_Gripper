import sys
import os
import time
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
# Settings
# =========================
PRESSURE_BAUD = 1000000
LOADCELL_BAUD = 115200

BATCH_SIZE = 5
GUI_REFRESH_MS = 10

MAX_STORED_POINTS = 200000
PLOT_WINDOW_SEC = 20

pg.setConfigOptions(antialias=False)


# =========================
# Parser (unused for loadcell now)
# =========================
def parse_line(line: str):
    vals = []
    for part in line.replace("\t", ",").split(","):
        part = part.strip()
        if not part:
            continue
        if ":" in part:
            part = part.split(":", 1)[1]
        try:
            vals.append(float(part))
        except:
            return None
    return vals


# =========================
# Pressure worker
# =========================
def pressure_worker(port, baud, q, stop):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

    t0 = time.perf_counter()
    bt, by = [], []

    while not stop.is_set():
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode(errors="ignore").strip()
        if not line or line.startswith("LC:"):
            continue

        vals = parse_line(line)
        if vals is None:
            continue

        # must be full packet: T1..T6 + Force
        if len(vals) != 7:
            continue

        t = time.perf_counter() - t0

        bt.append(t)
        by.append(vals)

        if len(bt) >= BATCH_SIZE:
            q.put(("pressure", bt, by))
            bt, by = [], []

    ser.close()


# =========================
# Load cell worker
# =========================
def loadcell_worker(port, baud, q, stop):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

    t0 = time.perf_counter()
    bt, by = [], []

    while not stop.is_set():
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode(errors="ignore").strip()
        if not line.startswith("LC:"):
            continue

        try:
            v = float(line.split(":")[1])
        except:
            continue

        t = time.perf_counter() - t0

        bt.append(t)
        by.append(v)

        if len(bt) >= BATCH_SIZE:
            q.put(("loadcell", bt, by))
            bt, by = [], []

    ser.close()


# =========================
# UI
# =========================
class PortDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Ports")

        layout = QtWidgets.QVBoxLayout(self)

        ports = list(serial.tools.list_ports.comports())

        self.p1 = QtWidgets.QComboBox()
        self.p2 = QtWidgets.QComboBox()

        for p in ports:
            self.p1.addItem(p.device, p.device)
            self.p2.addItem(p.device, p.device)

        layout.addWidget(QtWidgets.QLabel("Pressure"))
        layout.addWidget(self.p1)
        layout.addWidget(QtWidgets.QLabel("Load Cell"))
        layout.addWidget(self.p2)

        btn = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        btn.accepted.connect(self.accept)
        btn.rejected.connect(self.reject)
        layout.addWidget(btn)

    def selected(self):
        return self.p1.currentData(), self.p2.currentData()


# =========================
# Main plotter
# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port):
        super().__init__()

        self.setWindowTitle("Dual Sensor Plotter")

        self.q = mp.Queue()
        self.stop = mp.Event()

        # buffers (LOCKED LENGTH DESIGN)
        self.t = deque(maxlen=MAX_STORED_POINTS)

        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)

        self.lt = deque(maxlen=MAX_STORED_POINTS)
        self.lv = deque(maxlen=MAX_STORED_POINTS)

        self.p_proc = mp.Process(target=pressure_worker,
                                 args=(p_port, PRESSURE_BAUD, self.q, self.stop),
                                 daemon=True)

        self.l_proc = mp.Process(target=loadcell_worker,
                                 args=(l_port, LOADCELL_BAUD, self.q, self.stop),
                                 daemon=True)

        self.p_proc.start()
        self.l_proc.start()

        self._ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(GUI_REFRESH_MS)

        self.last_fps_t = time.perf_counter()
        self.fps = 0.0
        self.fps_count = 0

    def _ui(self):
        c = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(c)

        self.top = pg.PlotWidget(title="Pressure (T1-T6)")
        self.mid = pg.PlotWidget(title="Force (from pressure stream)")
        self.bot = pg.PlotWidget(title="Load Cell")

        self.curves = [self.top.plot(pen=pg.intColor(i, 6)) for i in range(6)]
        self.fcurve = self.mid.plot(pen="y")
        self.lcurve = self.bot.plot(pen="g")
        self.fps_label = QtWidgets.QLabel("Refresh: -- Hz")

        v.addWidget(self.top)
        v.addWidget(self.mid)
        v.addWidget(self.bot)
        v.addWidget(self.fps_label)

        self.setCentralWidget(c)

    # =========================
    # SAFE UPDATE
    # =========================
    def update(self):
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "pressure":
                _, t_batch, y_batch = msg

                for ti, vals in zip(t_batch, y_batch):
                    # atomic packet: 6 + force
                    self.t.append(ti)

                    self.force.append(vals[6])

                    for i in range(6):
                        self.ch[i].append(vals[i])

            elif msg[0] == "loadcell":
                _, t_batch, y_batch = msg

                for ti, v in zip(t_batch, y_batch):
                    self.lt.append(ti)
                    self.lv.append(v)

        self.redraw()
        self.fps_count += 1

        now = time.perf_counter()
        dt = now - self.last_fps_t

        if dt >= 0.5:  # update twice per second
            self.fps = self.fps_count / dt
            self.fps_count = 0
            self.last_fps_t = now

        self.fps_label.setText(f"Refresh: {self.fps:.1f} Hz")

    # =========================
    # SAFE REDRAW (NO MASK BUGS)
    # =========================
    def redraw(self):
        if len(self.t) < 10:
            return

        t = np.array(self.t)
        t0 = t[-1] - PLOT_WINDOW_SEC

        idx = t >= t0

        t_view = t[idx]

        # pressure channels
        for i, c in enumerate(self.curves):
            y = np.array(self.ch[i])
            n = min(len(t), len(y))
            c.setData(t[:n][idx[:n]], y[:n][idx[:n]])

        # force
        f = np.array(self.force)
        n = min(len(t), len(f))
        self.fcurve.setData(t[:n][idx[:n]], f[:n][idx[:n]])

        # load cell
        if len(self.lt) > 10:
            lt = np.array(self.lt)
            lv = np.array(self.lv)

            m = lt >= t0
            self.lcurve.setData(lt[m], lv[m])

    def closeEvent(self, e):
        self.stop.set()
        self.p_proc.terminate()
        self.l_proc.terminate()
        e.accept()


# =========================
# MAIN
# =========================
def main():
    mp.set_start_method("spawn", force=True)

    app = QtWidgets.QApplication(sys.argv)

    dlg = PortDialog()
    if dlg.exec_() != QtWidgets.QDialog.Accepted:
        return

    p, l = dlg.selected()

    w = Plotter(p, l)
    w.resize(1400, 800)
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


"""
Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
"""