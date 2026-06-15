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

LOG_HZ = 10  # log at 10 Hz


pg.setConfigOptions(antialias=False)


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
            part = part.split(":", 1)[1]
        try:
            vals.append(float(part))
        except:
            return None
    return vals


# =========================
# Pressure worker
# =========================
def pressure_worker(port, baud, q, stop, start_event, t0_shared):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

    start_event.wait()
    t0 = t0_shared.value

    bt, by = [], []

    while not stop.is_set():
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode(errors="ignore").strip()
        if not line or line.startswith("LC:"):
            continue

        vals = parse_line(line)
        if vals is None or len(vals) != 7:
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
def loadcell_worker(port, baud, q, stop, start_event, t0_shared):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

    start_event.wait()
    t0 = t0_shared.value

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
        self.setWindowTitle("Select Ports + Output")

        layout = QtWidgets.QVBoxLayout(self)

        ports = list(serial.tools.list_ports.comports())

        self.p1 = QtWidgets.QComboBox()
        self.p2 = QtWidgets.QComboBox()

        for p in ports:
            self.p1.addItem(p.device, p.device)
            self.p2.addItem(p.device, p.device)

        self.out_path = QtWidgets.QLineEdit(
            os.path.abspath(f"capture_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        )

        layout.addWidget(QtWidgets.QLabel("Pressure Port"))
        layout.addWidget(self.p1)

        layout.addWidget(QtWidgets.QLabel("Load Cell Port"))
        layout.addWidget(self.p2)

        layout.addWidget(QtWidgets.QLabel("Output File"))
        layout.addWidget(self.out_path)

        btn = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        btn.accepted.connect(self.accept)
        btn.rejected.connect(self.reject)
        layout.addWidget(btn)

    def selected(self):
        return (
            self.p1.currentData(),
            self.p2.currentData(),
            self.out_path.text()
        )


# =========================
# MAIN PLOTTER
# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port, out_path):
        super().__init__()

        self.setWindowTitle("Dual Sensor Plotter (Fast Mode)")

        # sync
        self.q = mp.Queue()
        self.stop = mp.Event()
        self.start_event = mp.Event()
        self.t0_shared = mp.Value('d', time.perf_counter())

        # buffers
        self.t = deque(maxlen=MAX_STORED_POINTS)
        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)

        self.lt = deque(maxlen=MAX_STORED_POINTS)
        self.lv = deque(maxlen=MAX_STORED_POINTS)

        self.out_path = out_path

        self.last_log = time.perf_counter()
        self.log_buffer = []

        # processes
        self.p_proc = mp.Process(
            target=pressure_worker,
            args=(p_port, PRESSURE_BAUD, self.q, self.stop, self.start_event, self.t0_shared),
            daemon=True
        )

        self.l_proc = mp.Process(
            target=loadcell_worker,
            args=(l_port, LOADCELL_BAUD, self.q, self.stop, self.start_event, self.t0_shared),
            daemon=True
        )

        self.p_proc.start()
        self.l_proc.start()

        time.sleep(1.0)
        self.t0_shared.value = time.perf_counter()
        self.start_event.set()

        self.file = open(self.out_path, "w")

        self._ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(GUI_REFRESH_MS)

        self.fps_t = time.perf_counter()
        self.fps = 0
        self.fps_count = 0

    # ---------------- UI
    def _ui(self):
        c = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(c)

        ctrl = QtWidgets.QHBoxLayout()

        self.curves = []
        self.checkboxes = []

        self.top = pg.PlotWidget(title="Pressure (T1-T6)")
        self.mid = pg.PlotWidget(title="Force")
        self.bot = pg.PlotWidget(title="Load Cell")

        for i in range(6):
            cb = QtWidgets.QCheckBox(f"T{i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda s, i=i: self.curves[i].setVisible(s == 2))
            ctrl.addWidget(cb)

            self.curves.append(self.top.plot(pen=pg.intColor(i, 6)))

        self.fcurve = self.mid.plot(pen="y")
        self.lcurve = self.bot.plot(pen="g")

        self.fps_label = QtWidgets.QLabel("Refresh: -- Hz")

        v.addLayout(ctrl)
        v.addWidget(self.top)
        v.addWidget(self.mid)
        v.addWidget(self.bot)
        v.addWidget(self.fps_label)

        self.setCentralWidget(c)

    # ---------------- UPDATE
    def update(self):
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "pressure":
                _, t_batch, y_batch = msg

                for ti, vals in zip(t_batch, y_batch):
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
        self.fps_update()
        self.log()

    # ---------------- FPS
    def fps_update(self):
        self.fps_count += 1
        now = time.perf_counter()

        if now - self.fps_t > 0.5:
            self.fps = self.fps_count / (now - self.fps_t)
            self.fps_count = 0
            self.fps_t = now

        self.fps_label.setText(f"Refresh: {self.fps:.1f} Hz")

    # ---------------- LOG (FAST)
    def log(self):
        now = time.perf_counter()
        if now - self.last_log < 1.0 / LOG_HZ:
            return

        self.last_log = now

        if not self.t:
            return

        i = -1

        line = (
            f"Timestamp: {self.t[i]:.4f} "
            + " ".join([f"T{j+1}: {self.ch[j][i]:.3f}" for j in range(6)])
            + f" Force: {self.force[i]:.3f} "
            + f"LoadCell: {self.lv[i] if self.lv else 0.0:.3f}"
        )

        self.log_buffer.append(line)

        if len(self.log_buffer) > 50:
            self.file.write("\n".join(self.log_buffer) + "\n")
            self.log_buffer.clear()

    # ---------------- REDRAW (FAST PATH)
    def redraw(self):
        if len(self.t) < 10:
            return

        t = np.array(self.t)
        t0 = t[-1] - PLOT_WINDOW_SEC
        idx = t >= t0

        for i, c in enumerate(self.curves):
            y = np.array(self.ch[i])
            n = min(len(t), len(y))
            c.setData(t[:n][idx[:n]], y[:n][idx[:n]])

        f = np.array(self.force)
        n = min(len(t), len(f))
        self.fcurve.setData(t[:n][idx[:n]], f[:n][idx[:n]])

        if len(self.lt) > 2:
            lt = np.array(self.lt)
            lv = np.array(self.lv)
            m = lt >= t0
            self.lcurve.setData(lt[m], lv[m])

    # ---------------- CLOSE
    def closeEvent(self, e):
        self.stop.set()
        self.p_proc.terminate()
        self.l_proc.terminate()

        if self.log_buffer:
            self.file.write("\n".join(self.log_buffer) + "\n")

        self.file.close()
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

    p, l, out = dlg.selected()

    w = Plotter(p, l, out)
    w.resize(1400, 800)
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()