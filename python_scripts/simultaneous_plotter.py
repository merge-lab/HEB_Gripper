import sys
import os
import time
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

LOG_INTERVAL_SEC = 0.1  # 10 Hz logging


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
# Workers
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
        if vals is None or len(vals) != 7:
            continue

        t = time.perf_counter() - t0

        bt.append(t)
        by.append(vals)

        if len(bt) >= BATCH_SIZE:
            q.put(("pressure", bt, by))
            bt, by = [], []

    ser.close()


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
# Dialog
# =========================
class PortDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Ports + Log File")

        layout = QtWidgets.QVBoxLayout(self)

        ports = list(serial.tools.list_ports.comports())

        self.p1 = QtWidgets.QComboBox()
        self.p2 = QtWidgets.QComboBox()

        for p in ports:
            self.p1.addItem(p.device, p.device)
            self.p2.addItem(p.device, p.device)

        self.file_edit = QtWidgets.QLineEdit("log_output.txt")
        self.file_btn = QtWidgets.QPushButton("Browse")

        self.file_btn.clicked.connect(self.browse)

        layout.addWidget(QtWidgets.QLabel("Pressure Port"))
        layout.addWidget(self.p1)
        layout.addWidget(QtWidgets.QLabel("Load Cell Port"))
        layout.addWidget(self.p2)

        layout.addWidget(QtWidgets.QLabel("Output Log File"))
        layout.addWidget(self.file_edit)
        layout.addWidget(self.file_btn)

        btn = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        btn.accepted.connect(self.accept)
        btn.rejected.connect(self.reject)
        layout.addWidget(btn)

    def browse(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Log File")
        if path:
            self.file_edit.setText(path)

    def selected(self):
        return (
            self.p1.currentData(),
            self.p2.currentData(),
            self.file_edit.text()
        )


# =========================
# Main
# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port, log_path):
        super().__init__()

        self.setWindowTitle("Dual Sensor Plotter")

        self.q = mp.Queue()
        self.stop = mp.Event()

        # buffers
        self.t = deque(maxlen=MAX_STORED_POINTS)
        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)

        self.lt = deque(maxlen=MAX_STORED_POINTS)
        self.lv = deque(maxlen=MAX_STORED_POINTS)

        # logging
        self.log_path = log_path
        self.log_file = open(log_path, "w", buffering=1)

        self.last_log_time = 0.0

        # processes
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

    # -------------------------
    # UI
    # -------------------------
    def _ui(self):
        c = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(c)

        ctrl = QtWidgets.QHBoxLayout()
        v.addLayout(ctrl)

        self.top = pg.PlotWidget(title="Pressure (T1-T6)")
        self.mid = pg.PlotWidget(title="Force")
        self.bot = pg.PlotWidget(title="Load Cell")

        v.addWidget(self.top)
        v.addWidget(self.mid)
        v.addWidget(self.bot)

        self.fps_label = QtWidgets.QLabel("Refresh: -- Hz")
        v.addWidget(self.fps_label)

        self.curves = []

        for i in range(6):
            cb = QtWidgets.QCheckBox(f"T{i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda state, idx=i: self.toggle(idx, state))
            ctrl.addWidget(cb)

            self.curves.append(self.top.plot(pen=pg.intColor(i, 6)))

        self.fcurve = self.mid.plot(pen="y")
        self.lcurve = self.bot.plot(pen="g")

        self.setCentralWidget(c)

    def toggle(self, i, state):
        self.curves[i].setVisible(state == 2)

    # -------------------------
    # UPDATE
    # -------------------------
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
        self.update_fps()
        self.log_data()

    # -------------------------
    # LOGGING (10 Hz)
    # -------------------------
    def log_data(self):
        if not self.t:
            return

        now = self.t[-1]

        if now - self.last_log_time < LOG_INTERVAL_SEC:
            return

        self.last_log_time = now

        t = self.t[-1]
        f = self.force[-1] if self.force else 0.0
        lc = self.lv[-1] if self.lv else 0.0

        vals = [ch[-1] if ch else 0.0 for ch in self.ch]

        line = (
            f"Timestamp: {t:.4f} "
            f"T1: {vals[0]} T2: {vals[1]} T3: {vals[2]} "
            f"T4: {vals[3]} T5: {vals[4]} T6: {vals[5]} "
            f"Force: {f} LoadCell: {lc}\n"
        )

        self.log_file.write(line)

    # -------------------------
    # FPS
    # -------------------------
    def update_fps(self):
        self.fps_count += 1
        now = time.perf_counter()
        dt = now - self.last_fps_t

        if dt >= 0.5:
            self.fps = self.fps_count / dt
            self.fps_count = 0
            self.last_fps_t = now

        self.fps_label.setText(f"Refresh: {self.fps:.1f} Hz")

    # -------------------------
    # PLOT
    # -------------------------
    def redraw(self):
        if len(self.t) < 10:
            return

        t = np.array(self.t)
        t0 = t[-1] - PLOT_WINDOW_SEC
        m = t >= t0

        for i, c in enumerate(self.curves):
            y = np.array(self.ch[i])
            n = min(len(t), len(y))
            c.setData(t[:n][m[:n]], y[:n][m[:n]])

        f = np.array(self.force)
        n = min(len(t), len(f))
        self.fcurve.setData(t[:n][m[:n]], f[:n][m[:n]])

        if self.lt:
            lt = np.array(self.lt)
            lv = np.array(self.lv)
            mm = lt >= t0
            self.lcurve.setData(lt[mm], lv[mm])

    # -------------------------
    def closeEvent(self, e):
        self.stop.set()
        self.log_file.close()
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

    p, l, log_path = dlg.selected()

    w = Plotter(p, l, log_path)
    w.resize(1400, 800)
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()