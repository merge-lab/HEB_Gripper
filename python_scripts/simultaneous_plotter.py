import sys
import time
import queue
import threading
import multiprocessing as mp
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


"""
Logs data from HEB UMI and load cell at 100hz and displays it.

HEB UMI is ussualy on ACM0
Loadcell arduini is ussualy on USB0
"""

PRESSURE_BAUD = 1000000
LOADCELL_BAUD = 115200

GUI_REFRESH_MS = 10
PLOT_WINDOW_SEC = 20
MAX_STORED_POINTS = 200000

LOG_HZ = 100
LOG_DT = 1.0 / LOG_HZ

pg.setConfigOptions(antialias=False)


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
def pressure_worker(port, baud, q, stop):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

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

        q.put(("pressure", vals))

    ser.close()


# =========================
def loadcell_worker(port, baud, q, stop):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

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

        q.put(("loadcell", v))

    ser.close()


# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port):
        super().__init__()
        self.setWindowTitle("Dual Sensor Plotter (stable sync)")

        self.q = mp.Queue()
        self.stop = mp.Event()

        # start time
        self.t0 = time.perf_counter()

        # latest values (atomic-ish in CPython OK)
        self.latest_p = [0.0]*6
        self.latest_f = 0.0
        self.latest_l = 0.0
        self.latest_t = 0.0

        # buffers
        self.t = deque(maxlen=MAX_STORED_POINTS)
        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)
        self.loadcell = deque(maxlen=MAX_STORED_POINTS)

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

        # 🔥 REAL logger thread
        self.log_file = open(f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt", "w")
        self.log_lock = threading.Lock()
        self.log_stop = threading.Event()

        self.logger = threading.Thread(target=self.logger_loop, daemon=True)
        self.logger.start()

    # =========================
    def logger_loop(self):
        # Wait 20 seconds before starting any logging
        start_logging_at = self.t0 + 15.0

        while not self.log_stop.is_set():
            if time.perf_counter() >= start_logging_at:
                break
            time.sleep(0.01)

        next_t = time.perf_counter()

        while not self.log_stop.is_set():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(0.0005)
                continue

            next_t += LOG_DT

            with self.log_lock:
                self.log_file.write(
                    f"Timestamp:{self.latest_t:.4f} "
                    f"T1:{self.latest_p[0]:.3f} T2:{self.latest_p[1]:.3f} T3:{self.latest_p[2]:.3f} "
                    f"T4:{self.latest_p[3]:.3f} T5:{self.latest_p[4]:.3f} T6:{self.latest_p[5]:.3f} "
                    f"Force:{self.latest_f:.3f} "
                    f"LoadCell:{self.latest_l:.3f}\n"
                )

    # ========================= UI =========================
    def _ui(self):
        c = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(c)

        self.top = pg.PlotWidget(title="Pressure")
        self.mid = pg.PlotWidget(title="Force")
        self.bot = pg.PlotWidget(title="LoadCell")

        v.addWidget(self.top)
        v.addWidget(self.mid)
        v.addWidget(self.bot)

        self.curves = []
        ctrl = QtWidgets.QHBoxLayout()

        for i in range(6):
            cb = QtWidgets.QCheckBox(f"T{i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda s, i=i: self.toggle(i, s))
            ctrl.addWidget(cb)

            self.curves.append(self.top.plot(pen=pg.intColor(i, 6)))

        v.addLayout(ctrl)

        self.fcurve = self.mid.plot(pen="y")
        self.lcurve = self.bot.plot(pen="g")

        self.setCentralWidget(c)

    def toggle(self, i, s):
        self.curves[i].setVisible(s == 2)

    # =========================
    def update(self):
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "pressure":
                vals = msg[1]

                self.latest_p = vals[:6]
                self.latest_f = vals[6]
                self.latest_t = time.perf_counter() - self.t0

            elif msg[0] == "loadcell":
                self.latest_l = msg[1]

        # store for plot
        self.t.append(self.latest_t)
        for i in range(6):
            self.ch[i].append(self.latest_p[i])
        self.force.append(self.latest_f)
        self.loadcell.append(self.latest_l)

        self.redraw()

    # =========================
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

        l = np.array(self.loadcell)
        n = min(len(t), len(l))
        self.lcurve.setData(t[:n][m[:n]], l[:n][m[:n]])

    # =========================
    def closeEvent(self, e):
        self.stop.set()
        self.log_stop.set()
        self.p_proc.terminate()
        self.l_proc.terminate()
        self.log_file.close()
        e.accept()


# =========================
def main():
    mp.set_start_method("spawn", force=True)

    app = QtWidgets.QApplication(sys.argv)

    ports = list(serial.tools.list_ports.comports())

    dlg = QtWidgets.QDialog()
    dlg.setWindowTitle("Select Ports")

    layout = QtWidgets.QVBoxLayout(dlg)

    p1 = QtWidgets.QComboBox()
    p2 = QtWidgets.QComboBox()

    for p in ports:
        p1.addItem(p.device, p.device)
        p2.addItem(p.device, p.device)

    layout.addWidget(p1)
    layout.addWidget(p2)

    btn = QtWidgets.QPushButton("Start")
    layout.addWidget(btn)

    def go():
        dlg.accept()

    btn.clicked.connect(go)

    if dlg.exec_() != QtWidgets.QDialog.Accepted:
        return

    w = Plotter(p1.currentData(), p2.currentData())
    w.resize(1400, 800)
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()