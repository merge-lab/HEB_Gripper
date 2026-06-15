import sys
import time
import threading
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

PLOT_WINDOW_SEC = 20
MAX_STORED_POINTS = 200000

LOG_HZ = 100
LOG_DT = 1.0 / LOG_HZ

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
# Shared system state (CRITICAL)
# =========================
class SystemState:
    def __init__(self):
        self.lock = threading.Lock()

        self.t0 = time.perf_counter()

        self.t = 0.0
        self.pressure = [0.0] * 6
        self.force = 0.0
        self.loadcell = 0.0

    def update_pressure(self, vals):
        with self.lock:
            self.pressure = vals[:6]
            self.force = vals[6]
            self.t = time.perf_counter() - self.t0

    def update_loadcell(self, v):
        with self.lock:
            self.loadcell = v

    def snapshot(self):
        with self.lock:
            return {
                "t": self.t,
                "p": self.pressure.copy(),
                "f": self.force,
                "l": self.loadcell
            }


# =========================
# Pressure worker
# =========================
def pressure_worker(port, baud, state, stop):
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

        state.update_pressure(vals)

    ser.close()


# =========================
# Load cell worker
# =========================
def loadcell_worker(port, baud, state, stop):
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

        state.update_loadcell(v)

    ser.close()


# =========================
# Port dialog
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

        layout.addWidget(QtWidgets.QLabel("Pressure Port"))
        layout.addWidget(self.p1)

        layout.addWidget(QtWidgets.QLabel("Load Cell Port"))
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
# Plotter
# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port):
        super().__init__()

        self.setWindowTitle("Dual Sensor Plotter (Snapshot Logger)")

        self.state = SystemState()

        self.stop = mp.Event()

        # buffers
        self.t = deque(maxlen=MAX_STORED_POINTS)
        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)
        self.loadcell = deque(maxlen=MAX_STORED_POINTS)

        # processes
        self.p_proc = mp.Process(
            target=pressure_worker,
            args=(p_port, PRESSURE_BAUD, self.state, self.stop),
            daemon=True
        )

        self.l_proc = mp.Process(
            target=loadcell_worker,
            args=(l_port, LOADCELL_BAUD, self.state, self.stop),
            daemon=True
        )

        self.p_proc.start()
        self.l_proc.start()

        self._ui()

        # timers
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(GUI_REFRESH_MS)

        self.log_timer = time.perf_counter()

        # FPS
        self.last_gui = time.perf_counter()
        self.frames = 0
        self.fps = 0.0

    # =========================
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

        self.fps_label = QtWidgets.QLabel("FPS: --")
        v.addWidget(self.fps_label)

        self.curves = []
        for i in range(6):
            cb = QtWidgets.QCheckBox(f"T{i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda s, i=i: self.toggle(i, s))
            ctrl.addWidget(cb)

            self.curves.append(self.top.plot(pen=pg.intColor(i, 6)))

        self.fcurve = self.mid.plot(pen="y")
        self.lcurve = self.bot.plot(pen="g")

        self.setCentralWidget(c)

    def toggle(self, i, state):
        self.curves[i].setVisible(state == 2)

    # =========================
    # UPDATE LOOP
    # =========================
    def update(self):
        snap = self.state.snapshot()

        t = snap["t"]

        self.t.append(t)

        for i in range(6):
            self.ch[i].append(snap["p"][i])

        self.force.append(snap["f"])
        self.loadcell.append(snap["l"])

        # ----- 100 Hz logging (true fixed rate) -----
        now = time.perf_counter()
        if now - self.log_timer >= LOG_DT:
            self.log_timer += LOG_DT
            self._log_snapshot(snap)

        self.redraw()

        # FPS
        self.frames += 1
        dt = now - self.last_gui
        if dt >= 0.5:
            self.fps = self.frames / dt
            self.frames = 0
            self.last_gui = now
            self.fps_label.setText(f"FPS: {self.fps:.1f}")

    # =========================
    def _log_snapshot(self, s):
        print(
            f"Timestamp:{s['t']:.4f} "
            f"T1:{s['p'][0]:.3f} T2:{s['p'][1]:.3f} T3:{s['p'][2]:.3f} "
            f"T4:{s['p'][3]:.3f} T5:{s['p'][4]:.3f} T6:{s['p'][5]:.3f} "
            f"Force:{s['f']:.3f} "
            f"LoadCell:{s['l']:.3f}"
        )

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