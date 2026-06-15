import sys
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

LOG_INTERVAL_SEC = 0.1

pg.setConfigOptions(antialias=False)


# =========================
# Parsers
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
# WORKERS (NO TIMESTAMPS ANYMORE)
# =========================
def pressure_worker(port, baud, q, stop):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

    bt, by = [], []
    first_packet = False

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

        bt.append(True)
        by.append(vals)

        # 🔴 KEY FIX: flush immediately on first valid packet
        if not first_packet:
            q.put(("pressure", [vals]))
            first_packet = True
            bt, by = [], []
            continue

        if len(bt) >= BATCH_SIZE:
            q.put(("pressure", by))
            bt, by = [], []

    ser.close()


def loadcell_worker(port, baud, q, stop):
    ser = serial.Serial(port, baud, timeout=0.001)
    time.sleep(2)
    ser.reset_input_buffer()

    bt, by = [], []
    first_packet = False

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

        bt.append(True)
        by.append(v)

        # 🔴 KEY FIX: immediate first sample
        if not first_packet:
            q.put(("loadcell", [v]))
            first_packet = True
            bt, by = [], []
            continue

        if len(bt) >= BATCH_SIZE:
            q.put(("loadcell", by))
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

        self.file_edit = QtWidgets.QLineEdit("log.txt")

        layout.addWidget(QtWidgets.QLabel("Pressure"))
        layout.addWidget(self.p1)
        layout.addWidget(QtWidgets.QLabel("Load Cell"))
        layout.addWidget(self.p2)
        layout.addWidget(QtWidgets.QLabel("Log File"))
        layout.addWidget(self.file_edit)

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
            self.file_edit.text()
        )


# =========================
# MAIN
# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port, log_path):
        super().__init__()

        self.setWindowTitle("Synchronized Dual Stream Plotter")

        self.q = mp.Queue()
        self.stop = mp.Event()

        # unified time base (HOST CLOCK)
        self.t = deque(maxlen=MAX_STORED_POINTS)

        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)

        self.lt = deque(maxlen=MAX_STORED_POINTS)
        self.lv = deque(maxlen=MAX_STORED_POINTS)

        self.log_file = open(log_path, "w")

        self.last_log = 0.0

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

        self.last_fps = time.perf_counter()
        self.fps = 0

    # =========================
    def _ui(self):
        c = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(c)

        ctrl = QtWidgets.QHBoxLayout()

        self.curves = []
        self.top = pg.PlotWidget(title="Pressure")
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

    # =========================
    def update(self):
        host_time = time.perf_counter()

        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "pressure":
                _, batch = msg
                for vals in batch:
                    self.t.append(host_time)

                    self.force.append(vals[6])
                    for i in range(6):
                        self.ch[i].append(vals[i])

            elif msg[0] == "loadcell":
                _, batch = msg
                for v in batch:
                    self.lt.append(host_time)
                    self.lv.append(v)

        self.redraw()
        self.update_fps()
        self.log(host_time)

    # =========================
    def log(self, t):
        if len(self.t) == 0:
            return

        if t - self.last_log < LOG_INTERVAL_SEC:
            return

        self.last_log = t

        line = (
            f"Timestamp: {t:.4f} "
            f"T1:{self.ch[0][-1] if self.ch[0] else 0} "
            f"T2:{self.ch[1][-1] if self.ch[1] else 0} "
            f"T3:{self.ch[2][-1] if self.ch[2] else 0} "
            f"T4:{self.ch[3][-1] if self.ch[3] else 0} "
            f"T5:{self.ch[4][-1] if self.ch[4] else 0} "
            f"T6:{self.ch[5][-1] if self.ch[5] else 0} "
            f"Force:{self.force[-1] if self.force else 0} "
            f"LoadCell:{self.lv[-1] if self.lv else 0}\n"
        )

        self.log_file.write(line)

    # =========================
    def update_fps(self):
        self.fps += 1
        now = time.perf_counter()

        if now - self.last_fps > 0.5:
            self.fps_label.setText(f"Refresh: {self.fps/(now-self.last_fps):.1f} Hz")
            self.fps = 0
            self.last_fps = now

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

        if len(self.lt) > 10:
            lt = np.array(self.lt)
            lv = np.array(self.lv)
            mm = lt >= t0
            self.lcurve.setData(lt[mm], lv[mm])

    # =========================
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
    if not dlg.exec_():
        return

    p, l, log_path = dlg.selected()

    w = Plotter(p, l, log_path)
    w.resize(1400, 800)
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()