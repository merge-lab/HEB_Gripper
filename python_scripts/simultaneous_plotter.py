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
# Serial parsing
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
# Writer thread (100 Hz logger)
# =========================
class DataWriter(threading.Thread):
    def __init__(self, buffer_ref, path, stop_event):
        super().__init__(daemon=True)
        self.buffer = buffer_ref
        self.path = path
        self.stop_event = stop_event

    def run(self):
        with open(self.path, "w") as f:
            while not self.stop_event.is_set():
                time.sleep(LOG_DT)

                try:
                    entry = self.buffer.get_nowait()
                except queue.Empty:
                    continue

                # unified log line
                f.write(
                    f"Timestamp: {entry['t']:.4f} "
                    f"T1:{entry['t1']:.3f} T2:{entry['t2']:.3f} T3:{entry['t3']:.3f} "
                    f"T4:{entry['t4']:.3f} T5:{entry['t5']:.3f} T6:{entry['t6']:.3f} "
                    f"Force:{entry['force']:.3f} "
                    f"LoadCell:{entry['loadcell']:.3f}\n"
                )


# =========================
# Main plotter
# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port, log_path="log.txt"):
        super().__init__()

        self.setWindowTitle("Dual Sensor Plotter")

        # queues
        self.q = mp.Queue()
        self.stop = mp.Event()

        # writer buffer
        self.log_buffer = queue.Queue(maxsize=10000)
        self.writer_stop = threading.Event()

        self.writer = DataWriter(self.log_buffer, log_path, self.writer_stop)
        self.writer.start()

        # buffers
        self.t = deque(maxlen=MAX_STORED_POINTS)
        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)

        self.lt = deque(maxlen=MAX_STORED_POINTS)
        self.lv = deque(maxlen=MAX_STORED_POINTS)

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

        self.last_gui = time.perf_counter()
        self.fps = 0.0
        self.frames = 0

    # =========================
    def _ui(self):
        c = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(c)

        ctrl = QtWidgets.QHBoxLayout()

        self.checkboxes = []
        self.curves = []

        for i in range(6):
            cb = QtWidgets.QCheckBox(f"T{i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda s, i=i: self.toggle(i, s))
            ctrl.addWidget(cb)

            self.curves.append(self.top_plot().plot(pen=pg.intColor(i, 6)))

        v.addLayout(ctrl)

        self.top = self.top_plot()
        self.mid = pg.PlotWidget(title="Force")
        self.bot = pg.PlotWidget(title="Load Cell")

        self.fcurve = self.mid.plot(pen="y")
        self.lcurve = self.bot.plot(pen="g")

        v.addWidget(self.top)
        v.addWidget(self.mid)
        v.addWidget(self.bot)

        self.fps_label = QtWidgets.QLabel("FPS: --")
        v.addWidget(self.fps_label)

        self.setCentralWidget(c)

    def top_plot(self):
        if hasattr(self, "_top"):
            return self._top
        self._top = pg.PlotWidget(title="Pressure")
        return self._top

    def toggle(self, i, state):
        self.curves[i].setVisible(state == 2)

    # =========================
    def update(self):
        # read serial queue
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "pressure":
                _, t_batch, y_batch = msg

                for t, vals in zip(t_batch, y_batch):
                    self.t.append(t)

                    # push log entry
                    try:
                        self.log_buffer.put_nowait({
                            "t": t,
                            "t1": vals[0],
                            "t2": vals[1],
                            "t3": vals[2],
                            "t4": vals[3],
                            "t5": vals[4],
                            "t6": vals[5],
                            "force": vals[6],
                            "loadcell": float("nan")
                        })
                    except queue.Full:
                        pass

                    self.force.append(vals[6])

                    for i in range(6):
                        self.ch[i].append(vals[i])

            elif msg[0] == "loadcell":
                _, t_batch, y_batch = msg

                for t, v in zip(t_batch, y_batch):
                    self.lt.append(t)
                    self.lv.append(v)

                    try:
                        self.log_buffer.put_nowait({
                            "t": t,
                            "t1": np.nan,
                            "t2": np.nan,
                            "t3": np.nan,
                            "t4": np.nan,
                            "t5": np.nan,
                            "t6": np.nan,
                            "force": np.nan,
                            "loadcell": v
                        })
                    except queue.Full:
                        pass

        self.redraw()

        self.frames += 1
        now = time.perf_counter()
        dt = now - self.last_gui

        if dt >= 0.5:
            self.fps = self.frames / dt
            self.frames = 0
            self.last_gui = now
            self.fps_label.setText(f"FPS: {self.fps:.1f}")

    # =========================
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

        if len(self.lt) > 10:
            lt = np.array(self.lt)
            lv = np.array(self.lv)
            m = lt >= t0
            self.lcurve.setData(lt[m], lv[m])

    # =========================
    def closeEvent(self, e):
        self.stop.set()
        self.writer_stop.set()

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