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

"""
Timestamp:2.4345 T1:0.051 T2:0.393 T3:0.018 T4:0.628 T5:0.612 T6:0.796 Force:11.870 LoadCell:nan
Timestamp:2.4354 T1:0.042 T2:0.376 T3:0.032 T4:0.652 T5:0.609 T6:0.783 Force:11.740 LoadCell:nan
Timestamp:2.4363 T1:0.053 T2:0.373 T3:0.032 T4:0.631 T5:0.609 T6:0.799 Force:11.740 LoadCell:nan
Timestamp:2.4373 T1:0.042 T2:0.378 T3:0.042 T4:0.635 T5:0.609 T6:0.792 Force:11.740 LoadCell:nan
Timestamp:2.4384 T1:0.072 T2:0.373 T3:0.040 T4:0.648 T5:0.629 T6:0.784 Force:11.740 LoadCell:nan
Timestamp:2.4394 T1:0.048 T2:0.378 T3:0.042 T4:0.642 T5:0.588 T6:0.796 Force:11.740 LoadCell:nan
Timestamp:2.4403 T1:0.055 T2:0.366 T3:0.051 T4:0.644 T5:0.607 T6:0.795 Force:11.740 LoadCell:nan
Timestamp:2.4415 T1:0.045 T2:0.373 T3:0.016 T4:0.633 T5:0.607 T6:0.791 Force:11.740 LoadCell:nan
Timestamp:2.4424 T1:0.064 T2:0.384 T3:0.045 T4:0.633 T5:0.604 T6:0.792 Force:11.740 LoadCell:nan
Timestamp:2.4435 T1:0.051 T2:0.376 T3:0.037 T4:0.604 T5:0.601 T6:0.776 Force:11.740 LoadCell:nan
"""

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
# Port + file dialog
# =========================
class PortDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Ports and Log File")

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

        # ---- LOG FILE PICKER ----
        self.log_path = QtWidgets.QLineEdit("log.txt")
        self.browse = QtWidgets.QPushButton("Browse")
        self.browse.clicked.connect(self.pick_file)

        layout.addWidget(QtWidgets.QLabel("Log File"))
        layout.addWidget(self.log_path)
        layout.addWidget(self.browse)

        btn = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        btn.accepted.connect(self.accept)
        btn.rejected.connect(self.reject)
        layout.addWidget(btn)

    def pick_file(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Select Log File",
            "",
            "Text Files (*.txt);;All Files (*)"
        )
        if path:
            self.log_path.setText(path)

    def selected(self):
        return (
            self.p1.currentData(),
            self.p2.currentData(),
            self.log_path.text()
        )


# =========================
# Writer thread (100 Hz)
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
                    e = self.buffer.get_nowait()
                except queue.Empty:
                    continue

                f.write(
                    f"Timestamp:{e['t']:.4f} "
                    f"T1:{e['t1']:.3f} T2:{e['t2']:.3f} T3:{e['t3']:.3f} "
                    f"T4:{e['t4']:.3f} T5:{e['t5']:.3f} T6:{e['t6']:.3f} "
                    f"Force:{e['force']:.3f} "
                    f"LoadCell:{e['loadcell']:.3f}\n"
                )


# =========================
# Main plotter
# =========================
class Plotter(QtWidgets.QMainWindow):
    def __init__(self, p_port, l_port, log_path):
        super().__init__()

        self.setWindowTitle("Dual Sensor Plotter")

        self.q = mp.Queue()
        self.stop = mp.Event()

        # logging buffer
        self.log_buffer = queue.Queue(maxsize=20000)
        self.writer_stop = threading.Event()

        self.writer = DataWriter(self.log_buffer, log_path, self.writer_stop)
        self.writer.start()

        # buffers
        self.t = deque(maxlen=MAX_STORED_POINTS)
        self.ch = [deque(maxlen=MAX_STORED_POINTS) for _ in range(6)]
        self.force = deque(maxlen=MAX_STORED_POINTS)

        self.lt = deque(maxlen=MAX_STORED_POINTS)
        self.lv = deque(maxlen=MAX_STORED_POINTS)

        # workers
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
        self.frames = 0
        self.fps = 0.0

    # =========================
    def _ui(self):
        c = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(c)

        ctrl = QtWidgets.QHBoxLayout()
        v.addLayout(ctrl)

        self.top = pg.PlotWidget(title="Pressure (T1-T6)")
        self.mid = pg.PlotWidget(title="Force (from pressure stream)")
        self.bot = pg.PlotWidget(title="Load Cell")

        v.addWidget(self.top)
        v.addWidget(self.mid)
        v.addWidget(self.bot)

        self.fps_label = QtWidgets.QLabel("Refresh: -- Hz")
        v.addWidget(self.fps_label)

        self.checkboxes = []
        self.curves = []

        for i in range(6):
            cb = QtWidgets.QCheckBox(f"T{i+1}")
            cb.setChecked(True)
            cb.stateChanged.connect(
                lambda state, idx=i: self.toggle_channel(idx, state)
            )
            ctrl.addWidget(cb)
            self.checkboxes.append(cb)

            # FIX: plot on EXISTING widget, not a temporary one
            curve = self.top.plot(pen=pg.intColor(i, 6))
            self.curves.append(curve)

        self.fcurve = self.mid.plot(pen="y")
        self.lcurve = self.bot.plot(pen="g")

        self.setCentralWidget(c)

        
    def toggle(self, i, state):
        self.curves[i].setVisible(state == 2)

    # =========================
    def update(self):
        while True:
            try:
                msg = self.q.get_nowait()
            except queue.Empty:
                break

            if msg[0] == "pressure":
                _, tb, yb = msg

                for t, v in zip(tb, yb):
                    self.t.append(t)

                    try:
                        self.log_buffer.put_nowait({
                            "t": t,
                            "t1": v[0],
                            "t2": v[1],
                            "t3": v[2],
                            "t4": v[3],
                            "t5": v[4],
                            "t6": v[5],
                            "force": v[6],
                            "loadcell": np.nan
                        })
                    except queue.Full:
                        pass

                    self.force.append(v[6])
                    for i in range(6):
                        self.ch[i].append(v[i])

            elif msg[0] == "loadcell":
                _, tb, yb = msg

                for t, v in zip(tb, yb):
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

    p, l, log_path = dlg.selected()

    w = Plotter(p, l, log_path)
    w.resize(1400, 800)
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()