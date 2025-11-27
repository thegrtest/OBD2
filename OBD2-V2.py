import sys
import time
import csv
import threading
from queue import Queue

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt

# Third-party libs
try:
    import obd
except ImportError:
    print("The 'obd' (python-OBD) package is not installed. Run: pip install obd")
    sys.exit(1)

try:
    from serial.tools import list_ports
except ImportError:
    print("The 'pyserial' package is not installed. Run: pip install pyserial")
    sys.exit(1)

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


class OBDMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("OBD-II Interface - 2015 Camaro")
        self.resize(1300, 800)

        # Dark theme
        self._setup_dark_theme()

        # OBD state
        self.connection = None
        self.polling = False
        self.polling_thread = None

        # Queues for thread-safe comms
        self.log_queue = Queue()
        self.data_queue = Queue()

        # CSV logging
        self.log_to_csv = True
        self.csv_file = None
        self.csv_writer = None
        self.csv_log_labels = []
        self.session_start_time = None

        # Data history for plotting: {label: {"t": [], "v": []}}
        self.data_history = {}

        # Broad set of PIDs – unsupported ones will just show N/A
        self.available_commands = {
            # Core driving data
            "Engine RPM":                obd.commands.RPM,
            "Vehicle Speed":             obd.commands.SPEED,
            "Coolant Temp":              obd.commands.COOLANT_TEMP,
            "Engine Load":               obd.commands.ENGINE_LOAD,
            "Throttle Position":         obd.commands.THROTTLE_POS,
            "Fuel Level":                obd.commands.FUEL_LEVEL,
            "Run Time":                  obd.commands.RUN_TIME,

            # Fuel trims / mixture
            "Short Fuel Trim Bank 1":    obd.commands.SHORT_FUEL_TRIM_1,
            "Long Fuel Trim Bank 1":     obd.commands.LONG_FUEL_TRIM_1,
            "Short Fuel Trim Bank 2":    obd.commands.SHORT_FUEL_TRIM_2,
            "Long Fuel Trim Bank 2":     obd.commands.LONG_FUEL_TRIM_2,

            # Airflow / manifold / timing
            "Intake Manifold Pressure":  obd.commands.INTAKE_PRESSURE,
            "Intake Air Temp":           obd.commands.INTAKE_TEMP,
            "MAF":                       obd.commands.MAF,
            "Timing Advance":            obd.commands.TIMING_ADVANCE,
            "Barometric Pressure":       obd.commands.BAROMETRIC_PRESSURE,

            # O2/ Lambda basic voltages
            "O2 B1S1 Voltage":           obd.commands.O2_B1S1,
            "O2 B1S2 Voltage":           obd.commands.O2_B1S2,
            "O2 B2S1 Voltage":           obd.commands.O2_B2S1,
            "O2 B2S2 Voltage":           obd.commands.O2_B2S2,

            # Misc
            "Distance w/ MIL On":        obd.commands.DISTANCE_W_MIL,
            "Distance Since DTC Clear":  obd.commands.DISTANCE_SINCE_DTC_CLEAR,
            "Control Module Voltage":    obd.commands.CONTROL_MODULE_VOLTAGE,
            "Oil Temp":                  obd.commands.OIL_TEMP,
            "Fuel Type":                 obd.commands.FUEL_TYPE,
        }

        self._build_ui()

        # Timer to process log/data queues
        self.queue_timer = QtCore.QTimer(self)
        self.queue_timer.timeout.connect(self._process_queues)
        self.queue_timer.start(100)

    # ------------------------------------------------------------------
    # Styling / theme
    # ------------------------------------------------------------------
    def _setup_dark_theme(self):
        app = QtWidgets.QApplication.instance()
        app.setStyle("Fusion")

        palette = QtGui.QPalette()
        bg = QtGui.QColor(32, 33, 36)
        bg_alt = QtGui.QColor(43, 45, 49)
        text = QtGui.QColor(232, 234, 237)
        disabled_text = QtGui.QColor(120, 120, 120)
        highlight = QtGui.QColor(26, 115, 232)

        palette.setColor(QtGui.QPalette.Window, bg)
        palette.setColor(QtGui.QPalette.WindowText, text)
        palette.setColor(QtGui.QPalette.Base, bg_alt)
        palette.setColor(QtGui.QPalette.AlternateBase, bg)
        palette.setColor(QtGui.QPalette.ToolTipBase, text)
        palette.setColor(QtGui.QPalette.ToolTipText, text)
        palette.setColor(QtGui.QPalette.Text, text)
        palette.setColor(QtGui.QPalette.Button, bg_alt)
        palette.setColor(QtGui.QPalette.ButtonText, text)
        palette.setColor(QtGui.QPalette.BrightText, Qt.red)
        palette.setColor(QtGui.QPalette.Link, highlight)
        palette.setColor(QtGui.QPalette.Highlight, highlight)
        palette.setColor(QtGui.QPalette.HighlightedText, Qt.white)
        palette.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.Text, disabled_text)
        palette.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, disabled_text)

        app.setPalette(palette)

    # ------------------------------------------------------------------
    # UI layout
    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # --- Connection group ---
        conn_group = QtWidgets.QGroupBox("Connection")
        main_layout.addWidget(conn_group)
        conn_layout = QtWidgets.QHBoxLayout(conn_group)

        conn_layout.addWidget(QtWidgets.QLabel("Serial Port:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(160)
        conn_layout.addWidget(self.port_combo)

        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(self.refresh_btn)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_obd)
        conn_layout.addWidget(self.connect_btn)

        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self.disconnect_obd)
        conn_layout.addWidget(self.disconnect_btn)

        conn_layout.addStretch()

        self.status_label = QtWidgets.QLabel("Disconnected")
        conn_layout.addWidget(self.status_label)

        # --- Live data & logging group ---
        live_group = QtWidgets.QGroupBox("Live Data & Logging")
        main_layout.addWidget(live_group)
        live_layout = QtWidgets.QVBoxLayout(live_group)

        # PID checkboxes (multi-row if many)
        pid_grid = QtWidgets.QGridLayout()
        live_layout.addLayout(pid_grid)

        self.pid_checkboxes = {}
        row, col = 0, 0
        for label in self.available_commands.keys():
            cb = QtWidgets.QCheckBox(label)
            if label in ("Engine RPM", "Vehicle Speed", "Coolant Temp",
                         "Engine Load", "Throttle Position"):
                cb.setChecked(True)
            self.pid_checkboxes[label] = cb
            pid_grid.addWidget(cb, row, col)
            col += 1
            if col >= 4:
                col = 0
                row += 1

        # Controls row
        controls_layout = QtWidgets.QHBoxLayout()
        live_layout.addLayout(controls_layout)

        self.start_btn = QtWidgets.QPushButton("Start Live Data")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.start_polling)
        controls_layout.addWidget(self.start_btn)

        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_polling)
        controls_layout.addWidget(self.stop_btn)

        controls_layout.addSpacing(20)
        controls_layout.addWidget(QtWidgets.QLabel("Read interval (s):"))

        self.interval_spin = QtWidgets.QDoubleSpinBox()
        self.interval_spin.setRange(0.1, 10.0)
        self.interval_spin.setSingleStep(0.1)
        self.interval_spin.setValue(0.5)
        controls_layout.addWidget(self.interval_spin)

        controls_layout.addSpacing(20)

        self.log_csv_checkbox = QtWidgets.QCheckBox("Log to CSV")
        self.log_csv_checkbox.setChecked(True)
        self.log_csv_checkbox.stateChanged.connect(self._on_log_csv_changed)
        controls_layout.addWidget(self.log_csv_checkbox)

        self.csv_label = QtWidgets.QLabel("Log file: (auto)")
        controls_layout.addWidget(self.csv_label)
        controls_layout.addStretch()

        # --- DTC group ---
        dtc_group = QtWidgets.QGroupBox("Diagnostic Trouble Codes")
        main_layout.addWidget(dtc_group)
        dtc_layout = QtWidgets.QHBoxLayout(dtc_group)

        self.read_dtc_btn = QtWidgets.QPushButton("Read Codes")
        self.read_dtc_btn.setEnabled(False)
        self.read_dtc_btn.clicked.connect(self.read_dtc)
        dtc_layout.addWidget(self.read_dtc_btn)

        self.clear_dtc_btn = QtWidgets.QPushButton("Clear Codes")
        self.clear_dtc_btn.setEnabled(False)
        self.clear_dtc_btn.clicked.connect(self.clear_dtc)
        dtc_layout.addWidget(self.clear_dtc_btn)

        dtc_layout.addStretch()

        # --- Bottom splitter: log + graphs ---
        splitter = QtWidgets.QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter, 1)

        # Left: log
        log_group = QtWidgets.QGroupBox("Event Log")
        log_layout = QtWidgets.QVBoxLayout(log_group)

        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        font = QtGui.QFont("Consolas")
        font.setPointSize(9)
        self.log_edit.setFont(font)
        self.log_edit.setStyleSheet("background-color: #000000; color: #e8eaed;")
        log_layout.addWidget(self.log_edit)

        splitter.addWidget(log_group)

        # Right: multi-graph
        graph_group = QtWidgets.QGroupBox("Live Graphs")
        graph_layout = QtWidgets.QVBoxLayout(graph_group)

        # Top bar: graph window
        top_graph_bar = QtWidgets.QHBoxLayout()
        graph_layout.addLayout(top_graph_bar)

        top_graph_bar.addWidget(QtWidgets.QLabel("Graph 1:"))
        self.graph_combo1 = QtWidgets.QComboBox()
        top_graph_bar.addWidget(self.graph_combo1)

        top_graph_bar.addSpacing(20)
        top_graph_bar.addWidget(QtWidgets.QLabel("Graph 2:"))
        self.graph_combo2 = QtWidgets.QComboBox()
        top_graph_bar.addWidget(self.graph_combo2)

        top_graph_bar.addSpacing(20)
        top_graph_bar.addWidget(QtWidgets.QLabel("Graph 3:"))
        self.graph_combo3 = QtWidgets.QComboBox()
        top_graph_bar.addWidget(self.graph_combo3)

        top_graph_bar.addSpacing(20)
        top_graph_bar.addWidget(QtWidgets.QLabel("Graph window (s, 0 = all):"))

        self.graph_window_spin = QtWidgets.QDoubleSpinBox()
        self.graph_window_spin.setRange(0.0, 3600.0)
        self.graph_window_spin.setSingleStep(5.0)
        self.graph_window_spin.setValue(0.0)
        top_graph_bar.addWidget(self.graph_window_spin)

        top_graph_bar.addStretch()

        # Graph combos list for convenience
        self.graph_combos = [self.graph_combo1, self.graph_combo2, self.graph_combo3]
        for combo in self.graph_combos:
            combo.currentIndexChanged.connect(self.refresh_plot)

        self.graph_window_spin.valueChanged.connect(self.refresh_plot)

        # Matplotlib figure with 3 axes
        self.fig = Figure(figsize=(5, 6), dpi=100)
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312, sharex=self.ax1)
        self.ax3 = self.fig.add_subplot(313, sharex=self.ax1)
        self.axes = [self.ax1, self.ax2, self.ax3]
        self._style_all_axes()

        self.canvas = FigureCanvas(self.fig)
        graph_layout.addWidget(self.canvas)

        splitter.addWidget(graph_group)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)

        # Initial port scan
        self.refresh_ports()

    def _style_axis(self, ax, xlabel=False, ylabel=""):
        ax.cla()
        ax.set_facecolor("#202124")
        self.fig.patch.set_facecolor("#202124")
        ax.tick_params(colors="#e8eaed")
        for spine in ax.spines.values():
            spine.set_color("#e8eaed")
        if xlabel:
            ax.set_xlabel("Time (s)", color="#e8eaed")
        if ylabel:
            ax.set_ylabel(ylabel, color="#e8eaed")
        else:
            ax.set_ylabel("", color="#e8eaed")

    def _style_all_axes(self):
        # Only bottom axis gets the X label
        for i, ax in enumerate(self.axes):
            self._style_axis(ax, xlabel=(i == len(self.axes) - 1))

    # ------------------------------------------------------------------
    # Logging & queue handling
    # ------------------------------------------------------------------
    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{ts}] {msg}\n")

    def _process_queues(self):
        # Logs
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.log_edit.moveCursor(QtGui.QTextCursor.End)
                self.log_edit.insertPlainText(msg)
                self.log_edit.moveCursor(QtGui.QTextCursor.End)
        except Exception:
            pass

        # Data
        try:
            while True:
                sample = self.data_queue.get_nowait()
                self._handle_sample(sample)
        except Exception:
            pass

    def _handle_sample(self, sample: dict):
        if self.session_start_time is None:
            self.session_start_time = sample["timestamp"]
        t_rel = sample["timestamp"] - self.session_start_time

        # Store full session
        for label, val in sample.items():
            if label == "timestamp":
                continue
            if label not in self.data_history:
                self.data_history[label] = {"t": [], "v": []}
            if val is not None:
                self.data_history[label]["t"].append(t_rel)
                self.data_history[label]["v"].append(val)

        # CSV write
        if self.csv_writer is not None:
            row = [time.strftime("%Y-%m-%d %H:%M:%S",
                                 time.localtime(sample["timestamp"]))]
            for lbl in self.csv_log_labels:
                v = sample.get(lbl)
                row.append("" if v is None else v)
            try:
                self.csv_writer.writerow(row)
            except Exception as e:
                self.log(f"CSV write error: {e}")
                self._close_csv()

        # Update all plots
        self.refresh_plot()

    # ------------------------------------------------------------------
    # Serial ports / connection
    # ------------------------------------------------------------------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if ports:
            self.status_label.setText("Select a port and connect")
        else:
            self.status_label.setText("No serial ports found")

    def connect_obd(self):
        if self.connection and self.connection.is_connected():
            self.log("Already connected.")
            return

        port = self.port_combo.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port selected.")
            return

        self.log(f"Connecting to {port}...")
        try:
            self.connection = obd.OBD(port, fast=False, timeout=2.0)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open port {port}:\n{e}")
            self.log(f"Failed to open port {port}: {e}")
            self.connection = None
            return

        if not self.connection or not self.connection.is_connected():
            QtWidgets.QMessageBox.critical(self, "Error", f"Could not connect to ECU on {port}.")
            self.log("ECU connection failed.")
            self.connection = None
            return

        self.log("Connected to ECU.")
        self.status_label.setText(f"Connected ({port})")

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.start_btn.setEnabled(True)
        self.read_dtc_btn.setEnabled(True)
        self.clear_dtc_btn.setEnabled(True)

    def disconnect_obd(self):
        self.stop_polling()
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None
        self.status_label.setText("Disconnected")
        self.log("Disconnected from ECU.")

        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.read_dtc_btn.setEnabled(False)
        self.clear_dtc_btn.setEnabled(False)

    # ------------------------------------------------------------------
    # Polling & CSV
    # ------------------------------------------------------------------
    def _on_log_csv_changed(self, state):
        self.log_to_csv = (state == Qt.Checked)
        if not self.log_to_csv:
            self._close_csv()
            self.csv_label.setText("Log file: (none)")
        else:
            self.csv_label.setText("Log file: (auto)")

    def start_polling(self):
        if not self.connection or not self.connection.is_connected():
            QtWidgets.QMessageBox.warning(self, "Warning", "Not connected to ECU.")
            return

        labels = [lbl for lbl, cb in self.pid_checkboxes.items() if cb.isChecked()]
        if not labels:
            QtWidgets.QMessageBox.warning(self, "Warning", "Select at least one PID.")
            return

        interval = float(self.interval_spin.value())
        if interval <= 0:
            QtWidgets.QMessageBox.warning(self, "Warning", "Interval must be > 0.")
            return

        if self.polling:
            self.log("Already polling.")
            return

        commands = [self.available_commands[lbl] for lbl in labels]

        # Reset in-memory data
        with self.data_queue.mutex:
            self.data_queue.queue.clear()
        self.data_history.clear()
        self.session_start_time = None

        # CSV setup
        if self.log_to_csv:
            self._open_new_csv(labels)
        else:
            self._close_csv()

        # Populate graph combos (first option blank = "(None)")
        for combo in self.graph_combos:
            combo.clear()
            combo.addItem("(None)")
            combo.addItems(labels)
            combo.setCurrentIndex(0)

        self.polling = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        self.log(f"Starting live data: {', '.join(labels)} (every {interval:.2f}s)")

        self.polling_thread = threading.Thread(
            target=self._poll_worker,
            args=(labels, commands, interval),
            daemon=True,
        )
        self.polling_thread.start()

    def stop_polling(self):
        if self.polling:
            self.polling = False
            self.log("Stopping live data...")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self._close_csv()

    def _poll_worker(self, labels, commands, interval):
        while self.polling and self.connection and self.connection.is_connected():
            sample = {"timestamp": time.time()}
            line_parts = []
            for label, cmd in zip(labels, commands):
                try:
                    resp = self.connection.query(cmd)
                    if resp.is_null():
                        sample[label] = None
                        line_parts.append(f"{label}=N/A")
                    else:
                        v = resp.value
                        num = None
                        try:
                            if hasattr(v, "magnitude"):
                                num = float(v.magnitude)
                            else:
                                num = float(v)
                        except Exception:
                            num = None
                        sample[label] = num
                        line_parts.append(f"{label}={v}")
                except Exception as e:
                    self.log(f"Error querying {label}: {e}")
                    sample[label] = None

            self.log(" | ".join(line_parts))
            self.data_queue.put(sample)

            time.sleep(interval)

    def _open_new_csv(self, labels):
        self._close_csv()
        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = f"obd_log_{ts}.csv"
        try:
            self.csv_file = open(filename, mode="w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_log_labels = list(labels)
            header = ["Timestamp"] + self.csv_log_labels
            self.csv_writer.writerow(header)
            self.csv_label.setText(f"Log file: {filename}")
            self.log(f"Logging to CSV: {filename}")
        except Exception as e:
            self.log(f"Failed to open CSV file: {e}")
            self._close_csv()
            self.csv_label.setText("Log file: (none)")

    def _close_csv(self):
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None
        self.csv_log_labels = []

    # ------------------------------------------------------------------
    # DTCs
    # ------------------------------------------------------------------
    def read_dtc(self):
        if not self.connection or not self.connection.is_connected():
            QtWidgets.QMessageBox.warning(self, "Warning", "Not connected to ECU.")
            return

        self.log("Reading DTCs...")
        try:
            resp = self.connection.query(obd.commands.GET_DTC)
            if resp.is_null():
                self.log("No DTC data or unsupported command.")
                return
            dtcs = resp.value
            if not dtcs:
                self.log("No stored DTCs.")
            else:
                self.log("Stored DTCs:")
                for code, desc in dtcs:
                    self.log(f"  {code}: {desc}")
        except Exception as e:
            self.log(f"Error reading DTCs: {e}")

    def clear_dtc(self):
        if not self.connection or not self.connection.is_connected():
            QtWidgets.QMessageBox.warning(self, "Warning", "Not connected to ECU.")
            return

        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm",
            "Clear all DTCs? Check Engine Light may turn off.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        self.log("Clearing DTCs...")
        try:
            resp = self.connection.query(obd.commands.CLEAR_DTC)
            if resp.is_null():
                self.log("No response or unsupported command.")
            else:
                self.log("Clear DTC command sent.")
        except Exception as e:
            self.log(f"Error clearing DTCs: {e}")

    # ------------------------------------------------------------------
    # Plotting (three panes)
    # ------------------------------------------------------------------
    def refresh_plot(self):
        # Re-style all axes, but only bottom shows x-label
        for i, ax in enumerate(self.axes):
            self._style_axis(ax, xlabel=(i == len(self.axes) - 1))

        window_sec = float(self.graph_window_spin.value())

        for ax, combo in zip(self.axes, self.graph_combos):
            label = combo.currentText()
            if not label or label == "(None)" or label not in self.data_history:
                continue

            t_all = self.data_history[label]["t"]
            v_all = self.data_history[label]["v"]
            if not t_all or not v_all:
                continue

            if window_sec > 0:
                t_last = t_all[-1]
                t0 = t_last - window_sec
                filtered = [(t, v) for t, v in zip(t_all, v_all) if t >= t0]
                if filtered:
                    t, v = zip(*filtered)
                else:
                    t, v = [], []
            else:
                t, v = t_all, v_all

            if t and v:
                ax.set_ylabel(label, color="#e8eaed")
                ax.plot(t, v, linewidth=1.5, color="#1a73e8")

        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Close event
    # ------------------------------------------------------------------
    def closeEvent(self, event: QtGui.QCloseEvent):
        self.stop_polling()
        if self.connection and self.connection.is_connected():
            try:
                self.connection.close()
            except Exception:
                pass
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = OBDMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
