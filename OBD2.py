import threading
import time
from queue import Queue
import sys
import csv

import tkinter as tk
from tkinter import ttk, messagebox

# ---- Third-party libs ----
try:
    import obd
except ImportError:
    print("The 'python-OBD' package is not installed. Run: pip install python-OBD")
    sys.exit(1)

try:
    from serial.tools import list_ports
except ImportError:
    print("The 'pyserial' package is not installed. Run: pip install pyserial")
    sys.exit(1)

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class OBDGuiApp:
    def __init__(self, root):
        self.root = root
        self.root.title("OBD-II Interface - 2015 Camaro")
        self.root.geometry("1100x700")

        # ---- Dark theme styling ----
        self._setup_style()

        # OBD connection and state
        self.connection = None
        self.polling_thread = None
        self.polling = False

        # Queues for thread-safe communication
        self.log_queue = Queue()
        self.data_queue = Queue()

        # CSV logging
        self.log_to_csv_var = tk.BooleanVar(value=True)
        self.csv_file = None
        self.csv_writer = None
        self.csv_log_labels = []
        self.session_start_time = None

        # Data history for plotting: label -> {"t": [], "v": []}
        self.data_history = {}

        # Mapping of label -> python-OBD command
        self.available_commands = {
            "Engine RPM":       obd.commands.RPM,
            "Vehicle Speed":    obd.commands.SPEED,
            "Coolant Temp":     obd.commands.COOLANT_TEMP,
            "Throttle Position":obd.commands.THROTTLE_POS,
            "Engine Load":      obd.commands.ENGINE_LOAD,
            "Fuel Level":       obd.commands.FUEL_LEVEL,
        }

        self._build_ui()

        # Periodically process queued log/data from worker thread
        self.root.after(100, self._process_queues)

    # ---------------------------
    # Styling
    # ---------------------------
    def _setup_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        bg = "#202124"
        fg = "#e8eaed"
        accent = "#1a73e8"
        frame_bg = "#2b2d31"

        self.root.configure(bg=bg)

        style.configure(".", background=bg, foreground=fg, fieldbackground=frame_bg)
        style.configure("TFrame", background=frame_bg)
        style.configure("TLabelframe", background=frame_bg, foreground=fg)
        style.configure("TLabelframe.Label", background=frame_bg, foreground=fg)
        style.configure("TLabel", background=frame_bg, foreground=fg)
        style.configure("TButton", background=accent, foreground="white")
        style.map("TButton", background=[("active", "#185abc")])
        style.configure("TCheckbutton", background=frame_bg, foreground=fg)
        style.configure("TCombobox", fieldbackground=frame_bg, background=frame_bg, foreground=fg)

    # ---------------------------
    # UI construction
    # ---------------------------
    def _build_ui(self):
        # ---- Connection frame ----
        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.pack(fill="x", padx=10, pady=8)

        ttk.Label(conn_frame, text="Serial Port:").pack(side="left", padx=(10, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.pack(side="left", padx=5)

        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).pack(side="left", padx=5)

        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect)
        self.connect_button.pack(side="left", padx=10)

        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect",
                                            command=self.disconnect, state="disabled")
        self.disconnect_button.pack(side="left", padx=5)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(conn_frame, textvariable=self.status_var).pack(side="right", padx=10)

        # ---- Live data + logging frame ----
        live_frame = ttk.LabelFrame(self.root, text="Live Data & Logging")
        live_frame.pack(fill="x", padx=10, pady=8)

        # PIDs selection
        pid_frame = ttk.Frame(live_frame)
        pid_frame.pack(fill="x", padx=5, pady=5)

        self.pid_vars = {}
        for i, (label, _) in enumerate(self.available_commands.items()):
            var = tk.BooleanVar(value=(label in ["Engine RPM", "Vehicle Speed", "Coolant Temp"]))
            cb = ttk.Checkbutton(pid_frame, text=label, variable=var)
            cb.grid(row=0, column=i, padx=5, pady=2, sticky="w")
            self.pid_vars[label] = var

        # Controls
        control_frame = ttk.Frame(live_frame)
        control_frame.pack(fill="x", padx=5, pady=5)

        self.start_button = ttk.Button(control_frame, text="Start Live Data",
                                       command=self.start_polling, state="disabled")
        self.start_button.pack(side="left", padx=5)

        self.stop_button = ttk.Button(control_frame, text="Stop",
                                      command=self.stop_polling, state="disabled")
        self.stop_button.pack(side="left", padx=5)

        ttk.Label(control_frame, text="Interval (s):").pack(side="left", padx=(20, 5))
        self.interval_var = tk.DoubleVar(value=0.5)
        ttk.Entry(control_frame, textvariable=self.interval_var, width=6).pack(side="left", padx=5)

        ttk.Checkbutton(control_frame, text="Log to CSV",
                        variable=self.log_to_csv_var).pack(side="left", padx=(20, 5))
        self.csv_path_var = tk.StringVar(value="Log file: (auto)")
        ttk.Label(control_frame, textvariable=self.csv_path_var).pack(side="left", padx=5)

        # ---- DTC frame ----
        dtc_frame = ttk.LabelFrame(self.root, text="Diagnostic Trouble Codes")
        dtc_frame.pack(fill="x", padx=10, pady=8)

        self.read_dtc_button = ttk.Button(dtc_frame, text="Read Codes",
                                          command=self.read_dtc, state="disabled")
        self.read_dtc_button.pack(side="left", padx=5, pady=5)

        self.clear_dtc_button = ttk.Button(dtc_frame, text="Clear Codes",
                                           command=self.clear_dtc, state="disabled")
        self.clear_dtc_button.pack(side="left", padx=5, pady=5)

        # ---- Bottom split: left = log, right = graph ----
        bottom_frame = ttk.Frame(self.root)
        bottom_frame.pack(fill="both", expand=True, padx=10, pady=8)

        # Log/output
        output_frame = ttk.LabelFrame(bottom_frame, text="Event Log")
        output_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        self.output_text = tk.Text(output_frame, wrap="word", state="disabled", height=10,
                                   bg="#000000", fg="#e8eaed", insertbackground="#e8eaed")
        self.output_text.pack(fill="both", expand=True, padx=5, pady=5)

        scrollbar = ttk.Scrollbar(self.output_text, command=self.output_text.yview)
        self.output_text["yscrollcommand"] = scrollbar.set
        scrollbar.pack(side="right", fill="y")

        # Graph area
        graph_frame = ttk.LabelFrame(bottom_frame, text="Live Graph")
        graph_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))

        # Graph variable selector
        top_graph_frame = ttk.Frame(graph_frame)
        top_graph_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(top_graph_frame, text="Graph variable:").pack(side="left")
        self.graph_var = tk.StringVar()
        self.graph_combo = ttk.Combobox(top_graph_frame, textvariable=self.graph_var,
                                        state="readonly", width=25)
        self.graph_combo.pack(side="left", padx=5)
        self.graph_combo.bind("<<ComboboxSelected>>", lambda e: self._refresh_plot())

        # Matplotlib figure
        self.fig = Figure(figsize=(5, 3), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#202124")
        self.fig.patch.set_facecolor("#202124")
        self.ax.tick_params(colors="#e8eaed")
        for spine in self.ax.spines.values():
            spine.set_color("#e8eaed")
        self.ax.set_xlabel("Time (s)", color="#e8eaed")

        self.line, = self.ax.plot([], [], linestyle="-", marker="", linewidth=1.5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)

        # Initial port scan
        self.refresh_ports()

    # ---------------------------
    # Logging + queue handling
    # ---------------------------
    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{ts}] {msg}\n")

    def _process_queues(self):
        # Logs
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.output_text.config(state="normal")
                self.output_text.insert("end", msg)
                self.output_text.see("end")
                self.output_text.config(state="disabled")
        except Exception:
            pass

        # Data samples
        try:
            while True:
                sample = self.data_queue.get_nowait()
                self._handle_sample(sample)
        except Exception:
            pass

        self.root.after(100, self._process_queues)

    def _handle_sample(self, sample):
        # sample: {"timestamp": float, "label1": val1, ...}
        if not self.session_start_time:
            self.session_start_time = sample["timestamp"]
        t_rel = sample["timestamp"] - self.session_start_time

        # Update in-memory time series
        for label, val in sample.items():
            if label == "timestamp":
                continue
            if label not in self.data_history:
                self.data_history[label] = {"t": [], "v": []}
            if val is not None:
                self.data_history[label]["t"].append(t_rel)
                self.data_history[label]["v"].append(val)

        # CSV logging
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

        # Update the plot every time new data arrives
        self._refresh_plot()

    # ---------------------------
    # Serial ports + connection
    # ---------------------------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
            self.status_var.set("Select a port and connect")
        elif not ports:
            self.port_var.set("")
            self.status_var.set("No serial ports found")

    def connect(self):
        if self.connection and self.connection.is_connected():
            messagebox.showinfo("Info", "Already connected.")
            return

        port = self.port_var.get()
        if not port:
            messagebox.showwarning("Warning", "No serial port selected.")
            return

        self.log(f"Connecting to {port}...")
        try:
            self.connection = obd.OBD(port, fast=False, timeout=2.0)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open port {port}:\n{e}")
            self.log(f"Failed to open port {port}: {e}")
            self.connection = None
            return

        if not self.connection or not self.connection.is_connected():
            messagebox.showerror("Error", f"Could not connect to ECU on {port}.")
            self.log("ECU connection failed.")
            self.connection = None
            return

        self.log("Connected to ECU.")
        self.status_var.set(f"Connected ({port})")
        self.connect_button.config(state="disabled")
        self.disconnect_button.config(state="normal")
        self.start_button.config(state="normal")
        self.read_dtc_button.config(state="normal")
        self.clear_dtc_button.config(state="normal")

    def disconnect(self):
        self.stop_polling()
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None
        self.status_var.set("Disconnected")
        self.log("Disconnected from ECU.")
        self.connect_button.config(state="normal")
        self.disconnect_button.config(state="disabled")
        self.start_button.config(state="disabled")
        self.read_dtc_button.config(state="disabled")
        self.clear_dtc_button.config(state="disabled")

    # ---------------------------
    # Live polling + CSV
    # ---------------------------
    def start_polling(self):
        if not self.connection or not self.connection.is_connected():
            messagebox.showwarning("Warning", "Not connected to ECU.")
            return

        selected_labels = [lbl for lbl, var in self.pid_vars.items() if var.get()]
        if not selected_labels:
            messagebox.showwarning("Warning", "Select at least one PID to monitor.")
            return

        try:
            interval = float(self.interval_var.get())
            if interval <= 0:
                raise ValueError
        except ValueError:
            messagebox.showwarning("Warning", "Interval must be a positive number.")
            return

        if self.polling:
            messagebox.showinfo("Info", "Already polling.")
            return

        commands = [self.available_commands[lbl] for lbl in selected_labels]

        # Reset previous data
        self.data_queue.queue.clear()
        self.data_history.clear()
        self.session_start_time = None

        # Setup CSV if enabled
        if self.log_to_csv_var.get():
            self._open_new_csv(selected_labels)
        else:
            self._close_csv()

        # Update graph dropdown
        self.graph_combo["values"] = selected_labels
        if selected_labels and not self.graph_var.get():
            self.graph_var.set(selected_labels[0])

        self.polling = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

        self.log(f"Starting live data: {', '.join(selected_labels)} (every {interval:.2f}s)")
        self.polling_thread = threading.Thread(
            target=self._poll_worker,
            args=(selected_labels, commands, interval),
            daemon=True
        )
        self.polling_thread.start()

    def stop_polling(self):
        if self.polling:
            self.polling = False
            self.log("Stopping live data...")
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self._close_csv()

    def _poll_worker(self, labels, commands, interval):
        while self.polling and self.connection and self.connection.is_connected():
            sample = {"timestamp": time.time()}
            line_parts = []
            for label, cmd in zip(labels, commands):
                try:
                    resp = self.connection.query(cmd)
                    if resp.is_null():
                        val = None
                        sample[label] = None
                        line_parts.append(f"{label}=N/A")
                    else:
                        v = resp.value
                        # Convert to numeric if possible (for plotting/logging)
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
            # One compact log line per cycle
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
            self.csv_path_var.set(f"Log file: {filename}")
            self.log(f"Logging to CSV: {filename}")
        except Exception as e:
            self.log(f"Failed to open CSV log file: {e}")
            self._close_csv()

    def _close_csv(self):
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None        # type: ignore
        self.csv_log_labels = []
        self.csv_path_var.set("Log file: (none)")

    # ---------------------------
    # DTC (codes)
    # ---------------------------
    def read_dtc(self):
        if not self.connection or not self.connection.is_connected():
            messagebox.showwarning("Warning", "Not connected to ECU.")
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
            messagebox.showwarning("Warning", "Not connected to ECU.")
            return
        if not messagebox.askyesno("Confirm",
                                   "Clear all DTCs? Check engine light may turn off."):
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

    # ---------------------------
    # Plotting
    # ---------------------------
    def _refresh_plot(self):
        label = self.graph_var.get()
        self.ax.cla()
        self.ax.set_facecolor("#202124")
        self.fig.patch.set_facecolor("#202124")
        self.ax.tick_params(colors="#e8eaed")
        for spine in self.ax.spines.values():
            spine.set_color("#e8eaed")
        self.ax.set_xlabel("Time (s)", color="#e8eaed")

        if not label or label not in self.data_history:
            self.ax.set_ylabel("", color="#e8eaed")
            self.canvas.draw_idle()
            return

        t = self.data_history[label]["t"]
        v = self.data_history[label]["v"]
        self.ax.set_ylabel(label, color="#e8eaed")

        if t and v:
            self.ax.plot(t, v, linewidth=1.5)

        self.canvas.draw_idle()


def main():
    root = tk.Tk()
    app = OBDGuiApp(root)

    def on_close():
        app.stop_polling()
        app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
