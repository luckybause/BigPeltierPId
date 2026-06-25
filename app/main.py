"""
Peltier PID Controller - Aplikacja desktopowa
Wymagania: pip install customtkinter pyserial matplotlib
"""

import tkinter as tk
import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import json
import csv
import time
import queue
from datetime import datetime
from collections import deque
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

HIST_LEN = 300
CHART_BG = "#1a1a2e"
C_T1     = "#e94560"
C_T2L    = "#4fc3f7"
C_SP     = "#f5a623"
C_PELT   = "#7ed321"
C_FAN    = "#9b59b6"


class PeltierApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Peltier PID Controller")
        self.geometry("1200x780")
        self.minsize(1000, 680)

        self.ser = None
        self.running = False
        self.data_queue = queue.Queue()
        self.log_file = None
        self.log_writer = None
        self.logging_active = False

        self.ts_hist  = deque(maxlen=HIST_LEN)
        self.t1_hist  = deque(maxlen=HIST_LEN)
        self.t2_hist  = deque(maxlen=HIST_LEN)
        self.sp_hist  = deque(maxlen=HIST_LEN)
        self.pw_hist  = deque(maxlen=HIST_LEN)
        self.fan_hist = deque(maxlen=HIST_LEN)
        self.t_start  = time.time()

        self._build_ui()
        self._build_chart()
        self._refresh_ports()

        self.ani = animation.FuncAnimation(
            self.fig, self._update_chart, interval=500, cache_frame_data=False
        )
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(300, self._poll_queue)

    def _build_ui(self):
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        panel = ctk.CTkScrollableFrame(self, width=280, corner_radius=0)
        panel.grid(row=0, column=0, sticky="nsew")
        self._build_panel(panel)
        self.chart_frame = ctk.CTkFrame(self, corner_radius=0, fg_color=CHART_BG)
        self.chart_frame.grid(row=0, column=1, sticky="nsew")
        self.chart_frame.grid_rowconfigure(0, weight=1)
        self.chart_frame.grid_columnconfigure(0, weight=1)
        self.status_var = tk.StringVar(value="Rozlaczono")
        ctk.CTkLabel(self, textvariable=self.status_var, anchor="w",
                     fg_color="#111111", text_color="#888888", height=24).grid(
            row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=2)

    def _section(self, parent, text):
        ctk.CTkLabel(parent, text=text.upper(), font=("Courier", 10, "bold"),
                     text_color="#555577").pack(anchor="w", padx=12, pady=(14, 2))
        ctk.CTkFrame(parent, height=1, fg_color="#333355").pack(fill="x", padx=8, pady=(0, 6))

    def _build_panel(self, p):
        ctk.CTkLabel(p, text="Peltier PID", font=("Helvetica", 18, "bold")).pack(pady=(16, 4))
        ctk.CTkLabel(p, text="Sterownik temperatury", font=("Helvetica", 11),
                     text_color="#888888").pack(pady=(0, 8))

        self._section(p, "Polaczenie")
        self.port_var = tk.StringVar()
        self.port_menu = ctk.CTkOptionMenu(p, variable=self.port_var, values=[], width=240)
        self.port_menu.pack(padx=12, pady=4)
        row = ctk.CTkFrame(p, fg_color="transparent")
        row.pack(fill="x", padx=12, pady=4)
        ctk.CTkButton(row, text="Odswiez", width=110, command=self._refresh_ports).pack(side="left")
        self.connect_btn = ctk.CTkButton(row, text="Polacz", width=110,
                                          command=self._toggle_connect, fg_color="#1a6b3a")
        self.connect_btn.pack(side="right")

        self._section(p, "Temperatury")
        tf = ctk.CTkFrame(p, fg_color="#111122", corner_radius=8)
        tf.pack(fill="x", padx=12, pady=4)
        self.t1_var = tk.StringVar(value="---")
        self.t2_var = tk.StringVar(value="---")
        ctk.CTkLabel(tf, text="Czujnik 1 (CS9)", text_color=C_T1,
                     font=("Courier", 11)).grid(row=0, column=0, padx=10, pady=6, sticky="w")
        ctk.CTkLabel(tf, textvariable=self.t1_var, font=("Courier", 20, "bold"),
                     text_color=C_T1).grid(row=0, column=1, padx=10)
        ctk.CTkLabel(tf, text="Czujnik 2 (CS10)", text_color=C_T2L,
                     font=("Courier", 11)).grid(row=1, column=0, padx=10, pady=6, sticky="w")
        ctk.CTkLabel(tf, textvariable=self.t2_var, font=("Courier", 20, "bold"),
                     text_color=C_T2L).grid(row=1, column=1, padx=10)

        self._section(p, "PID")
        self._labeled_entry(p, "Setpoint [C]", "25.0", "sp_entry")
        self._labeled_entry(p, "Kp", "5.0", "kp_entry")
        self._labeled_entry(p, "Ki", "0.1", "ki_entry")
        self._labeled_entry(p, "Kd", "1.0", "kd_entry")
        self.pid_var  = tk.BooleanVar(value=False)
        self.heat_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(p, text="Wlacz PID", variable=self.pid_var,
                        command=self._send_settings).pack(anchor="w", padx=16, pady=4)
        ctk.CTkCheckBox(p, text="Tryb grzania (odznacz = chlodzenie)",
                        variable=self.heat_var, command=self._send_settings).pack(anchor="w", padx=16, pady=2)
        ctk.CTkButton(p, text="Wyslij ustawienia PID", command=self._send_settings).pack(fill="x", padx=12, pady=6)
        ctk.CTkButton(p, text="Reset calki PID", fg_color="#5a2020",
                      command=self._reset_pid).pack(fill="x", padx=12, pady=2)

        self._section(p, "Wentylator")
        self.fan_auto_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(p, text="Automatyczny", variable=self.fan_auto_var,
                        command=self._send_settings).pack(anchor="w", padx=16, pady=4)
        ctk.CTkLabel(p, text="Reczna moc [%]", anchor="w").pack(fill="x", padx=12)
        self.fan_slider = ctk.CTkSlider(p, from_=0, to=100, number_of_steps=100,
                                         command=lambda v: self._on_fan_slide(v))
        self.fan_slider.set(0)
        self.fan_slider.pack(fill="x", padx=12, pady=4)
        self.fan_pct_var = tk.StringVar(value="0 %")
        ctk.CTkLabel(p, textvariable=self.fan_pct_var, font=("Courier", 13)).pack(pady=2)

        self._section(p, "Wyjscia")
        of = ctk.CTkFrame(p, fg_color="#111122", corner_radius=8)
        of.pack(fill="x", padx=12, pady=4)
        self.pelt_var    = tk.StringVar(value="---")
        self.fan_out_var = tk.StringVar(value="---")
        ctk.CTkLabel(of, text="Peltier",    text_color=C_PELT).grid(row=0, column=0, padx=10, pady=6)
        ctk.CTkLabel(of, textvariable=self.pelt_var,    font=("Courier", 16, "bold"),
                     text_color=C_PELT).grid(row=0, column=1, padx=10)
        ctk.CTkLabel(of, text="Wentylator", text_color=C_FAN).grid(row=1, column=0, padx=10, pady=6)
        ctk.CTkLabel(of, textvariable=self.fan_out_var, font=("Courier", 16, "bold"),
                     text_color=C_FAN).grid(row=1, column=1, padx=10)

        self._section(p, "Zapis CSV")
        self.log_btn = ctk.CTkButton(p, text="Rozpocznij zapis", fg_color="#1a4a6a",
                                      command=self._toggle_logging)
        self.log_btn.pack(fill="x", padx=12, pady=6)
        self.log_path_var = tk.StringVar(value="")
        ctk.CTkLabel(p, textvariable=self.log_path_var, font=("Courier", 9),
                     text_color="#555577", wraplength=250).pack(padx=12)

    def _labeled_entry(self, parent, label, default, attr):
        ctk.CTkLabel(parent, text=label, anchor="w").pack(fill="x", padx=12, pady=(4, 0))
        entry = ctk.CTkEntry(parent, placeholder_text=default)
        entry.insert(0, default)
        entry.pack(fill="x", padx=12, pady=2)
        setattr(self, attr, entry)

    def _build_chart(self):
        self.fig = Figure(figsize=(6, 4), facecolor=CHART_BG)
        self.fig.subplots_adjust(left=0.07, right=0.93, top=0.92, bottom=0.10, hspace=0.3)

        self.ax_temp = self.fig.add_subplot(2, 1, 1, facecolor="#0d0d1a")
        self.ax_temp.set_title("Temperatura", color="#cccccc", fontsize=10)
        self.ax_temp.set_ylabel("C", color="#888888")
        self.ax_temp.tick_params(colors="#666666")
        for sp in self.ax_temp.spines.values(): sp.set_edgecolor("#333355")

        self.ax_pwr = self.fig.add_subplot(2, 1, 2, facecolor="#0d0d1a")
        self.ax_pwr.set_title("Moc wyjsc [%]", color="#cccccc", fontsize=10)
        self.ax_pwr.set_ylabel("%", color="#888888")
        self.ax_pwr.set_ylim(0, 105)
        self.ax_pwr.tick_params(colors="#666666")
        for sp in self.ax_pwr.spines.values(): sp.set_edgecolor("#333355")

        self.line_t1, = self.ax_temp.plot([], [], color=C_T1,  lw=2, label="T1")
        self.line_t2, = self.ax_temp.plot([], [], color=C_T2L, lw=2, label="T2", ls="--")
        self.line_sp, = self.ax_temp.plot([], [], color=C_SP,  lw=1, label="SP", ls=":")
        self.ax_temp.legend(loc="upper left", facecolor="#111122", labelcolor="#cccccc", fontsize=9)

        self.line_pw,  = self.ax_pwr.plot([], [], color=C_PELT, lw=2, label="Peltier")
        self.line_fan, = self.ax_pwr.plot([], [], color=C_FAN,  lw=2, label="Went.", ls="--")
        self.ax_pwr.legend(loc="upper left", facecolor="#111122", labelcolor="#cccccc", fontsize=9)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.chart_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _update_chart(self, _frame):
        if not self.ts_hist: return
        xs = list(self.ts_hist)
        def safe(lst): return [v if v is not None else float("nan") for v in lst]
        self.line_t1.set_data(xs, safe(self.t1_hist))
        self.line_t2.set_data(xs, safe(self.t2_hist))
        self.line_sp.set_data(xs, list(self.sp_hist))
        self.line_pw.set_data(xs, list(self.pw_hist))
        self.line_fan.set_data(xs, list(self.fan_hist))
        for ax in (self.ax_temp, self.ax_pwr):
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=(ax is self.ax_temp))
        self.canvas.draw_idle()

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_menu.configure(values=ports if ports else ["(brak portow)"])
        if ports: self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.running: self._disconnect()
        else: self._connect()

    def _connect(self):
        port = self.port_var.get()
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.running = True
            threading.Thread(target=self._read_loop, daemon=True).start()
            self.connect_btn.configure(text="Rozlacz", fg_color="#6a1a1a")
            self.status_var.set(f"Polaczono: {port}")
        except Exception as e:
            self.status_var.set(f"Blad: {e}")

    def _disconnect(self):
        self.running = False
        if self.ser:
            try: self.ser.close()
            except: pass
            self.ser = None
        self.connect_btn.configure(text="Polacz", fg_color="#1a6b3a")
        self.status_var.set("Rozlaczono")

    def _read_loop(self):
        buf = ""
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1).decode("utf-8", errors="ignore")
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line.startswith("{"):
                        try: self.data_queue.put(json.loads(line))
                        except: pass
            except:
                if self.running: self.after(0, self._disconnect)
                break

    def _poll_queue(self):
        try:
            while True: self._handle_data(self.data_queue.get_nowait())
        except queue.Empty: pass
        self.after(200, self._poll_queue)

    def _handle_data(self, d):
        t1 = d.get("t1"); t2 = d.get("t2")
        sp = d.get("setpoint", 0)
        pp = d.get("peltier_pct", 0); fp = d.get("fan_pct", 0)
        self.t1_var.set(f"{t1:.1f} C" if t1 is not None else "BLAD")
        self.t2_var.set(f"{t2:.1f} C" if t2 is not None else "BLAD")
        self.pelt_var.set(f"{pp:.0f} %"); self.fan_out_var.set(f"{fp:.0f} %")
        ts = time.time() - self.t_start
        self.ts_hist.append(ts); self.t1_hist.append(t1); self.t2_hist.append(t2)
        self.sp_hist.append(sp); self.pw_hist.append(pp); self.fan_hist.append(fp)
        if self.logging_active and self.log_writer:
            self.log_writer.writerow([datetime.now().isoformat(timespec="seconds"),
                                       t1, t2, sp, pp, fp, d.get("pid_on"), d.get("heat_mode")])
            self.log_file.flush()

    def _send_settings(self):
        if not self.ser: return
        try:
            cmd = {"setpoint": float(self.sp_entry.get()),
                   "kp": float(self.kp_entry.get()), "ki": float(self.ki_entry.get()),
                   "kd": float(self.kd_entry.get()), "pid_enabled": self.pid_var.get(),
                   "heat_mode": self.heat_var.get(), "fan_auto": self.fan_auto_var.get(),
                   "fan_manual": float(self.fan_slider.get())}
            self.ser.write((json.dumps(cmd) + "\n").encode())
        except Exception as e:
            self.status_var.set(f"Blad wysylania: {e}")

    def _reset_pid(self):
        if not self.ser: return
        self.ser.write((json.dumps({"reset_pid": True}) + "\n").encode())

    def _on_fan_slide(self, val):
        self.fan_pct_var.set(f"{int(float(val))} %")
        self._send_settings()

    def _toggle_logging(self):
        if self.logging_active:
            self.logging_active = False
            if self.log_file: self.log_file.close(); self.log_file = None; self.log_writer = None
            self.log_btn.configure(text="Rozpocznij zapis", fg_color="#1a4a6a")
            self.log_path_var.set("")
        else:
            fname = f"peltier_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.log_file = open(fname, "w", newline="", encoding="utf-8")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(["timestamp","T1_C","T2_C","setpoint_C","peltier_pct","fan_pct","pid_on","heat_mode"])
            self.logging_active = True
            self.log_btn.configure(text="Zatrzymaj zapis", fg_color="#6a1a1a")
            self.log_path_var.set(fname)

    def _on_close(self):
        self._disconnect()
        if self.logging_active: self._toggle_logging()
        self.destroy()


if __name__ == "__main__":
    app = PeltierApp()
    app.mainloop()
