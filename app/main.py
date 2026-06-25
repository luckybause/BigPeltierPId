"""
Peltier PID Controller - Aplikacja PC
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

HIST_LEN = 600
CHART_BG  = "#1a1a2e"
C_T1   = "#e94560"
C_T2L  = "#4fc3f7"
C_SP   = "#f5a623"
C_PELT = "#7ed321"
C_FAN  = "#9b59b6"


class PeltierApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Peltier PID Controller")
        self.geometry("1280x800")
        self.minsize(1000, 680)

        self.ser = None
        self.running = False
        self._lock = threading.Lock()
        self.data_queue = queue.Queue()
        self.log_file = None
        self.log_writer = None
        self.logging_active = False

        self.ts_hist  = deque(maxlen=HIST_LEN)
        self.t1_hist  = deque(maxlen=HIST_LEN)
        self.t2_hist  = deque(maxlen=HIST_LEN)
        self.sp_hist  = deque(maxlen=HIST_LEN)
        self.pw_hist  = deque(maxlen=HIST_LEN)
        self.fn_hist  = deque(maxlen=HIST_LEN)
        self.t_start  = time.time()

        self._build_ui()
        self._build_chart()
        self._refresh_ports()

        self.ani = animation.FuncAnimation(
            self.fig, self._update_chart, interval=500, cache_frame_data=False
        )
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(150, self._poll_queue)

    # ─── UI ───────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        panel = ctk.CTkScrollableFrame(self, width=295, corner_radius=0)
        panel.grid(row=0, column=0, sticky="nsew")
        self._build_panel(panel)
        self.chart_frame = ctk.CTkFrame(self, corner_radius=0, fg_color=CHART_BG)
        self.chart_frame.grid(row=0, column=1, sticky="nsew")
        self.chart_frame.grid_rowconfigure(0, weight=1)
        self.chart_frame.grid_columnconfigure(0, weight=1)
        self.status_var = tk.StringVar(value="Rozlaczono")
        ctk.CTkLabel(self, textvariable=self.status_var, anchor="w",
                     fg_color="#0a0a0a", text_color="#666666", height=22).grid(
            row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=1)

    def _section(self, p, text):
        ctk.CTkLabel(p, text=text.upper(), font=("Courier", 10, "bold"),
                     text_color="#444466").pack(anchor="w", padx=12, pady=(12, 2))
        ctk.CTkFrame(p, height=1, fg_color="#333355").pack(fill="x", padx=8, pady=(0, 5))

    def _build_panel(self, p):
        ctk.CTkLabel(p, text="Peltier PID", font=("Helvetica", 17, "bold")).pack(pady=(14, 2))
        ctk.CTkLabel(p, text="ItsyBitsy M0 + Cytron MDD10A", font=("Helvetica", 10),
                     text_color="#666688").pack(pady=(0, 6))

        # ── Port ──
        self._section(p, "Polaczenie")
        self.port_var = tk.StringVar()
        self.port_menu = ctk.CTkOptionMenu(p, variable=self.port_var, values=[], width=260)
        self.port_menu.pack(padx=12, pady=3)
        row = ctk.CTkFrame(p, fg_color="transparent")
        row.pack(fill="x", padx=12, pady=3)
        ctk.CTkButton(row, text="Odswiez porty", width=120,
                      command=self._refresh_ports).pack(side="left")
        self.connect_btn = ctk.CTkButton(row, text="Polacz", width=120,
                                          command=self._toggle_connect,
                                          fg_color="#1a5c33")
        self.connect_btn.pack(side="right")

        # ── Temperatury ──
        self._section(p, "Temperatury")
        tf = ctk.CTkFrame(p, fg_color="#0e0e20", corner_radius=8)
        tf.pack(fill="x", padx=12, pady=3)
        self.t1_var = tk.StringVar(value="---")
        self.t2_var = tk.StringVar(value="---")
        ctk.CTkLabel(tf, text="T1 (CS9) regulacja",
                     text_color=C_T1, font=("Courier", 10)).grid(
            row=0, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(tf, textvariable=self.t1_var,
                     font=("Courier", 22, "bold"), text_color=C_T1).grid(
            row=0, column=1, padx=10)
        ctk.CTkLabel(tf, text="T2 (CS10) pomiar",
                     text_color=C_T2L, font=("Courier", 10)).grid(
            row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(tf, textvariable=self.t2_var,
                     font=("Courier", 22, "bold"), text_color=C_T2L).grid(
            row=1, column=1, padx=10)

        # ── PID ──
        self._section(p, "Nastawy PID")
        self._entry_row(p, "Setpoint [C]",  "25.0", "sp_entry")
        self._entry_row(p, "Kp",            "10.0", "kp_entry")
        self._entry_row(p, "Ki",            "0.3",  "ki_entry")
        self._entry_row(p, "Kd",            "0.8",  "kd_entry")

        self.pid_var  = tk.BooleanVar(value=False)
        self.heat_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(p, text="Wlacz PID", variable=self.pid_var,
                        command=self._on_pid_toggle).pack(anchor="w", padx=16, pady=3)
        ctk.CTkCheckBox(p, text="Tryb GRZANIA (odznacz = chlodzenie)",
                        variable=self.heat_var,
                        command=self._send_settings).pack(anchor="w", padx=16, pady=2)

        ctk.CTkButton(p, text="Wyslij nastawy PID",
                      command=self._send_settings).pack(fill="x", padx=12, pady=5)
        ctk.CTkButton(p, text="STOP (wylacz Peltier)", fg_color="#6a1010",
                      command=self._send_stop).pack(fill="x", padx=12, pady=2)

        # ── Wentylator ──
        self._section(p, "Wentylator")
        self.fan_auto_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(p, text="Automatyczny", variable=self.fan_auto_var,
                        command=self._send_settings).pack(anchor="w", padx=16, pady=3)
        ctk.CTkLabel(p, text="Reczna moc [%]", anchor="w").pack(fill="x", padx=12)
        self.fan_slider = ctk.CTkSlider(p, from_=0, to=100, number_of_steps=100,
                                         command=self._on_fan_slide)
        self.fan_slider.set(0)
        self.fan_slider.pack(fill="x", padx=12, pady=3)
        self.fan_pct_lbl = tk.StringVar(value="0 %")
        ctk.CTkLabel(p, textvariable=self.fan_pct_lbl,
                     font=("Courier", 12)).pack(pady=1)

        # ── Wyjscia ──
        self._section(p, "Wyjscia aktualne")
        of = ctk.CTkFrame(p, fg_color="#0e0e20", corner_radius=8)
        of.pack(fill="x", padx=12, pady=3)
        self.pelt_var   = tk.StringVar(value="---")
        self.fanout_var = tk.StringVar(value="---")
        ctk.CTkLabel(of, text="Peltier",    text_color=C_PELT).grid(
            row=0, column=0, padx=10, pady=5)
        ctk.CTkLabel(of, textvariable=self.pelt_var,
                     font=("Courier", 15, "bold"), text_color=C_PELT).grid(
            row=0, column=1, padx=10)
        ctk.CTkLabel(of, text="Wentylator", text_color=C_FAN).grid(
            row=1, column=0, padx=10, pady=5)
        ctk.CTkLabel(of, textvariable=self.fanout_var,
                     font=("Courier", 15, "bold"), text_color=C_FAN).grid(
            row=1, column=1, padx=10)

        # ── Zapis ──
        self._section(p, "Zapis CSV")
        self.log_btn = ctk.CTkButton(p, text="Rozpocznij zapis",
                                      fg_color="#1a3a5a",
                                      command=self._toggle_logging)
        self.log_btn.pack(fill="x", padx=12, pady=5)
        self.log_lbl = tk.StringVar(value="")
        ctk.CTkLabel(p, textvariable=self.log_lbl, font=("Courier", 8),
                     text_color="#445566", wraplength=260).pack(padx=12)

    def _entry_row(self, parent, label, default, attr):
        ctk.CTkLabel(parent, text=label, anchor="w").pack(fill="x", padx=12, pady=(3, 0))
        e = ctk.CTkEntry(parent)
        e.insert(0, default)
        e.pack(fill="x", padx=12, pady=2)
        setattr(self, attr, e)

    # ─── WYKRES ───────────────────────────────────────────────────────────────
    def _build_chart(self):
        self.fig = Figure(figsize=(6, 4), facecolor=CHART_BG)
        self.fig.subplots_adjust(left=0.07, right=0.94, top=0.92,
                                  bottom=0.09, hspace=0.35)

        self.ax_t = self.fig.add_subplot(2, 1, 1, facecolor="#080818")
        self.ax_t.set_title("Temperatura [C]", color="#aaaaaa", fontsize=10)
        self.ax_t.tick_params(colors="#555555")
        for sp in self.ax_t.spines.values(): sp.set_edgecolor("#222244")

        self.ax_p = self.fig.add_subplot(2, 1, 2, facecolor="#080818")
        self.ax_p.set_title("Moc wyjsc [%]", color="#aaaaaa", fontsize=10)
        self.ax_p.set_ylim(0, 105)
        self.ax_p.tick_params(colors="#555555")
        for sp in self.ax_p.spines.values(): sp.set_edgecolor("#222244")

        self.ln_t1, = self.ax_t.plot([], [], color=C_T1,  lw=2, label="T1")
        self.ln_t2, = self.ax_t.plot([], [], color=C_T2L, lw=1.5, ls="--", label="T2")
        self.ln_sp, = self.ax_t.plot([], [], color=C_SP,  lw=1,   ls=":",  label="SP")
        self.ax_t.legend(loc="upper left", facecolor="#111122",
                          labelcolor="#cccccc", fontsize=8)

        self.ln_pw, = self.ax_p.plot([], [], color=C_PELT, lw=2,   label="Peltier")
        self.ln_fn, = self.ax_p.plot([], [], color=C_FAN,  lw=1.5, ls="--", label="Wentylator")
        self.ax_p.legend(loc="upper left", facecolor="#111122",
                          labelcolor="#cccccc", fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.chart_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _update_chart(self, _):
        if not self.ts_hist: return
        xs = list(self.ts_hist)
        def s(lst): return [v if v is not None else float("nan") for v in lst]
        self.ln_t1.set_data(xs, s(self.t1_hist))
        self.ln_t2.set_data(xs, s(self.t2_hist))
        self.ln_sp.set_data(xs, list(self.sp_hist))
        self.ln_pw.set_data(xs, list(self.pw_hist))
        self.ln_fn.set_data(xs, list(self.fn_hist))
        self.ax_t.relim(); self.ax_t.autoscale_view()
        self.ax_p.relim(); self.ax_p.autoscale_view(scaley=False)
        self.canvas.draw_idle()

    # ─── SERIAL ───────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            self.port_menu.configure(values=["(brak portow)"])
            self.status_var.set("Brak portow COM - podlacz ItsyBitsy")
            return
        self.port_menu.configure(values=ports)
        self.port_var.set(ports[0])
        self.status_var.set(f"Znaleziono {len(ports)} port(ow)")

    def _toggle_connect(self):
        if self.running: self._disconnect()
        else:            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port or port.startswith("("):
            self.status_var.set("Wybierz prawidlowy port COM")
            return
        try:
            with self._lock:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=2,
                    write_timeout=2
                )
            self.running = True
            t = threading.Thread(target=self._read_loop, daemon=True)
            t.start()
            self.connect_btn.configure(text="Rozlacz", fg_color="#6a1a1a")
            self.status_var.set(f"Polaczono: {port}  |  115200 baud")
        except serial.SerialException as e:
            self.status_var.set(f"Blad polaczenia: {e}")

    def _disconnect(self):
        self.running = False
        with self._lock:
            if self.ser:
                try: self.ser.close()
                except: pass
                self.ser = None
        self.connect_btn.configure(text="Polacz", fg_color="#1a5c33")
        self.status_var.set("Rozlaczono")

    def _read_loop(self):
        """Watek czytajacy dane z Serial. Buforuje i parsuje JSON."""
        buf = ""
        while self.running:
            try:
                with self._lock:
                    ser = self.ser
                if ser is None or not ser.is_open:
                    break
                # Czytaj dostepne bajty (nieblokujaco)
                n = ser.in_waiting
                if n > 0:
                    chunk = ser.read(n).decode("utf-8", errors="replace")
                    buf += chunk
                    # Przetwarzaj pelne linie
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line.startswith("{"):
                            try:
                                data = json.loads(line)
                                self.data_queue.put(data)
                            except json.JSONDecodeError:
                                pass
                else:
                    time.sleep(0.02)
            except serial.SerialException:
                if self.running:
                    self.after(0, self._on_serial_error)
                break
            except Exception:
                time.sleep(0.05)

    def _on_serial_error(self):
        self._disconnect()
        self.status_var.set("Utracono polaczenie - kliknij Polacz ponownie")

    def _poll_queue(self):
        try:
            while True:
                data = self.data_queue.get_nowait()
                self._handle_data(data)
        except queue.Empty:
            pass
        self.after(150, self._poll_queue)

    def _handle_data(self, d):
        dtype = d.get("type", "data")
        if dtype == "status":
            self.status_var.set(f"Status: {d.get('msg','')}")
            return
        if dtype == "cfg":
            # Synchronizuj UI z nastawami z urzadzenia
            self._sync_cfg(d)
            return
        if dtype != "data":
            return

        t1  = d.get("t1")
        t2  = d.get("t2")
        sp  = d.get("sp",  0)
        pct = d.get("pct", 0)
        fn  = d.get("fan", 0)
        pid = d.get("pid_on", False)

        self.t1_var.set(f"{t1:.1f} C" if t1 is not None else "BLAD")
        self.t2_var.set(f"{t2:.1f} C" if t2 is not None else "---")
        self.pelt_var.set(f"{pct:.0f} %")
        self.fanout_var.set(f"{fn:.0f} %")

        ts = time.time() - self.t_start
        self.ts_hist.append(ts)
        self.t1_hist.append(t1)
        self.t2_hist.append(t2)
        self.sp_hist.append(sp)
        self.pw_hist.append(pct)
        self.fn_hist.append(fn)

        if self.logging_active and self.log_writer:
            self.log_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"),
                t1, t2, sp, pct, fn, pid, d.get("heat"), d.get("kp"), d.get("ki"), d.get("kd")
            ])
            self.log_file.flush()

    def _sync_cfg(self, d):
        """Synchronizuj pola UI z nastawami odczytanymi z urzadzenia."""
        try:
            self.sp_entry.delete(0, "end"); self.sp_entry.insert(0, str(d.get("sp", 25)))
            self.kp_entry.delete(0, "end"); self.kp_entry.insert(0, str(d.get("kp", 10)))
            self.ki_entry.delete(0, "end"); self.ki_entry.insert(0, str(d.get("ki", 0.3)))
            self.kd_entry.delete(0, "end"); self.kd_entry.insert(0, str(d.get("kd", 0.8)))
            self.pid_var.set(bool(d.get("pid_on", False)))
            self.heat_var.set(bool(d.get("heat", True)))
            self.fan_auto_var.set(bool(d.get("fan_auto", True)))
            self.fan_slider.set(float(d.get("fan_man", 0)))
        except Exception:
            pass

    # ─── WYSYLANIE KOMEND ─────────────────────────────────────────────────────
    def _send(self, cmd: str):
        """Wyslij komende do urzadzenia. cmd bez znaku nowej linii."""
        with self._lock:
            ser = self.ser
        if ser is None or not ser.is_open:
            self.status_var.set("Nie polaczono - najpierw kliknij Polacz")
            return
        try:
            ser.write((cmd + "\n").encode("utf-8"))
        except serial.SerialException as e:
            self.status_var.set(f"Blad wysylania: {e}")

    def _send_settings(self):
        try:
            sp = float(self.sp_entry.get())
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
        except ValueError:
            self.status_var.set("Blad: sprawdz wartosci numeryczne")
            return
        self._send(f"SP:{sp}")
        self._send(f"KP:{kp}")
        self._send(f"KI:{ki}")
        self._send(f"KD:{kd}")
        self._send(f"HEAT:{1 if self.heat_var.get() else 0}")
        self._send(f"FANAUTO:{1 if self.fan_auto_var.get() else 0}")
        self._send(f"FAN:{self.fan_slider.get()}")

    def _on_pid_toggle(self):
        if self.pid_var.get():
            self._send_settings()
            self._send("START")
        else:
            self._send("STOP")

    def _send_stop(self):
        self.pid_var.set(False)
        self._send("STOP")

    def _on_fan_slide(self, val):
        self.fan_pct_lbl.set(f"{int(float(val))} %")
        if not self.fan_auto_var.get():
            self._send(f"FAN:{int(float(val))}")

    # ─── LOGOWANIE ────────────────────────────────────────────────────────────
    def _toggle_logging(self):
        if self.logging_active:
            self.logging_active = False
            if self.log_file:
                self.log_file.close()
                self.log_file = self.log_writer = None
            self.log_btn.configure(text="Rozpocznij zapis", fg_color="#1a3a5a")
            self.log_lbl.set("")
        else:
            fname = f"peltier_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.log_file = open(fname, "w", newline="", encoding="utf-8")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "timestamp","T1_C","T2_C","setpoint_C",
                "peltier_pct","fan_pct","pid_on","heat_mode",
                "Kp","Ki","Kd"
            ])
            self.logging_active = True
            self.log_btn.configure(text="Zatrzymaj zapis", fg_color="#6a1a1a")
            self.log_lbl.set(fname)

    # ─── ZAMKNIECIE ───────────────────────────────────────────────────────────
    def _on_close(self):
        self._send("STOP")
        self._disconnect()
        if self.logging_active:
            self._toggle_logging()
        self.destroy()


if __name__ == "__main__":
    app = PeltierApp()
    app.mainloop()
