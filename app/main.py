#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PeltierControl - BRUTALIST
Panel sterowania PID Peltiera (Feed-Forward) z dwukierunkowa komunikacja JSON.
Firmware: ItsyBitsy M0 + Cytron MDD10A + 2x MAX31856
"""

import sys, os, time, csv, json, threading, queue, socket
from datetime import datetime
from pathlib import Path

try:
    import serial, serial.tools.list_ports
except ImportError:
    print("pip install pyserial"); input(); sys.exit(1)
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError:
    print("brak tkinter"); input(); sys.exit(1)
try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except ImportError as e:
    print(f"pip install matplotlib\n{e}"); input(); sys.exit(1)

# ════════════════════════════════════════════════════════
#  MOTYW BRUTALIST
# ════════════════════════════════════════════════════════
C = {
    'bg':      '#3a3d42', 'bg2': '#2b2d31', 'panel': '#33363b',
    'panel2':  '#2b2d31', 'panel3': '#42454a',
    'border':  '#4a4d52', 'border2': '#5a5d63',
    'text':    '#f0f0f0', 'dim': '#b0b3b8', 'dim2': '#6a6d72',
    'blue':    '#4d9fff', 'orange': '#e8a33d', 'yellow': '#e8c63d',
    'green':   '#5fc77f', 'red': '#d4452e', 'cyan': '#4db8d4',
    'purple':  '#a87dd4', 'grid': '#42454a',
}

FONT = 'Consolas'
FS = 1.0
def fsz(n): return max(6, int(round(n * FS)))

def _lighten(hex_color, amount=0.15):
    h = hex_color.lstrip('#')
    r = min(255, int(int(h[0:2],16) + (255-int(h[0:2],16))*amount))
    g = min(255, int(int(h[2:4],16) + (255-int(h[2:4],16))*amount))
    b = min(255, int(int(h[4:6],16) + (255-int(h[4:6],16))*amount))
    return f'#{r:02x}{g:02x}{b:02x}'

def mk_btn(parent, text, cmd, bg=None, fg='#1a1c1f', **kw):
    bg = bg or C['green']
    b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg,
                  font=(FONT, fsz(10), 'bold'), padx=16, pady=8,
                  relief='flat', cursor='hand2', bd=0,
                  activebackground=_lighten(bg, 0.15), activeforeground=fg, **kw)
    def on_enter(e):
        if b['state'] != 'disabled': b.config(bg=_lighten(bg, 0.15))
    def on_leave(e):
        if b['state'] != 'disabled': b.config(bg=bg)
    b.bind('<Enter>', on_enter); b.bind('<Leave>', on_leave)
    return b

def mk_btn_outline(parent, text, cmd, color, **kw):
    return tk.Button(parent, text=text, command=cmd, bg=C['bg2'], fg=color,
                  font=(FONT, fsz(10), 'bold'), padx=14, pady=7,
                  relief='flat', cursor='hand2', bd=0,
                  highlightthickness=2, highlightbackground=color,
                  highlightcolor=color,
                  activebackground=C['panel3'], activeforeground=color, **kw)

# ════════════════════════════════════════════════════════
#  SLIDER + POLE LICZBOWE
# ════════════════════════════════════════════════════════
class SliderField:
    def __init__(self, parent, label, vmin, vmax, vinit, color,
                 unit='', decimals=1, on_change=None, width=170):
        self.vmin=vmin; self.vmax=vmax; self.color=color
        self.decimals=decimals; self.on_change=on_change
        self._last_sent=None; self._after_id=None

        self.frame = tk.Frame(parent, bg=C['bg2'])
        self.frame.pack(fill='x', pady=(0, 14))

        top = tk.Frame(self.frame, bg=C['bg2'])
        top.pack(fill='x')
        tk.Label(top, text=label, bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9)), anchor='w').pack(side='left')
        if unit:
            tk.Label(top, text=unit, bg=C['bg2'], fg=C['dim2'],
                     font=(FONT, fsz(8)), anchor='e').pack(side='right')

        row = tk.Frame(self.frame, bg=C['bg2'])
        row.pack(fill='x', pady=(4, 0))

        self.entry = tk.Entry(row, width=7, bg=C['panel'], fg=color,
                              font=(FONT, fsz(12), 'bold'), justify='center',
                              relief='flat', bd=0,
                              highlightthickness=1, highlightbackground=color,
                              insertbackground=color)
        self.entry.pack(side='right', ipady=4, padx=(8, 0))
        self.entry.bind('<Return>', self._on_entry)
        self.entry.bind('<FocusOut>', self._on_entry)

        self.var = tk.DoubleVar(value=vinit)
        self.scale = tk.Scale(row, from_=vmin, to=vmax, resolution=10**(-decimals),
                             orient='horizontal', variable=self.var,
                             showvalue=False, bg=C['bg2'], fg=color,
                             troughcolor=C['panel'], highlightthickness=0,
                             bd=0, sliderrelief='flat', sliderlength=18,
                             activebackground=color, length=width,
                             command=self._on_slide)
        self.scale.pack(side='right', fill='x', expand=True)
        self._set_entry(vinit)

    def _set_entry(self, v):
        self.entry.delete(0, 'end')
        self.entry.insert(0, f"{v:.{self.decimals}f}")

    def _on_slide(self, val):
        v = float(val); self._set_entry(v); self._debounced(v)

    def _on_entry(self, evt=None):
        try:
            v = float(self.entry.get().replace(',', '.'))
            v = max(self.vmin, min(self.vmax, v))
            self.var.set(v); self._set_entry(v); self._debounced(v)
        except ValueError:
            self._set_entry(self.var.get())

    def _debounced(self, v):
        if self._after_id: self.frame.after_cancel(self._after_id)
        self._after_id = self.frame.after(150, lambda: self._emit(v))

    def _emit(self, v):
        if self.on_change and v != self._last_sent:
            self._last_sent = v
            self.on_change(v)

    def get(self): return self.var.get()

    def set(self, v, silent=True):
        v = max(self.vmin, min(self.vmax, v))
        if silent: self._last_sent = v
        self.var.set(v); self._set_entry(v)

    def set_enabled(self, en):
        st = 'normal' if en else 'disabled'
        self.scale.config(state=st); self.entry.config(state=st)

# ════════════════════════════════════════════════════════
#  KEITHLEY 2611B - klient TSP przez raw socket (port 5025)
# ════════════════════════════════════════════════════════
class KeithleyClient:
    """Komunikacja z Keithley 2611B przez TSP (Lua) na porcie 5025 (scpi-raw).
    Domyslny jezyk komend serii 2600B to TSP, nie SCPI."""

    PORT = 5025
    TIMEOUT = 2.0

    def __init__(self):
        self.sock = None
        self.connected = False
        self.ip = ""
        self.idn = ""

    def connect(self, ip):
        self.ip = ip
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.TIMEOUT)
        s.connect((ip, self.PORT))
        s.settimeout(self.TIMEOUT)
        self.sock = s
        self.connected = True
        try:
            self.idn = self._query("*IDN?")
        except Exception:
            self.idn = ""
        return self.idn

    def disconnect(self):
        self.connected = False
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None

    def _send(self, cmd):
        if not self.sock:
            raise ConnectionError("Keithley not connected")
        self.sock.sendall((cmd + "\n").encode("ascii"))

    def _recv_line(self):
        buf = b""
        while True:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            if b"\n" in buf:
                break
        return buf.decode("ascii", errors="replace").strip()

    def _query(self, cmd):
        self._send(cmd)
        return self._recv_line()

    def _exec(self, cmd):
        """Wykonaj komende TSP bez oczekiwania odpowiedzi (np. przypisania)."""
        self._send(cmd)

    def setup_source_v_measure_i(self, channel="a", voltage=0.0, ilimit=0.1):
        """Konfiguruje SMU: zrodlo napiecia, pomiar pradu, dany limit pradowy (compliance)."""
        ch = f"smu{channel}"
        self._exec(f"{ch}.reset()")
        self._exec(f"{ch}.source.func = {ch}.OUTPUT_DCVOLTS")
        self._exec(f"{ch}.source.levelv = {voltage:.6f}")
        self._exec(f"{ch}.source.limiti = {ilimit:.6f}")
        self._exec(f"{ch}.measure.nplc = 0.1")
        self._exec(f"{ch}.measure.autorangei = {ch}.AUTORANGE_ON")

    def output_on(self, channel="a"):
        self._exec(f"smu{channel}.source.output = smu{channel}.OUTPUT_ON")

    def output_off(self, channel="a"):
        self._exec(f"smu{channel}.source.output = smu{channel}.OUTPUT_OFF")

    def set_voltage(self, channel="a", voltage=0.0):
        self._exec(f"smu{channel}.source.levelv = {voltage:.6f}")

    def measure_iv(self, channel="a"):
        """Zwraca (prad_A, napiecie_V) z jednego zapytania (szybsze niz dwa osobne)."""
        resp = self._query(f"print(smu{channel}.measure.i(), smu{channel}.measure.v())")
        parts = resp.replace(",", " ").split()
        i_val = float(parts[0])
        v_val = float(parts[1]) if len(parts) > 1 else float('nan')
        return i_val, v_val

    def measure_i(self, channel="a"):
        resp = self._query(f"print(smu{channel}.measure.i())")
        return float(resp)


# ════════════════════════════════════════════════════════
#  APLIKACJA GLOWNA
# ════════════════════════════════════════════════════════
class PeltierControl:
    def __init__(self, root):
        self.root = root
        self.root.title("PeltierControl - BRUTALIST")
        self.root.configure(bg=C['bg'])
        self.root.geometry("1280x800")
        self.root.minsize(1100, 720)

        self.ser = None
        self.port_name = None
        self.baud = 115200
        self.running = False
        self.connected = False
        self._lock = threading.Lock()

        self.maxlen = 3000
        self.t = []; self.temp1 = []; self.temp2 = []
        self.spt = []; self.spa = []; self.pwm = []; self.fanv = []
        self.t0 = None
        self.data_queue = queue.Queue()

        self.raw_maxrows = 2000
        self.raw_rows = []
        self.raw_paused = False
        self.raw_autoscroll = True

        # Keithley 2611B (SMU) - pomiar pradu przez LAN/TSP, synchronizowany z PID
        self.keithley = KeithleyClient()
        self.keithley_connected = False
        self.keithley_running = False
        self.keithley_thread = None
        self.keithley_lock = threading.Lock()
        self.keithley_last_i = None
        self.keithley_last_v = None
        self.keithley_last_ts = None
        self.keithley_ip = ""
        self.keithley_voltage = 1.0
        self.keithley_ilimit = 0.1
        self.keithley_period_s = 0.1
        self.keithley_queue = queue.Queue()

        self.reach_start_t = None
        self.reach_start_temp = None
        self.reach_target = None
        self.reach_done = False
        self.reach_time = None
        self.reach_avg_rate = None
        self.reach_dir = None
        self.last_setpoint_target = None

        self.chart_paused = False
        self.chart_window = 0

        self.log_dir = Path.home() / "PeltierLogi"
        self.log_dir.mkdir(exist_ok=True)
        self.cyc_on = False; self.cyc_file = None; self.cyc_wr = None
        self.cyc_t0 = None; self.cyc_fn = None; self.cyc_rows = 0

        self._cfg_synced = False
        self.is_running = False
        self.fan_on = False
        self._cmd_buf = ""
        self._pulse_state = 0

        self._build_styles()
        self._build_ui()
        self._pulse()
        self.tick()
        self.root.after(800, self._auto_connect)

    # ─── AUTO-CONNECT ────────────────────────────────────
    def _auto_connect(self):
        if self.connected: return
        try: ports = list(serial.tools.list_ports.comports())
        except: return
        if not ports: return
        def score(p):
            d = (p.description or '').lower()
            s = 0
            for kw in ['itsybitsy', 'adafruit', 'usb serial', 'usb-serial']:
                if kw in d: s += 10
            if hasattr(p, 'vid') and p.vid == 0x239A: s += 20
            return s
        best = max(ports, key=score)
        if score(best) > 0 or len(ports) == 1:
            self.connect(best.device)

    def _build_styles(self):
        st = ttk.Style()
        try: st.theme_use('clam')
        except: pass
        st.configure('TNotebook', background=C['bg2'], borderwidth=0, tabmargins=[0,0,0,0])
        st.configure('TNotebook.Tab', background=C['bg2'], foreground=C['dim'],
                     padding=[20, 10], font=(FONT, fsz(10), 'bold'), borderwidth=0)
        st.map('TNotebook.Tab',
               background=[('selected', C['bg'])],
               foreground=[('selected', C['text'])])

    # ─── SERIAL ──────────────────────────────────────────
    def send(self, cmd):
        with self._lock: ser = self.ser
        if ser and ser.is_open:
            try: ser.write((cmd + '\n').encode())
            except Exception as e: print(f"send err: {e}")

    def connect(self, port):
        try:
            with self._lock:
                self.ser = serial.Serial(port, self.baud, timeout=0.5, write_timeout=2)
            self.port_name = port
            self.clear_buf()
            self._cfg_synced = False
            self.set_status(True, f"{port} - 115200")
            self.running = True
            threading.Thread(target=self.reader, daemon=True).start()
            self.root.after(1200, lambda: self.send("GET"))
        except Exception as e:
            messagebox.showerror("Error", f"{port}:\n{e}")
            self.set_status(False, "")

    def disconnect(self):
        self.running = False
        if self.cyc_on: self.cyc_stop("Rozlaczono")
        with self._lock:
            if self.ser:
                try: self.ser.close()
                except: pass
                self.ser = None
        self.set_status(False, "")

    def clear_buf(self):
        for a in [self.t, self.temp1, self.temp2, self.spt, self.spa, self.pwm, self.fanv]:
            a.clear()
        self.t0 = None

    def _parse_csv_line(self, line):
        # Format firmware: czas_s,temp_C,setpoint_akt,setpoint_cel,PWM,Kp,Ki,Kd,stan,temp2_C
        p = line.split(',')
        if len(p) < 9:
            return
        try:
            ts = float(p[0])
            temp = float(p[1])
            sa = float(p[2])
            st = float(p[3])
            pwm_raw = float(p[4])
            kp = float(p[5]); ki = float(p[6]); kd = float(p[7])
            state = p[8].strip()
        except (ValueError, IndexError):
            return
        temp2v = None
        if len(p) >= 10:
            try:
                v2 = float(p[9])
                temp2v = v2 if v2 != 0 else None
            except ValueError:
                pass
        pid_on = state.startswith('AUTO') or state.startswith('ST') or state.startswith('CAL') or state.startswith('FREEZE')
        d = {
            'type': 'data',
            'ts': ts * 1000.0,
            't1': temp,
            't2': temp2v,
            'sp': st,
            'spa': sa,
            'pct': abs(pwm_raw) / 255.0 * 100.0,
            'fan': self.sl_fan.get() if (self.fan_on and hasattr(self, 'sl_fan')) else 0.0,
            'pid_on': pid_on,
            'heat': pwm_raw >= 0,
            'kp': kp, 'ki': ki, 'kd': kd,
            'state': state,
        }
        self.data_queue.put(d)

    def _parse_cfg_line(self, cfg):
        # Format: SP=25.50,RU=2.00,RD=2.00,TMAX=110.0,KP=10.000,KI=0.3000,KD=0.800,...
        d = {}
        for part in cfg.split(','):
            if '=' in part:
                k, v = part.split('=', 1)
                d[k.strip()] = v.strip()
        out = {'type': 'cfg'}
        try:
            if 'SP' in d:   out['sp'] = float(d['SP'])
            if 'RU' in d:   out['ru'] = float(d['RU'])
            if 'KP' in d:   out['kp'] = float(d['KP'])
            if 'KI' in d:   out['ki'] = float(d['KI'])
            if 'KD' in d:   out['kd'] = float(d['KD'])
            if 'KFFH' in d: out['kffh'] = float(d['KFFH'])
            if 'KFFR' in d: out['kffr'] = float(d['KFFR'])
            if 'OFFSET' in d: out['offset'] = float(d['OFFSET'])
            if 'FAN' in d:
                fv = float(d['FAN'])
                self.fan_on = fv > 0
        except ValueError:
            pass
        self.data_queue.put(out)

    def reader(self):
        with self._lock: ser = self.ser
        if ser and ser.is_open:
            try: ser.reset_input_buffer()
            except: pass
        buf = ""
        while self.running:
            try:
                with self._lock: ser = self.ser
                if not ser or not ser.is_open: break
                n = ser.in_waiting
                if n > 0:
                    chunk = ser.read(n).decode('utf-8', errors='replace')
                    buf += chunk
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        if line.startswith('{'):
                            try: self.data_queue.put(json.loads(line))
                            except: pass
                        elif line.startswith('CFG:'):
                            self._parse_cfg_line(line[4:])
                        elif line[0].isdigit() or (line[0]=='-' and len(line)>1 and line[1].isdigit()):
                            self._parse_csv_line(line)
                else:
                    time.sleep(0.02)
            except serial.SerialException:
                self.running = False
                self.root.after(0, lambda: self.set_status(False, "Utracono polaczenie"))
                break
            except Exception as e:
                if self.running: print(f"reader err: {e}")
                time.sleep(0.2)

    # ─── UI ──────────────────────────────────────────────
    def _build_ui(self):
        top = tk.Frame(self.root, bg=C['bg2'], height=44)
        top.pack(fill='x'); top.pack_propagate(False)
        tk.Frame(top, bg=C['red'], width=6).pack(side='left', fill='y')
        tk.Label(top, text="  PELTIER CONTROL", bg=C['bg2'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(side='left', padx=(8, 0))
        tk.Label(top, text="ItsyBitsy M0 + Cytron MDD10A", bg=C['bg2'], fg=C['dim2'],
                 font=(FONT, fsz(9))).pack(side='left', padx=8)

        sf = tk.Frame(top, bg=C['bg2'])
        sf.pack(side='right', padx=16)
        self.s_dot = tk.Canvas(sf, width=14, height=14, bg=C['bg2'], highlightthickness=0)
        self.s_dot.pack(side='left', padx=(0, 8))
        self._draw_dot(C['dim2'], glow=False)
        self.s_lbl = tk.Label(sf, text="DISCONNECTED", bg=C['bg2'], fg=C['dim'],
                              font=(FONT, fsz(10)))
        self.s_lbl.pack(side='left')

        nb = ttk.Notebook(self.root)
        nb.pack(fill='both', expand=True)
        t1 = tk.Frame(nb, bg=C['bg']); nb.add(t1, text='CONTROL')
        t2 = tk.Frame(nb, bg=C['bg']); nb.add(t2, text='ADVANCED')
        t5 = tk.Frame(nb, bg=C['bg']); nb.add(t5, text='RAW DATA')
        t3 = tk.Frame(nb, bg=C['bg']); nb.add(t3, text='ARCHIVE')
        t4 = tk.Frame(nb, bg=C['bg']); nb.add(t4, text='CONNECTION')
        self.build_live(t1)
        self.build_advanced(t2)
        self.build_raw(t5)
        self.build_arch(t3)
        self.build_conn(t4)

    def _draw_dot(self, color, glow=True):
        self.s_dot.delete('all')
        if glow:
            self.s_dot.create_oval(0, 0, 14, 14, fill='', outline=color, width=1)
        self.s_dot.create_rectangle(3, 3, 11, 11, fill=color, outline='')

    def _pulse(self):
        if self.connected:
            self._pulse_state = (self._pulse_state + 1) % 20
            phase = abs(self._pulse_state - 10) / 10.0
            self._draw_dot(_lighten(C['green'], phase * 0.4))
        self.root.after(80, self._pulse)

    def set_status(self, connected, msg):
        self.connected = connected
        if connected:
            self._draw_dot(C['green'])
            self.s_lbl.config(text=msg or "CONNECTED", fg=C['green'])
        else:
            self._draw_dot(C['dim2'], glow=False)
            self.s_lbl.config(text=msg or "DISCONNECTED", fg=C['dim'])
        if hasattr(self, 'btn_run'):
            self._set_panel_enabled(connected)

    # ─── EKRAN LIVE ──────────────────────────────────────
    def build_live(self, parent):
        topbar = tk.Frame(parent, bg=C['bg'])
        topbar.pack(fill='x', padx=16, pady=(10, 6))

        cards = tk.Frame(topbar, bg=C['bg'])
        cards.pack(side='left', fill='x', expand=True)
        self.cards = {}
        self.cards['temp']  = self._stat_card(cards, "TEMP T1", "°C", C['blue'])
        self.cards['temp2'] = self._stat_card(cards, "TEMP T2", "°C", C['cyan'])
        self.cards['sp']    = self._stat_card(cards, "SETPOINT", "°C", C['orange'])
        self.cards['rate']  = self._stat_card(cards, "AVG RATE", "°C/min", C['yellow'])
        self.cards['pwm']   = self._stat_card(cards, "PWM", "%", C['green'])
        self.cards['kcur']  = self._stat_card(cards, "I KEITHLEY", "A", C['orange'])

        ctrl = tk.Frame(topbar, bg=C['bg'])
        ctrl.pack(side='right', padx=(8, 0))
        self.btn_run = tk.Button(ctrl, text="▶ START", command=self.toggle_run,
                                 bg=C['green'], fg='#1a1c1f', font=(FONT, fsz(12), 'bold'),
                                 relief='flat', cursor='hand2', bd=0, padx=16, pady=12,
                                 activebackground=_lighten(C['green'], 0.15))
        self.btn_run.pack(side='left', padx=(0, 4), fill='y')
        self.btn_estop = tk.Button(ctrl, text="⛔", command=self.do_estop,
                                   bg=C['red'], fg='#fff', font=(FONT, fsz(14), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=12, pady=12,
                                   activebackground=_lighten(C['red'], 0.15))
        self.btn_estop.pack(side='left', fill='y')

        main = tk.Frame(parent, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=(0, 12))

        # PRAWO - panel sterowania (PAKOWANY PIERWSZY zeby zachowac szerokosc)
        self._build_panel(main)
        # LEWO - wykres
        self._build_chart(main)

    def _stat_card(self, parent, title, unit, color):
        card = tk.Frame(parent, bg=C['panel'])
        card.pack(side='left', fill='x', expand=True, padx=(0, 4))
        tk.Frame(card, bg=color, height=3).pack(fill='x')
        inner = tk.Frame(card, bg=C['panel'])
        inner.pack(fill='both', expand=True, padx=7, pady=5)
        tk.Label(inner, text=title, bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(7)), anchor='w').pack(anchor='w')
        vrow = tk.Frame(inner, bg=C['panel'])
        vrow.pack(anchor='w', pady=(1, 0))
        val = tk.Label(vrow, text="--", bg=C['panel'], fg=color,
                       font=(FONT, fsz(16), 'bold'))
        val.pack(side='left')
        unit_lbl = tk.Label(vrow, text=" " + unit, bg=C['panel'], fg=C['dim2'],
                            font=(FONT, fsz(7)))
        unit_lbl.pack(side='left', pady=(4, 0))
        return {'val': val, 'unit': unit, 'unit_lbl': unit_lbl}

    def _build_chart(self, parent):
        wrap = tk.Frame(parent, bg=C['panel'])
        wrap.pack(side='left', fill='both', expand=True, padx=(0, 12))
        tk.Frame(wrap, bg=C['border2'], height=3).pack(fill='x')

        hd = tk.Frame(wrap, bg=C['panel'])
        hd.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(hd, text="LIVE CHART", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')

        self.reach_lbl = tk.Label(hd, text="", bg=C['panel'], fg=C['green'],
                                  font=(FONT, fsz(9), 'bold'))
        self.reach_lbl.pack(side='right')

        self.fig = Figure(figsize=(9, 6), facecolor=C['panel'], dpi=100)
        gs = self.fig.add_gridspec(2, 1, height_ratios=[3, 1], hspace=0.2,
                                   left=0.07, right=0.97, top=0.97, bottom=0.08)
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax2 = self.fig.add_subplot(gs[1], sharex=self.ax1)
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor(C['panel2'])

        self.cv = FigureCanvasTkAgg(self.fig, master=wrap)
        self.cv.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(0, 4))

        toolbar_row = tk.Frame(wrap, bg=C['panel'])
        toolbar_row.pack(fill='x', padx=8, pady=(0, 8))

        self.btn_pause = tk.Button(toolbar_row, text="⏸ PAUSE", command=self.toggle_pause,
                                   bg=C['bg2'], fg=C['yellow'], font=(FONT, fsz(9), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=12, pady=6,
                                   highlightthickness=1, highlightbackground=C['yellow'],
                                   activebackground=C['panel3'])
        self.btn_pause.pack(side='left', padx=(0, 6))

        tk.Label(toolbar_row, text="WINDOW:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8))).pack(side='left', padx=(8, 4))
        for label, secs in [("ALL", 0), ("5m", 300), ("2m", 120), ("1m", 60)]:
            b = tk.Button(toolbar_row, text=label,
                         command=lambda s=secs: self.set_chart_window(s),
                         bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(8)),
                         relief='flat', cursor='hand2', bd=0, padx=10, pady=5,
                         activebackground=C['panel3'])
            b.pack(side='left', padx=2)

        tb_frame = tk.Frame(toolbar_row, bg=C['panel'])
        tb_frame.pack(side='right')
        try:
            self.mpl_toolbar = NavigationToolbar2Tk(self.cv, tb_frame, pack_toolbar=False)
            self.mpl_toolbar.config(bg=C['panel'])
            self.mpl_toolbar.update()
            self.mpl_toolbar.pack(side='right')
        except Exception as e:
            print(f"toolbar err: {e}")

    def toggle_pause(self):
        self.chart_paused = not self.chart_paused
        if self.chart_paused:
            self.btn_pause.config(text="▶ RESUME", fg=C['green'], highlightbackground=C['green'])
        else:
            self.btn_pause.config(text="⏸ PAUSE", fg=C['yellow'], highlightbackground=C['yellow'])

    def set_chart_window(self, secs):
        self.chart_window = secs

    def _build_panel(self, parent):
        panel = tk.Frame(parent, bg=C['bg2'], width=312)
        panel.pack(side='right', fill='y')
        panel.pack_propagate(False)
        tk.Frame(panel, bg=C['red'], width=6).pack(side='left', fill='y')

        scroll_wrap = tk.Frame(panel, bg=C['bg2'])
        scroll_wrap.pack(side='left', fill='both', expand=True)
        pcanvas = tk.Canvas(scroll_wrap, bg=C['bg2'], highlightthickness=0, width=290)
        psb = tk.Scrollbar(scroll_wrap, orient='vertical', command=pcanvas.yview)
        pcanvas.configure(yscrollcommand=psb.set)
        psb.pack(side='right', fill='y')
        pcanvas.pack(side='left', fill='both', expand=True)

        inner = tk.Frame(pcanvas, bg=C['bg2'])
        inner_id = pcanvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: pcanvas.configure(scrollregion=pcanvas.bbox('all')))
        pcanvas.bind('<Configure>', lambda e: pcanvas.itemconfig(inner_id, width=e.width))
        pcanvas.bind('<Enter>', lambda e: pcanvas.bind_all('<MouseWheel>',
                     lambda ev: pcanvas.yview_scroll(int(-ev.delta/120), 'units')))
        pcanvas.bind('<Leave>', lambda e: pcanvas.unbind_all('<MouseWheel>'))

        inner = tk.Frame(inner, bg=C['bg2'])
        inner.pack(fill='both', expand=True, padx=16, pady=14)

        tk.Label(inner, text="CONTROL", bg=C['bg2'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w')
        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(8, 12))

        self.sl_sp = SliderField(inner, "TARGET", -15, 100, 25.0,
                                 C['orange'], "°C", 1,
                                 on_change=lambda v: self.send(f"SP:{v:.1f}"))
        self.sl_ru = SliderField(inner, "HEAT/COOL RATE", 0.5, 80, 2.0,
                                 C['yellow'], "°C/min", 1,
                                 on_change=lambda v: self.send(f"RU:{v:.1f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        fan_hd = tk.Frame(inner, bg=C['bg2'])
        fan_hd.pack(fill='x', pady=(0, 4))
        tk.Label(fan_hd, text="FANS", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        self.btn_fan = tk.Button(fan_hd, text="○ OFF", command=self.toggle_fan,
                                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(9), 'bold'),
                                 relief='flat', cursor='hand2', bd=0, padx=12, pady=4,
                                 highlightthickness=1, highlightbackground=C['dim'],
                                 activebackground=C['panel3'])
        self.btn_fan.pack(side='right')
        self.sl_fan = SliderField(inner, "FAN SPEED", 0, 100, 100,
                                  C['blue'], "%", 0,
                                  on_change=lambda v: self.set_fan_speed(v))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # Tryb grzania/chlodzenia
        auto_lbl = tk.Frame(inner, bg=C['bg2'], highlightthickness=1,
                            highlightbackground=C['green'])
        auto_lbl.pack(fill='x', pady=(0, 10))
        tk.Label(auto_lbl, text="AUTO: kierunek wg setpointu", bg=C['bg2'],
                 fg=C['green'], font=(FONT, fsz(9))).pack(padx=8, pady=6)

        tk.Label(inner, text="▶ START uses panel values",
                 bg=C['bg2'], fg=C['green'], font=(FONT, fsz(8))).pack(anchor='w', pady=(4, 0))
        tk.Label(inner, text="PID + Feed-Forward tuning → ADVANCED tab",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8))).pack(anchor='w', pady=(2, 0))

        self._set_panel_enabled(False)

    def build_advanced(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=20, pady=16)

        acanvas = tk.Canvas(wrap, bg=C['bg'], highlightthickness=0)
        asb = tk.Scrollbar(wrap, orient='vertical', command=acanvas.yview)
        acanvas.configure(yscrollcommand=asb.set)
        asb.pack(side='right', fill='y')
        acanvas.pack(side='left', fill='both', expand=True)
        col = tk.Frame(acanvas, bg=C['bg'])
        cid = acanvas.create_window((0, 0), window=col, anchor='nw')
        col.bind('<Configure>', lambda e: acanvas.configure(scrollregion=acanvas.bbox('all')))
        acanvas.bind('<Configure>', lambda e: acanvas.itemconfig(cid, width=e.width))
        acanvas.bind('<Enter>', lambda e: acanvas.bind_all('<MouseWheel>',
                     lambda ev: acanvas.yview_scroll(int(-ev.delta/120), 'units')))
        acanvas.bind('<Leave>', lambda e: acanvas.unbind_all('<MouseWheel>'))

        inner = tk.Frame(col, bg=C['bg'])
        inner.pack(fill='x', padx=4, pady=4)
        inner.configure(width=560)

        tk.Label(inner, text="ADVANCED — PID + FEED-FORWARD", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Manual gains tuning",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 16))

        sec1 = self._adv_section(inner, "PID TUNING", C['cyan'])
        self.sl_kp = SliderField(sec1, "Kp", 1, 30, 10.0, C['cyan'], "", 1,
                                 on_change=lambda v: self.send(f"KP:{v:.1f}"))
        self.sl_ki = SliderField(sec1, "Ki", 0, 1.5, 0.3, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KI:{v:.2f}"))
        self.sl_kd = SliderField(sec1, "Kd", 0, 80, 0.8, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KD:{v:.2f}"))

        sec2 = self._adv_section(inner, "FEED-FORWARD", C['yellow'])
        tk.Label(sec2, text="HOLD = moc bazowa na utrzymanie temp\nRAMP = dodatkowa moc na dynamike rampy",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8)),
                 justify='left').pack(anchor='w', pady=(0, 8))
        self.sl_kffh = SliderField(sec2, "FF HOLD (KFFH)", 0, 8, 2.5, C['yellow'], "PWM/10°C", 2,
                                   on_change=lambda v: self.send(f"KFFH:{v:.2f}"))
        self.sl_kffr = SliderField(sec2, "FF RAMP (KFFR)", 0, 4, 1.0, C['yellow'], "PWM/(°C/min)", 2,
                                   on_change=lambda v: self.send(f"KFFR:{v:.2f}"))

        sec3 = self._adv_section(inner, "THERMOCOUPLE", C['purple'])
        self.sl_off = SliderField(sec3, "CAL OFFSET", -20, 20, 0.0,
                                  C['purple'], "°C", 1,
                                  on_change=lambda v: self.send(f"OFFSET:{v:.1f}"))

        sec4 = self._adv_section(inner, "RESET", C['red'])
        mk_btn_outline(sec4, "↺ RESET PID GAINS", self.do_reset, C['red']).pack(fill='x')

    def _adv_section(self, parent, title, color):
        tk.Frame(parent, bg=color, height=2).pack(fill='x', pady=(12, 0))
        tk.Label(parent, text=title, bg=C['bg'], fg=color,
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(4, 6))
        box = tk.Frame(parent, bg=C['bg2'])
        box.pack(fill='x')
        inner = tk.Frame(box, bg=C['bg2'])
        inner.pack(fill='x', padx=12, pady=10)
        return inner

    def _set_panel_enabled(self, en):
        for sl in ['sl_sp', 'sl_ru', 'sl_kp', 'sl_ki', 'sl_kd', 'sl_kffh', 'sl_kffr', 'sl_off', 'sl_fan']:
            if hasattr(self, sl): getattr(self, sl).set_enabled(True)
        for b in ['btn_run', 'btn_estop', 'btn_fan']:
            if hasattr(self, b): getattr(self, b).config(state='normal')

    # ─── AKCJE ───────────────────────────────────────────
    def toggle_run(self):
        if self.is_running: self.do_stop()
        else: self.do_start()

    def _update_run_button(self, running):
        self.is_running = running
        if running:
            self.btn_run.config(text="■ STOP", bg=C['red'], fg='#fff',
                               activebackground=_lighten(C['red'], 0.15))
        else:
            self.btn_run.config(text="▶ START", bg=C['green'], fg='#1a1c1f',
                               activebackground=_lighten(C['green'], 0.15))

    def do_start(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        self.reach_start_t = None
        self.reach_start_temp = None
        self.reach_target = self.sl_sp.get()
        self.reach_done = False
        self.reach_time = None
        self.reach_avg_rate = None
        self.reach_dir = None
        self.last_setpoint_target = None
        if hasattr(self, 'reach_lbl'):
            self.reach_lbl.config(text="→ starting...", fg=C['dim'])
        self.send(f"SP:{self.sl_sp.get():.1f}")
        self.send(f"RU:{self.sl_ru.get():.1f}")
        self.send(f"RD:{self.sl_ru.get():.1f}")
        self.send(f"KP:{self.sl_kp.get():.1f}")
        self.send(f"KI:{self.sl_ki.get():.2f}")
        self.send(f"KD:{self.sl_kd.get():.2f}")
        self.send(f"KFFH:{self.sl_kffh.get():.2f}")
        self.send(f"KFFR:{self.sl_kffr.get():.2f}")
        self.send(f"OFFSET:{self.sl_off.get():.1f}")
        time.sleep(0.05)
        self.send("START")
        self._update_run_button(True)
        self.keithley_start_measurement()

    def do_stop(self):
        self.send("STOP")
        self._update_run_button(False)
        self.keithley_stop_measurement()

    def do_estop(self):
        self.send("STOP")
        self._update_run_button(False)
        self.keithley_stop_measurement()

    def toggle_fan(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        self.fan_on = not self.fan_on
        if self.fan_on:
            spd = int(self.sl_fan.get())
            if spd == 0: spd = 100; self.sl_fan.set(100, silent=True)
            self.send(f"FAN:{spd}")
            self.btn_fan.config(text="● ON", fg=C['green'], highlightbackground=C['green'])
        else:
            self.send("FANOFF")
            self.btn_fan.config(text="○ OFF", fg=C['dim2'], highlightbackground=C['dim'])

    def set_fan_speed(self, v):
        spd = int(v)
        self.send(f"FAN:{spd}")
        if spd > 0:
            self.fan_on = True
            self.btn_fan.config(text="● ON", fg=C['green'], highlightbackground=C['green'])
        else:
            self.fan_on = False
            self.btn_fan.config(text="○ OFF", fg=C['dim2'], highlightbackground=C['dim'])

    def do_reset(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if messagebox.askyesno("Reset PID gains", "Restore default Kp/Ki/Kd/FF?"):
            self.send("RESET")

    # ─── ZAKLADKA CONNECTION ─────────────────────────────
    # ─── ZAKLADKA RAW DATA ───────────────────────────────
    def build_raw(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=14, pady=14)

        hd = tk.Frame(wrap, bg=C['bg'])
        hd.pack(fill='x', pady=(0, 10))
        tk.Label(hd, text="RAW THERMOCOUPLE DATA", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')
        tk.Label(hd, text="  surowy strumien T1/T2 z urzadzenia, 10 Hz",
                 bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8))).pack(side='left', padx=(8, 0))

        self.btn_raw_pause = tk.Button(hd, text="⏸ PAUSE", command=self._toggle_raw_pause,
                                       bg=C['bg2'], fg=C['yellow'], font=(FONT, fsz(9), 'bold'),
                                       relief='flat', cursor='hand2', bd=0, padx=12, pady=6,
                                       highlightthickness=1, highlightbackground=C['yellow'],
                                       activebackground=C['panel3'])
        self.btn_raw_pause.pack(side='right', padx=(6, 0))
        mk_btn_outline(hd, "⤓ EXPORT CSV", self.export_raw_csv, C['green']).pack(side='right', padx=(6, 0))
        mk_btn_outline(hd, "WYCZYSC", self.clear_raw, C['dim']).pack(side='right', padx=(6, 0))

        self.raw_count_lbl = tk.Label(hd, text="0 probek", bg=C['bg'], fg=C['dim'],
                                      font=(FONT, fsz(9)))
        self.raw_count_lbl.pack(side='right', padx=(6, 12))

        # Tabela (Treeview)
        table_wrap = tk.Frame(wrap, bg=C['panel'])
        table_wrap.pack(fill='both', expand=True)
        tk.Frame(table_wrap, bg=C['blue'], height=3).pack(fill='x')

        cols = ('idx', 'czas_fw', 'pc_time', 't1', 't2', 'sp', 'spa', 'pct', 'fan', 'k_i', 'k_v', 'state')
        headers = {
            'idx': '#', 'czas_fw': 'czas FW [s]', 'pc_time': 'czas PC',
            't1': 'T1 [C]', 't2': 'T2 [C]', 'sp': 'SP cel [C]',
            'spa': 'SP akt [C]', 'pct': 'Peltier %', 'fan': 'Fan %',
            'k_i': 'I Keithley [A]', 'k_v': 'V Keithley [V]', 'state': 'stan'
        }
        widths = {
            'idx': 50, 'czas_fw': 90, 'pc_time': 110, 't1': 80, 't2': 80,
            'sp': 90, 'spa': 90, 'pct': 80, 'fan': 70,
            'k_i': 110, 'k_v': 100, 'state': 70
        }

        style = ttk.Style()
        style.configure('Raw.Treeview', background=C['bg2'], fieldbackground=C['bg2'],
                        foreground=C['text'], font=(FONT, fsz(9)), rowheight=22, borderwidth=0)
        style.configure('Raw.Treeview.Heading', background=C['panel'], foreground=C['dim'],
                        font=(FONT, fsz(9), 'bold'), borderwidth=0)
        style.map('Raw.Treeview', background=[('selected', C['panel3'])])

        tree_frame = tk.Frame(table_wrap, bg=C['panel'])
        tree_frame.pack(fill='both', expand=True, padx=8, pady=8)

        ysb = ttk.Scrollbar(tree_frame, orient='vertical')
        ysb.pack(side='right', fill='y')

        self.raw_tree = ttk.Treeview(tree_frame, columns=cols, show='headings',
                                     style='Raw.Treeview', yscrollcommand=ysb.set)
        for c in cols:
            self.raw_tree.heading(c, text=headers.get(c, c))
            self.raw_tree.column(c, width=widths.get(c, 80), anchor='center')
        self.raw_tree.pack(side='left', fill='both', expand=True)
        ysb.config(command=self.raw_tree.yview)

        # Wykrywaj reczne przewijanie - wylacz autoscroll
        def on_scroll(*a):
            self.raw_autoscroll = (self.raw_tree.yview()[1] >= 0.999)
        self.raw_tree.bind('<MouseWheel>', lambda e: setattr(self, 'raw_autoscroll', False))
        self.raw_tree.bind('<Button-4>', lambda e: setattr(self, 'raw_autoscroll', False))
        self.raw_tree.bind('<Button-5>', lambda e: setattr(self, 'raw_autoscroll', False))

        info = tk.Label(wrap, text="Tabela pokazuje ostatnie probki (max 2000). Pelny zapis surowych danych od START do STOP jest w zakladce ARCHIVE.",
                        bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8)))
        info.pack(anchor='w', pady=(8, 0))

    def _toggle_raw_pause(self):
        self.raw_paused = not self.raw_paused
        if self.raw_paused:
            self.btn_raw_pause.config(text="▶ RESUME", fg=C['green'], highlightbackground=C['green'])
        else:
            self.btn_raw_pause.config(text="⏸ PAUSE", fg=C['yellow'], highlightbackground=C['yellow'])

    def clear_raw(self):
        self.raw_rows = []
        if hasattr(self, 'raw_tree'):
            for item in self.raw_tree.get_children():
                self.raw_tree.delete(item)
        if hasattr(self, 'raw_count_lbl'):
            self.raw_count_lbl.config(text="0 probek")

    def _raw_append(self, row):
        # row = (czas_fw, pc_time, t1, t2, sp, spa, pct, fan, state, k_i, k_v)
        self.raw_rows.append(row)
        if len(self.raw_rows) > self.raw_maxrows:
            self.raw_rows = self.raw_rows[-self.raw_maxrows:]
            if hasattr(self, 'raw_tree'):
                children = self.raw_tree.get_children()
                if len(children) > self.raw_maxrows:
                    for item in children[:len(children)-self.raw_maxrows]:
                        self.raw_tree.delete(item)

        if self.raw_paused or not hasattr(self, 'raw_tree'):
            return
        idx = len(self.raw_rows)
        czas_fw, pc_time, t1, t2, sp, spa, pct, fan, state, k_i, k_v = row
        t1s = f"{t1:.3f}" if t1 is not None else "—"
        t2s = f"{t2:.3f}" if t2 is not None else "—"
        kis = f"{k_i:.6e}" if k_i is not None else "—"
        kvs = f"{k_v:.4f}" if k_v is not None else "—"
        # W trybie MAN firmware wysyla T1 zamiast spA (brak aktywnej rampy) -
        # pokazujemy myslnik zeby nie mylic uzytkownika ze rampa dziala
        spas = "—" if state == "MAN" else f"{spa:.2f}"
        self.raw_tree.insert('', 'end', values=(
            idx, f"{czas_fw:.2f}", pc_time, t1s, t2s,
            f"{sp:.2f}", spas, f"{pct:.1f}", f"{fan:.1f}", kis, kvs, state
        ))
        if self.raw_autoscroll:
            children = self.raw_tree.get_children()
            if children:
                self.raw_tree.see(children[-1])
        self.raw_count_lbl.config(text=f"{len(self.raw_rows)} probek")

    def export_raw_csv(self):
        if not self.raw_rows:
            messagebox.showinfo("Brak danych", "Brak danych do eksportu.")
            return
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Eksportuj surowe dane", defaultextension=".csv",
            initialfile=f"raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            filetypes=[("CSV", "*.csv")])
        if not dest:
            return
        try:
            with open(dest, 'w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(['czas_firmware_s', 'timestamp_pc', 'temperatura1_C',
                           'temperatura2_C', 'setpoint_cel_C', 'setpoint_aktywny_C',
                           'peltier_pct', 'fan_pct', 'keithley_prad_A',
                           'keithley_napiecie_V', 'stan'])
                for row in self.raw_rows:
                    czas_fw, pc_time, t1, t2, sp, spa, pct, fan, state, k_i, k_v = row
                    spas = "" if state == "MAN" else f"{spa:.3f}"
                    w.writerow([
                        f"{czas_fw:.3f}", pc_time,
                        f"{t1:.3f}" if t1 is not None else "",
                        f"{t2:.3f}" if t2 is not None else "",
                        f"{sp:.3f}", spas, f"{pct:.2f}", f"{fan:.2f}",
                        f"{k_i:.9e}" if k_i is not None else "",
                        f"{k_v:.6f}" if k_v is not None else "",
                        state
                    ])
            messagebox.showinfo("Zapisano", f"Wyeksportowano {len(self.raw_rows)} probek do:\n{dest}")
        except Exception as e:
            messagebox.showerror("Blad eksportu", str(e))

    def build_conn(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=24, pady=24)

        card = tk.Frame(wrap, bg=C['panel'])
        card.pack(fill='x', pady=(0, 16))
        tk.Frame(card, bg=C['blue'], height=3).pack(fill='x')
        inner = tk.Frame(card, bg=C['panel'])
        inner.pack(fill='x', padx=20, pady=16)

        tk.Label(inner, text="SERIAL CONNECTION", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(anchor='w', pady=(0, 12))
        tk.Label(inner, text="Available ports:", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10))).pack(anchor='w')

        lf = tk.Frame(inner, bg=C['panel'])
        lf.pack(fill='x', pady=8)
        sb = tk.Scrollbar(lf)
        sb.pack(side='right', fill='y')
        self.conn_list = tk.Listbox(lf, bg=C['bg2'], fg=C['text'],
                                    font=(FONT, fsz(10)), height=6,
                                    selectbackground=C['blue'], borderwidth=0,
                                    highlightthickness=1, highlightbackground=C['border'],
                                    yscrollcommand=sb.set, activestyle='none')
        self.conn_list.pack(side='left', fill='both', expand=True)
        sb.config(command=self.conn_list.yview)

        br = tk.Frame(inner, bg=C['panel'])
        br.pack(fill='x', pady=(8, 0))
        mk_btn(br, "REFRESH", self.refresh_ports, C['cyan']).pack(side='left', padx=(0, 8))
        mk_btn(br, "CONNECT", self.conn_from_tab, C['green']).pack(side='left', padx=(0, 8))
        mk_btn_outline(br, "DISCONNECT", self.disconnect, C['red']).pack(side='left')

        # ── KEITHLEY 2611B (LAN/TSP) ──
        kcard = tk.Frame(wrap, bg=C['panel'])
        kcard.pack(fill='x', pady=(0, 16))
        tk.Frame(kcard, bg=C['orange'], height=3).pack(fill='x')
        kinner = tk.Frame(kcard, bg=C['panel'])
        kinner.pack(fill='x', padx=20, pady=16)

        khd = tk.Frame(kinner, bg=C['panel'])
        khd.pack(fill='x', pady=(0, 12))
        tk.Label(khd, text="KEITHLEY 2611B (LAN)", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')
        self.keithley_status_lbl = tk.Label(khd, text="● not connected", bg=C['panel'],
                                            fg=C['dim2'], font=(FONT, fsz(9)))
        self.keithley_status_lbl.pack(side='right')

        krow1 = tk.Frame(kinner, bg=C['panel'])
        krow1.pack(fill='x', pady=4)
        tk.Label(krow1, text="IP address", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9)), width=14, anchor='w').pack(side='left')
        self.keithley_ip_entry = tk.Entry(krow1, bg=C['bg2'], fg=C['text'],
                                          font=(FONT, fsz(10)), relief='flat', bd=0,
                                          insertbackground=C['orange'],
                                          highlightthickness=1, highlightbackground=C['border'])
        self.keithley_ip_entry.pack(side='left', fill='x', expand=True, ipady=4)
        self.keithley_ip_entry.insert(0, "192.168.1.50")

        krow2 = tk.Frame(kinner, bg=C['panel'])
        krow2.pack(fill='x', pady=4)
        tk.Label(krow2, text="Napiecie [V]", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9)), width=14, anchor='w').pack(side='left')
        self.keithley_v_entry = tk.Entry(krow2, bg=C['bg2'], fg=C['text'],
                                         font=(FONT, fsz(10)), relief='flat', bd=0,
                                         insertbackground=C['orange'], width=10,
                                         highlightthickness=1, highlightbackground=C['border'])
        self.keithley_v_entry.pack(side='left', ipady=4, padx=(0, 16))
        self.keithley_v_entry.insert(0, "1.0")

        tk.Label(krow2, text="Limit pradu [A]", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(side='left', padx=(0, 6))
        self.keithley_i_entry = tk.Entry(krow2, bg=C['bg2'], fg=C['text'],
                                         font=(FONT, fsz(10)), relief='flat', bd=0,
                                         insertbackground=C['orange'], width=10,
                                         highlightthickness=1, highlightbackground=C['border'])
        self.keithley_i_entry.pack(side='left', ipady=4)
        self.keithley_i_entry.insert(0, "0.1")

        kbr = tk.Frame(kinner, bg=C['panel'])
        kbr.pack(fill='x', pady=(10, 0))
        mk_btn(kbr, "TEST CONNECTION", self.keithley_test_connect, C['orange']).pack(
            side='left', padx=(0, 8))
        mk_btn_outline(kbr, "DISCONNECT", self.keithley_disconnect, C['red']).pack(side='left')

        tk.Label(kinner, text="Pomiar pradu startuje/zatrzymuje sie razem z PID (START/STOP w CONTROL).\n"
                 "Domyslny protokol 2600B to TSP (Lua) przez port 5025.",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)),
                 justify='left').pack(anchor='w', pady=(10, 0))

        info = tk.Frame(wrap, bg=C['panel'])
        info.pack(fill='x')
        tk.Frame(info, bg=C['dim2'], height=3).pack(fill='x')
        ii = tk.Frame(info, bg=C['panel'])
        ii.pack(fill='x', padx=20, pady=16)
        tk.Label(ii, text="INSTRUCTIONS", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(11), 'bold')).pack(anchor='w', pady=(0, 8))
        for line in [
            "1. Wgraj firmware PeltierPID.ino na ItsyBitsy M0 (Arduino IDE)",
            "2. Polacz przez USB, wybierz port COM, kliknij CONNECT",
            "3. Suwaki synchronizuja sie automatycznie po polaczeniu",
            "4. Ustaw TARGET i RATE, kliknij START",
            "5. Wykres na zywo + zapis CSV w ~/PeltierLogi",
        ]:
            tk.Label(ii, text=line, bg=C['panel'], fg=C['dim'],
                     font=(FONT, fsz(9)), anchor='w').pack(anchor='w', pady=1)

        self.refresh_ports()

    def refresh_ports(self):
        self.conn_list.delete(0, 'end')
        self._ports = list(serial.tools.list_ports.comports())
        for p in self._ports:
            self.conn_list.insert('end', f"  {p.device}   {p.description or '?'}")
        if self._ports: self.conn_list.selection_set(0)

    def conn_from_tab(self):
        s = self.conn_list.curselection()
        if s and self._ports:
            self.connect(self._ports[s[0]].device)

    # ─── KEITHLEY 2611B ──────────────────────────────────
    def _read_keithley_settings(self):
        try:
            ip = self.keithley_ip_entry.get().strip()
            v = float(self.keithley_v_entry.get().replace(',', '.'))
            ilim = float(self.keithley_i_entry.get().replace(',', '.'))
            return ip, v, ilim
        except ValueError:
            return None, None, None

    def keithley_test_connect(self):
        ip, v, ilim = self._read_keithley_settings()
        if not ip:
            messagebox.showwarning("Brak IP", "Wpisz adres IP Keithleya.")
            return
        self.keithley_status_lbl.config(text="● laczenie...", fg=C['yellow'])
        self.root.update_idletasks()

        def worker():
            try:
                idn = self.keithley.connect(ip)
                self.keithley_connected = True
                self.keithley_ip = ip
                self.root.after(0, lambda: self.keithley_status_lbl.config(
                    text=f"● polaczono: {idn[:40]}", fg=C['green']))
            except Exception as e:
                self.keithley_connected = False
                self.root.after(0, lambda: self.keithley_status_lbl.config(
                    text=f"● blad: {e}", fg=C['red']))
        threading.Thread(target=worker, daemon=True).start()

    def keithley_disconnect(self):
        self.keithley_running = False
        self.keithley.disconnect()
        self.keithley_connected = False
        self.keithley_status_lbl.config(text="● not connected", fg=C['dim2'])

    def keithley_start_measurement(self):
        """Wywolywane razem z do_start() PID - konfiguruje SMU, wlacza output, startuje watek pomiarowy."""
        ip, v, ilim = self._read_keithley_settings()
        if not ip:
            return  # brak konfiguracji Keithleya - kontynuuj bez niego
        self.keithley_voltage = v if v is not None else 1.0
        self.keithley_ilimit = ilim if ilim is not None else 0.1

        def worker():
            try:
                if not self.keithley_connected:
                    idn = self.keithley.connect(ip)
                    self.keithley_connected = True
                    self.keithley_ip = ip
                    self.root.after(0, lambda: self.keithley_status_lbl.config(
                        text=f"● polaczono: {idn[:40]}", fg=C['green']))
                self.keithley.setup_source_v_measure_i(
                    "a", self.keithley_voltage, self.keithley_ilimit)
                self.keithley.output_on("a")
                self.keithley_running = True
                self.root.after(0, lambda: self.keithley_status_lbl.config(
                    text=f"● MIERZY  {self.keithley_voltage:.2f}V / lim {self.keithley_ilimit:.3f}A",
                    fg=C['green']))
                threading.Thread(target=self._keithley_poll_loop, daemon=True).start()
            except Exception as e:
                self.keithley_running = False
                self.root.after(0, lambda: self.keithley_status_lbl.config(
                    text=f"● blad startu: {e}", fg=C['red']))
        threading.Thread(target=worker, daemon=True).start()

    def keithley_stop_measurement(self):
        """Wywolywane razem z do_stop() PID - wylacza output, zatrzymuje watek."""
        self.keithley_running = False
        if self.keithley_connected:
            def worker():
                try:
                    self.keithley.output_off("a")
                except Exception:
                    pass
                self.root.after(0, lambda: self.keithley_status_lbl.config(
                    text="● polaczono (output OFF)", fg=C['dim']))
            threading.Thread(target=worker, daemon=True).start()

    def _keithley_poll_loop(self):
        """Watek probkujacy prad/napiecie z Keithleya co keithley_period_s."""
        while self.keithley_running and self.keithley_connected:
            t_start = time.time()
            try:
                with self.keithley_lock:
                    i_val, v_val = self.keithley.measure_iv("a")
                ts_ms = time.time() * 1000.0
                self.keithley_last_i = i_val
                self.keithley_last_v = v_val
                self.keithley_last_ts = ts_ms
                self.keithley_queue.put((ts_ms, i_val, v_val))
            except Exception as e:
                self.root.after(0, lambda e=e: self.keithley_status_lbl.config(
                    text=f"● blad pomiaru: {e}", fg=C['red']))
                time.sleep(0.5)
                continue
            elapsed = time.time() - t_start
            sleep_t = max(0.0, self.keithley_period_s - elapsed)
            time.sleep(sleep_t)

    def _keithley_latest(self, max_age_s=0.5):
        """Zwraca (i, v) ostatniego pomiaru jesli swiezy, inaczej (None, None)."""
        if self.keithley_last_ts is None:
            return None, None
        age = (time.time() * 1000.0 - self.keithley_last_ts) / 1000.0
        if age > max_age_s:
            return None, None
        return self.keithley_last_i, self.keithley_last_v


    # ─── ZAKLADKA ARCHIVE ────────────────────────────────
    def build_arch(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=16, pady=16)

        hd = tk.Frame(wrap, bg=C['bg'])
        hd.pack(fill='x', pady=(0, 12))
        tk.Label(hd, text="CYCLE ARCHIVE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')
        mk_btn(hd, "REFRESH", self.refresh_arch, C['cyan']).pack(side='right')

        body = tk.Frame(wrap, bg=C['bg'])
        body.pack(fill='both', expand=True)

        lf = tk.Frame(body, bg=C['panel'], width=340)
        lf.pack(side='left', fill='y', padx=(0, 12))
        lf.pack_propagate(False)
        tk.Frame(lf, bg=C['purple'], height=3).pack(fill='x')
        lhd = tk.Frame(lf, bg=C['panel'])
        lhd.pack(fill='x', padx=12, pady=8)
        tk.Label(lhd, text="SAVED CYCLES", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        mk_btn_outline(lhd, "CLEAR", self._arch_clear_sel, C['dim']).pack(side='right')

        list_wrap = tk.Frame(lf, bg=C['bg2'])
        list_wrap.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        asb = tk.Scrollbar(list_wrap)
        asb.pack(side='right', fill='y')
        self.arch_canvas = tk.Canvas(list_wrap, bg=C['bg2'], highlightthickness=0,
                                    yscrollcommand=asb.set)
        self.arch_canvas.pack(side='left', fill='both', expand=True)
        asb.config(command=self.arch_canvas.yview)
        self.arch_items = tk.Frame(self.arch_canvas, bg=C['bg2'])
        self._arch_win = self.arch_canvas.create_window((0, 0), window=self.arch_items, anchor='nw')
        self.arch_items.bind('<Configure>',
            lambda e: self.arch_canvas.config(scrollregion=self.arch_canvas.bbox('all')))
        self.arch_canvas.bind('<Configure>',
            lambda e: self.arch_canvas.itemconfig(self._arch_win, width=e.width))
        self.arch_canvas.bind('<Enter>', lambda e: self.arch_canvas.bind_all(
            '<MouseWheel>', lambda ev: self.arch_canvas.yview_scroll(int(-ev.delta/120), 'units')))
        self.arch_canvas.bind('<Leave>', lambda e: self.arch_canvas.unbind_all('<MouseWheel>'))

        self.arch_vars = {}

        cf = tk.Frame(body, bg=C['panel'])
        cf.pack(side='left', fill='both', expand=True)
        tk.Frame(cf, bg=C['border2'], height=3).pack(fill='x')
        self.fig_a = Figure(figsize=(8, 6), facecolor=C['panel'], dpi=100)
        self.ax_a = self.fig_a.add_subplot(111)
        self.ax_a.set_facecolor(C['panel2'])
        self.cv_a = FigureCanvasTkAgg(self.fig_a, master=cf)
        self.cv_a.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(8, 4))
        self.cv_a.draw()

        tbf = tk.Frame(cf, bg='#3a3f44')
        tbf.pack(fill='x', padx=8, pady=(4, 0))
        try:
            self.mpl_toolbar_a = NavigationToolbar2Tk(self.cv_a, tbf, pack_toolbar=False)
            self.mpl_toolbar_a.config(bg='#3a3f44')
            self.mpl_toolbar_a.update()
            self.mpl_toolbar_a.pack(side='left', fill='x')
        except Exception as e:
            print(f"arch toolbar err: {e}")

        atb = tk.Frame(cf, bg=C['panel'])
        atb.pack(fill='x', padx=8, pady=(2, 8))
        mk_btn_outline(atb, "📁", self.open_log_folder, C['dim']).pack(side='right', padx=(4, 0))
        mk_btn_outline(atb, "⤓ PNG", self.save_arch_chart, C['cyan']).pack(side='right', padx=(4, 0))
        mk_btn(atb, "⤓ POBIERZ CSV (zaznaczony cykl)", self.export_selected_cycle_csv, C['green']).pack(
            side='right', padx=(4, 0))

        self._arch_colors = [C['blue'], C['orange'], C['green'], C['red'],
                            C['cyan'], C['purple'], C['yellow'], '#ff8fab']
        self.refresh_arch()
        self._redraw_arch()

    def _cycle_display_name(self, path):
        from pathlib import Path as _P
        s = _P(path).stem
        if s.startswith('cykl_'): s = s[5:]
        elif s.startswith('c_'): s = s[2:]
        return s.replace('_', ' ')

    def refresh_arch(self):
        for w in self.arch_items.winfo_children(): w.destroy()
        self.arch_vars = {}
        files = sorted([f for f in self.log_dir.glob("*.csv")
                        if (f.name.startswith("cykl_") or f.name.startswith("c_"))
                        and not f.name.startswith("_tmp")],
                       key=lambda f: f.stat().st_mtime, reverse=True)
        if not files:
            tk.Label(self.arch_items, text="No saved cycles yet.",
                     bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(9))).pack(
                     anchor='w', padx=12, pady=12)
            return
        for i, f in enumerate(files):
            col = self._arch_colors[i % len(self._arch_colors)]
            row = tk.Frame(self.arch_items, bg=C['bg2'])
            row.pack(fill='x', pady=1)
            var = tk.BooleanVar(value=False)
            self.arch_vars[str(f)] = var
            delb = tk.Button(row, text="✕", command=lambda p=f: self._delete_cycle(p),
                            bg=C['bg2'], fg=C['red'], font=(FONT, fsz(10), 'bold'),
                            relief='flat', cursor='hand2', bd=0, padx=8,
                            activebackground=C['red'], activeforeground='#fff')
            delb.pack(side='right', padx=(2, 4))
            tk.Frame(row, bg=col, width=8).pack(side='left', fill='y')
            name = self._cycle_display_name(f)
            disp = name if len(name) <= 24 else name[:22]+"…"
            tk.Checkbutton(row, text=disp, variable=var, command=self._redraw_arch,
                           bg=C['bg2'], fg=C['text'], selectcolor=C['panel'],
                           activebackground=C['bg2'], activeforeground=col,
                           font=(FONT, fsz(9)), bd=0, highlightthickness=0,
                           anchor='w').pack(side='left', fill='x', expand=True)

    def _delete_cycle(self, path):
        if messagebox.askyesno("Delete", f"Delete: {path.name}?"):
            try: path.unlink(); self.refresh_arch(); self._redraw_arch()
            except Exception as e: messagebox.showerror("Error", str(e))

    def _arch_clear_sel(self):
        for v in self.arch_vars.values(): v.set(False)
        self._redraw_arch()

    def _load_cycle(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
            t,t1,t2,sp,pwm = [],[],[],[],[]
            for r in rows:
                try:
                    t.append(float(r.get('czas_od_startu_s',0)))
                    v1 = r.get('temperatura1_C','')
                    t1.append(float(v1) if v1 else None)
                    v2 = r.get('temperatura2_C','')
                    t2.append(float(v2) if v2 else None)
                    sp.append(float(r.get('setpoint_cel_C',0)))
                    pwm.append(float(r.get('peltier_pct',0)))
                except: continue
            return (t,t1,t2,sp,pwm) if t else None
        except: return None

    def _redraw_arch(self):
        sel = [(p,v) for p,v in self.arch_vars.items() if v.get()]
        self.ax_a.clear(); self.ax_a.set_facecolor(C['panel2'])
        if not sel:
            self.ax_a.text(0.5,0.5,"Tick a cycle to display",
                           ha='center',va='center',color=C['dim2'],
                           fontsize=11,transform=self.ax_a.transAxes)
            self.cv_a.draw(); return
        files = sorted([f for f in self.log_dir.glob("*.csv")
                        if (f.name.startswith("cykl_") or f.name.startswith("c_"))
                        and not f.name.startswith("_tmp")], reverse=True)
        forder = {str(f):i for i,f in enumerate(files)}
        for path,_ in sel:
            d = self._load_cycle(path)
            if not d: continue
            t,t1,t2,sp,pwm = d
            ci = forder.get(path,0)%len(self._arch_colors)
            col = self._arch_colors[ci]
            t0 = t[0]; tx=[x-t0 for x in t]
            nm = self._cycle_display_name(path)
            self.ax_a.plot(tx, sp, color=C['orange'], lw=1, ls='--', alpha=0.5)
            self.ax_a.plot(tx, t1, color=col, lw=2, label=nm[:20])
        self.ax_a.set_xlabel('czas [s]', color=C['dim'], fontsize=9)
        self.ax_a.set_ylabel('temperatura [°C]', color=C['dim'], fontsize=9)
        self.ax_a.tick_params(colors=C['dim'], labelsize=8)
        self.ax_a.legend(facecolor=C['panel'], edgecolor=C['border'],
                         labelcolor=C['dim'], fontsize=8)
        self.ax_a.grid(True, alpha=0.3, color=C['grid'])
        for sp2 in self.ax_a.spines.values(): sp2.set_color(C['border'])
        self.fig_a.tight_layout()
        self.cv_a.draw()

    def save_arch_chart(self):
        if not any(v.get() for v in self.arch_vars.values()):
            messagebox.showinfo("No selection", "Tick a cycle first."); return
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(title="Save chart",
               defaultextension=".png", initialfile="wykres.png",
               filetypes=[("PNG","*.png")])
        if dest:
            self.fig_a.savefig(dest, dpi=150, facecolor=C['panel'], bbox_inches='tight')
            messagebox.showinfo("Saved", f"{dest}")

    def export_selected_cycle_csv(self):
        """Kopiuje surowy plik CSV zaznaczonego cyklu (lub cykli) do wskazanej
        lokalizacji. Plik juz zawiera komplet danych raw: T1, T2, setpointy,
        PID, Keithley - dokladnie to co zostalo zapisane od START do STOP."""
        import shutil
        from pathlib import Path as _P
        selected = [_P(p) for p, v in self.arch_vars.items() if v.get()]
        if not selected:
            messagebox.showinfo("Brak zaznaczenia", "Zaznacz cykl (checkbox) na liscie po lewej.")
            return
        from tkinter import filedialog
        if len(selected) == 1:
            src = selected[0]
            dest = filedialog.asksaveasfilename(
                title="Zapisz raw dane cyklu", defaultextension=".csv",
                initialfile=src.name,
                filetypes=[("CSV", "*.csv")])
            if not dest:
                return
            try:
                shutil.copy(src, dest)
                messagebox.showinfo("Zapisano", f"Raw dane cyklu zapisane do:\n{dest}")
            except Exception as e:
                messagebox.showerror("Blad", str(e))
        else:
            dest_dir = filedialog.askdirectory(title="Wybierz folder docelowy dla CSV")
            if not dest_dir:
                return
            ok = 0
            for src in selected:
                try:
                    shutil.copy(src, _P(dest_dir) / src.name)
                    ok += 1
                except Exception:
                    pass
            messagebox.showinfo("Zapisano", f"Skopiowano {ok}/{len(selected)} plikow do:\n{dest_dir}")

    def open_log_folder(self):
        import subprocess
        p = str(self.log_dir)
        if sys.platform=='win32': os.startfile(p)
        elif sys.platform=='darwin': subprocess.run(['open',p])
        else: subprocess.run(['xdg-open',p])

    # ─── TICK ────────────────────────────────────────────
    def tick(self):
        try:
            rows = []
            while not self.data_queue.empty():
                rows.append(self.data_queue.get_nowait())

            for d in rows:
                dtype = d.get('type','data')
                if dtype == 'cfg':
                    self.root.after(0, lambda d=d: self._apply_cfg(d)); continue
                if dtype == 'status':
                    msg = d.get('msg','')
                    if msg == 'ON': self._update_run_button(True)
                    elif msg in ('STOP','RESET'): self._update_run_button(False)
                    continue
                if dtype != 'data': continue

                t1  = d.get('t1')
                t2  = d.get('t2')
                sp  = d.get('sp', 0)
                spa = d.get('spa', sp)
                pct = d.get('pct', 0)
                fn  = d.get('fan', 0)
                tsr = d.get('ts', 0) / 1000.0

                if self.t0 is None: self.t0 = tsr
                rel = tsr - self.t0

                self.t.append(rel); self.temp1.append(t1); self.temp2.append(t2)
                self.spt.append(sp); self.spa.append(spa)
                self.pwm.append(pct); self.fanv.append(fn)

                if len(self.t) > self.maxlen:
                    for a in [self.t,self.temp1,self.temp2,self.spt,self.spa,self.pwm,self.fanv]:
                        del a[0]

                pid_on = d.get('pid_on', False)
                if pid_on and not self.cyc_on:
                    self._cyc_start(t1 or 0)
                    self.reach_start_t = rel
                    self.reach_start_temp = t1
                    self.reach_target = sp
                    self.reach_done = False
                    self.reach_time = None
                    self.reach_avg_rate = None
                    self.last_setpoint_target = sp
                elif not pid_on and self.cyc_on:
                    self.cyc_stop("done")

                if self.cyc_on:
                    k_i, k_v = self._keithley_latest()
                    self.cyc_log(rel, t1, t2, sp, pct, fn,
                                 spa=spa, kp=d.get('kp'), ki=d.get('ki'), kd=d.get('kd'),
                                 fw_ts=tsr, state=d.get('state'),
                                 keithley_i=k_i, keithley_v=k_v)

                pc_now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                k_i, k_v = self._keithley_latest()
                self._raw_append((tsr, pc_now, t1, t2, sp, spa, pct, fn, d.get('state',''), k_i, k_v))

                if pid_on and self.last_setpoint_target is not None:
                    if abs(sp - self.last_setpoint_target) > 0.5:
                        self.reach_start_t = rel
                        self.reach_start_temp = t1
                        self.reach_target = sp
                        self.reach_done = False
                        self.last_setpoint_target = sp

                if (pid_on and not self.reach_done and self.reach_target is not None
                        and self.reach_start_t is not None and t1 is not None):
                    if abs(t1 - self.reach_target) <= 0.5:
                        self.reach_done = True
                        self.reach_time = rel - self.reach_start_t
                        delta = self.reach_target - (self.reach_start_temp or t1)
                        dT = abs(delta)
                        if self.reach_time > 0:
                            self.reach_avg_rate = dT / (self.reach_time/60.0)
                        self.reach_dir = "HEAT" if delta > 0 else "COOL"

                self.cards['temp']['val'].config(
                    text=f"{t1:.2f}" if t1 is not None else "ERR")
                self.cards['temp2']['val'].config(
                    text=f"{t2:.2f}" if t2 is not None else "--")
                self.cards['sp']['val'].config(text=f"{sp:.1f}")
                self.cards['pwm']['val'].config(text=f"{pct:.0f}")
                k_i_disp, _ = self._keithley_latest()
                if k_i_disp is not None:
                    self.cards['kcur']['val'].config(text=f"{k_i_disp*1000:.3f}m")
                elif self.keithley_running:
                    self.cards['kcur']['val'].config(text="...")
                else:
                    self.cards['kcur']['val'].config(text="--")

                avg_rate = 0.0
                if (self.reach_start_t is not None and self.reach_start_temp is not None
                        and t1 is not None and pid_on):
                    elapsed = rel - self.reach_start_t
                    if elapsed > 2:
                        avg_rate = (t1 - self.reach_start_temp) / (elapsed/60.0)
                if self.reach_done and self.reach_avg_rate is not None:
                    sign = 1 if self.reach_dir == 'HEAT' else -1
                    avg_rate = sign * self.reach_avg_rate
                self.cards['rate']['val'].config(text=f"{avg_rate:+.1f}")

                diff = sp - (t1 or sp)
                arrow = "▲HEAT" if diff>0.3 else ("▼COOL" if diff<-0.3 else "●HOLD")
                acol = C['red'] if diff>0.3 else (C['cyan'] if diff<-0.3 else C['dim2'])
                self.cards['pwm']['unit_lbl'].config(text=" % "+arrow, fg=acol)

                if self.reach_done and self.reach_time is not None:
                    m=int(self.reach_time//60); s=int(self.reach_time%60)
                    tstr=f"{m}m {s}s" if m>0 else f"{s}s"
                    rate_str = f"{self.reach_avg_rate:.2f}" if self.reach_avg_rate else "?"
                    dcol = C['red'] if self.reach_dir=='HEAT' else C['cyan']
                    self.reach_lbl.config(text=f"✓ {self.reach_dir} REACHED {tstr} · avg {rate_str}°C/min", fg=dcol)
                elif pid_on and self.reach_start_t is not None and not self.reach_done:
                    elapsed = rel - self.reach_start_t
                    m=int(elapsed//60); s=int(elapsed%60)
                    tstr=f"{m}m {s}s" if m>0 else f"{s}s"
                    self.reach_lbl.config(text=f"→ reaching {self.reach_target:.1f}°C · {tstr}", fg=C['yellow'])
                elif not pid_on:
                    self.reach_lbl.config(text="")

            if self.t and not self.chart_paused:
                self._draw_chart()

        except Exception as e:
            print(f"tick err: {e}")
        self.root.after(250, self.tick)

    def _apply_cfg(self, d):
        try:
            if not self._cfg_synced:
                if 'sp' in d and hasattr(self,'sl_sp'): self.sl_sp.set(float(d['sp']))
                if 'ru' in d and hasattr(self,'sl_ru'): self.sl_ru.set(float(d['ru']))
                if 'kp' in d and hasattr(self,'sl_kp'): self.sl_kp.set(float(d['kp']))
                if 'ki' in d and hasattr(self,'sl_ki'): self.sl_ki.set(float(d['ki']))
                if 'kd' in d and hasattr(self,'sl_kd'): self.sl_kd.set(float(d['kd']))
                if 'kffh' in d and hasattr(self,'sl_kffh'): self.sl_kffh.set(float(d['kffh']))
                if 'kffr' in d and hasattr(self,'sl_kffr'): self.sl_kffr.set(float(d['kffr']))
                if 'offset' in d and hasattr(self,'sl_off'): self.sl_off.set(float(d['offset']))
                self._cfg_synced = True
        except Exception as e: print(f"cfg err: {e}")

    def _draw_chart(self):
        t=self.t; t1=self.temp1; t2=self.temp2; sp=self.spt; spa=self.spa; pw=self.pwm
        if self.chart_window > 0 and len(t)>1:
            cutoff = t[-1]-self.chart_window
            i0 = next((i for i in range(len(t)) if t[i]>=cutoff), 0)
            t=t[i0:]; t1=t1[i0:]; t2=t2[i0:]; sp=sp[i0:]; spa=spa[i0:]; pw=pw[i0:]

        def safe(lst): return [v if v is not None else float('nan') for v in lst]

        self.ax1.clear(); self.ax1.set_facecolor(C['panel2'])
        self.ax1.plot(t, sp, color=C['orange'], lw=1.3, ls='--', label='target', alpha=0.7)
        self.ax1.plot(t, spa, color=C['cyan'], lw=1.5, ls=':', label='setpoint (ramp)')
        self.ax1.plot(t, safe(t1), color=C['blue'], lw=2.2, label='T1')
        self.ax1.plot(t, safe(t2), color=C['purple'], lw=1.3, ls='--', label='T2', alpha=0.6)
        self.ax1.set_ylabel('°C', color=C['dim'], fontsize=9)
        self.ax1.tick_params(colors=C['dim'], labelsize=8, length=0)
        self.ax1.grid(True, axis='y', alpha=0.35, color=C['grid'])
        for s in ['top','right']: self.ax1.spines[s].set_visible(False)
        for s in ['left','bottom']: self.ax1.spines[s].set_color(C['border'])
        self.ax1.legend(facecolor=C['panel'], edgecolor=C['border'],
                        labelcolor=C['dim'], fontsize=8, loc='upper right')

        self.ax2.clear(); self.ax2.set_facecolor(C['panel2'])
        self.ax2.fill_between(t, 0, pw, color=C['green'], alpha=0.3)
        self.ax2.plot(t, pw, color=C['green'], lw=1.5)
        self.ax2.set_ylabel('PWM %', color=C['dim'], fontsize=9)
        self.ax2.set_xlabel('time [s]', color=C['dim'], fontsize=9)
        self.ax2.set_ylim(-5, 105)
        self.ax2.tick_params(colors=C['dim'], labelsize=8, length=0)
        self.ax2.grid(True, axis='y', alpha=0.35, color=C['grid'])
        for s in ['top','right']: self.ax2.spines[s].set_visible(False)
        for s in ['left','bottom']: self.ax2.spines[s].set_color(C['border'])

        self.cv.draw_idle()

    # ─── CSV CYKLU ───────────────────────────────────────
    def _cyc_start(self, temp0):
        self.cyc_on = True; self.cyc_t0 = time.time()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.cyc_fn = self.log_dir / f"_tmp_cykl_{ts}.csv"
        self.cyc_file = open(self.cyc_fn, 'w', newline='', encoding='utf-8')
        self.cyc_wr = csv.writer(self.cyc_file)
        self.cyc_wr.writerow([
            'timestamp_pc', 'czas_firmware_s', 'czas_od_startu_s',
            'temperatura1_C', 'temperatura2_C',
            'setpoint_aktywny_C', 'setpoint_cel_C',
            'peltier_pct', 'fan_pct',
            'Kp', 'Ki', 'Kd', 'stan',
            'keithley_prad_A', 'keithley_napiecie_V',
        ])
        self.cyc_rows = 0
        print(f"CYC START T={temp0}")

    def cyc_log(self, t, t1, t2, sp, pct, fn, spa=None, kp=None, ki=None, kd=None,
                fw_ts=None, state=None, keithley_i=None, keithley_v=None):
        if self.cyc_wr:
            try:
                t1s = f"{t1:.3f}" if t1 is not None else ""
                t2s = f"{t2:.3f}" if t2 is not None else ""
                spas = f"{spa:.3f}" if spa is not None else ""
                kps = f"{kp:.4f}" if kp is not None else ""
                kis = f"{ki:.5f}" if ki is not None else ""
                kds = f"{kd:.4f}" if kd is not None else ""
                fwts = f"{fw_ts:.3f}" if fw_ts is not None else ""
                kis_a = f"{keithley_i:.9e}" if keithley_i is not None else ""
                kvs = f"{keithley_v:.6f}" if keithley_v is not None else ""
                pc_ts = datetime.now().isoformat(timespec="milliseconds")
                self.cyc_wr.writerow([
                    pc_ts, fwts, f"{t:.3f}",
                    t1s, t2s,
                    spas, f"{sp:.3f}",
                    f"{pct:.2f}", f"{fn:.2f}",
                    kps, kis, kds, state or "",
                    kis_a, kvs,
                ])
                self.cyc_file.flush(); self.cyc_rows += 1
            except: pass

    def cyc_stop(self, reason=""):
        if self.cyc_file:
            try: self.cyc_file.close()
            except: pass
        had = self.cyc_on and self.cyc_rows > 0
        tmp = self.cyc_fn
        self.cyc_on=False; self.cyc_file=None; self.cyc_wr=None
        print(f"CYC STOP: {reason} ({self.cyc_rows} probek)")
        if had and tmp and tmp.exists():
            self.root.after(0, lambda: self._ask_save_name(tmp))
        elif tmp and tmp.exists():
            try: tmp.unlink()
            except: pass

    def _ask_save_name(self, tmp_path):
        SaveCycleDialog(self.root, self, tmp_path)

    def save_cycle_as(self, tmp_path, name):
        import re as _re
        safe = _re.sub(r'[^\w\-\s]', '', name.strip())
        safe = _re.sub(r'\s+', '_', safe) or "cykl"
        dest = self.log_dir / f"c_{safe}.csv"
        if dest.exists():
            ts = datetime.now().strftime("%m%d_%H%M")
            dest = self.log_dir / f"c_{safe}_{ts}.csv"
        try:
            tmp_path.rename(dest)
            print(f"Zapisano: {dest.name}")
        except Exception as e: print(f"err: {e}")
        if hasattr(self, 'refresh_arch'):
            try: self.refresh_arch()
            except: pass

    def discard_cycle(self, tmp_path):
        try:
            if tmp_path.exists(): tmp_path.unlink()
        except: pass


# ════════════════════════════════════════════════════════
#  DIALOG ZAPISU CYKLU
# ════════════════════════════════════════════════════════
class SaveCycleDialog:
    def __init__(self, parent, app, tmp_path):
        self.app = app; self.tmp_path = tmp_path
        self.win = tk.Toplevel(parent)
        self.win.title("Save cycle")
        self.win.configure(bg=C['bg'])
        self.win.geometry("440x230")
        self.win.transient(parent)
        self.win.grab_set()

        tk.Frame(self.win, bg=C['green'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        tk.Label(inner, text="SAVE CYCLE TO ARCHIVE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w')
        rows = getattr(app, 'cyc_rows', 0)
        tk.Label(inner, text=f"Recorded {rows} samples",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(4, 16))

        tk.Label(inner, text="Cycle name:", bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(10))).pack(anchor='w')
        self.entry = tk.Entry(inner, bg=C['bg2'], fg=C['text'],
                              font=(FONT, fsz(12)), relief='flat', bd=0,
                              insertbackground=C['green'],
                              highlightthickness=2, highlightbackground=C['green'])
        self.entry.pack(fill='x', ipady=6, pady=(4, 16))
        default = datetime.now().strftime("test_%H%M")
        self.entry.insert(0, default)
        self.entry.select_range(0, 'end')
        self.entry.focus()
        self.entry.bind('<Return>', lambda e: self.save())

        bf = tk.Frame(inner, bg=C['bg'])
        bf.pack(fill='x')
        mk_btn(bf, "SAVE", self.save, C['green']).pack(side='left', fill='x', expand=True, padx=(0, 4))
        mk_btn_outline(bf, "DISCARD", self.discard, C['red']).pack(side='left', fill='x', expand=True, padx=(4, 0))
        self.win.protocol("WM_DELETE_WINDOW", self.save)

    def save(self):
        name = self.entry.get().strip()
        if not name: name = datetime.now().strftime("cykl_%H%M")
        self.app.save_cycle_as(self.tmp_path, name)
        self.win.destroy()

    def discard(self):
        if messagebox.askyesno("Discard?", "Discard this cycle?"):
            self.app.discard_cycle(self.tmp_path)
            self.win.destroy()


# ════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════
def _enable_dpi_awareness():
    if sys.platform != 'win32': return 1.0
    try:
        import ctypes
        try: ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except: ctypes.windll.user32.SetProcessDPIAware()
        hdc = ctypes.windll.user32.GetDC(0)
        dpi = ctypes.windll.gdi32.GetDeviceCaps(hdc, 88)
        ctypes.windll.user32.ReleaseDC(0, hdc)
        return dpi / 96.0
    except: return 1.0

def main():
    scale = _enable_dpi_awareness()
    global FS
    FS = scale if scale and scale > 1.05 else 1.0

    root = tk.Tk()
    try:
        if scale and scale > 1.05: root.tk.call('tk', 'scaling', scale)
    except: pass

    app = PeltierControl(root)

    def on_close():
        app.keithley_running = False
        app.keithley_disconnect()
        app.disconnect()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
