#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PeltierControl - BRUTALIST
Panel sterowania PID Peltiera (Feed-Forward) z dwukierunkowa komunikacja JSON.
Firmware: ItsyBitsy M0 + Cytron MDD10A + 2x MAX31856
"""

import sys, os, time, csv, json, threading, queue, socket, bisect
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
def px(n): return max(1, int(round(n * FS)))  # skalowanie wymiarow (szerokosci/wysokosci) wg DPI

_SI_PREFIXES = [(1e0, ''), (1e-3, 'm'), (1e-6, 'µ'), (1e-9, 'n'), (1e-12, 'p')]
def fmt_si(value, digits=3):
    """Formatuje mala wartosc z prefiksem SI, np. 4.83e-8 -> ('48.30', 'n').
    Zwraca (tekst_wartosci, prefiks). Dla None zwraca ('--', '')."""
    if value is None:
        return "--", ""
    av = abs(value)
    if av == 0:
        return f"{0:.{digits}f}", ""
    if av >= 1.0:
        return f"{value:.{digits}f}", ""
    for scale, pref in _SI_PREFIXES:
        if av >= scale:
            return f"{value/scale:.{digits}f}", pref
    return f"{value/1e-12:.{digits}f}", "p"

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
    """Komunikacja z Keithley 2611B przez USB (protokol TMC488) uzywajac PyVISA.
    Recznie zweryfikowane: stabilne, wielokrotne komendy na tym samym polaczeniu
    dzialaja bez zrywania - w przeciwienstwie do Ethernetu przez tani adapter
    USB-Ethernet, ktory mial problemy sprzetowe (fizyczne odlaczanie/podlaczanie).
    Wymaga: pip install pyvisa pyvisa-py pyusb libusb-package
    Oraz zainstalowanego sterownika WinUSB dla urzadzenia (przez Zadig), bo Windows
    domyslnie nie ma sterownika dla USBTMC device na Keithleyu."""

    TIMEOUT_MS = 5000

    def __init__(self):
        self.rm = None
        self.inst = None
        self.connected = False
        self.idn = ""
        self.resource_str = ""
        self._ensure_libusb_on_path()

    @staticmethod
    def _ensure_libusb_on_path():
        """Dodaje folder z libusb-1.0.dll (z pakietu libusb_package) do PATH,
        bo pyusb/pyvisa-py go tam nie znajdzie automatycznie."""
        try:
            import libusb_package
            lib_dir = os.path.dirname(libusb_package.__file__)
            if lib_dir not in os.environ.get("PATH", ""):
                os.environ["PATH"] = os.environ.get("PATH", "") + os.pathsep + lib_dir
        except Exception:
            pass  # jesli libusb_package niedostepny, pyvisa-py sprobuje domyslnych sciezek

    def find_usb_resource(self):
        """Znajdz pierwszy dostepny zasob USB (Keithley) przez PyVISA."""
        import pyvisa
        if self.rm is None:
            self.rm = pyvisa.ResourceManager('@py')
        resources = self.rm.list_resources()
        usb_resources = [r for r in resources if r.startswith('USB')]
        if not usb_resources:
            raise ConnectionError(
                "Nie znaleziono urzadzenia USB. Sprawdz czy Keithley jest podlaczony "
                "kablem USB i czy sterownik WinUSB jest zainstalowany (Zadig)."
            )
        return usb_resources[0]

    def connect(self, ip=None):
        """Parametr 'ip' zachowany dla zgodnosci z reszta aplikacji (nieuzywany -
        USB nie wymaga adresu, znajduje urzadzenie automatycznie)."""
        import pyvisa
        if self.rm is None:
            self.rm = pyvisa.ResourceManager('@py')
        self.resource_str = self.find_usb_resource()
        self.inst = self.rm.open_resource(self.resource_str)
        self.inst.timeout = self.TIMEOUT_MS
        self.connected = True
        try:
            self.idn = self.inst.query("*IDN?").strip()
        except Exception:
            self.idn = "Keithley (USB)"
        return self.idn

    def disconnect(self):
        self.connected = False
        if self.inst:
            try: self.inst.close()
            except Exception: pass
            self.inst = None

    def _exec(self, cmd):
        """Wykonaj komende TSP bez oczekiwania odpowiedzi (np. przypisania)."""
        if not self.inst:
            raise ConnectionError("Keithley not connected")
        try:
            self.inst.write(cmd)
        except Exception:
            self._reconnect()
            self.inst.write(cmd)

    def _query(self, cmd):
        if not self.inst:
            raise ConnectionError("Keithley not connected")
        try:
            return self.inst.query(cmd).strip()
        except Exception:
            self._reconnect()
            return self.inst.query(cmd).strip()

    def _reconnect(self):
        """Auto-reconnect jesli polaczenie USB padnie w trakcie pracy."""
        try:
            if self.inst:
                self.inst.close()
        except Exception:
            pass
        self.resource_str = self.find_usb_resource()
        self.inst = self.rm.open_resource(self.resource_str)
        self.inst.timeout = self.TIMEOUT_MS

    def setup_source_v_measure_i(self, channel="a", voltage=0.0, ilimit=0.1,
                                  nplc=1.0, filter_count=1):
        """Konfiguruje SMU: zrodlo napiecia, pomiar pradu, dany limit pradowy (compliance).
        nplc - czas integracji pomiaru w okresach sieci (wyzej = mniej szumu, wolniej).
        filter_count - ile pomiarow usredniac sprzetowo w jeden odczyt (1 = wylaczone).
        Dla malych sygnalow (pA-nA, np. prad piroelektryczny) NPLC=0.01 to za mala
        integracja - odczyt to praktycznie sam szum ADC, ktory dodatkowo wywoluje
        ciagle przelaczanie zakresu (autorange hunting) i daje pilokształtne skoki +/-."""
        ch = f"smu{channel}"
        self._exec(f"{ch}.reset()")
        self._exec(f"{ch}.source.func = {ch}.OUTPUT_DCVOLTS")
        self._exec(f"{ch}.source.levelv = {voltage:.6f}")
        self._exec(f"{ch}.source.limiti = {ilimit:.6f}")
        self._exec(f"{ch}.measure.nplc = {nplc:.4f}")
        self._exec(f"{ch}.measure.autozero = {ch}.AUTOZERO_AUTO")
        self._exec(f"{ch}.measure.autorangei = {ch}.AUTORANGE_ON")
        self._configure_filter(ch, filter_count)

    def setup_source_i_measure_v(self, channel="a", current=0.0, vlimit=1.0,
                                  nplc=1.0, filter_count=1):
        """Konfiguruje SMU: zrodlo pradu, pomiar napiecia, dany limit napieciowy (compliance).
        nplc / filter_count - patrz setup_source_v_measure_i."""
        ch = f"smu{channel}"
        self._exec(f"{ch}.reset()")
        self._exec(f"{ch}.source.func = {ch}.OUTPUT_DCAMPS")
        self._exec(f"{ch}.source.leveli = {current:.6f}")
        self._exec(f"{ch}.source.limitv = {vlimit:.6f}")
        self._exec(f"{ch}.measure.nplc = {nplc:.4f}")
        self._exec(f"{ch}.measure.autozero = {ch}.AUTOZERO_AUTO")
        self._exec(f"{ch}.measure.autorangev = {ch}.AUTORANGE_ON")
        self._configure_filter(ch, filter_count)

    def _configure_filter(self, ch, filter_count):
        """Wlacza sprzetowy filtr usredniajacy (repeating average) - kazdy odczyt
        to srednia z filter_count pomiarow, zamiast pojedynczej zaszumionej probki.
        filter_count<=1 wylacza filtr (najszybszy, ale najbardziej zaszumiony pomiar)."""
        if filter_count and filter_count > 1:
            self._exec(f"{ch}.measure.filter.type = {ch}.FILTER_REPEAT_AVG")
            self._exec(f"{ch}.measure.filter.count = {int(filter_count)}")
            self._exec(f"{ch}.measure.filter.enable = {ch}.FILTER_ON")
        else:
            self._exec(f"{ch}.measure.filter.enable = {ch}.FILTER_OFF")

    def output_on(self, channel="a"):
        self._exec(f"smu{channel}.source.output = smu{channel}.OUTPUT_ON")

    def output_off(self, channel="a"):
        self._exec(f"smu{channel}.source.output = smu{channel}.OUTPUT_OFF")

    def set_voltage(self, channel="a", voltage=0.0):
        self._exec(f"smu{channel}.source.levelv = {voltage:.6f}")

    def set_current(self, channel="a", current=0.0):
        self._exec(f"smu{channel}.source.leveli = {current:.6f}")

    def measure_iv(self, channel="a"):
        """Zwraca (prad_A, napiecie_V) z jednego zapytania (szybsze niz dwa osobne)."""
        resp = self._query(f"print(smu{channel}.measure.i(), smu{channel}.measure.v())")
        parts = resp.replace(",", " ").split()
        i_val = float(parts[0])
        v_val = float(parts[1]) if len(parts) > 1 else float('nan')
        return i_val, v_val

    def set_voltage_and_measure(self, channel="a", voltage=0.0, settle_s=0.0):
        """Ustawia napiecie, czeka settle_s (delay() PO STRONIE INSTRUMENTU, wiec
        czas ustalenia jest realny/niezmieniony), i mierzy I/V - wszystko w JEDNEJ
        komendzie/jednym przejezdzie USB (zamiast osobnego write + sleep + query).
        Polowa narzutu komunikacyjnego na kazdy punkt sweepu."""
        ch = f"smu{channel}"
        if settle_s > 0:
            resp = self._query(
                f"{ch}.source.levelv = {voltage:.6f}; delay({settle_s:.6f}); "
                f"print({ch}.measure.i(), {ch}.measure.v())")
        else:
            resp = self._query(
                f"{ch}.source.levelv = {voltage:.6f}; print({ch}.measure.i(), {ch}.measure.v())")
        parts = resp.replace(",", " ").split()
        return float(parts[0]), float(parts[1]) if len(parts) > 1 else float('nan')

    def set_current_and_measure(self, channel="a", current=0.0, settle_s=0.0):
        """Jak set_voltage_and_measure, ale dla trybu zrodla pradu."""
        ch = f"smu{channel}"
        if settle_s > 0:
            resp = self._query(
                f"{ch}.source.leveli = {current:.6f}; delay({settle_s:.6f}); "
                f"print({ch}.measure.i(), {ch}.measure.v())")
        else:
            resp = self._query(
                f"{ch}.source.leveli = {current:.6f}; print({ch}.measure.i(), {ch}.measure.v())")
        parts = resp.replace(",", " ").split()
        return float(parts[0]), float(parts[1]) if len(parts) > 1 else float('nan')

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
        self.root.geometry(f"{px(1280)}x{px(800)}")
        self.root.minsize(px(1100), px(720))

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
        self._raw_last_ui_ts = 0.0
        self.raw_ui_interval = 0.2  # throttling: max 5 aktualizacji Treeview / sekunde

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

        # Sweep V/I (zakladka KEITHLEY)
        self.sweep_running = False
        self.sweep_abort = False
        self.sweep_queue = queue.Queue()
        self.sweep_points = []  # lista (v_set_lub_i_set, i_meas, v_meas)
        self.sweep_mode = "V"   # "V" = source V/measure I, "I" = source I/measure V
        self.sweep_saved_settings = {
            "V": {"start": "0.000001", "stop": "0.00005", "steps": "50", "limit": "0.0001", "value": "0.000001"},
            "I": {"start": "0.000001", "stop": "0.00005", "steps": "50", "limit": "1.0", "value": "0.000001"},
        }
        self.sweep_total = 0
        self.sweep_done = 0
        self.sweep_loop_count = 0
        self.sweep_t0 = None
        self.last_known_rel = 0.0
        self.last_known_t1 = None
        self.last_known_t2 = None
        self.last_known_sp = None

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

        self.log_dir = Path.home() / "BigPeltierPidLogi"
        self.log_dir.mkdir(exist_ok=True)
        self.cyc_on = False; self.cyc_file = None; self.cyc_wr = None
        self.cyc_t0 = None; self.cyc_fn = None; self.cyc_rows = 0
        self.cyc_write_errors = 0
        self._recover_tmp_cycles()

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
        top = tk.Frame(self.root, bg=C['bg2'], height=px(44))
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
        t6 = tk.Frame(nb, bg=C['bg']); nb.add(t6, text='KEITHLEY')
        t3 = tk.Frame(nb, bg=C['bg']); nb.add(t3, text='ARCHIVE')
        t4 = tk.Frame(nb, bg=C['bg']); nb.add(t4, text='CONNECTION')
        self.nb = nb
        self.raw_tab_frame = t5
        self.raw_tab_visible = False
        nb.bind('<<NotebookTabChanged>>', self._on_tab_changed)
        self.build_live(t1)
        self.build_advanced(t2)
        self.build_raw(t5)
        self.build_keithley_tab(t6)
        self.build_arch(t3)
        self.build_conn(t4)

    def _on_tab_changed(self, event):
        """RAW DATA to jedyna zakladka gdzie odswiezanie UI jest kosztowne
        (Treeview.insert 10x/s) - aktualizujemy ja tylko gdy jest faktycznie
        widoczna, zeby nie obciazac programu w tle gdy uzytkownik patrzy
        np. na CONTROL albo ARCHIVE."""
        try:
            was_visible = self.raw_tab_visible
            self.raw_tab_visible = (self.nb.select() == str(self.raw_tab_frame))
            if self.raw_tab_visible and not was_visible:
                self._raw_rebuild_tree()
        except Exception:
            pass

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
        self.btn_stop_peltier = tk.Button(ctrl, text="⏹ STOP\nPELTIER", command=self.do_stop_peltier_only,
                                   bg=C['bg2'], fg=C['cyan'], font=(FONT, fsz(8), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=8, pady=6,
                                   highlightthickness=1, highlightbackground=C['cyan'],
                                   activebackground=C['panel3'])
        self.btn_stop_peltier.pack(side='left', padx=(0, 4), fill='y')
        self.btn_stop_keithley = tk.Button(ctrl, text="⏹ STOP\nKEITHLEY", command=self.keithley_sweep_stop,
                                   bg=C['bg2'], fg=C['orange'], font=(FONT, fsz(8), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=8, pady=6,
                                   highlightthickness=1, highlightbackground=C['orange'],
                                   activebackground=C['panel3'])
        self.btn_stop_keithley.pack(side='left', padx=(0, 4), fill='y')
        self.btn_estop = tk.Button(ctrl, text="⛔", command=self.do_estop,
                                   bg=C['red'], fg='#fff', font=(FONT, fsz(14), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=12, pady=12,
                                   activebackground=_lighten(C['red'], 0.15))
        self.btn_estop.pack(side='left', fill='y')

        main = tk.Frame(parent, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=(0, 12))

        # PRAWO - panel sterowania (PAKOWANY PIERWSZY zeby zachowac szerokosc)
        self._build_panel(main)
        # LEWO - stos wykresow: temperatura (gora) + I-V sweep (dol)
        chart_area = tk.Frame(main, bg=C['bg'])
        chart_area.pack(side='left', fill='both', expand=True, padx=(0, 12))
        self._build_chart(chart_area)
        self._build_sweep_mini_chart(chart_area)

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
        wrap.pack(side='top', fill='both', expand=True, pady=(0, 8))
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

    def _build_sweep_mini_chart(self, parent):
        """Kompaktowy wykres I-V na ekranie CONTROL, obok wykresu temperatury -
        pokazuje ten sam sweep co zakladka KEITHLEY, zeby widziec oba na raz."""
        wrap = tk.Frame(parent, bg=C['panel'], height=px(220))
        wrap.pack(side='top', fill='x')
        wrap.pack_propagate(False)
        tk.Frame(wrap, bg=C['orange'], height=3).pack(fill='x')

        hd = tk.Frame(wrap, bg=C['panel'])
        hd.pack(fill='x', padx=14, pady=(8, 2))
        tk.Label(hd, text="WYKRES KEITHLEY", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9), 'bold')).pack(side='left')
        self.sweep_mini_status_lbl = tk.Label(hd, text="--", bg=C['panel'], fg=C['dim2'],
                                              font=(FONT, fsz(8)))
        self.sweep_mini_status_lbl.pack(side='right')

        self.sweep_fig_mini = Figure(figsize=(9, 2), facecolor=C['panel'], dpi=100)
        self.sweep_ax_mini = self.sweep_fig_mini.add_subplot(111)
        self.sweep_ax_mini.set_facecolor(C['panel2'])
        self.sweep_ax_mini.tick_params(colors=C['dim'], labelsize=6)
        for spine in self.sweep_ax_mini.spines.values():
            spine.set_color(C['border'])
        self.sweep_ax_mini.grid(True, color=C['grid'], linewidth=0.5, alpha=0.5)
        self.sweep_line_mini, = self.sweep_ax_mini.plot([], [], color=C['orange'], marker='o',
                                                         markersize=2.5, linewidth=1.0)
        self.sweep_fig_mini.tight_layout(pad=1.0)

        self.sweep_cv_mini = FigureCanvasTkAgg(self.sweep_fig_mini, master=wrap)
        self.sweep_cv_mini.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(0, 8))

    def _live_toolbar_busy(self):
        """True gdy w toolbarze matplotliba aktywny jest tryb ZOOM lub PAN -
        wtedy wstrzymujemy auto-odswiezanie, zeby przyblizenie nie znikalo.
        Wylaczenie narzedzia (ponowne klikniecie lupy) wznawia live."""
        tb = getattr(self, 'mpl_toolbar', None)
        if tb is None:
            return False
        mode = getattr(tb, 'mode', '')
        busy = bool(mode) and str(mode) != ''
        # wizualna informacja na przycisku pauzy
        if hasattr(self, 'btn_pause') and not self.chart_paused:
            if busy and self.btn_pause['text'] != "🔍 ZOOM (live wstrzymane)":
                self.btn_pause.config(text="🔍 ZOOM (live wstrzymane)", fg=C['cyan'],
                                      highlightbackground=C['cyan'])
            elif not busy and self.btn_pause['text'] != "⏸ PAUSE":
                self.btn_pause.config(text="⏸ PAUSE", fg=C['yellow'],
                                      highlightbackground=C['yellow'])
        return busy

    def toggle_pause(self):
        self.chart_paused = not self.chart_paused
        if self.chart_paused:
            self.btn_pause.config(text="▶ RESUME", fg=C['green'], highlightbackground=C['green'])
        else:
            self.btn_pause.config(text="⏸ PAUSE", fg=C['yellow'], highlightbackground=C['yellow'])

    def set_chart_window(self, secs):
        self.chart_window = secs

    def _build_panel(self, parent):
        panel = tk.Frame(parent, bg=C['bg2'], width=px(312))
        panel.pack(side='right', fill='y')
        panel.pack_propagate(False)
        tk.Frame(panel, bg=C['red'], width=px(6)).pack(side='left', fill='y')

        scroll_wrap = tk.Frame(panel, bg=C['bg2'])
        scroll_wrap.pack(side='left', fill='both', expand=True)
        pcanvas = tk.Canvas(scroll_wrap, bg=C['bg2'], highlightthickness=0, width=px(290))
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
        self.keithley_sweep_start()

    def do_stop(self):
        self.send("STOP")
        self._update_run_button(False)
        self.sweep_abort = True
        self.keithley_stop_measurement()

    def do_stop_peltier_only(self):
        """Zatrzymuje TYLKO PID Peltiera, nie ruszajac sweepu/pomiaru Keithleya."""
        self.send("STOP")
        self._update_run_button(False)

    def do_estop(self):
        self.send("STOP")
        self._update_run_button(False)
        self.sweep_abort = True
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

        cols = ('idx', 'czas_fw', 'pc_time', 't1', 't2', 'sp', 'spa', 'pct', 'fan', 'dir', 'k_i', 'k_v', 'state')
        headers = {
            'idx': '#', 'czas_fw': 'czas FW [s]', 'pc_time': 'czas PC',
            't1': 'T1 [C]', 't2': 'T2 [C]', 'sp': 'SP cel [C]',
            'spa': 'SP akt [C]', 'pct': 'Peltier %', 'fan': 'Fan %', 'dir': 'Kierunek',
            'k_i': 'I Keithley', 'k_v': 'V Keithley', 'state': 'stan'
        }
        widths = {
            'idx': 50, 'czas_fw': 90, 'pc_time': 110, 't1': 80, 't2': 80,
            'sp': 90, 'spa': 90, 'pct': 80, 'fan': 70, 'dir': 80,
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

        info = tk.Label(wrap, text="Tabela aktualizuje sie tylko gdy ta zakladka jest otwarta (max 5x/s) - dlatego nie "
                        "obciaza programu w tle. Bufor (max 2000 probek) zbiera dane zawsze, wiec EXPORT CSV "
                        "dziala niezaleznie. Pelny zapis surowych danych od START do STOP jest w zakladce ARCHIVE.",
                        bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8)), wraplength=900, justify='left')
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

    def _raw_row_values(self, row, idx):
        """Formatuje jeden wiersz (krotke danych) na wartosci wyswietlane w Treeview."""
        czas_fw, pc_time, t1, t2, sp, spa, pct, fan, state, k_i, k_v, heat = row
        t1s = f"{t1:.3f}" if t1 is not None else "—"
        t2s = f"{t2:.3f}" if t2 is not None else "—"
        if k_i is not None:
            _v, _p = fmt_si(k_i, 4); kis = f"{_v} {_p}A"
        else: kis = "—"
        if k_v is not None:
            _v, _p = fmt_si(k_v, 4); kvs = f"{_v} {_p}V"
        else: kvs = "—"
        dirs = "▲ HEAT" if heat else "▼ COOL"
        if pct < 0.5:
            dirs = "—"  # PWM prawie zero - kierunek bez znaczenia
        spas = "—" if state == "MAN" else f"{spa:.2f}"
        return (idx, f"{czas_fw:.2f}", pc_time, t1s, t2s,
                f"{sp:.2f}", spas, f"{pct:.1f}", f"{fan:.1f}", dirs, kis, kvs, state)

    def _raw_rebuild_tree(self):
        """Pelne przebudowanie tabeli z bufora self.raw_rows - wywolywane raz,
        gdy uzytkownik dopiero co przelaczyl sie na zakladke RAW DATA (zeby od
        razu zobaczyc aktualny stan, a nie czekac na throttlowane aktualizacje)."""
        if not hasattr(self, 'raw_tree'):
            return
        for item in self.raw_tree.get_children():
            self.raw_tree.delete(item)
        start_idx = max(1, len(self.raw_rows) - self.raw_maxrows + 1)
        for i, row in enumerate(self.raw_rows[-self.raw_maxrows:]):
            self.raw_tree.insert('', 'end', values=self._raw_row_values(row, start_idx + i))
        if self.raw_autoscroll:
            children = self.raw_tree.get_children()
            if children:
                self.raw_tree.see(children[-1])
        if hasattr(self, 'raw_count_lbl'):
            self.raw_count_lbl.config(text=f"{len(self.raw_rows)} probek")
        self._raw_last_ui_ts = time.time()

    def _raw_append(self, row):
        # row = (czas_fw, pc_time, t1, t2, sp, spa, pct, fan, state, k_i, k_v, heat)
        # Bufor w pamieci rosnie zawsze (tani append+trim na liscie Pythona) -
        # to on zasila EXPORT CSV i pelne przebudowanie tabeli, niezaleznie od
        # tego czy ktokolwiek aktualnie patrzy na zakladke.
        self.raw_rows.append(row)
        if len(self.raw_rows) > self.raw_maxrows:
            self.raw_rows = self.raw_rows[-self.raw_maxrows:]

        if self.raw_paused or not hasattr(self, 'raw_tree'):
            return

        # Kosztowna czesc (Treeview.insert, .see(), przewijanie) wykonujemy
        # TYLKO gdy zakladka RAW DATA jest faktycznie widoczna na ekranie,
        # i nie czesciej niz raw_ui_interval - inaczej caly program przycinal
        # sie od aktualizacji tabeli 10x/s w tle, nawet gdy nikt na nia nie patrzy.
        if not self.raw_tab_visible:
            return
        now = time.time()
        if now - self._raw_last_ui_ts < self.raw_ui_interval:
            return
        self._raw_last_ui_ts = now

        idx = len(self.raw_rows)
        self.raw_tree.insert('', 'end', values=self._raw_row_values(row, idx))
        children = self.raw_tree.get_children()
        if len(children) > self.raw_maxrows:
            for item in children[:len(children) - self.raw_maxrows]:
                self.raw_tree.delete(item)
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
                           'peltier_pct', 'fan_pct', 'kierunek', 'keithley_prad_A',
                           'keithley_napiecie_V', 'stan'])
                for row in self.raw_rows:
                    czas_fw, pc_time, t1, t2, sp, spa, pct, fan, state, k_i, k_v, heat = row
                    spas = "" if state == "MAN" else f"{spa:.3f}"
                    dirs = "HEAT" if heat else "COOL"
                    w.writerow([
                        f"{czas_fw:.3f}", pc_time,
                        f"{t1:.3f}" if t1 is not None else "",
                        f"{t2:.3f}" if t2 is not None else "",
                        f"{sp:.3f}", spas, f"{pct:.2f}", f"{fan:.2f}", dirs,
                        f"{k_i:.9e}" if k_i is not None else "",
                        f"{k_v:.9e}" if k_v is not None else "",
                        state
                    ])
            messagebox.showinfo("Zapisano", f"Wyeksportowano {len(self.raw_rows)} probek do:\n{dest}")
        except Exception as e:
            messagebox.showerror("Blad eksportu", str(e))

    # ─── ZAKLADKA KEITHLEY (SWEEP V/I) ───────────────────
    def build_keithley_tab(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=16, pady=12)

        # ── PANEL LEWY: konfiguracja sweep (SCROLLOWALNY - zeby przyciski
        # START/STOP nigdy nie znikaly niezaleznie od wysokosci okna) ──
        left = tk.Frame(wrap, bg=C['panel'], width=px(300))
        left.pack(side='left', fill='y', padx=(0, 12))
        left.pack_propagate(False)
        tk.Frame(left, bg=C['orange'], height=3).pack(fill='x')

        scroll_wrap = tk.Frame(left, bg=C['panel'])
        scroll_wrap.pack(fill='both', expand=True)
        pcanvas = tk.Canvas(scroll_wrap, bg=C['panel'], highlightthickness=0, width=px(280))
        psb = tk.Scrollbar(scroll_wrap, orient='vertical', command=pcanvas.yview)
        pcanvas.configure(yscrollcommand=psb.set)
        psb.pack(side='right', fill='y')
        pcanvas.pack(side='left', fill='both', expand=True)

        linner = tk.Frame(pcanvas, bg=C['panel'])
        linner_id = pcanvas.create_window((0, 0), window=linner, anchor='nw')
        linner.bind('<Configure>', lambda e: pcanvas.configure(scrollregion=pcanvas.bbox('all')))
        pcanvas.bind('<Configure>', lambda e: pcanvas.itemconfig(linner_id, width=e.width))
        pcanvas.bind('<Enter>', lambda e: pcanvas.bind_all('<MouseWheel>',
                     lambda ev: pcanvas.yview_scroll(int(-ev.delta/120), 'units')))
        pcanvas.bind('<Leave>', lambda e: pcanvas.unbind_all('<MouseWheel>'))
        linner_pad = tk.Frame(linner, bg=C['panel'])
        linner_pad.pack(fill='both', expand=True, padx=16, pady=14)
        linner = linner_pad  # reszta kodu buduje wewnatrz tego z paddingiem

        tk.Label(linner, text="SWEEP V/I", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w', pady=(0, 4))
        tk.Label(linner, text="Krokowe przemiatanie napiecia lub pradu\nz pomiarem i wykresem I-V",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)),
                 wraplength=250, justify='left').pack(anchor='w', pady=(0, 14))

        # Wybor trybu
        mode_row = tk.Frame(linner, bg=C['panel'])
        mode_row.pack(fill='x', pady=(0, 10))
        tk.Label(mode_row, text="TRYB", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9), 'bold')).pack(anchor='w', pady=(0, 4))
        self.sweep_mode_var = tk.StringVar(value="V")
        mrow = tk.Frame(mode_row, bg=C['panel'])
        mrow.pack(fill='x')
        self.btn_mode_v = tk.Button(mrow, text="ZRODLO V", command=lambda: self._set_sweep_mode("V"),
                                    bg=C['orange'], fg='#1a1c1f', font=(FONT, fsz(9), 'bold'),
                                    relief='flat', cursor='hand2', bd=0, padx=4, pady=6)
        self.btn_mode_v.pack(side='left', fill='x', expand=True, padx=(0, 4))
        self.btn_mode_i = tk.Button(mrow, text="ZRODLO I", command=lambda: self._set_sweep_mode("I"),
                                    bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                    relief='flat', cursor='hand2', bd=0, padx=4, pady=6)
        self.btn_mode_i.pack(side='left', fill='x', expand=True)

        # Tylko pomiar (bez sweepu) - pojedyncza stala wartosc, ciagly pomiar
        self.sweep_continuous_var = tk.BooleanVar(value=False)
        cont_row = tk.Frame(linner, bg=C['panel'])
        cont_row.pack(fill='x', pady=(10, 0))
        tk.Checkbutton(cont_row, text="Tylko pomiar (bez sweepu - stala wartosc)",
                      variable=self.sweep_continuous_var, command=lambda: self._toggle_continuous_mode(),
                      bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
                      font=(FONT, fsz(9)), activebackground=C['panel'],
                      activeforeground=C['text'], wraplength=240,
                      justify='left').pack(anchor='w')

        def _field(label, default, unit=""):
            row = tk.Frame(linner, bg=C['panel'])
            row.pack(fill='x', pady=4)
            lbl = tk.Label(row, text=label, bg=C['panel'], fg=C['dim'],
                     font=(FONT, fsz(9)), width=13, anchor='w')
            lbl.pack(side='left')
            e = tk.Entry(row, bg=C['bg2'], fg=C['text'], font=(FONT, fsz(10)),
                        relief='flat', bd=0, insertbackground=C['orange'],
                        highlightthickness=1, highlightbackground=C['border'])
            e.pack(side='left', fill='x', expand=True, ipady=4)
            e.insert(0, str(default))
            ulbl = tk.Label(row, text=unit, bg=C['panel'], fg=C['dim2'],
                            font=(FONT, fsz(8)), width=3, anchor='w')
            ulbl.pack(side='left', padx=(4, 0))
            e.unit_lbl = ulbl
            e.field_lbl = lbl
            return e

        # ── PARAMETRY - zwijana sekcja (oszczedza miejsce) ──
        self.params_expanded = tk.BooleanVar(value=True)
        phd = tk.Frame(linner, bg=C['panel'])
        phd.pack(fill='x', pady=(12, 2))
        self.btn_params_toggle = tk.Button(phd, text="▼ PARAMETRY", command=self._toggle_params_section,
                                           bg=C['panel'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                           relief='flat', cursor='hand2', bd=0, anchor='w')
        self.btn_params_toggle.pack(fill='x')

        self.params_body = tk.Frame(linner, bg=C['panel'])
        self.params_body.pack(fill='x')

        self.params_desc_lbl = tk.Label(self.params_body,
                 text="START/STOP - zakres wartosci zadanej (V lub A,\n"
                      "zalezy od trybu powyzej). KROKI - ile punktow\n"
                      "pomiarowych rozlozonych rownomiernie miedzy\n"
                      "START a STOP. LIMIT - compliance (np. przy\n"
                      "zrodle V to maks. dopuszczalny prad - chroni\n"
                      "probke/kontakty). SETTLE TIME - ile ms czekac\n"
                      "po kazdej zmianie wartosci zadanej, zanim\n"
                      "instrument zmierzy wynik (czas na ustalenie\n"
                      "sie sygnalu elektrycznego).",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)),
                 wraplength=250, justify='left')
        self.params_desc_lbl.pack(anchor='w', pady=(0, 8))

        # Pola sweepu (zakres) - ukrywane w trybie "tylko pomiar"
        self._sweep_range_frame = tk.Frame(self.params_body, bg=C['panel'])
        self._sweep_range_frame.pack(fill='x')
        old_linner = linner
        linner = self._sweep_range_frame
        self.sweep_start_entry = _field("START", "0.000001", "V")
        self.sweep_stop_entry  = _field("STOP", "0.00005", "V")
        self.sweep_step_entry  = _field("KROKI", "50", "")
        linner = old_linner

        # Pole pojedynczej wartosci - widoczne TYLKO w trybie "tylko pomiar"
        self._single_value_frame = tk.Frame(self.params_body, bg=C['panel'])
        linner = self._single_value_frame
        self.sweep_value_entry = _field("WARTOSC", "0.000001", "V")
        linner = old_linner
        self._single_value_frame.pack_forget()  # ukryte domyslnie (tryb sweep aktywny)

        # LIMIT i SETTLE TIME zawsze widoczne (potrzebne w obu trybach)
        linner = self.params_body
        self.sweep_limit_entry = _field("LIMIT", "0.0001", "A")
        self.sweep_settle_entry = _field("SETTLE TIME", "50", "ms")
        self.sweep_nplc_entry = _field("NPLC", "1.0", "")
        self.sweep_avg_entry = _field("USREDNIANIE", "1", "x")
        linner = old_linner

        # Sweep dwukierunkowy (tam i z powrotem) - przydatne np. do histerezy
        self.sweep_bidir_var = tk.BooleanVar(value=False)
        self._bidir_row = tk.Frame(self.params_body, bg=C['panel'])
        self._bidir_row.pack(fill='x', pady=(6, 0))
        tk.Checkbutton(self._bidir_row, text="Sweep tam i z powrotem", variable=self.sweep_bidir_var,
                      bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
                      font=(FONT, fsz(9)), activebackground=C['panel'],
                      activeforeground=C['text'], wraplength=240,
                      justify='left').pack(anchor='w')

        # Petla - powtarzaj sweep w kolko, np. przez caly czas rampy PID
        self.sweep_loop_var = tk.BooleanVar(value=False)
        self._loop_chk_row = tk.Frame(self.params_body, bg=C['panel'])
        self._loop_chk_row.pack(fill='x', pady=(4, 0))
        self.chk_loop = tk.Checkbutton(self._loop_chk_row, text="Petla (powtarzaj do STOP)",
                      variable=self.sweep_loop_var,
                      command=lambda: self._toggle_loop_pause_field(),
                      bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
                      font=(FONT, fsz(9)), activebackground=C['panel'],
                      activeforeground=C['text'], wraplength=240,
                      justify='left')
        self.chk_loop.pack(anchor='w')

        linner = self.params_body
        self.sweep_loop_pause_entry = _field("PRZERWA", "0", "ms")
        linner = old_linner
        self.sweep_loop_pause_entry.master.pack_forget()  # ukryte dopoki petla wylaczona
        self._loop_pause_row = self.sweep_loop_pause_entry.master

        btn_row = tk.Frame(linner, bg=C['panel'])
        self._sweep_btn_row = btn_row
        btn_row.pack(fill='x', pady=(16, 0))
        self.btn_sweep_start = tk.Button(btn_row, text="▶ START SWEEP", command=self.keithley_sweep_start,
                                         bg=C['green'], fg='#1a1c1f', font=(FONT, fsz(10), 'bold'),
                                         relief='flat', cursor='hand2', bd=0, padx=12, pady=10)
        self.btn_sweep_start.pack(fill='x', pady=(0, 6))
        self.btn_sweep_stop = tk.Button(btn_row, text="⛔ STOP SWEEP", command=self.keithley_sweep_stop,
                                        bg=C['red'], fg='#fff', font=(FONT, fsz(10), 'bold'),
                                        relief='flat', cursor='hand2', bd=0, padx=12, pady=10,
                                        state='disabled')
        self.btn_sweep_stop.pack(fill='x')

        exp_row = tk.Frame(linner, bg=C['panel'])
        exp_row.pack(fill='x', pady=(20, 10))
        mk_btn_outline(exp_row, "⤓ EKSPORTUJ CSV", self.export_sweep_csv, C['green']).pack(fill='x', pady=(0, 4))
        mk_btn_outline(exp_row, "WYCZYSC WYKRES", self.clear_sweep, C['dim']).pack(fill='x')

        # ── PANEL PRAWY: wykres (I-V / V(t) / I(t)) ──
        right = tk.Frame(wrap, bg=C['panel'])
        right.pack(side='left', fill='both', expand=True)
        tk.Frame(right, bg=C['border2'], height=3).pack(fill='x')

        rhd = tk.Frame(right, bg=C['panel'])
        rhd.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(rhd, text="WYKRES", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        self.sweep_pts_lbl = tk.Label(rhd, text="0 punktow", bg=C['panel'], fg=C['dim2'],
                                      font=(FONT, fsz(9)))
        self.sweep_pts_lbl.pack(side='right')

        # Przelacznik typu wykresu
        chart_sel = tk.Frame(right, bg=C['panel'])
        chart_sel.pack(fill='x', padx=14, pady=(0, 6))
        self.sweep_chart_view = "IV"  # "IV", "VT", "IT"
        self.btn_chart_iv = tk.Button(chart_sel, text="I-V", command=lambda: self._set_chart_view("IV"),
                                      bg=C['orange'], fg='#1a1c1f', font=(FONT, fsz(9), 'bold'),
                                      relief='flat', cursor='hand2', bd=0, padx=10, pady=5)
        self.btn_chart_iv.pack(side='left', padx=(0, 4))
        self.btn_chart_vt = tk.Button(chart_sel, text="V(t)", command=lambda: self._set_chart_view("VT"),
                                      bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                      relief='flat', cursor='hand2', bd=0, padx=10, pady=5)
        self.btn_chart_vt.pack(side='left', padx=(0, 4))
        self.btn_chart_it = tk.Button(chart_sel, text="I(t)", command=lambda: self._set_chart_view("IT"),
                                      bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                      relief='flat', cursor='hand2', bd=0, padx=10, pady=5)
        self.btn_chart_it.pack(side='left')

        self.sweep_autoscale_var = tk.BooleanVar(value=True)
        tk.Checkbutton(chart_sel, text="Auto-skalowanie", variable=self.sweep_autoscale_var,
                      command=lambda: self._redraw_sweep_chart(),
                      bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
                      font=(FONT, fsz(9)), activebackground=C['panel'],
                      activeforeground=C['text']).pack(side='right')

        # Duze karty na zywo: napiecie, prad, postep kroku
        scards_wrap = tk.Frame(right, bg=C['panel'])
        scards_wrap.pack(fill='x', padx=14, pady=(0, 10))
        self.sweep_cards = {}
        self.sweep_cards['v'] = self._stat_card(scards_wrap, "NAPIECIE", "V", C['orange'])
        self.sweep_cards['i'] = self._stat_card(scards_wrap, "PRAD", "A", C['blue'])
        self.sweep_cards['step'] = self._stat_card(scards_wrap, "KROK", "", C['green'])
        self.sweep_progress_lbl = self.sweep_cards['step']['val']  # alias - karta pokazuje "X / Y"
        self.sweep_cards['step']['unit_lbl'].config(text="")

        self.sweep_fig = Figure(figsize=(7, 5.3), facecolor=C['panel'], dpi=100)
        self.sweep_ax = self.sweep_fig.add_subplot(111)
        self.sweep_ax.set_facecolor(C['panel2'])
        self.sweep_ax.set_xlabel("Napiecie [V]", color=C['dim'], fontsize=8)
        self.sweep_ax.set_ylabel("Prad [A]", color=C['dim'], fontsize=8)
        self.sweep_ax.tick_params(colors=C['dim'], labelsize=7)
        for spine in self.sweep_ax.spines.values():
            spine.set_color(C['border'])
        self.sweep_ax.grid(True, color=C['grid'], linewidth=0.5, alpha=0.5)
        self.sweep_line, = self.sweep_ax.plot([], [], color=C['orange'], marker='o',
                                               markersize=3, linewidth=1.2)
        self.sweep_fig.tight_layout()

        self.sweep_cv = FigureCanvasTkAgg(self.sweep_fig, master=right)
        self.sweep_cv.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(0, 4))

        sweep_tbf = tk.Frame(right, bg='#3a3f44')
        sweep_tbf.pack(fill='x', padx=8, pady=(0, 8))
        try:
            self.mpl_toolbar_sweep = NavigationToolbar2Tk(self.sweep_cv, sweep_tbf, pack_toolbar=False)
            self.mpl_toolbar_sweep.config(bg='#3a3f44')
            self.mpl_toolbar_sweep.update()
            self.mpl_toolbar_sweep.pack(side='left', fill='x')
        except Exception as e:
            print(f"sweep toolbar err: {e}")

        self._sweep_tick()  # startuje petle odswiezania wykresu/tabeli

    def _set_chart_view(self, view):
        self.sweep_chart_view = view
        for v, btn in [("IV", self.btn_chart_iv), ("VT", self.btn_chart_vt), ("IT", self.btn_chart_it)]:
            if v == view:
                btn.config(bg=C['orange'], fg='#1a1c1f')
            else:
                btn.config(bg=C['bg2'], fg=C['dim'])
        self._redraw_sweep_chart()

    def _redraw_sweep_chart(self):
        """Przerysowuje caly wykres wg aktualnie wybranego widoku (I-V / V-t / I-t)."""
        try:
            from matplotlib.ticker import EngFormatter, ScalarFormatter
        except Exception:
            EngFormatter = None
        def _fmt(ax, x_unit, y_unit):
            # czytelne osie: 50 µA zamiast 5e-05; brak jednostki = zwykly format
            if EngFormatter is None: return
            ax.xaxis.set_major_formatter(EngFormatter(unit=x_unit) if x_unit else ScalarFormatter())
            ax.yaxis.set_major_formatter(EngFormatter(unit=y_unit) if y_unit else ScalarFormatter())
        pts = self.sweep_points
        if self.sweep_chart_view == "IV":
            if self.sweep_mode == "V":
                xs = [p[1] for p in pts]; ys = [p[2] for p in pts]
                self.sweep_ax.set_xlabel("Napiecie zadane", color=C['dim'], fontsize=8)
                self.sweep_ax.set_ylabel("Prad zmierzony", color=C['dim'], fontsize=8)
                _fmt(self.sweep_ax, 'V', 'A'); mini_units = ('V', 'A')
            else:
                xs = [p[1] for p in pts]; ys = [p[3] for p in pts]
                self.sweep_ax.set_xlabel("Prad zadany", color=C['dim'], fontsize=8)
                self.sweep_ax.set_ylabel("Napiecie zmierzone", color=C['dim'], fontsize=8)
                _fmt(self.sweep_ax, 'A', 'V'); mini_units = ('A', 'V')
            self.sweep_line.set_marker('o')
        elif self.sweep_chart_view == "VT":
            xs = [p[0] for p in pts]; ys = [p[3] for p in pts]
            self.sweep_ax.set_xlabel("Czas [s]", color=C['dim'], fontsize=8)
            self.sweep_ax.set_ylabel("Napiecie zmierzone", color=C['dim'], fontsize=8)
            _fmt(self.sweep_ax, '', 'V'); mini_units = ('', 'V')
            self.sweep_line.set_marker('')
        else:  # IT
            xs = [p[0] for p in pts]; ys = [p[2] for p in pts]
            self.sweep_ax.set_xlabel("Czas [s]", color=C['dim'], fontsize=8)
            self.sweep_ax.set_ylabel("Prad zmierzony", color=C['dim'], fontsize=8)
            _fmt(self.sweep_ax, '', 'A'); mini_units = ('', 'A')
            self.sweep_line.set_marker('')
        self.sweep_line.set_data(xs, ys)
        if getattr(self, 'sweep_autoscale_var', None) is None or self.sweep_autoscale_var.get():
            self.sweep_ax.relim(); self.sweep_ax.autoscale_view()
        self.sweep_cv.draw_idle()
        if hasattr(self, 'sweep_line_mini'):
            _fmt(self.sweep_ax_mini, mini_units[0], mini_units[1])
            self.sweep_line_mini.set_data(xs, ys)
            self.sweep_ax_mini.relim(); self.sweep_ax_mini.autoscale_view()
            self.sweep_cv_mini.draw_idle()

    def _toggle_params_section(self):
        if self.params_expanded.get():
            self.params_body.pack_forget()
            self.btn_params_toggle.config(text="▶ PARAMETRY")
            self.params_expanded.set(False)
        else:
            self.params_body.pack(fill='x', after=self.btn_params_toggle)
            self.btn_params_toggle.config(text="▼ PARAMETRY")
            self.params_expanded.set(True)

    def _toggle_continuous_mode(self):
        if self.sweep_continuous_var.get():
            self._sweep_range_frame.pack_forget()
            self._single_value_frame.pack(fill='x', before=self.sweep_limit_entry.master)
            self._bidir_row.pack_forget()
            self.sweep_loop_var.set(True)
            self.chk_loop.config(state='disabled')
            self._loop_pause_row.pack(fill='x', pady=4, before=self._sweep_btn_row)
            self.sweep_loop_pause_entry.field_lbl.config(text="INTERWAL")
            self.params_desc_lbl.config(
                text="WARTOSC - stala wartosc zadana (V lub A,\n"
                     "zalezy od trybu powyzej), Keithley bedzie ja\n"
                     "utrzymywal i ciagle mierzyl. LIMIT - compliance\n"
                     "(chroni probke/kontakty). To DWIE OSOBNE rzeczy:\n"
                     "SETTLE TIME - czas fizycznego ustalenia sie\n"
                     "sygnalu PO zmianie wartosci (tu bez znaczenia,\n"
                     "bo wartosc jest stala - mozna dac 0). INTERWAL -\n"
                     "odstep miedzy KOLEJNYMI probkami [ms] - TO steruje\n"
                     "czestotliwoscia zbierania danych (np. 100ms =\n"
                     "10 probek/s). NPLC/USREDNIANIE - patrz nizej.")
        else:
            self._single_value_frame.pack_forget()
            self._sweep_range_frame.pack(fill='x', before=self.sweep_limit_entry.master)
            self._bidir_row.pack(fill='x', pady=(6, 0), before=self._loop_chk_row)
            self.chk_loop.config(state='normal')
            self.sweep_loop_var.set(False)
            self._loop_pause_row.pack_forget()
            self.sweep_loop_pause_entry.field_lbl.config(text="PRZERWA")
            self.params_desc_lbl.config(
                text="START/STOP - zakres wartosci zadanej (V lub A,\n"
                     "zalezy od trybu powyzej). KROKI - ile punktow\n"
                     "pomiarowych rozlozonych rownomiernie miedzy\n"
                     "START a STOP. LIMIT - compliance (np. przy\n"
                     "zrodle V to maks. dopuszczalny prad - chroni\n"
                     "probke/kontakty). SETTLE TIME - ile ms czekac\n"
                     "po kazdej zmianie wartosci zadanej, zanim\n"
                     "instrument zmierzy wynik (czas na ustalenie\n"
                     "sie sygnalu elektrycznego). NPLC/USREDNIANIE -\n"
                     "patrz nizej: kontroluja szum pomiaru pradu/napiecia.")

    def _toggle_loop_pause_field(self):
        if self.sweep_loop_var.get():
            self._loop_pause_row.pack(fill='x', pady=4, before=self._sweep_btn_row)
        else:
            self._loop_pause_row.pack_forget()

    def _set_sweep_mode(self, mode):
        if mode == self.sweep_mode:
            return
        # Zapisz biezace wartosci pod STARY tryb, zanim przelaczymy
        old = self.sweep_mode
        self.sweep_saved_settings[old] = {
            'start': self.sweep_start_entry.get(),
            'stop': self.sweep_stop_entry.get(),
            'steps': self.sweep_step_entry.get(),
            'limit': self.sweep_limit_entry.get(),
            'value': self.sweep_value_entry.get(),
        }
        self.sweep_mode = mode
        saved = self.sweep_saved_settings[mode]

        if mode == "V":
            self.btn_mode_v.config(bg=C['orange'], fg='#1a1c1f')
            self.btn_mode_i.config(bg=C['bg2'], fg=C['dim'])
            self.sweep_start_entry.unit_lbl.config(text="V")
            self.sweep_stop_entry.unit_lbl.config(text="V")
            self.sweep_value_entry.unit_lbl.config(text="V")
            self.sweep_limit_entry.unit_lbl.config(text="A")
        else:
            self.btn_mode_i.config(bg=C['orange'], fg='#1a1c1f')
            self.btn_mode_v.config(bg=C['bg2'], fg=C['dim'])
            self.sweep_start_entry.unit_lbl.config(text="A")
            self.sweep_stop_entry.unit_lbl.config(text="A")
            self.sweep_value_entry.unit_lbl.config(text="A")
            self.sweep_limit_entry.unit_lbl.config(text="V")

        # Przywroc zapamietane wartosci dla tego trybu (kazdy tryb ma wlasne,
        # niezalezne ustawienia - przelaczanie nie kasuje juz wpisanych danych)
        for entry, key in [(self.sweep_start_entry, 'start'), (self.sweep_stop_entry, 'stop'),
                           (self.sweep_step_entry, 'steps'), (self.sweep_limit_entry, 'limit'),
                           (self.sweep_value_entry, 'value')]:
            entry.delete(0, 'end'); entry.insert(0, saved[key])

        self._redraw_sweep_chart()

    def clear_sweep(self):
        self.sweep_points = []
        self.sweep_line.set_data([], [])
        self.sweep_ax.relim(); self.sweep_ax.autoscale_view()
        self.sweep_cv.draw_idle()
        self.sweep_pts_lbl.config(text="0 punktow")
        self.sweep_cards['v']['val'].config(text="--")
        self.sweep_cards['i']['val'].config(text="--")
        self.sweep_progress_lbl.config(text="--", fg=C['dim2'])

    def keithley_sweep_start(self):
        if self.sweep_running:
            return
        if self.keithley_running:
            messagebox.showwarning("Keithley zajety",
                "Pomiar ciagly PID uzywa teraz Keithleya. Zatrzymaj PID (STOP) przed sweepem.")
            return

        continuous = self.sweep_continuous_var.get()
        try:
            if continuous:
                val = float(self.sweep_value_entry.get().replace(',', '.'))
                v0 = v1 = val
                n_steps = 2  # bez znaczenia gdy v0==v1, generuje pojedynczy punkt
            else:
                v0 = float(self.sweep_start_entry.get().replace(',', '.'))
                v1 = float(self.sweep_stop_entry.get().replace(',', '.'))
                n_steps = int(round(float(self.sweep_step_entry.get().replace(',', '.'))))
            limit = float(self.sweep_limit_entry.get().replace(',', '.'))
            settle_ms = float(self.sweep_settle_entry.get().replace(',', '.'))
            nplc = float(self.sweep_nplc_entry.get().replace(',', '.'))
            avg_count = int(round(float(self.sweep_avg_entry.get().replace(',', '.'))))
            if nplc <= 0: nplc = 1.0
            if avg_count < 1: avg_count = 1
        except ValueError:
            messagebox.showerror("Blad", "Sprawdz wartosci liczbowe pol sweep.")
            return
        if not continuous and n_steps < 2:
            messagebox.showerror("Blad", "Liczba krokow musi byc co najmniej 2.")
            return

        loop = True if continuous else self.sweep_loop_var.get()
        loop_pause_ms = 0.0
        if loop:
            try:
                loop_pause_ms = float(self.sweep_loop_pause_entry.get().replace(',', '.'))
            except ValueError:
                loop_pause_ms = 0.0

        self.clear_sweep()
        self.sweep_abort = False
        self.sweep_running = True
        self.sweep_loop_count = 0
        self.sweep_t0 = time.time()
        self.btn_sweep_start.config(state='disabled')
        self.btn_sweep_stop.config(state='normal')

        bidir = False if continuous else self.sweep_bidir_var.get()
        mode = self.sweep_mode

        # ── AUTOMATYCZNY ZAPIS NA DYSK - pelna precyzja, korelacja z temp/czasem,
        # zapisywany na biezaco (flush po kazdym punkcie) zeby nic nie zgineło
        # nawet przy awarii/zamknieciu programu w trakcie dlugiego sweepu w petli.
        ts_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        sweep_log_path = self.log_dir / f"sweep_{ts_name}.csv"
        sweep_log_file = open(sweep_log_path, 'w', newline='', encoding='utf-8')
        sweep_log_wr = csv.writer(sweep_log_file)
        sweep_log_wr.writerow([
            'timestamp_pc', 'czas_od_startu_sweep_s', 'czas_od_startu_PID_s',
            'temperatura1_C', 'temperatura2_C', 'setpoint_C',
            'petla_nr', 'wartosc_zadana', 'jednostka_zadana',
            'prad_zmierzony_A', 'napiecie_zmierzone_V',
        ])
        sweep_log_file.flush()
        self.root.after(0, lambda: self.sweep_mini_status_lbl.config(
            text=f"zapis: {sweep_log_path.name}", fg=C['dim2']))

        def worker():
            try:
                if not self.keithley_connected:
                    idn = self.keithley.connect()
                    self.keithley_connected = True
                    self.root.after(0, lambda: self.keithley_status_lbl.config(
                        text=f"● polaczono: {idn[:40]}", fg=C['green']))

                # zbuduj liste punktow - n_steps to LICZBA punktow, nie wielkosc kroku.
                # W trybie ciaglym (continuous) jest to zawsze jeden, staly punkt.
                if continuous:
                    points = [v0]
                else:
                    n = n_steps
                    fwd = [v0 + (v1 - v0) * i / (n - 1) for i in range(n)] if n > 1 else [v0]
                    points = fwd + list(reversed(fwd)) if bidir else fwd
                unit = "V" if mode == "V" else "A"

                if mode == "V":
                    self.keithley.setup_source_v_measure_i("a", points[0], limit, nplc, avg_count)
                else:
                    self.keithley.setup_source_i_measure_v("a", points[0], limit, nplc, avg_count)
                self.keithley.output_on("a")

                first_pass = True
                while True:
                    if self.sweep_abort:
                        break
                    self.sweep_loop_count += 1
                    if not first_pass:
                        # sygnal dla _sweep_tick (glowny watek): wyczysc wykres na nowa petle
                        # (w trybie ciaglym NIE czyscimy - chcemy widziec cala historie V(t)/I(t))
                        if not continuous:
                            self.sweep_queue.put(("__NEW_LOOP__", None, None, None))
                    first_pass = False

                    self.sweep_total = len(points)
                    self.sweep_done = 0

                    for p in points:
                        if self.sweep_abort:
                            break
                        settle_s = max(0.0, settle_ms / 1000.0)
                        if mode == "V":
                            i_meas, v_meas = self.keithley.set_voltage_and_measure("a", p, settle_s)
                        else:
                            i_meas, v_meas = self.keithley.set_current_and_measure("a", p, settle_s)
                        self.sweep_done += 1
                        t_elapsed = time.time() - self.sweep_t0
                        self.sweep_queue.put((t_elapsed, p, i_meas, v_meas))

                        # Nakarm te same "ostatnie znane" pola co stary tryb ciaglego
                        # pomiaru - dzieki temu RAW DATA / ARCHIVE / karta na CONTROL
                        # tez widza swieze dane Keithleya podczas sweepu, nie tylko
                        # wykres w zakladce KEITHLEY.
                        self.keithley_last_i = i_meas
                        self.keithley_last_v = v_meas
                        self.keithley_last_ts = time.time() * 1000.0

                        # Zapis pelnej precyzji na dysk NATYCHMIAST, z korelacja
                        # do aktualnej temperatury/czasu PID (odczyt atomowy, bezpieczny
                        # z tego watku - proste przypisania atrybutow sa thread-safe w CPythonie)
                        pc_ts = datetime.now().isoformat(timespec="microseconds")
                        t_rel = self.last_known_rel
                        t1_now = self.last_known_t1
                        t2_now = self.last_known_t2
                        sp_now = self.last_known_sp
                        t1_str = f"{t1_now:.4f}" if t1_now is not None else ""
                        t2_str = f"{t2_now:.4f}" if t2_now is not None else ""
                        sp_str = f"{sp_now:.4f}" if sp_now is not None else ""
                        try:
                            sweep_log_wr.writerow([
                                pc_ts, f"{t_elapsed:.3f}", f"{t_rel:.3f}", t1_str, t2_str, sp_str,
                                self.sweep_loop_count, f"{p:.9f}", unit,
                                f"{i_meas:.12e}", f"{v_meas:.9f}",
                            ])
                            sweep_log_file.flush()
                        except Exception:
                            pass  # nie przerywaj sweepu z powodu bledu zapisu pojedynczego wiersza

                    if not loop or self.sweep_abort:
                        break
                    if loop_pause_ms > 0:
                        time.sleep(loop_pause_ms / 1000.0)

                self.keithley.output_off("a")
            except Exception as e:
                self.root.after(0, lambda e=e: messagebox.showerror("Blad sweep", str(e)))
            finally:
                self.sweep_running = False
                try:
                    sweep_log_file.close()
                except Exception:
                    pass
                self.root.after(0, lambda: self.btn_sweep_start.config(state='normal'))
                self.root.after(0, lambda: self.btn_sweep_stop.config(state='disabled'))

        threading.Thread(target=worker, daemon=True).start()

    def keithley_sweep_stop(self):
        self.sweep_abort = True

    def _sweep_tick(self):
        """Odswieza wykres/karty na zywo na podstawie kolejki z watku sweep."""
        got_new = False
        last_pt = None
        while not self.sweep_queue.empty():
            try:
                pt = self.sweep_queue.get_nowait()
            except queue.Empty:
                break
            if pt[0] == "__NEW_LOOP__":
                # nowa iteracja petli - wyczysc wykres, zacznij od nowa
                self.sweep_points = []
                got_new = True
                continue
            self.sweep_points.append(pt)
            last_pt = pt
            got_new = True

        if got_new:
            self._redraw_sweep_chart()
            self.sweep_pts_lbl.config(text=f"{len(self.sweep_points)} punktow")

            if last_pt is not None:
                _, p_val, i_meas, v_meas = last_pt
                vv, vpref = fmt_si(v_meas, 3)
                iv, ipref = fmt_si(i_meas, 3)
                self.sweep_cards['v']['val'].config(text=vv)
                self.sweep_cards['v']['unit_lbl'].config(text=f" {vpref}V")
                self.sweep_cards['i']['val'].config(text=iv)
                self.sweep_cards['i']['unit_lbl'].config(text=f" {ipref}A")

        looping = getattr(self, 'sweep_loop_var', None) is not None and self.sweep_loop_var.get()
        loop_prefix = f"P{self.sweep_loop_count} " if looping else ""

        if hasattr(self, 'sweep_mini_status_lbl'):
            if self.sweep_running:
                self.sweep_mini_status_lbl.config(
                    text=f"{loop_prefix}{self.sweep_done}/{self.sweep_total}", fg=C['orange'])
            elif self.sweep_points:
                self.sweep_mini_status_lbl.config(text="koniec", fg=C['green'])
            else:
                self.sweep_mini_status_lbl.config(text="--", fg=C['dim2'])

        if self.sweep_running:
            self.sweep_progress_lbl.config(
                text=f"{loop_prefix}{self.sweep_done}/{self.sweep_total}", fg=C['green'])
        elif self.sweep_points:
            self.sweep_progress_lbl.config(text="Koniec", fg=C['green'])
        else:
            self.sweep_progress_lbl.config(text="--", fg=C['dim2'])

        self.root.after(150, self._sweep_tick)

    def export_sweep_csv(self):
        if not self.sweep_points:
            messagebox.showinfo("Brak danych", "Brak punktow sweep do eksportu.\n\n"
                "Uwaga: pelny, ciagly zapis WSZYSTKICH petli z korelacja czasowa/temperatura\n"
                "jest juz automatycznie zapisywany w ~/BigPeltierPidLogi/ podczas kazdego sweepu.")
            return
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Eksportuj biezacy widok sweep", defaultextension=".csv",
            initialfile=f"sweep_widok_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            filetypes=[("CSV", "*.csv")])
        if not dest:
            return
        try:
            with open(dest, 'w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if self.sweep_mode == "V":
                    w.writerow(['czas_s', 'napiecie_zadane_V', 'prad_zmierzony_A', 'napiecie_zmierzone_V'])
                else:
                    w.writerow(['czas_s', 'prad_zadany_A', 'prad_zmierzony_A', 'napiecie_zmierzone_V'])
                for t, p, i_m, v_m in self.sweep_points:
                    w.writerow([f"{t:.3f}", f"{p:.9f}", f"{i_m:.12e}", f"{v_m:.9f}"])
            messagebox.showinfo("Zapisano",
                f"Wyeksportowano {len(self.sweep_points)} punktow (biezaca petla) do:\n{dest}\n\n"
                f"Pelny zapis wszystkich petli z korelacja czasowa jest w ~/BigPeltierPidLogi/")
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
        tk.Label(khd, text="KEITHLEY 2611B (USB)", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')
        self.keithley_status_lbl = tk.Label(khd, text="● not connected", bg=C['panel'],
                                            fg=C['dim2'], font=(FONT, fsz(9)))
        self.keithley_status_lbl.pack(side='right')

        kbr = tk.Frame(kinner, bg=C['panel'])
        kbr.pack(fill='x', pady=(0, 0))
        mk_btn(kbr, "TEST CONNECTION", self.keithley_test_connect, C['orange']).pack(
            side='left', padx=(0, 8))
        mk_btn_outline(kbr, "DISCONNECT", self.keithley_disconnect, C['red']).pack(side='left')

        tk.Label(kinner, text="Ta zakladka sluzy TYLKO do sprawdzenia polaczenia USB z instrumentem.\n"
                 "Napiecie/prad, zakres sweepu i limity ustawiasz w zakladce KEITHLEY.\n"
                 "START na CONTROL uruchamia jednoczesnie PID i sweep skonfigurowany tam.\n"
                 "Polaczenie przez USB (protokol TMC488) - wymaga sterownika WinUSB (Zadig).",
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
            "5. Wykres na zywo + zapis CSV w ~/BigPeltierPidLogi",
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
    def keithley_test_connect(self):
        self.keithley_status_lbl.config(text="● szukam urzadzenia USB...", fg=C['yellow'])
        self.root.update_idletasks()

        def worker():
            try:
                idn = self.keithley.connect()
                self.keithley_connected = True
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
        consecutive_errors = 0
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
                consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                self.root.after(0, lambda e=e: self.keithley_status_lbl.config(
                    text=f"● blad pomiaru: {e}", fg=C['red']))
                # Backoff rosnie z liczba kolejnych bledow - zapobiega zalewaniu
                # adaptera USB-Ethernet ciaglymi probami reconnect (co powodowalo
                # fizyczne odlaczanie/podlaczanie adaptera w Windows)
                backoff = min(0.5 * consecutive_errors, 5.0)
                time.sleep(backoff)
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

        lf = tk.Frame(body, bg=C['panel'], width=px(340))
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
        # Osie tworzone dynamicznie w _redraw_arch: sama temperatura, albo
        # temperatura (gora) + prad Keithleya (dol) jeden pod drugim, wspolna os X.
        self.ax_a = self.fig_a.add_subplot(111)
        self.ax_a.set_facecolor(C['panel2'])
        self.ax_a_current = None
        self.cv_a = FigureCanvasTkAgg(self.fig_a, master=cf)
        self.cv_a.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(8, 4))
        self.cv_a.draw()

        # ─── HOVER: kursor pokazujacy czas/temperature/prad w punkcie ───
        self._arch_hover_series = []
        self._arch_hover_mode = 'time'
        self._arch_annot = None
        self._arch_vline_top = None
        self._arch_vline_bottom = None
        self._arch_marker_top = None
        self._arch_marker_bottom = None
        self._arch_marker_main = None
        self._arch_hover_last_key = None
        self.cv_a.mpl_connect('motion_notify_event', self._on_arch_hover)
        self.cv_a.mpl_connect('figure_leave_event', lambda e: self._arch_hide_hover())
        self.cv_a.mpl_connect('axes_leave_event', lambda e: self._arch_hide_hover())

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
        mk_btn_outline(atb, "❓ CSV w Excelu", self._show_excel_help, C['yellow']).pack(
            side='right', padx=(4, 0))

        atb2 = tk.Frame(cf, bg=C['panel'])
        atb2.pack(fill='x', padx=8, pady=(0, 8))
        tk.Label(atb2, text="WYKRES:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(0, 6))
        self.arch_chart_mode = tk.StringVar(value='time')
        def _mode_btn(text, val):
            b = tk.Radiobutton(atb2, text=text, variable=self.arch_chart_mode, value=val,
                          command=self._redraw_arch, bg=C['panel'], fg=C['dim'],
                          selectcolor=C['bg2'], font=(FONT, fsz(9)),
                          activebackground=C['panel'], activeforeground=C['text'],
                          indicatoron=False, padx=10, pady=3, relief='flat',
                          bd=1, highlightthickness=0)
            b.pack(side='left', padx=(0, 4))
            return b
        _mode_btn("temperatura / czas", 'time')
        _mode_btn("prad Keithley / temperatura", 'iT')
        self.arch_show_current_var = tk.BooleanVar(value=True)
        self.arch_show_current_chk = tk.Checkbutton(
            atb2, text="Pokaz prad Keithley (wykres pod temperatura)",
            variable=self.arch_show_current_var, command=lambda: self._redraw_arch(),
            bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
            font=(FONT, fsz(9)), activebackground=C['panel'],
            activeforeground=C['text'])
        self.arch_show_current_chk.pack(side='left', padx=(12, 0))

        atb3 = tk.Frame(cf, bg=C['panel'])
        atb3.pack(fill='x', padx=8, pady=(0, 8))
        tk.Label(atb3, text="WYROWNAJ:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(0, 6))
        self.arch_align_mode = tk.StringVar(value='none')
        def _align_btn(text, val):
            b = tk.Radiobutton(atb3, text=text, variable=self.arch_align_mode, value=val,
                          command=self._on_align_mode_change, bg=C['panel'], fg=C['dim'],
                          selectcolor=C['bg2'], font=(FONT, fsz(9)),
                          activebackground=C['panel'], activeforeground=C['text'],
                          indicatoron=False, padx=8, pady=3, relief='flat',
                          bd=1, highlightthickness=0)
            b.pack(side='left', padx=(0, 4))
            return b
        _align_btn("brak (czas od startu)", 'none')
        self.arch_align_temp_btn = _align_btn("wg temperatury =", 'temp')
        self.arch_align_cur_btn = _align_btn("wg pradu =", 'current')
        self.arch_align_entry = tk.Entry(atb3, bg=C['bg2'], fg=C['text'],
                                        insertbackground=C['text'], relief='flat',
                                        font=(FONT, fsz(9)), width=10,
                                        highlightthickness=1, highlightbackground=C['border'])
        self.arch_align_entry.insert(0, "25.0")
        self.arch_align_entry.pack(side='left', padx=(2, 2))
        self.arch_align_entry.bind('<Return>', lambda e: self._redraw_arch())
        self.arch_align_unit_lbl = tk.Label(atb3, text="°C", bg=C['panel'], fg=C['dim2'],
                                            font=(FONT, fsz(9)))
        self.arch_align_unit_lbl.pack(side='left', padx=(0, 4))
        mk_btn_outline(atb3, "ZASTOSUJ", self._redraw_arch, C['cyan']).pack(side='left', padx=(4, 0))
        self.arch_align_entry.config(state='disabled')

        self._arch_colors = [C['blue'], C['orange'], C['green'], C['red'],
                            C['cyan'], C['purple'], C['yellow'], '#ff8fab']
        self.refresh_arch()
        self._redraw_arch()

    def _on_align_mode_change(self):
        mode = self.arch_align_mode.get()
        if mode == 'none':
            self.arch_align_entry.config(state='disabled')
        else:
            self.arch_align_entry.config(state='normal')
            self.arch_align_unit_lbl.config(text="°C" if mode == 'temp' else "A (mozna np. 50n, 2u, 0.001)")
        self._redraw_arch()

    def _parse_align_value(self, mode):
        txt = self.arch_align_entry.get().strip().replace(',', '.')
        if not txt:
            return None
        try:
            if mode == 'current' and txt and txt[-1] in ('n', 'u', 'µ', 'm', 'p'):
                mult = {'p': 1e-12, 'n': 1e-9, 'u': 1e-6, 'µ': 1e-6, 'm': 1e-3}[txt[-1]]
                return float(txt[:-1]) * mult
            return float(txt)
        except ValueError:
            return None

    def _find_threshold_crossing(self, series, target):
        """Zwraca indeks pierwszego momentu, w ktorym seria (lista wartosci,
        moze zawierac None) przekracza target - w kierunku wyznaczonym przez
        pierwsza dostepna probke (rosnaco jesli target > start, malejaco w
        przeciwnym wypadku). Zwraca None jesli target nigdy nie zostal
        osiagniety (np. cykl zatrzymal sie wczesniej)."""
        first = next((v for v in series if v is not None), None)
        if first is None:
            return None
        rising = target >= first
        for i, v in enumerate(series):
            if v is None:
                continue
            if (rising and v >= target) or (not rising and v <= target):
                return i
        return None

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
            dlb = tk.Button(row, text="⤓", command=lambda p=f: self._export_single_cycle_csv(p),
                            bg=C['bg2'], fg=C['green'], font=(FONT, fsz(10), 'bold'),
                            relief='flat', cursor='hand2', bd=0, padx=8,
                            activebackground=C['green'], activeforeground='#fff')
            dlb.pack(side='right', padx=(2, 4))
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
            t,t1,t2,sp,pwm,ki = [],[],[],[],[],[]
            for r in rows:
                try:
                    t.append(float(r.get('czas_od_startu_s',0)))
                    v1 = r.get('temperatura1_C','')
                    t1.append(float(v1) if v1 else None)
                    v2 = r.get('temperatura2_C','')
                    t2.append(float(v2) if v2 else None)
                    sp.append(float(r.get('setpoint_cel_C',0)))
                    pwm.append(float(r.get('peltier_pct',0)))
                    vki = r.get('keithley_prad_A','')
                    ki.append(float(vki) if vki else None)
                except: continue
            return (t,t1,t2,sp,pwm,ki) if t else None
        except: return None

    def _style_arch_ax(self, ax):
        ax.set_facecolor(C['panel2'])
        ax.tick_params(colors=C['dim'], labelsize=8)
        ax.grid(True, alpha=0.3, color=C['grid'])
        for sp2 in ax.spines.values(): sp2.set_color(C['border'])

    # ─── HOVER: kursor "co sie stalo w tym punkcie" ─────────
    def _arch_reset_hover(self):
        """Czysci stan hover przy pustym wykresie (brak zaznaczonych cykli) -
        stare artysty (linie/adnotacja) zostaly juz usuniete przez fig.clear()."""
        self._arch_hover_series = []
        self._arch_annot = None
        self._arch_vline_top = None
        self._arch_vline_bottom = None
        self._arch_marker_top = None
        self._arch_marker_bottom = None
        self._arch_marker_main = None
        self._arch_hover_last_key = None

    def _arch_setup_hover(self, mode, hover_series):
        """Tworzy niewidoczne na starcie artysty (linia pionowa + kropka +
        dymek z tekstem) uzywane przez _on_arch_hover. Wywolywane po kazdym
        przebudowaniu wykresu (fig.clear() kasuje poprzednie artysty)."""
        self._arch_hover_mode = mode
        self._arch_hover_series = hover_series
        self._arch_hover_last_key = None
        if mode == 'time':
            self._arch_vline_top = self.ax_a.axvline(
                0, color=C['text'], lw=0.8, ls=':', alpha=0)
            self._arch_marker_top, = self.ax_a.plot(
                [], [], 'o', ms=5, alpha=0, zorder=9)
            if self.ax_a_current is not None:
                self._arch_vline_bottom = self.ax_a_current.axvline(
                    0, color=C['text'], lw=0.8, ls=':', alpha=0)
                self._arch_marker_bottom, = self.ax_a_current.plot(
                    [], [], 'o', ms=5, alpha=0, zorder=9)
            else:
                self._arch_vline_bottom = None
                self._arch_marker_bottom = None
            self._arch_marker_main = None
        else:  # iT
            self._arch_marker_main, = self.ax_a.plot(
                [], [], 'o', ms=7, alpha=0, zorder=9)
            self._arch_vline_top = None
            self._arch_vline_bottom = None
            self._arch_marker_top = None
            self._arch_marker_bottom = None
        self._arch_annot = self.ax_a.annotate(
            '', xy=(0, 0), xytext=(14, 14), textcoords='offset points',
            bbox=dict(boxstyle='round,pad=0.4', fc=C['panel2'], ec=C['border'], alpha=0.95),
            fontsize=fsz(8), color=C['text'], family=FONT, visible=False, zorder=10)

    def _arch_hover_axes(self):
        if getattr(self, '_arch_hover_mode', 'time') == 'time':
            axes = [self.ax_a]
            if self.ax_a_current is not None:
                axes.append(self.ax_a_current)
            return axes
        return [self.ax_a]

    def _arch_nearest_index(self, xs, target, sorted_asc):
        """Zwraca indeks najblizszego punktu w xs do target. Dla danych
        posortowanych (czas) uzywa bisekcji - szybkie nawet dla dlugich
        cykli. Dla niesortowanych (temperatura w trybie I-T, gdzie moze
        rosnac i malec w jednym cyklu) robi skan liniowy."""
        n = len(xs)
        if n == 0:
            return None
        if sorted_asc:
            idx = bisect.bisect_left(xs, target)
            cands = [i for i in (idx - 1, idx) if 0 <= i < n]
            if not cands:
                return 0
            return min(cands, key=lambda i: abs(xs[i] - target))
        return min(range(n), key=lambda i: abs(xs[i] - target))

    def _on_arch_hover(self, event):
        series = getattr(self, '_arch_hover_series', None)
        if not series or getattr(self, '_arch_annot', None) is None:
            return
        if event.inaxes not in self._arch_hover_axes() or event.xdata is None:
            self._arch_hide_hover()
            return

        mode = self._arch_hover_mode
        # Dopasowanie do panelu pod kursorem: gorny = temperatura, dolny = prad
        # (dla trybu iT jest tylko jeden panel = prad vs temperatura).
        if mode == 'time' and event.inaxes is self.ax_a_current:
            field = 'current'
        elif mode == 'time':
            field = 'temp'
        else:
            field = 'current'

        xlo, xhi = event.inaxes.get_xlim()
        ylo, yhi = event.inaxes.get_ylim()
        xr = (xhi - xlo) or 1.0
        yr = (yhi - ylo) or 1.0

        best = None  # (score, series_dict, idx)
        for s in series:
            idx = self._arch_nearest_index(s['x'], event.xdata, s['sorted'])
            if idx is None:
                continue
            yv = s.get(field, [None]*len(s['x']))[idx]
            if yv is None:
                # brak wartosci Keithleya w tym punkcie (np. sweep byl wylaczony) -
                # ciagle liczy sie jako kandydat po samym X, tylko z gorszym scorem
                dy = yr
            else:
                dy = abs(yv - (event.ydata if event.ydata is not None else yv))
            dx = abs(s['x'][idx] - event.xdata)
            score = (dx / xr) ** 2 + (dy / yr) ** 2
            if best is None or score < best[0]:
                best = (score, s, idx)

        if best is None:
            self._arch_hide_hover()
            return
        _, s, idx = best
        key = (s['name'], idx)
        if key == self._arch_hover_last_key:
            return
        self._arch_hover_last_key = key

        t_val = s['t'][idx]
        temp_val = s['temp'][idx] if idx < len(s['temp']) else None
        cur_val = s['current'][idx] if idx < len(s['current']) else None

        lines = [s['name']]
        lines.append(f"czas: {t_val:.2f} s")
        if temp_val is not None:
            lines.append(f"T1: {temp_val:.3f} °C")
        if cur_val is not None:
            v, p = fmt_si(cur_val, 3)
            lines.append(f"I: {v} {p}A")
        else:
            lines.append("I: --")
        text = "\n".join(lines)

        self._arch_annot.set_text(text)
        self._arch_annot.set_visible(True)
        self._arch_annot.get_bbox_patch().set_edgecolor(s['color'])

        if mode == 'time':
            xv = s['x'][idx]
            self._arch_annot.xy = (xv, temp_val if event.inaxes is self.ax_a else (cur_val or 0))
            self._arch_vline_top.set_xdata([xv, xv]); self._arch_vline_top.set_alpha(0.6)
            if temp_val is not None:
                self._arch_marker_top.set_data([xv], [temp_val])
                self._arch_marker_top.set_color(s['color']); self._arch_marker_top.set_alpha(1)
            if self._arch_vline_bottom is not None:
                self._arch_vline_bottom.set_xdata([xv, xv]); self._arch_vline_bottom.set_alpha(0.6)
                if cur_val is not None:
                    self._arch_marker_bottom.set_data([xv], [cur_val])
                    self._arch_marker_bottom.set_color(s['color']); self._arch_marker_bottom.set_alpha(1)
                else:
                    self._arch_marker_bottom.set_alpha(0)
        else:
            xv = s['x'][idx]
            self._arch_annot.xy = (xv, cur_val if cur_val is not None else 0)
            if cur_val is not None:
                self._arch_marker_main.set_data([xv], [cur_val])
                self._arch_marker_main.set_color(s['color']); self._arch_marker_main.set_alpha(1)

        self.cv_a.draw_idle()

    def _arch_hide_hover(self):
        if getattr(self, '_arch_annot', None) is None:
            return
        if self._arch_hover_last_key is None:
            return
        self._arch_hover_last_key = None
        self._arch_annot.set_visible(False)
        if self._arch_hover_mode == 'time':
            self._arch_vline_top.set_alpha(0)
            self._arch_marker_top.set_alpha(0)
            if self._arch_vline_bottom is not None:
                self._arch_vline_bottom.set_alpha(0)
                self._arch_marker_bottom.set_alpha(0)
        elif self._arch_marker_main is not None:
            self._arch_marker_main.set_alpha(0)
        self.cv_a.draw_idle()

    def _redraw_arch(self):
        sel = [(p,v) for p,v in self.arch_vars.items() if v.get()]
        show_current = getattr(self, 'arch_show_current_var', None) is not None \
                       and self.arch_show_current_var.get()
        mode = getattr(self, 'arch_chart_mode', None)
        mode = mode.get() if mode is not None else 'time'

        # Najpierw wczytaj dane - dopiero potem decyduj o ukladzie osi
        files = sorted([f for f in self.log_dir.glob("*.csv")
                        if (f.name.startswith("cykl_") or f.name.startswith("c_"))
                        and not f.name.startswith("_tmp")], reverse=True)
        forder = {str(f):i for i,f in enumerate(files)}
        loaded = []
        any_current_data = False
        for path,_ in sel:
            d = self._load_cycle(path)
            if not d: continue
            t,t1,t2,sp,pwm,ki = d
            has_i = any(v is not None for v in ki)
            any_current_data = any_current_data or has_i
            loaded.append((path, t, t1, sp, ki, has_i))

        # checkbox "pokaz prad pod spodem" i wyrownanie maja sens tylko w widoku czasowym
        if hasattr(self, 'arch_show_current_chk'):
            self.arch_show_current_chk.config(state='normal' if mode == 'time' else 'disabled')
        if hasattr(self, 'arch_align_temp_btn'):
            align_state = 'normal' if mode == 'time' else 'disabled'
            self.arch_align_temp_btn.config(state=align_state)
            self.arch_align_cur_btn.config(state=align_state)
            if mode != 'time':
                self.arch_align_entry.config(state='disabled')
            elif self.arch_align_mode.get() != 'none':
                self.arch_align_entry.config(state='normal')

        if mode == 'iT':
            self._redraw_arch_iT(sel, loaded, forder)
            return

        # Przebuduj uklad figury od zera (jedna os lub dwie jedna pod druga)
        self.fig_a.clear()
        two_panels = show_current and any_current_data
        if two_panels:
            gs = self.fig_a.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.12,
                                         left=0.1, right=0.97, top=0.96, bottom=0.09)
            self.ax_a = self.fig_a.add_subplot(gs[0])
            ax_i = self.fig_a.add_subplot(gs[1], sharex=self.ax_a)
        else:
            self.ax_a = self.fig_a.add_subplot(111)
            self.fig_a.subplots_adjust(left=0.1, right=0.97, top=0.96, bottom=0.09)
            ax_i = None
        self.ax_a_current = ax_i
        self._style_arch_ax(self.ax_a)
        if ax_i is not None: self._style_arch_ax(ax_i)

        if not sel:
            self.ax_a.text(0.5,0.5,"Tick a cycle to display",
                           ha='center',va='center',color=C['dim2'],
                           fontsize=11,transform=self.ax_a.transAxes)
            self._arch_reset_hover()
            self.cv_a.draw(); return

        hover_series = []
        align_mode = getattr(self, 'arch_align_mode', None)
        align_mode = align_mode.get() if align_mode is not None else 'none'
        align_val = self._parse_align_value(align_mode) if align_mode != 'none' else None
        align_missed = []  # nazwy cykli ktore nie osiagnely progu - uzyty naturalny start
        for path, t, t1, sp, ki, has_i in loaded:
            ci = forder.get(path,0)%len(self._arch_colors)
            col = self._arch_colors[ci]
            nm = self._cycle_display_name(path)
            t_ref = t[0]
            if align_mode == 'temp' and align_val is not None:
                idx = self._find_threshold_crossing(t1, align_val)
                if idx is not None: t_ref = t[idx]
                else: align_missed.append(nm)
            elif align_mode == 'current' and align_val is not None:
                idx = self._find_threshold_crossing(ki, align_val)
                if idx is not None: t_ref = t[idx]
                else: align_missed.append(nm)
            tx = [x - t_ref for x in t]
            self.ax_a.plot(tx, sp, color=C['orange'], lw=1, ls='--', alpha=0.5)
            self.ax_a.plot(tx, t1, color=col, lw=2, label=nm[:20])
            if ax_i is not None and has_i:
                # ten sam kolor co T1 danego cyklu - latwo skojarzyc pary krzywych
                ax_i.plot(tx, ki, color=col, lw=1.4)
            # dane do hover-kursora: tx jest chronologicznie rosnace (bisect OK)
            hover_series.append({'name': nm, 'color': col, 'x': tx, 't': tx,
                                  'temp': t1, 'current': ki, 'sorted': True})

        if align_mode == 'temp' and align_val is not None:
            xlabel = f"czas wzgledem T={align_val:g}°C [s]"
        elif align_mode == 'current' and align_val is not None:
            av, ap = fmt_si(align_val, 2)
            xlabel = f"czas wzgledem I={av} {ap}A [s]"
        else:
            xlabel = "czas [s]"

        if ax_i is not None:
            try:
                from matplotlib.ticker import EngFormatter
                ax_i.yaxis.set_major_formatter(EngFormatter(unit='A'))
            except Exception:
                pass
            ax_i.set_ylabel('prad Keithley', color=C['dim'], fontsize=9)
            ax_i.set_xlabel(xlabel, color=C['dim'], fontsize=9)
            # ukryj etykiety X gornego wykresu - wspolna os czasu na dole
            self.ax_a.tick_params(labelbottom=False)
        else:
            self.ax_a.set_xlabel(xlabel, color=C['dim'], fontsize=9)
            if show_current and not any_current_data:
                self.ax_a.text(0.02, 0.02, "brak danych Keithleya w zaznaczonych cyklach",
                               color=C['dim2'], fontsize=8, transform=self.ax_a.transAxes)
        if align_missed:
            names = ", ".join(align_missed[:3]) + ("…" if len(align_missed) > 3 else "")
            self.ax_a.text(0.02, 0.98,
                           f"⚠ nie osiagnieto progu: {names} (uzyto naturalnego startu)",
                           color=C['yellow'], fontsize=7, transform=self.ax_a.transAxes,
                           va='top')

        self.ax_a.set_ylabel('temperatura [°C]', color=C['dim'], fontsize=9)
        self.ax_a.legend(facecolor=C['panel'], edgecolor=C['border'],
                         labelcolor=C['dim'], fontsize=8)
        # zresetuj stos nawigacji toolbara (HOME wraca do nowego widoku, nie starego)
        if hasattr(self, 'mpl_toolbar_a'):
            try: self.mpl_toolbar_a.update()
            except Exception: pass
        self._arch_setup_hover('time', hover_series)
        self.cv_a.draw()

    def _redraw_arch_iT(self, sel, loaded, forder):
        """Wykres prad Keithleya (Y) w funkcji temperatury T1 (X), oddzielnie dla
        kazdego zaznaczonego cyklu. Punkty polaczone linia w kolejnosci czasowej,
        wiec widac petle grzanie/chlodzenie (histereza) jesli wystepuje - typowe
        dla sygnalu piroelektrycznego, ktory zalezy od dT/dt, a nie tylko od T."""
        self.fig_a.clear()
        self.ax_a = self.fig_a.add_subplot(111)
        self.fig_a.subplots_adjust(left=0.12, right=0.97, top=0.96, bottom=0.11)
        self.ax_a_current = None
        self._style_arch_ax(self.ax_a)

        if not sel:
            self.ax_a.text(0.5, 0.5, "Tick a cycle to display",
                           ha='center', va='center', color=C['dim2'],
                           fontsize=11, transform=self.ax_a.transAxes)
            self._arch_reset_hover()
            self.cv_a.draw(); return

        any_i = False
        hover_series = []
        for path, t, t1, sp, ki, has_i in loaded:
            if not has_i:
                continue
            any_i = True
            ci = forder.get(path, 0) % len(self._arch_colors)
            col = self._arch_colors[ci]
            nm = self._cycle_display_name(path)
            t0 = t[0]
            xs, ys, ts = [], [], []
            for tt, ii, ttime in zip(t1, ki, t):
                if ii is not None and tt is not None:
                    xs.append(tt); ys.append(ii); ts.append(ttime - t0)
            if not xs:
                continue
            self.ax_a.plot(xs, ys, color=col, lw=1.1, alpha=0.85,
                           marker='.', markersize=2, label=nm[:20])
            # temperatura nie zawsze rosnie monotonicznie (grzanie+chlodzenie
            # w jednym cyklu, szum) - wyszukiwanie najblizszego punktu liniowe
            hover_series.append({'name': nm, 'color': col, 'x': xs, 't': ts,
                                  'temp': xs, 'current': ys, 'sorted': False})

        if not any_i:
            self.ax_a.text(0.5, 0.5, "brak danych Keithleya w zaznaczonych cyklach",
                           ha='center', va='center', color=C['dim2'],
                           fontsize=10, transform=self.ax_a.transAxes)
            self._arch_reset_hover()
            self.cv_a.draw(); return

        try:
            from matplotlib.ticker import EngFormatter
            self.ax_a.yaxis.set_major_formatter(EngFormatter(unit='A'))
        except Exception:
            pass
        self.ax_a.set_xlabel('temperatura T1 [°C]', color=C['dim'], fontsize=9)
        self.ax_a.set_ylabel('prad Keithley', color=C['dim'], fontsize=9)
        self.ax_a.legend(facecolor=C['panel'], edgecolor=C['border'],
                         labelcolor=C['dim'], fontsize=8)
        if hasattr(self, 'mpl_toolbar_a'):
            try: self.mpl_toolbar_a.update()
            except Exception: pass
        self._arch_setup_hover('iT', hover_series)
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
        """Kopiuje surowy plik CSV zaznaczonego cyklu (lub cykli - checkboxy na
        liscie po lewej) do wskazanej lokalizacji. Plik juz zawiera komplet
        danych raw: T1, T2, setpointy, PID, Keithley - dokladnie to co zostalo
        zapisane od START do STOP. Dla pojedynczego pliku mozna tez uzyc
        przycisku ⤓ bezposrednio przy pozycji na liscie (bez zaznaczania)."""
        import shutil
        from pathlib import Path as _P
        selected = [_P(p) for p, v in self.arch_vars.items() if v.get()]
        if not selected:
            messagebox.showinfo("Brak zaznaczenia", "Zaznacz cykl (checkbox) na liscie po lewej, "
                                "albo uzyj przycisku ⤓ bezposrednio przy wybranej pozycji.")
            return
        if len(selected) == 1:
            self._export_single_cycle_csv(selected[0])
            return
        from tkinter import filedialog
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

    def _export_single_cycle_csv(self, src):
        """Zapisuje jeden konkretny plik cyklu (przycisk ⤓ przy pozycji na liscie -
        dziala od razu, bez zaznaczania checkboxa)."""
        from pathlib import Path as _P
        import shutil
        src = _P(src)
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Zapisz raw dane cyklu", defaultextension=".csv",
            initialfile=src.name, filetypes=[("CSV", "*.csv")])
        if not dest:
            return
        try:
            shutil.copy(src, dest)
            messagebox.showinfo("Zapisano", f"Raw dane cyklu zapisane do:\n{dest}")
        except Exception as e:
            messagebox.showerror("Blad", str(e))

    def _show_excel_help(self):
        """Instrukcja: dlaczego Excel czesto psuje ten CSV przy zwyklym
        dwuklikni?ciu, i jak poprawnie zaimportowac dane (osobne kolumny,
        poprawny separator dziesietny), zeby dalsza obrobka byla latwa."""
        win = tk.Toplevel(self.root)
        win.title("Jak otworzyc CSV w Excelu")
        win.configure(bg=C['bg'])
        w, h = px(620), px(560)
        try:
            self.root.update_idletasks()
            x = self.root.winfo_rootx() + (self.root.winfo_width() - w)//2
            y = self.root.winfo_rooty() + (self.root.winfo_height() - h)//2
            win.geometry(f"{w}x{h}+{max(0,x)}+{max(0,y)}")
        except Exception:
            win.geometry(f"{w}x{h}")
        win.minsize(w, h)
        win.transient(self.root)

        tk.Label(win, text="Import CSV do Excela", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w', padx=16, pady=(14, 4))

        body = tk.Frame(win, bg=C['bg'])
        body.pack(fill='both', expand=True, padx=16, pady=(0, 12))
        txt = tk.Text(body, bg=C['panel2'], fg=C['text'], font=(FONT, fsz(9)),
                       wrap='word', relief='flat', padx=12, pady=12,
                       highlightthickness=1, highlightbackground=C['border'])
        sb = tk.Scrollbar(body, command=txt.yview)
        txt.config(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')
        txt.pack(side='left', fill='both', expand=True)

        content = (
            "PROBLEM: nasz plik CSV rozdziela kolumny PRZECINKIEM, a liczby "
            "dziesietne maja KROPKE (np. 48.30, nie 48,30). Polski Excel domyslnie "
            "oczekuje odwrotnie: srednika jako separatora kolumn i przecinka jako "
            "separatora dziesietnego. Dlatego zwykle dwuklikniecie na plik CSV czesto "
            "wrzuca WSZYSTKO do jednej kolumny albo zamienia liczby na tekst.\n\n"
            "NAJLEPSZY SPOSOB (Excel 2016+, zawsze dziala poprawnie):\n"
            "1. Otworz PUSTY skoroszyt Excela (nie klikaj CSV dwa razy).\n"
            "2. Zakladka DANE -> \"Z tekstu/CSV\" (Get Data From Text/CSV).\n"
            "3. Wskaz nasz plik cykl_....csv.\n"
            "4. W oknie podgladu ustaw:\n"
            "     Ogranicznik (Delimiter): Przecinek\n"
            "     Pochodzenie pliku: UTF-8\n"
            "5. Kliknij \"Przeksztalc dane\" (Transform Data) albo od razu \"Zaladuj\".\n"
            "6. Jesli liczby wyjda jako tekst (wyrownane do lewej), zaznacz kolumny "
            "z liczbami -> DANE -> Tekst jako kolumny -> Dalej -> Dalej -> w kroku 3 "
            "wybierz \"Kropka\" jako separator dziesietny (przycisk Zaawansowane) "
            "-> Zakoncz.\n\n"
            "SZYBSZY SPOSOB (dwuklik na plik, gdy nie chce sie robic importu):\n"
            "1. Otworz plik zwyklym dwuklikiem - dane wpadna w jedna kolumne.\n"
            "2. Zaznacz cala kolumne A.\n"
            "3. DANE -> Tekst jako kolumny.\n"
            "4. Wybierz \"Rozdzielany\" -> Dalej.\n"
            "5. Zaznacz ogranicznik \"Przecinek\" (odznacz inne) -> Dalej.\n"
            "6. Kliknij \"Zaawansowane\" (Advanced) w prawym dolnym rogu -> ustaw "
            "\"Separator dziesietny: kropka\", \"Separator tysiecy: (puste)\" -> OK.\n"
            "7. Zakoncz.\n\n"
            "KOLUMNY W PLIKU CYKLU (cykl_*.csv / c_*.csv):\n"
            "  timestamp_pc          - znacznik czasu komputera (ISO 8601)\n"
            "  czas_firmware_s       - czas wg zegara mikrokontrolera [s]\n"
            "  czas_od_startu_s      - czas od poczatku cyklu [s] (najwygodniejszy do wykresow)\n"
            "  temperatura1_C        - temperatura z czujnika 1 [°C]\n"
            "  temperatura2_C        - temperatura z czujnika 2 [°C]\n"
            "  setpoint_aktywny_C    - biezacy punkt zadany (w trakcie rampy) [°C]\n"
            "  setpoint_cel_C        - docelowy punkt zadany [°C]\n"
            "  peltier_pct           - moc Peltiera [%]\n"
            "  fan_pct               - moc wentylatora [%]\n"
            "  kierunek              - HEAT / COOL\n"
            "  Kp, Ki, Kd            - nastawy regulatora PID\n"
            "  stan                  - stan automatu (RUN / IDLE / itp.)\n"
            "  keithley_prad_A       - prad zmierzony przez SMU [A], notacja naukowa np. 4.83e-08\n"
            "  keithley_napiecie_V   - napiecie zmierzone przez SMU [V]\n\n"
            "UWAGA na notacje naukowa (np. 4.83e-08): Excel poprawnie ja rozumie jako "
            "liczbe, o ile format kolumny to \"Ogolny\" lub \"Naukowy\" - NIE formatuj "
            "tych kolumn jako \"Tekst\" przed importem, bo zostana zamienione na string "
            "i nie da sie ich uzyc w wykresach/formulach.\n\n"
            "SZYBKI WYKRES W EXCELU (I vs T albo I vs czas):\n"
            "1. Po poprawnym imporcie zaznacz dwie kolumny, np. temperatura1_C i "
            "keithley_prad_A (przytrzymaj Ctrl, zeby zaznaczyc niesasiadujace kolumny).\n"
            "2. WSTAWIANIE -> Wykres punktowy (XY Scatter) -> \"Punktowy z liniami "
            "prostymi\" jesli chcesz zachowac kolejnosc czasowa (widac wtedy petle "
            "histerezy grzanie/chlodzenie), albo zwykly \"Punktowy\" dla samej chmury "
            "punktow."
        )
        txt.insert('1.0', content)
        txt.config(state='disabled')

        mk_btn(win, "ZAMKNIJ", win.destroy, C['cyan']).pack(pady=(0, 14))

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

                # Ostatnia znana temp/czas - do odczytu przez watek sweep (korelacja
                # kazdego punktu I-V z temperatura w momencie pomiaru)
                self.last_known_rel = rel
                self.last_known_t1 = t1
                self.last_known_t2 = t2
                self.last_known_sp = sp

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

                # Jedno wywolanie na probke - ta sama wartosc uzyta wszedzie
                # (log cyklu, RAW DATA, karta na zywo), zeby wykluczyc jakakolwiek
                # niespojnosc miedzy oddzielnymi odczytami w tej samej iteracji.
                k_i, k_v = self._keithley_latest()
                heat_dir = d.get('heat', True)

                if self.cyc_on:
                    self.cyc_log(rel, t1, t2, sp, pct, fn,
                                 spa=spa, kp=d.get('kp'), ki=d.get('ki'), kd=d.get('kd'),
                                 fw_ts=tsr, state=d.get('state'),
                                 keithley_i=k_i, keithley_v=k_v, heat=heat_dir)

                pc_now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self._raw_append((tsr, pc_now, t1, t2, sp, spa, pct, fn, d.get('state',''), k_i, k_v, heat_dir))

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
                if k_i is not None:
                    kv, kpref = fmt_si(k_i, 3)
                    self.cards['kcur']['val'].config(text=kv)
                    self.cards['kcur']['unit_lbl'].config(text=f" {kpref}A")
                elif self.keithley_running:
                    self.cards['kcur']['val'].config(text="...")
                else:
                    self.cards['kcur']['val'].config(text="--")
                    self.cards['kcur']['unit_lbl'].config(text=" A")

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

            # Nie przerysowuj gdy uzytkownik ma aktywne narzedzie ZOOM/PAN z
            # toolbara - inaczej ax.clear() co 250 ms kasowalby przyblizenie
            # i zoom na wykresie live nigdy nie dzialal.
            if self.t and not self.chart_paused and not self._live_toolbar_busy():
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
    def _recover_tmp_cycles(self):
        """Po awarii/wymuszonym zamknieciu programu pliki _tmp_cykl_* zostaja na
        dysku i nigdy nie trafialy do archiwum (filtr _tmp je ukrywal) - dane
        przepadaly. Teraz przy starcie zmieniamy je na c_odzyskane_* zeby byly
        widoczne w ARCHIVE. Puste pliki (sam naglowek) sa usuwane."""
        try:
            for f in self.log_dir.glob("_tmp_cykl_*.csv"):
                try:
                    if f.stat().st_size < 300:  # sam naglowek / prawie puste
                        f.unlink()
                        continue
                    ts = f.stem.replace("_tmp_cykl_", "")
                    dest = self.log_dir / f"c_odzyskane_{ts}.csv"
                    if not dest.exists():
                        f.rename(dest)
                        print(f"Odzyskano przerwany cykl -> {dest.name}")
                except Exception as e:
                    print(f"recover err ({f.name}): {e}")
        except Exception as e:
            print(f"recover scan err: {e}")

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
            'peltier_pct', 'fan_pct', 'kierunek',
            'Kp', 'Ki', 'Kd', 'stan',
            'keithley_prad_A', 'keithley_napiecie_V',
        ])
        self.cyc_rows = 0
        print(f"CYC START T={temp0}")

    def cyc_log(self, t, t1, t2, sp, pct, fn, spa=None, kp=None, ki=None, kd=None,
                fw_ts=None, state=None, keithley_i=None, keithley_v=None, heat=None):
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
                kvs = f"{keithley_v:.9e}" if keithley_v is not None else ""
                dirs = "" if heat is None else ("HEAT" if heat else "COOL")
                pc_ts = datetime.now().isoformat(timespec="milliseconds")
                self.cyc_wr.writerow([
                    pc_ts, fwts, f"{t:.3f}",
                    t1s, t2s,
                    spas, f"{sp:.3f}",
                    f"{pct:.2f}", f"{fn:.2f}", dirs,
                    kps, kis, kds, state or "",
                    kis_a, kvs,
                ])
                self.cyc_file.flush(); self.cyc_rows += 1
            except Exception as e:
                # NIE gub bledow po cichu - licz je i pokaz w naglowku statusu,
                # zeby brak danych w CSV nie byl niespodzianka po eksperymencie
                self.cyc_write_errors = getattr(self, 'cyc_write_errors', 0) + 1
                print(f"cyc_log write err #{self.cyc_write_errors}: {e}")
                if self.cyc_write_errors == 1 and hasattr(self, 's_lbl'):
                    try:
                        self.s_lbl.config(text="UWAGA: blad zapisu CSV cyklu!", fg=C['red'])
                    except Exception:
                        pass

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
        w, h = px(440), px(230)
        # wysrodkuj nad oknem glownym (a nie w rogu ekranu)
        try:
            parent.update_idletasks()
            x = parent.winfo_rootx() + (parent.winfo_width() - w) // 2
            y = parent.winfo_rooty() + (parent.winfo_height() - h) // 2
            self.win.geometry(f"{w}x{h}+{max(0, x)}+{max(0, y)}")
        except Exception:
            self.win.geometry(f"{w}x{h}")
        self.win.minsize(w, h)
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
        app.sweep_abort = True
        app.keithley_running = False
        app.keithley_disconnect()
        app.disconnect()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
