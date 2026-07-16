#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PeltierControl - BRUTALIST
Panel sterowania PID Peltiera (Feed-Forward) z dwukierunkowa komunikacja JSON.
Firmware: ItsyBitsy M0 + Cytron MDD10A + 2x MAX31856
"""

import sys, os, time, csv, json, threading, queue, socket, bisect
from datetime import datetime, timedelta
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
    # WAZNE: path.simplify CELOWO WYLACZONE. Wczesniej probowalismy je
    # wlaczyc jako "czysto renderingowa" optymalizacje predkosci (nie
    # dotykajaca danych), ale przy threshold=1.0 dawalo to widocznie
    # zniejszony, niepoprawny wyglad wykresu (linie wygladaly na uproszczone/
    # znieksztalcone na ekranie, mimo ze dane w pamieci byly kompletne).
    # Uzytkownik wyraznie zastrzegl, ze nie chce ZADNEGO zniekształcenia
    # rysowania - nawet czysto wizualnego, na poziomie pikseli - wiec
    # zostawiamy domyslne, surowe renderowanie matplotlib (kazdy punkt
    # rysowany dokladnie tak jak jest, bez zadnej optymalizacji upraszczajacej
    # geometrie linii).
    matplotlib.rcParams['path.simplify'] = False
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
FS = 1.12  # delikatne globalne powiekszenie czcionki calej aplikacji
def fsz(n): return max(6, int(round(n * FS)))
def px(n): return max(1, int(round(n * FS)))  # skalowanie wymiarow (szerokosci/wysokosci) wg DPI

# ── TWARDE LIMITY SPRZETOWE KEITHLEY 2611B (wg specyfikacji producenta) ──
# Zrodlo napiecia: max 200V (ciaglo DC, ograniczone termicznie przy wiekszym
# pradzie - patrz SMU_MAX_POWER). Zrodlo pradu: max 1.5A DC ciagle (10A to
# TYLKO tryb impulsowy, ktorego ta aplikacja nie uzywa). Min. compliance
# pradowej wg specyfikacji: 10nA.
SMU_V_MAX = 200.0      # V - maksymalne napiecie zrodla
SMU_I_MAX = 1.5         # A - maksymalny prad zrodla (DC ciagly, nie pulse)
SMU_I_COMPLIANCE_MIN = 10e-9   # A - minimalna sensowna compliance wg specyfikacji
SMU_MAX_POWER = 30.3    # W - maksymalna moc na kanal (V*I nie moze przekroczyc)

# ── Zakresy sterownika Peltiera (zgodne z limitami firmware SP_MIN/SP_MAX/
# RAMP_MIN/RAMP_MAX) - uzywane do walidacji krokow PROGRAMATORA po stronie PC,
# zanim cokolwiek zostanie wyslane do urzadzenia. ──
SP_MIN_UI = -15.0
SP_MAX_UI = 100.0
RAMP_MIN_UI = 0.5
RAMP_MAX_UI = 150.0

# Widoczny w gornym pasku aplikacji numer builda - podbijany przy kazdej
# wiekszej zmianie. Pozwala od razu sprawdzic "na oko" czy uruchomiony plik
# to faktycznie najnowsza wersja main.py, bez zgadywania po samym wygladzie.
BUILD_VERSION = "2026-07-11.19"

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

def mk_segmented(parent, options, variable, command=None, accent=None, font_size=9):
    """Grupa przyciskow-przelacznikow wygladajaca jak JEDNA spojna kontrolka
    (segmented control) zamiast oddzielnych, rozjechanych 'kwadratow' -
    cienka wspolna obwodka, przyciski stykaja sie bez przerw, aktywny segment
    dostaje pelne wypelnienie kolorem akcentu z ciemnym tekstem. Zastepuje
    wczesniejsze uzycie tk.Radiobutton(indicatoron=False) ktore w praktyce
    nie dawalo wyraznej roznicy miedzy stanem wybranym a niewybranym.
    options: lista (etykieta, wartosc). Zwraca dict {wartosc: widget}."""
    accent = accent or C['cyan']
    outer = tk.Frame(parent, bg=C['border'])
    outer.pack(side='left')
    inner = tk.Frame(outer, bg=C['panel'])
    inner.pack(padx=1, pady=1)
    btns = {}
    def _select(val, fire_command=True):
        variable.set(val)
        for v, b in btns.items():
            if v == val:
                b.config(bg=accent, fg='#12181a')
            else:
                b.config(bg=C['panel'], fg=C['dim'])
        if fire_command and command: command()
    for i, (label, val) in enumerate(options):
        b = tk.Button(inner, text=label, command=lambda v=val: _select(v),
                     bg=C['panel'], fg=C['dim'], font=(FONT, fsz(font_size)),
                     relief='flat', cursor='hand2', bd=0, padx=10, pady=6,
                     activebackground=C['panel3'])
        b.pack(side='left', padx=(0 if i == 0 else 1, 0))
        btns[val] = b
    # Ustawiamy wizualnie POCZATKOWY wybrany segment BEZ odpalania command() -
    # w chwili tworzenia tej kontrolki widgety utworzone PO niej (np. pole
    # wpisu wartosci progu w ARCHIVE) moga jeszcze nie istniec, a command
    # czesto sie do nich odwoluje. Uzytkownik i tak nie klikal, wiec nic nie
    # powinno sie jeszcze wykonywac - tylko wyglad ma odzwierciedlac stan.
    _select(variable.get(), fire_command=False)
    return btns

def mk_checkbox(parent, text, variable, command=None, bg=None, wraplength=250, font_size=11, side=None, accent=None):
    """Wlasny checkbox (kwadrat-znacznik + etykieta, obsluga kliknieciem)
    zamiast natywnego tk.Checkbutton. Powod: na niektorych platformach/
    motywach natywny wskaznik Tk nie daje WYRAZNEJ roznicy miedzy stanem
    zaznaczonym i niezaznaczonym (oba wygladaly niemal identycznie), przez
    co nie dalo sie stwierdzic czy opcja jest wlaczona - zglaszany, powtarzajacy
    sie problem. Ten widget zawsze jednoznacznie pokazuje stan: wypelniony,
    ZIELONY kwadrat z checkmarkiem gdy ON, pusty, SZARY kontur gdy OFF -
    duzy, latwo widoczny, niezalezny od platformy/motywu, bo rysujemy to
    sami zamiast polegac na natywnym rendererze. wraplength ustawiony
    domyslnie (dlugi tekst etykiety zawijal sie zle/przycinal bez tego przy
    natywnym Checkbutton).
    side: None (domyslnie) pakuje blokowo (anchor='w', fill='x') - do uzycia
    w pionowych panelach formularzy. Podaj 'right'/'left' do uzycia w
    poziomym pasku narzedzi (np. obok innych kontrolek w jednym wierszu).
    accent: kolor stanu ON - domyslnie zielony; podaj inny (np. kolor cyklu
    w ARCHIVE) gdy zaznaczenie ma tez pelnic funkcje kodowania kolorem."""
    bg = bg or C['panel']
    accent = accent or C['green']
    row = tk.Frame(parent, bg=bg)
    if side:
        row.pack(side=side)
    else:
        row.pack(anchor='w', fill='x')
    box = tk.Label(row, text="☐", width=2, font=(FONT, fsz(font_size+5), 'bold'),
                   bg=bg, fg=C['dim2'])
    box.pack(side='left', padx=(0, 8), pady=(1, 0), anchor='n')
    lbl = tk.Label(row, text=text, bg=bg, fg=C['dim'], font=(FONT, fsz(font_size)),
                   justify='left', wraplength=wraplength, anchor='w')
    lbl.pack(side='left', fill='x', expand=True)

    def render():
        if variable.get():
            box.config(text="☑", fg=(accent if state['enabled'] else C['dim2']))
            lbl.config(fg=(C['text'] if state['enabled'] else C['dim2']))
        else:
            box.config(text="☐", fg=C['dim2'])
            lbl.config(fg=(C['dim'] if state['enabled'] else C['dim2']))

    def toggle(event=None):
        if not state['enabled']:
            return
        variable.set(not variable.get())
        render()
        if command: command()

    state = {'enabled': True}
    for w in (row, box, lbl):
        w.bind("<Button-1>", toggle)
        w.config(cursor='hand2')
    render()
    row.render = render  # pozwala wymusic odswiezenie wygladu z zewnatrz (np. po zmianie variable bez kliknięcia)

    def set_enabled(enabled):
        """Blokuje/odblokowuje klikalnosc i przygasza wyglad - zamiennik dla
        .config(state='disabled'/'normal') na natywnych Tk widgetach, ktorego
        ten wlasny checkbox (Frame+Label, nie prawdziwy Checkbutton) nie
        wspiera natywnie."""
        state['enabled'] = enabled
        row.config(cursor='hand2' if enabled else 'arrow')
        for w in (row, box, lbl):
            w.config(cursor='hand2' if enabled else 'arrow')
        render()
    row.set_enabled = set_enabled
    row.is_enabled = lambda: state['enabled']
    row.variable = variable
    return row

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
                "No USB device found. Check that the Keithley is connected "
                "via USB cable and that the WinUSB driver is installed (Zadig)."
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
                                  nplc=1.0, filter_count=1, autozero=True, autorange=True):
        """Konfiguruje SMU: zrodlo napiecia, pomiar pradu, dany limit pradowy (compliance).
        nplc - czas integracji pomiaru w okresach sieci (wyzej = mniej szumu, wolniej).
        filter_count - ile pomiarow usredniac sprzetowo w jeden odczyt (1 = wylaczone).
        autozero - AUTOZERO_AUTO (dokladniejsze, ale wg dokumentacji/forum Keithleya i NI
        potrafi znaczaco spowolnic pomiar - realne testy pokazuja nawet ~1.75x wolniej niz
        sam czysty czas NPLC) vs AUTOZERO_OFF (szybsze, ale odczyt moze z czasem dryfowac -
        warto okresowo recznie wywolac zero raz na jakis czas jesli to wylaczone na dluzej).
        autorange - AUTORANGE_ON (wygodne, ale przy sygnale blisko granicy zakresu potrafi
        "polowac" miedzy zakresami, dodajac narzut) vs stale ustawiony zakres (szybsze i
        stabilniejsze w czasie, ale trzeba znac przyblizona skale sygnalu z gory - uzyj
        ustawionego LIMIT jako przyblizenia najgorszego przypadku).
        Dla malych sygnalow (pA-nA, np. prad piroelektryczny) NPLC=0.01 to za mala
        integracja - odczyt to praktycznie sam szum ADC, ktory dodatkowo wywoluje
        ciagle przelaczanie zakresu (autorange hunting) i daje pilokształtne skoki +/-."""
        ch = f"smu{channel}"
        self._exec(f"{ch}.reset()")
        self._exec(f"{ch}.source.func = {ch}.OUTPUT_DCVOLTS")
        self._exec(f"{ch}.source.levelv = {voltage:.6f}")
        self._exec(f"{ch}.source.limiti = {ilimit:.6f}")
        self._exec(f"{ch}.measure.nplc = {nplc:.4f}")
        self._exec(f"{ch}.measure.autozero = {ch}.{'AUTOZERO_AUTO' if autozero else 'AUTOZERO_OFF'}")
        if autorange:
            self._exec(f"{ch}.measure.autorangei = {ch}.AUTORANGE_ON")
        else:
            self._exec(f"{ch}.measure.autorangei = {ch}.AUTORANGE_OFF")
            self._exec(f"{ch}.measure.rangei = {ilimit:.6f}")
        self._configure_filter(ch, filter_count)

    def setup_source_i_measure_v(self, channel="a", current=0.0, vlimit=1.0,
                                  nplc=1.0, filter_count=1, autozero=True, autorange=True):
        """Konfiguruje SMU: zrodlo pradu, pomiar napiecia, dany limit napieciowy (compliance).
        nplc / filter_count / autozero / autorange - patrz setup_source_v_measure_i."""
        ch = f"smu{channel}"
        self._exec(f"{ch}.reset()")
        self._exec(f"{ch}.source.func = {ch}.OUTPUT_DCAMPS")
        self._exec(f"{ch}.source.leveli = {current:.6f}")
        self._exec(f"{ch}.source.limitv = {vlimit:.6f}")
        self._exec(f"{ch}.measure.nplc = {nplc:.4f}")
        self._exec(f"{ch}.measure.autozero = {ch}.{'AUTOZERO_AUTO' if autozero else 'AUTOZERO_OFF'}")
        if autorange:
            self._exec(f"{ch}.measure.autorangev = {ch}.AUTORANGE_ON")
        else:
            self._exec(f"{ch}.measure.autorangev = {ch}.AUTORANGE_OFF")
            self._exec(f"{ch}.measure.rangev = {vlimit:.6f}")
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
        """Zwraca (prad_A, napiecie_V) z jednego zapytania (szybsze niz dwa
        osobne). Nie dotyka source.levelv i nie czeka na settle - do uzycia w
        trybie ciaglym/'tylko pomiar' PO pierwszej probce, kiedy wartosc
        zrodlowa juz jest ustawiona i sie nie zmienia miedzy pomiarami."""
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

        self.raw_maxrows = 100  # bufor w pamieci dla EXPORT CSV (live-wiersz pokazuje tylko ostatnia probke)
        self.selftune_running = False
        self.selftune_start_ts = 0.0
        self.raw_rows = []
        self.raw_paused = False
        self._raw_last_ui_ts = 0.0

        # Keithley 2611B (SMU) - pomiar pradu przez LAN/TSP, synchronizowany z PID
        self.keithley = KeithleyClient()
        self.keithley_connected = False
        self._keithley_autoconnect_enabled = True
        self._keithley_autoconnect_busy = False
        self.keithley_running = False
        self.keithley_thread = None
        self.keithley_lock = threading.Lock()
        self.keithley_last_i = None
        self.keithley_last_v = None
        self.keithley_last_ts = None
        self._last_logged_keithley_ts = None  # do wykrywania "swiezosci" probki przy zapisie do CSV cyklu
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
        self.SWEEP_SETTLE_SKIP_S = 3.0  # pomijane na wykresie po starcie/nowej petli
        self.last_known_rel = 0.0
        self.last_known_rel_wallclock_ts = time.time()  # do ekstrapolacji "t" dla wierszy wstawianych miedzy tykniecami PID (patrz cyc_log z watku Keithleya)
        self.last_known_t1 = None
        self.last_known_t2 = None
        self.last_known_sp = None
        self.last_known_spa = None
        self.last_known_pct = 0.0
        self.last_known_fn = 0.0
        self.last_known_heat_dir = True
        self.last_known_kp = None
        self.last_known_ki = None
        self.last_known_kd = None
        self.last_known_state = None
        self.cyc_log_lock = threading.Lock()  # chroni cyc_wr.writerow() przed jednoczesnym zapisem z 2 watkow (glowny tick + watek Keithleya)

        self.reach_start_t = None
        self.reach_start_temp = None
        self.reach_target = None
        self.reach_done = False
        self.reach_time = None
        self.reach_avg_rate = None
        self.reach_dir = None
        self.last_setpoint_target = None

        self.chart_paused = False

        # ── PROGRAMATOR: sekwencja krokow grzej/trzymaj/schlodz/trzymaj ──
        self.program_steps = []       # lista dict: {'target':C, 'ramp':C/min lub None, 'hold_min':min}
        self.program_running = False
        self.program_state = 'idle'   # 'idle' | 'ramping' | 'holding' | 'done'
        self.program_step_idx = 0
        self.program_step_sent = False
        self.program_stable_since = None
        self.program_hold_start = None
        self.program_tolerance = 0.5      # C - tolerancja uznania ze cel osiagniety (globalna, domyslna)
        self.prog_editing_idx = None      # indeks kroku aktualnie edytowanego (None = tryb "dodaj nowy")
        self.program_stable_needed_s = 5  # s - jak dlugo w tolerancji zanim zaczniemy hold
        self.chart_window = 0

        self.log_dir = Path.home() / "BigPeltierPidLogi"
        self.log_dir.mkdir(exist_ok=True)
        self.programs_dir = self.log_dir / "programy"
        self.programs_dir.mkdir(exist_ok=True)
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
        st.configure('TNotebook', background=C['bg2'], borderwidth=0, tabmargins=[2, 4, 2, 0])
        st.configure('TNotebook.Tab', background=C['bg2'], foreground=C['dim'],
                     padding=[22, 12], font=(FONT, fsz(10), 'bold'), borderwidth=0)
        # Aktywna zakladka: wyrazny akcentowy kolor tekstu (cyan) + jasniejsze
        # tlo + delikatny obrys w kolorze akcentu, zeby od razu bylo widac
        # ktora zakladka jest otwarta bez wpatrywania sie w subtelna roznice
        # szarosci. Stan 'active' (najechanie mysza) tez dostaje podswietlenie
        # dla lepszej informacji zwrotnej przed klikneciem.
        st.map('TNotebook.Tab',
               background=[('selected', C['bg']), ('active', C['panel3'])],
               foreground=[('selected', C['cyan']), ('active', C['text'])],
               bordercolor=[('selected', C['cyan'])],
               lightcolor=[('selected', C['bg'])],
               darkcolor=[('selected', C['bg'])])

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
        if self.cyc_on: self.cyc_stop("Disconnected")
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
            if 'KFFHC' in d: out['kffhc'] = float(d['KFFHC'])
            if 'KFFRC' in d: out['kffrc'] = float(d['KFFRC'])
            if 'OFFSET' in d: out['offset'] = float(d['OFFSET'])
            if 'OPPDIR' in d: out['oppdir'] = (d['OPPDIR'].strip() == '1')
            if 'TC2' in d: out['tc2_enabled'] = (d['TC2'].strip() == '1')
            if 'FAN' in d:
                fv = float(d['FAN'])
                out['fan_on'] = fv > 0
                out['fan_speed'] = fv
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
                self.root.after(0, lambda: self.set_status(False, "Connection lost"))
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
        tk.Label(top, text=f"build {BUILD_VERSION}", bg=C['bg2'], fg=C['panel3'],
                 font=(FONT, fsz(8))).pack(side='left', padx=(4, 0))

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
        t7 = tk.Frame(nb, bg=C['bg']); nb.add(t7, text='PROGRAM')
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
        self.build_program(t7)
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
        # width=7 + anchor='e': liczba jest wyrownana do prawej wewnatrz
        # STALEJ szerokosci (font jest monospace, wiec to daje pikselowo
        # stabilna pozycje) - bez tego kazda zmiana dlugosci liczby (np.
        # "24.5" -> "-14.3" -> "100.0") przesuwala jednostke obok i caly
        # dalszy uklad w wierszu kart, co wygladalo jak "skakanie" wartosci.
        val = tk.Label(vrow, text="--", bg=C['panel'], fg=color,
                       font=(FONT, fsz(16), 'bold'), width=7, anchor='e')
        val.pack(side='left')
        unit_lbl = tk.Label(vrow, text=" " + unit, bg=C['panel'], fg=C['dim2'],
                            font=(FONT, fsz(7)), width=9, anchor='w')
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
            ax.tick_params(colors=C['dim'], labelsize=8, length=0)
            ax.grid(True, axis='y', alpha=0.35, color=C['grid'])
            for s in ['top', 'right']: ax.spines[s].set_visible(False)
            for s in ['left', 'bottom']: ax.spines[s].set_color(C['border'])

        # ── Obiekty linii tworzone RAZ, aktualizowane pozniej przez set_data() ──
        # (zamiast ax.clear()+replot co 250ms - to bylo bardzo kosztowne i
        # dodatkowo kasowalo kazde przyblizenie/przesuniecie uzytkownika, bo
        # clear() zawsze resetuje granice osi do autoscale)
        self.ln_target, = self.ax1.plot([], [], color=C['orange'], lw=1.3, ls='--',
                                         label='target', alpha=0.7)
        self.ln_spa, = self.ax1.plot([], [], color=C['cyan'], lw=1.5, ls=':',
                                      label='setpoint (ramp)')
        self.ln_t1, = self.ax1.plot([], [], color=C['blue'], lw=2.2, label='T1')
        self.ln_t2, = self.ax1.plot([], [], color=C['purple'], lw=1.3, ls='--',
                                     label='T2', alpha=0.6)
        self.ax1.set_ylabel('°C', color=C['dim'], fontsize=9)
        self.ax1.legend(facecolor=C['panel'], edgecolor=C['border'],
                        labelcolor=C['dim'], fontsize=8, loc='upper right')

        # PWM ze znakiem: dodatni = grzanie (pomaranczowy), ujemny = chlodzenie
        # (niebieski/cyan) - dwie osobne linie + wypelnienia, kazda widoczna
        # TYLKO po swojej stronie zera (patrz _draw_chart: maskowanie przez
        # NaN dla drugiej polowy), zeby przejscie grzanie<->chlodzenie bylo
        # od razu czytelne jako zmiana koloru, bez osobnego wskaznika obok.
        self.ln_pwm_heat, = self.ax2.plot([], [], color=C['orange'], lw=1.6)
        self.ln_pwm_cool, = self.ax2.plot([], [], color=C['cyan'], lw=1.6)
        self.pwm_fill_heat = None  # PolyCollection - przebudowywane co odswiezenie
        self.pwm_fill_cool = None
        self.ax2.axhline(0, color=C['border2'], lw=0.8, zorder=1)
        self.ax2.set_ylabel('PWM %  (heat / cool)', color=C['dim'], fontsize=9)
        self.ax2.set_xlabel('time [s]', color=C['dim'], fontsize=9)
        self.ax2.set_ylim(-105, 105)
        self.ax2.legend([self.ln_pwm_heat, self.ln_pwm_cool], ['heat', 'cool'],
                        facecolor=C['panel'], edgecolor=C['border'],
                        labelcolor=C['dim'], fontsize=8, loc='upper right')

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
            self.mpl_toolbar = None
        self._setup_scroll_drag_zoom(self.cv, self.mpl_toolbar,
                                     '_live_chart_frozen', '_live_chart_dragging')

    def _build_sweep_mini_chart(self, parent):
        """Kompaktowy wykres I-V na ekranie CONTROL, obok wykresu temperatury -
        pokazuje ten sam sweep co zakladka KEITHLEY, zeby widziec oba na raz."""
        wrap = tk.Frame(parent, bg=C['panel'], height=px(220))
        wrap.pack(side='top', fill='x')
        wrap.pack_propagate(False)
        tk.Frame(wrap, bg=C['orange'], height=3).pack(fill='x')

        hd = tk.Frame(wrap, bg=C['panel'])
        hd.pack(fill='x', padx=14, pady=(8, 2))
        tk.Label(hd, text="KEITHLEY CHART", bg=C['panel'], fg=C['dim'],
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
        """True gdy w toolbarze matplotliba aktywny jest tryb ZOOM/PAN, LUB
        uzytkownik przyblizyl wykres recznie (scroll/przeciagniecie prostokata)
        - w obu przypadkach wstrzymujemy TYLKO autoscale (dopasowanie osi),
        zeby recznie ustawione przyblizenie nie znikalo. Dane nadal splywaja
        na biezaco (set_data), wiec po przywroceniu (przycisk HOME w toolbarze)
        widok od razu dogoni najnowsze probki bez utraty ciaglosci."""
        tb = getattr(self, 'mpl_toolbar', None)
        tb_mode_busy = bool(tb is not None and getattr(tb, 'mode', '') and str(tb.mode) != '')
        manual_zoom = getattr(self, '_live_chart_frozen', False)
        busy = tb_mode_busy or manual_zoom
        # wizualna informacja na przycisku pauzy
        if hasattr(self, 'btn_pause') and not self.chart_paused:
            if busy and self.btn_pause['text'] != "🔍 ZOOM (widok zablokowany)":
                self.btn_pause.config(text="🔍 ZOOM (widok zablokowany)", fg=C['cyan'],
                                      highlightbackground=C['cyan'])
            elif not busy and self.btn_pause['text'] != "⏸ PAUSE":
                self.btn_pause.config(text="⏸ PAUSE", fg=C['yellow'],
                                      highlightbackground=C['yellow'])
        return busy

    def _setup_scroll_drag_zoom(self, canvas, toolbar, frozen_attr, dragging_attr):
        """Dodaje do wykresu dwa sposoby przyblizania, ktore dzialaja ZAWSZE
        (bez klikania ikony lupy w toolbarze):
          1) scroll kolkiem myszy - przyblizenie/oddalenie wycentrowane na
             pozycji kursora, dziala na osi ktora jest akurat pod kursorem.
          2) przeciagniecie lewym przyciskiem myszy - zaznaczony prostokat
             ZAWSZE przybliza widok do tego obszaru po puszczeniu przycisku.
        Jesli w toolbarze aktywne jest natywne narzedzie PAN, ustepujemy mu
        miejsca (nie przechwytujemy przeciagniecia) zeby nie kolidowac.
        frozen_attr / dragging_attr - nazwy atrybutow self uzywanych do
        sledzenia stanu (rozne dla kazdego wykresu, zeby sie nie mieszaly)."""
        from matplotlib.patches import Rectangle
        setattr(self, frozen_attr, False)
        setattr(self, dragging_attr, False)
        drag = {'ax': None, 'x0': None, 'y0': None, 'px0': 0, 'py0': 0, 'rect': None}

        def on_scroll(event):
            if event.inaxes is None or event.xdata is None or event.ydata is None:
                return
            ax = event.inaxes
            if event.button == 'up':
                scale = 0.85   # scroll w gore = przyblizenie
            elif event.button == 'down':
                scale = 1/0.85  # scroll w dol = oddalenie
            else:
                return
            xlo, xhi = ax.get_xlim(); ylo, yhi = ax.get_ylim()
            xd, yd = event.xdata, event.ydata
            ax.set_xlim(xd - (xd - xlo) * scale, xd + (xhi - xd) * scale)
            ax.set_ylim(yd - (yd - ylo) * scale, yd + (yhi - yd) * scale)
            setattr(self, frozen_attr, True)
            canvas.draw_idle()

        def on_press(event):
            if event.inaxes is None:
                return
            if event.button == 3:
                # PRAWY przycisk = reset przyblizenia do pelnego zakresu danych,
                # na WSZYSTKICH osiach tej figury na raz (zeby np. panel
                # temperatury i panel pradu, ktore wspoldziela os X, zresetowaly
                # sie razem, niezaleznie na ktory z nich kliknieto).
                for a in canvas.figure.get_axes():
                    a.set_autoscale_on(True)
                    a.relim()
                    a.autoscale_view()
                setattr(self, frozen_attr, False)
                canvas.draw_idle()
                return
            tb_mode = str(getattr(toolbar, 'mode', '')) if toolbar is not None else ''
            if tb_mode == 'pan/zoom':
                return  # oddaj kontrole natywnemu narzedziu PAN z toolbara
            if event.button != 1:
                return
            drag['ax'] = event.inaxes
            drag['x0'] = event.xdata; drag['y0'] = event.ydata
            drag['px0'] = event.x; drag['py0'] = event.y
            setattr(self, dragging_attr, False)

        def on_motion(event):
            if drag['ax'] is None or event.inaxes is not drag['ax'] or event.xdata is None:
                return
            # prog kilku pikseli - zwykle klikniecie (bez ruchu) nie ma sie
            # zamieniac w zoom do zerowego prostokata
            if abs(event.x - drag['px0']) < 4 and abs(event.y - drag['py0']) < 4:
                return
            setattr(self, dragging_attr, True)
            ax = drag['ax']
            x0, x1 = sorted([drag['x0'], event.xdata])
            y0, y1 = sorted([drag['y0'], event.ydata])
            if drag['rect'] is None:
                drag['rect'] = Rectangle((x0, y0), x1 - x0, y1 - y0, fill=True,
                                         facecolor=C['cyan'], alpha=0.15,
                                         edgecolor=C['cyan'], linewidth=1, zorder=20)
                ax.add_patch(drag['rect'])
            else:
                drag['rect'].set_bounds(x0, y0, x1 - x0, y1 - y0)
            canvas.draw_idle()

        def on_release(event):
            if drag['ax'] is None:
                return
            ax = drag['ax']
            was_dragging = getattr(self, dragging_attr, False)
            if drag['rect'] is not None:
                try: drag['rect'].remove()
                except Exception: pass
                drag['rect'] = None
            if was_dragging and event.xdata is not None and event.ydata is not None:
                x0, x1 = sorted([drag['x0'], event.xdata])
                y0, y1 = sorted([drag['y0'], event.ydata])
                if x1 - x0 > 1e-12 and y1 - y0 > 1e-12:
                    ax.set_xlim(x0, x1)
                    ax.set_ylim(y0, y1)
                    setattr(self, frozen_attr, True)
            drag['ax'] = None
            setattr(self, dragging_attr, False)
            canvas.draw_idle()

        canvas.mpl_connect('scroll_event', on_scroll)
        canvas.mpl_connect('button_press_event', on_press)
        canvas.mpl_connect('motion_notify_event', on_motion)
        canvas.mpl_connect('button_release_event', on_release)

        # Zapamietane pod przewidywalna nazwa - przydatne do testow (wywolanie
        # bezposrednie, bez przechodzenia przez pelny dispatcher matplotlib
        # ktory wymagalby pelnego obiektu zdarzenia z atrybutem .name itp).
        if not hasattr(self, '_zoom_handlers'):
            self._zoom_handlers = {}
        self._zoom_handlers[frozen_attr] = {
            'scroll': on_scroll, 'press': on_press,
            'motion': on_motion, 'release': on_release,
        }

        # HOME w toolbarze resetuje widok do domyslnego - niech przy okazji
        # tez odmrozi autoscale, inaczej po powrocie do "calosci" wykres by
        # sie zablokowal na tym co pokazal HOME zamiast wrocic do zywego trybu.
        if toolbar is not None:
            orig_home = toolbar.home
            def wrapped_home(*a, **kw):
                setattr(self, frozen_attr, False)
                return orig_home(*a, **kw)
            toolbar.home = wrapped_home

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
        self.sl_ru = SliderField(inner, "HEAT/COOL RATE", RAMP_MIN_UI, RAMP_MAX_UI, 2.0,
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

        # ── Wybor programu uzywanego po klikni?ciu START ──
        # "Manual" = zwykle zachowanie (TARGET/RATE ze suwakow powyzej).
        # Wybranie zapisanego programu sprawia, ze START uruchamia CALA
        # sekwencje krokow (jak przycisk START PROGRAM w zakladce PROGRAM),
        # zamiast pojedynczego, stalego celu.
        prog_hd = tk.Frame(inner, bg=C['bg2'])
        prog_hd.pack(fill='x', pady=(0, 4))
        tk.Label(prog_hd, text="PROGRAM ON START", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        tk.Button(prog_hd, text="⟳", command=self._refresh_control_program_list,
                 bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(10), 'bold'),
                 relief='flat', cursor='hand2', bd=0, padx=6,
                 activebackground=C['panel3']).pack(side='right')

        style = ttk.Style()
        style.theme_use(style.theme_use())  # zachowaj biezacy motyw bazowy
        style.configure('Dark.TCombobox', fieldbackground=C['bg2'], background=C['bg2'],
                        foreground=C['text'], arrowcolor=C['dim'], bordercolor=C['border'],
                        lightcolor=C['bg2'], darkcolor=C['bg2'])
        style.map('Dark.TCombobox', fieldbackground=[('readonly', C['bg2'])],
                  foreground=[('readonly', C['text'])])

        self.CONTROL_PROGRAM_MANUAL = "Manual (TARGET/RATE)"
        self.control_program_var = tk.StringVar(value=self.CONTROL_PROGRAM_MANUAL)
        self.control_program_combo = ttk.Combobox(
            inner, textvariable=self.control_program_var, state='readonly',
            style='Dark.TCombobox', font=(FONT, fsz(9)))
        self.control_program_combo.pack(fill='x', pady=(0, 4))
        self.control_program_hint = tk.Label(
            inner, text="", bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8)),
            wraplength=260, justify='left')
        self.control_program_hint.pack(anchor='w', pady=(0, 4))

        # Zywy podglad postepu (widoczny tylko gdy program faktycznie dziala) -
        # dokladnie to samo co widac w zakladce PROGRAM, zeby nie trzeba bylo
        # przelaczac zakladek zeby sprawdzic ile zostalo do konca kroku.
        self.control_prog_status_lbl = tk.Label(
            inner, text="", bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'))
        self.control_prog_status_lbl.pack(anchor='w', pady=(0, 1))
        self.control_prog_detail_lbl = tk.Label(
            inner, text="", bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8)),
            wraplength=260, justify='left')
        self.control_prog_detail_lbl.pack(anchor='w', pady=(0, 10))
        self.control_program_var.trace_add('write', lambda *a: self._on_control_program_pick())

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # Tryb grzania/chlodzenia
        auto_lbl = tk.Frame(inner, bg=C['bg2'], highlightthickness=1,
                            highlightbackground=C['green'])
        auto_lbl.pack(fill='x', pady=(0, 10))
        tk.Label(auto_lbl, text="AUTO: direction based on setpoint", bg=C['bg2'],
                 fg=C['green'], font=(FONT, fsz(9))).pack(padx=8, pady=6)

        self.control_start_hint_lbl = tk.Label(inner, text="▶ START uses panel values",
                 bg=C['bg2'], fg=C['green'], font=(FONT, fsz(8)))
        self.control_start_hint_lbl.pack(anchor='w', pady=(4, 0))
        tk.Label(inner, text="PID + Feed-Forward tuning → ADVANCED tab",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8))).pack(anchor='w', pady=(2, 0))

        self._set_panel_enabled(False)
        self._refresh_control_program_list()

    # ════════════════════════════════════════════════════════════
    #  PROGRAMATOR: sekwencja krokow grzej->trzymaj->schlodz->trzymaj
    # ════════════════════════════════════════════════════════════
    def build_program(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=16, pady=12)

        tk.Label(wrap, text="PROGRAMMER", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(wrap, text="Step sequence: heat/cool to target, hold for a set time, repeat.",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 14))

        body = tk.Frame(wrap, bg=C['bg'])
        body.pack(fill='both', expand=True)

        # ── LEWY PANEL: formularz dodawania kroku (SCROLLOWALNY - zeby
        # przyciski SAVE/LOAD/CLEAR nigdy nie znikaly niezaleznie od
        # wysokosci okna i liczby pol w formularzu, tak samo jak w KEITHLEY) ──
        left = tk.Frame(body, bg=C['panel'], width=px(280))
        left.pack(side='left', fill='y', padx=(0, 12))
        left.pack_propagate(False)
        tk.Frame(left, bg=C['cyan'], height=3).pack(fill='x')

        scroll_wrap = tk.Frame(left, bg=C['panel'])
        scroll_wrap.pack(fill='both', expand=True)
        pcanvas = tk.Canvas(scroll_wrap, bg=C['panel'], highlightthickness=0, width=px(260))
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
        linner_pad.pack(fill='both', expand=True, padx=14, pady=14)
        linner = linner_pad  # reszta kodu buduje wewnatrz tego z paddingiem

        # Ustawienie GLOBALNE (dotyczy wszystkich krokow programu) - jak
        # blisko celu trzeba podejsc, zeby krok zaczal liczyc "osiagniety".
        # Wczesniej byla to sztywna wartosc 0.5C w kodzie - przy wolno
        # zbiegajacej rampie (np. asymptotyczne zblizanie sie do celu przy
        # nowej logice kierunku, gdzie PWM lagodnie opada do zera) osiagniecie
        # DOKLADNIE 0.5C potrafilo trwac bardzo dlugo, mimo ze temperatura
        # byla juz praktycznie na miejscu. Teraz uzytkownik moze poszerzyc
        # ten zakres, zeby odliczanie czasu przytrzymania zaczynalo sie
        # wczesniej.
        tol_row = tk.Frame(linner, bg=C['panel'])
        tol_row.pack(fill='x', pady=(0, 12))
        tk.Label(tol_row, text="TOLERANCE (± all steps)", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(anchor='w')
        tol_r2 = tk.Frame(tol_row, bg=C['panel'])
        tol_r2.pack(fill='x', pady=(2, 0))
        self.prog_tolerance_entry = tk.Entry(
            tol_r2, bg=C['bg2'], fg=C['text'], insertbackground=C['text'],
            relief='flat', font=(FONT, fsz(10)),
            highlightthickness=1, highlightbackground=C['border'])
        self.prog_tolerance_entry.insert(0, str(self.program_tolerance))
        self.prog_tolerance_entry.pack(side='left', fill='x', expand=True, ipady=4)
        tk.Label(tol_r2, text="°C", bg=C['panel'], fg=C['dim2'],
                font=(FONT, fsz(9))).pack(side='left', padx=(6, 0))
        self.prog_tolerance_entry.bind('<FocusOut>', lambda e: self._apply_program_tolerance())
        self.prog_tolerance_entry.bind('<Return>', lambda e: self._apply_program_tolerance())
        tk.Label(tol_row, text="Once T1 is within this range of a step's target, the "
                 "hold countdown for that step begins (a wider range starts it sooner, "
                 "since a slow final approach doesn't have to be exact).",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)),
                 justify='left', wraplength=240).pack(anchor='w', pady=(3, 0))

        tk.Frame(linner, bg=C['border'], height=1).pack(fill='x', pady=(0, 12))

        tk.Label(linner, text="NEW STEP", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(11), 'bold')).pack(anchor='w', pady=(0, 10))

        def _pfield(label, default, unit=""):
            row = tk.Frame(linner, bg=C['panel'])
            row.pack(fill='x', pady=4)
            tk.Label(row, text=label, bg=C['panel'], fg=C['dim'],
                     font=(FONT, fsz(9))).pack(anchor='w')
            r2 = tk.Frame(row, bg=C['panel'])
            r2.pack(fill='x', pady=(2, 0))
            e = tk.Entry(r2, bg=C['bg2'], fg=C['text'], insertbackground=C['text'],
                        relief='flat', font=(FONT, fsz(10)),
                        highlightthickness=1, highlightbackground=C['border'])
            e.insert(0, str(default))
            e.pack(side='left', fill='x', expand=True, ipady=4)
            if unit:
                tk.Label(r2, text=unit, bg=C['panel'], fg=C['dim2'],
                        font=(FONT, fsz(9))).pack(side='left', padx=(6, 0))
            return e

        self.prog_target_entry = _pfield("TARGET (temperature)", "25.0", "°C")

        ramp_row = tk.Frame(linner, bg=C['panel'])
        ramp_row.pack(fill='x', pady=(8, 0))
        self.prog_use_global_ramp = tk.BooleanVar(value=True)
        mk_checkbox(ramp_row, "use global ramp rate (CONTROL)", self.prog_use_global_ramp,
                   command=lambda: self.prog_ramp_entry.config(
                       state='disabled' if self.prog_use_global_ramp.get() else 'normal'))
        self.prog_ramp_entry = _pfield("RAMP (custom, optional)", "2.0", "°C/min")
        self.prog_ramp_entry.config(state='disabled')

        self.prog_hold_entry = _pfield("HOLD FOR", "10.0", "min")

        tol_step_row = tk.Frame(linner, bg=C['panel'])
        tol_step_row.pack(fill='x', pady=(8, 0))
        self.prog_use_global_tolerance = tk.BooleanVar(value=True)
        mk_checkbox(tol_step_row, "use global tolerance (above)", self.prog_use_global_tolerance,
                   command=lambda: self.prog_step_tolerance_entry.config(
                       state='disabled' if self.prog_use_global_tolerance.get() else 'normal'))
        self.prog_step_tolerance_entry = _pfield("TOLERANCE (custom, this step only)", "0.5", "°C")
        self.prog_step_tolerance_entry.config(state='disabled')

        self.prog_end_program_var = tk.BooleanVar(value=False)
        def _on_end_program_toggle():
            if self.prog_end_program_var.get():
                self.prog_hold_entry.config(state='disabled')
            else:
                self.prog_hold_entry.config(state='normal')
        mk_checkbox(linner, "End program here once reached (skip hold, ignore later steps)",
                   self.prog_end_program_var, command=_on_end_program_toggle)

        self.btn_add_step = mk_btn(linner, "+ ADD STEP", self.add_program_step, C['cyan'])
        self.btn_add_step.pack(fill='x', pady=(14, 0))
        self.btn_cancel_edit_step = mk_btn_outline(linner, "✕ Cancel edit", self._cancel_edit_program_step, C['dim2'])
        # nie pakujemy - pokazuje sie tylko w trybie edycji, patrz _edit_program_step

        tk.Frame(linner, bg=C['border'], height=1).pack(fill='x', pady=14)

        tk.Label(linner, text="PROGRAM FILE", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(0, 8))

        # Szybki wybor z listy juz zapisanych programow - zamiast zawsze
        # otwierac dialog wyboru pliku. "LOAD SELECTED" wczytuje to co jest
        # wybrane w liscie ponizej; "LOAD PROGRAM" (dalej) nadal otwiera
        # dialog plikowy, gdy program lezy gdzies indziej na dysku.
        pick_row = tk.Frame(linner, bg=C['panel'])
        pick_row.pack(fill='x', pady=(0, 4))
        self.prog_quick_pick_var = tk.StringVar(value="")
        self.prog_quick_pick_combo = ttk.Combobox(
            pick_row, textvariable=self.prog_quick_pick_var, state='readonly',
            style='Dark.TCombobox', font=(FONT, fsz(9)))
        self.prog_quick_pick_combo.pack(side='left', fill='x', expand=True)
        tk.Button(pick_row, text="⟳", command=self._refresh_prog_quick_pick,
                 bg=C['panel'], fg=C['dim'], font=(FONT, fsz(10), 'bold'),
                 relief='flat', cursor='hand2', bd=0, padx=6,
                 activebackground=C['panel3']).pack(side='left', padx=(4, 0))
        mk_btn_outline(linner, "▶ LOAD SELECTED", self._load_selected_quick_pick, C['cyan']).pack(fill='x', pady=(4, 6))

        mk_btn_outline(linner, "💾 SAVE PROGRAM", self.save_program, C['green']).pack(fill='x', pady=2)
        mk_btn_outline(linner, "📂 LOAD PROGRAM (browse file)", self.load_program, C['orange']).pack(fill='x', pady=2)
        mk_btn_outline(linner, "🗑 CLEAR ALL", self.clear_program, C['red']).pack(fill='x', pady=2)
        self._refresh_prog_quick_pick()

        # ── PRAWA STRONA: lista krokow + sterowanie ──
        right = tk.Frame(body, bg=C['bg'])
        right.pack(side='left', fill='both', expand=True)

        listwrap = tk.Frame(right, bg=C['panel'])
        listwrap.pack(fill='both', expand=True)
        tk.Frame(listwrap, bg=C['cyan'], height=3).pack(fill='x')
        lcanvas = tk.Canvas(listwrap, bg=C['panel'], highlightthickness=0)
        lsb = tk.Scrollbar(listwrap, orient='vertical', command=lcanvas.yview)
        lcanvas.configure(yscrollcommand=lsb.set)
        lsb.pack(side='right', fill='y')
        lcanvas.pack(side='left', fill='both', expand=True)
        self.prog_list_inner = tk.Frame(lcanvas, bg=C['panel'])
        prog_list_id = lcanvas.create_window((0, 0), window=self.prog_list_inner, anchor='nw')
        self.prog_list_inner.bind('<Configure>',
            lambda e: lcanvas.configure(scrollregion=lcanvas.bbox('all')))
        lcanvas.bind('<Configure>', lambda e: lcanvas.itemconfig(prog_list_id, width=e.width))
        lcanvas.bind('<Enter>', lambda e: lcanvas.bind_all('<MouseWheel>',
                     lambda ev: lcanvas.yview_scroll(int(-ev.delta/120), 'units')))
        lcanvas.bind('<Leave>', lambda e: lcanvas.unbind_all('<MouseWheel>'))

        # ── Pasek statusu + sterowanie wykonaniem ──
        ctrl = tk.Frame(right, bg=C['panel'])
        ctrl.pack(fill='x', pady=(10, 0))
        cinner = tk.Frame(ctrl, bg=C['panel'])
        cinner.pack(fill='x', padx=14, pady=12)

        self.prog_status_lbl = tk.Label(cinner, text="No active program.",
                                        bg=C['panel'], fg=C['dim'], font=(FONT, fsz(10), 'bold'))
        self.prog_status_lbl.pack(anchor='w')
        self.prog_detail_lbl = tk.Label(cinner, text="", bg=C['panel'], fg=C['dim2'],
                                        font=(FONT, fsz(9)))
        self.prog_detail_lbl.pack(anchor='w', pady=(2, 10))

        btnrow = tk.Frame(cinner, bg=C['panel'])
        btnrow.pack(fill='x')
        self.btn_prog_start = tk.Button(btnrow, text="▶ START PROGRAM", command=self.start_program,
                                        bg=C['green'], fg='#1a1c1f', font=(FONT, fsz(10), 'bold'),
                                        relief='flat', cursor='hand2', bd=0, padx=14, pady=8,
                                        activebackground=_lighten(C['green'], 0.15))
        self.btn_prog_start.pack(side='left', padx=(0, 8))
        self.btn_prog_stop = tk.Button(btnrow, text="■ STOP PROGRAM", command=self.stop_program,
                                       bg=C['bg2'], fg=C['red'], font=(FONT, fsz(10), 'bold'),
                                       relief='flat', cursor='hand2', bd=0, padx=14, pady=8,
                                       state='disabled', activebackground=C['panel3'])
        self.btn_prog_stop.pack(side='left')

        self._refresh_program_list()

    def _fmt_mmss(self, seconds):
        seconds = max(0, int(seconds))
        return f"{seconds//60:02d}:{seconds%60:02d}"

    def _refresh_program_list(self):
        for w in self.prog_list_inner.winfo_children():
            w.destroy()
        if not self.program_steps:
            tk.Label(self.prog_list_inner, text="No steps yet - add the first step on the left.",
                     bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(9))).pack(anchor='w', padx=12, pady=12)
            return
        for i, step in enumerate(self.program_steps):
            active = self.program_running and i == self.program_step_idx
            editing = (self.prog_editing_idx == i)
            row_bg = C['bg2'] if active else C['panel']
            row = tk.Frame(self.prog_list_inner, bg=row_bg,
                          highlightthickness=1 if (active or editing) else 0,
                          highlightbackground=C['orange'] if editing else C['cyan'])
            row.pack(fill='x', padx=8, pady=3)

            marker = "▶ " if active else ("✎ " if editing else "")
            ramp_txt = f"{step['ramp']:.1f}°C/min" if step['ramp'] is not None else "global ramp"
            step_tol = step.get('tolerance')
            tol_txt = f"±{step_tol:g}°C" if step_tol is not None else "global tol."
            if step.get('end_program'):
                tail = "⏹ END PROGRAM HERE (no hold)"
            else:
                tail = f"hold {step['hold_min']:.1f} min"
            txt = (f"{marker}STEP {i+1}: → {step['target']:.1f}°C  "
                   f"({ramp_txt}, {tol_txt})  •  {tail}")
            tk.Label(row, text=txt, bg=row_bg,
                    fg=C['orange'] if editing else (C['red'] if step.get('end_program') else (C['cyan'] if active else C['text'])),
                    font=(FONT, fsz(9), 'bold' if (active or editing) else 'normal')
                    ).pack(side='left', padx=10, pady=8, fill='x', expand=True)

            btns = tk.Frame(row, bg=row_bg)
            btns.pack(side='right', padx=6)
            for txt_b, cmd, color in [("✎", lambda i=i: self._edit_program_step(i), C['orange']),
                                        ("↑", lambda i=i: self._move_program_step(i, -1), C['dim']),
                                        ("↓", lambda i=i: self._move_program_step(i, 1), C['dim']),
                                        ("✕", lambda i=i: self._delete_program_step(i), C['red'])]:
                tk.Button(btns, text=txt_b, command=cmd, bg=row_bg,
                         fg=color,
                         font=(FONT, fsz(9), 'bold'), relief='flat', cursor='hand2',
                         bd=0, padx=6, activebackground=C['panel3']).pack(side='left')

    def add_program_step(self):
        if self.program_running:
            messagebox.showwarning("Program running", "Stop the program before editing the step list.")
            return
        try:
            target = float(self.prog_target_entry.get().replace(',', '.'))
        except ValueError:
            messagebox.showerror("Error", "Check the numeric value (TARGET).")
            return
        end_program = self.prog_end_program_var.get()
        hold_min = 0.0
        if not end_program:
            try:
                hold_min = float(self.prog_hold_entry.get().replace(',', '.'))
            except ValueError:
                messagebox.showerror("Error", "Check the numeric value (HOLD FOR).")
                return
            if hold_min < 0:
                messagebox.showerror("Error", "Hold time cannot be negative.")
                return
        if not (SP_MIN_UI <= target <= SP_MAX_UI):
            messagebox.showerror("Error", f"TARGET is out of controller range ({SP_MIN_UI:g}..{SP_MAX_UI:g}°C).")
            return
        ramp = None
        if not self.prog_use_global_ramp.get():
            try:
                ramp = float(self.prog_ramp_entry.get().replace(',', '.'))
            except ValueError:
                messagebox.showerror("Error", "Check the RAMP value.")
                return
            if not (RAMP_MIN_UI <= ramp <= RAMP_MAX_UI):
                messagebox.showerror("Error", f"RAMP is out of controller range ({RAMP_MIN_UI:g}..{RAMP_MAX_UI:g}°C/min).")
                return
        # Tolerancja PER-KROK (opcjonalna) - jesli "use global tolerance"
        # odznaczone, ten konkretny krok uzywa wlasnej wartosci zamiast
        # globalnego ustawienia z gory panelu. None = uzyj globalnej.
        tolerance = None
        if not self.prog_use_global_tolerance.get():
            try:
                tolerance = float(self.prog_step_tolerance_entry.get().replace(',', '.'))
            except ValueError:
                messagebox.showerror("Error", "Check the TOLERANCE value.")
                return
            if not (0.05 <= tolerance <= 10.0):
                messagebox.showerror("Error", "TOLERANCE should be between 0.05 and 10°C.")
                return
        new_step = {'target': target, 'ramp': ramp, 'hold_min': hold_min,
                    'end_program': end_program, 'tolerance': tolerance}
        if self.prog_editing_idx is not None:
            # Tryb EDYCJI - podmieniamy istniejacy krok w miejscu, nie dodajemy nowego
            idx = self.prog_editing_idx
            if 0 <= idx < len(self.program_steps):
                self.program_steps[idx] = new_step
            self._cancel_edit_program_step()  # wraca do trybu "dodaj nowy" + odswieza liste
        else:
            self.program_steps.append(new_step)
            self._refresh_program_list()

    def _edit_program_step(self, idx):
        """Wczytuje wartosci istniejacego kroku z powrotem do formularza po
        lewej i przelacza go w tryb edycji - klikniecie 'ADD STEP' (teraz
        pokazujace sie jako 'SAVE CHANGES') zaktualizuje ten krok w miejscu
        zamiast dodawac nowy na koncu listy."""
        if self.program_running:
            messagebox.showwarning("Program running", "Stop the program before editing the step list.")
            return
        if not (0 <= idx < len(self.program_steps)):
            return
        step = self.program_steps[idx]
        self.prog_editing_idx = idx

        self.prog_target_entry.delete(0, 'end'); self.prog_target_entry.insert(0, f"{step['target']:g}")

        if step['ramp'] is None:
            self.prog_use_global_ramp.set(True)
            self.prog_ramp_entry.config(state='normal')
            self.prog_ramp_entry.delete(0, 'end'); self.prog_ramp_entry.insert(0, "2.0")
            self.prog_ramp_entry.config(state='disabled')
        else:
            self.prog_use_global_ramp.set(False)
            self.prog_ramp_entry.config(state='normal')
            self.prog_ramp_entry.delete(0, 'end'); self.prog_ramp_entry.insert(0, f"{step['ramp']:g}")

        self.prog_hold_entry.config(state='normal')
        self.prog_hold_entry.delete(0, 'end'); self.prog_hold_entry.insert(0, f"{step['hold_min']:g}")

        step_tol = step.get('tolerance')
        if step_tol is None:
            self.prog_use_global_tolerance.set(True)
            self.prog_step_tolerance_entry.config(state='normal')
            self.prog_step_tolerance_entry.delete(0, 'end'); self.prog_step_tolerance_entry.insert(0, "0.5")
            self.prog_step_tolerance_entry.config(state='disabled')
        else:
            self.prog_use_global_tolerance.set(False)
            self.prog_step_tolerance_entry.config(state='normal')
            self.prog_step_tolerance_entry.delete(0, 'end'); self.prog_step_tolerance_entry.insert(0, f"{step_tol:g}")

        self.prog_end_program_var.set(step.get('end_program', False))
        if step.get('end_program', False):
            self.prog_hold_entry.config(state='disabled')

        self.btn_add_step.config(text="✓ SAVE CHANGES (step {})".format(idx+1), bg=C['orange'])
        self.btn_cancel_edit_step.pack(fill='x', pady=(6, 0))
        self._refresh_program_list()  # odswiez, zeby podswietlic edytowany wiersz

    def _cancel_edit_program_step(self):
        """Wychodzi z trybu edycji bez zapisywania zmian, wraca do trybu
        'dodaj nowy krok' i przywraca formularz do wartosci domyslnych."""
        self.prog_editing_idx = None
        self.btn_add_step.config(text="+ ADD STEP", bg=C['cyan'])
        self.btn_cancel_edit_step.pack_forget()
        self.prog_target_entry.delete(0, 'end'); self.prog_target_entry.insert(0, "25.0")
        self.prog_use_global_ramp.set(True)
        self.prog_ramp_entry.config(state='normal')
        self.prog_ramp_entry.delete(0, 'end'); self.prog_ramp_entry.insert(0, "2.0")
        self.prog_ramp_entry.config(state='disabled')
        self.prog_hold_entry.config(state='normal')
        self.prog_hold_entry.delete(0, 'end'); self.prog_hold_entry.insert(0, "10.0")
        self.prog_use_global_tolerance.set(True)
        self.prog_step_tolerance_entry.config(state='normal')
        self.prog_step_tolerance_entry.delete(0, 'end'); self.prog_step_tolerance_entry.insert(0, "0.5")
        self.prog_step_tolerance_entry.config(state='disabled')
        self.prog_end_program_var.set(False)
        self._refresh_program_list()

    def _delete_program_step(self, idx):
        if self.program_running:
            messagebox.showwarning("Program running", "Stop the program before editing the step list.")
            return
        if 0 <= idx < len(self.program_steps):
            del self.program_steps[idx]
            if self.prog_editing_idx == idx:
                self._cancel_edit_program_step()  # edytowany krok wlasnie zniknal - wyjdz z edycji
                return  # _cancel_edit_program_step juz odswieza liste
            elif self.prog_editing_idx is not None and self.prog_editing_idx > idx:
                self.prog_editing_idx -= 1  # przesuniecie po usunieciu wczesniejszego kroku
            self._refresh_program_list()

    def _move_program_step(self, idx, direction):
        if self.program_running:
            messagebox.showwarning("Program running", "Stop the program before editing the step list.")
            return
        j = idx + direction
        if 0 <= idx < len(self.program_steps) and 0 <= j < len(self.program_steps):
            self.program_steps[idx], self.program_steps[j] = self.program_steps[j], self.program_steps[idx]
            # jesli przesuwamy akurat edytowany krok, podazaj za nim indeksem
            if self.prog_editing_idx == idx: self.prog_editing_idx = j
            elif self.prog_editing_idx == j: self.prog_editing_idx = idx
            self._refresh_program_list()

    def clear_program(self):
        if self.program_running:
            messagebox.showwarning("Program running", "Stop the program before clearing the list.")
            return
        if self.program_steps and not messagebox.askyesno("Confirm", "Delete all program steps?"):
            return
        self.program_steps = []
        self._refresh_program_list()

    def save_program(self):
        if not self.program_steps:
            messagebox.showinfo("No steps", "Add at least one step before saving.")
            return
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Save program", defaultextension=".json",
            initialdir=str(self.programs_dir), filetypes=[("Program JSON", "*.json")])
        if not dest:
            return
        try:
            with open(dest, 'w', encoding='utf-8') as f:
                json.dump({'steps': self.program_steps}, f, indent=2, ensure_ascii=False)
            self._refresh_control_program_list()
            self._refresh_prog_quick_pick()
            messagebox.showinfo("Saved", f"Program saved to:\n{dest}")
        except Exception as e:
            messagebox.showerror("Save error", str(e))

    def _parse_program_json(self, path):
        """Wczytuje i waliduje plik programu (.json) - zwraca liste czystych
        krokow albo rzuca wyjatek z czytelnym opisem bledu. Wspoldzielone przez
        load_program() (dialog wyboru pliku) i selektor programu w CONTROL
        (wybor z listy rozwijanej, bez dialogu)."""
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        steps = data.get('steps', [])
        clean = []
        for s in steps:
            clean.append({'target': float(s['target']),
                          'ramp': (float(s['ramp']) if s.get('ramp') is not None else None),
                          'hold_min': float(s['hold_min']),
                          'end_program': bool(s.get('end_program', False)),
                          'tolerance': (float(s['tolerance']) if s.get('tolerance') is not None else None)})
        return clean

    def _refresh_prog_quick_pick(self):
        """Wypelnia liste szybkiego wyboru nazwami zapisanych programow (bez
        koniecznosci otwierania dialogu wyboru pliku za kazdym razem) -
        odswiezane po starcie aplikacji i po kazdym zapisie nowego programu."""
        if not hasattr(self, 'prog_quick_pick_combo'):
            return
        names = sorted(p.name for p in self.programs_dir.glob("*.json"))
        self.prog_quick_pick_combo.config(values=names)
        if self.prog_quick_pick_var.get() not in names:
            self.prog_quick_pick_var.set(names[0] if names else "")

    def _load_selected_quick_pick(self):
        """Wczytuje program wybrany w liscie rozwijanej - szybsza alternatywa
        dla LOAD PROGRAM (ktory zawsze otwiera dialog wyboru pliku)."""
        if self.program_running:
            messagebox.showwarning("Program running", "Stop the program before loading another one.")
            return
        name = self.prog_quick_pick_var.get()
        if not name:
            messagebox.showinfo("No programs", "No saved programs found - save one first, "
                                "or use LOAD PROGRAM to browse for a file elsewhere.")
            return
        try:
            clean = self._parse_program_json(self.programs_dir / name)
            self.program_steps = clean
            self._refresh_program_list()
            if hasattr(self, '_refresh_control_program_list'):
                self._refresh_control_program_list()
        except Exception as e:
            messagebox.showerror("Load error", f"Could not load '{name}':\n{e}")

    def load_program(self):
        if self.program_running:
            messagebox.showwarning("Program running", "Stop the program before loading another one.")
            return
        from tkinter import filedialog
        src = filedialog.askopenfilename(
            title="Load program", initialdir=str(self.programs_dir),
            filetypes=[("Program JSON", "*.json")])
        if not src:
            return
        try:
            clean = self._parse_program_json(src)
            self.program_steps = clean
            self._refresh_program_list()
            messagebox.showinfo("Loaded", f"Loaded {len(clean)} steps from:\n{src}")
        except Exception as e:
            messagebox.showerror("Load error", f"Could not load the program:\n{e}")

    def start_program(self):
        if not self.program_steps:
            messagebox.showinfo("No steps", "Add at least one step before starting.")
            return
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        self.program_running = True
        self.program_state = 'ramping'
        self.program_step_idx = 0
        self.program_step_sent = False
        self.program_stable_since = None
        self.program_hold_start = None
        self.btn_prog_start.config(state='disabled')
        self.btn_prog_stop.config(state='normal')
        if not self.is_running:
            self.do_start()
        self._refresh_program_list()

    def stop_program(self):
        self.program_running = False
        self.program_state = 'idle'
        self.btn_prog_start.config(state='normal')
        self.btn_prog_stop.config(state='disabled')
        self._set_program_status("Program stopped.", C['dim'], "")
        self._refresh_program_list()

    def _fmt_duration_min(self, minutes):
        """Formatuje czas trwania w minutach na czytelny tekst (Xh Ym albo Ym)."""
        if minutes < 0: minutes = 0
        total_s = int(round(minutes * 60))
        h, rem = divmod(total_s, 3600)
        mn, _s = divmod(rem, 60)
        if h > 0:
            return f"~{h}h {mn}m"
        return f"~{mn} min" if mn > 0 else "<1 min"

    def _estimate_program_duration(self, steps, start_temp=None):
        """Szacuje CALKOWITY czas trwania programu (rampy + przytrzymania), w
        minutach. To tylko PRZYBLIZENIE dla rampy: liczy dystans/tempo, bez
        uwzgledniania rzeczywistej bezwladnosci cieplnej ukladu (opoznienia,
        overshoot) - realny czas dojazdu moze byc dluzszy. Przytrzymanie jest
        dokladne (to zwykla stala z kroku)."""
        if not steps:
            return 0.0
        total = 0.0
        cur = start_temp if start_temp is not None else steps[0]['target']
        for s in steps:
            rate = s['ramp'] if s['ramp'] is not None else self.sl_ru.get()
            rate = max(rate, 0.1)  # unikaj dzielenia przez 0 przy nietypowej rampie
            dist = abs(s['target'] - cur)
            total += dist / rate + s['hold_min']
            cur = s['target']
            if s.get('end_program'):
                break  # kolejne kroki nigdy sie nie wykonaja - program konczy sie tutaj
        return total

    def _set_program_status(self, text, color, detail=""):
        """Aktualizuje status programu w OBU miejscach na raz: w zakladce
        PROGRAM i na CONTROL (jesli tam uruchomiono) - zeby uzytkownik widzial
        postep niezaleznie na ktorej zakladce akurat jest."""
        self.prog_status_lbl.config(text=text, fg=color)
        self.prog_detail_lbl.config(text=detail)
        if hasattr(self, 'control_prog_status_lbl'):
            self.control_prog_status_lbl.config(text=text, fg=color)
            self.control_prog_detail_lbl.config(text=detail)

    def _stop_peltier_and_keithley(self):
        """Wspolny helper wywolywany przy KAZDYM zakonczeniu programu (normalnym
        po ostatnim kroku, przez end_program, i przez skrot w galezi holding) -
        zatrzymuje PID w firmware (STOP) I, jesli akurat trwa pomiar Keithleya
        (np. w trybie ciaglym rownolegle do rampy), zatrzymuje go tez. Bez tego
        Peltier poprawnie sie wylaczal, ale Keithley zostawal dzialajacy w
        nieskonczonosc i dalej dopisywal probki do pliku sweep, mimo ze caly
        eksperyment juz sie skonczyl - bezuzyteczne dane po fakcie."""
        self.send("STOP")
        self._update_run_button(False)
        if self.sweep_running:
            self.keithley_sweep_stop()

    def _program_tick(self, current_temp):
        """Maszyna stanow programatora - wywolywana co kazdy tick() (250ms) gdy
        aktywny jest program. current_temp - biezaca temperatura T1 (moze byc
        None jesli jeszcze brak danych)."""
        if not self.program_running or current_temp is None:
            return
        if self.program_step_idx >= len(self.program_steps):
            self.program_running = False
            self.program_state = 'done'
            # NAPRAWIONE: wczesniej konczylismy TYLKO ksiegowosc po stronie
            # PC (program_running/program_state), nigdy nie mowiac firmware
            # zeby faktycznie przestal regulowac. Efekt: PID w firmware
            # nadal trzymal ostatnia temperature w nieskonczonosc, a log
            # cyklu (CSV) nigdy sie nie zamykal/zapisywal, bo to zamkniecie
            # jest wyzwalane przez przejscie stanu firmware AUTO->MAN (patrz
            # tick(): "elif not pid_on and self.cyc_on: self.cyc_stop(...)"),
            # a firmware nigdy nie dostawal komendy STOP. Teraz wysylamy ja
            # jawnie (i zatrzymujemy tez Keithleya jesli mierzyl) przez
            # wspolny helper - tak samo jak robi to zwykly przycisk STOP.
            self._stop_peltier_and_keithley()
            self.btn_prog_start.config(state='normal')
            self.btn_prog_stop.config(state='disabled')
            self._set_program_status("✓ Program complete.", C['green'], f"Completed {len(self.program_steps)} steps.")
            self._refresh_program_list()
            return

        step = self.program_steps[self.program_step_idx]

        if self.program_state == 'ramping':
            if not self.program_step_sent:
                ramp = step['ramp'] if step['ramp'] is not None else self.sl_ru.get()
                self.send(f"SP:{step['target']:.2f}")
                self.send(f"RU:{ramp:.2f}")
                self.send(f"RD:{ramp:.2f}")
                self.program_step_sent = True
                self.program_stable_since = None

            step_tolerance = step.get('tolerance')
            active_tolerance = step_tolerance if step_tolerance is not None else self.program_tolerance
            within_tol = abs(current_temp - step['target']) <= active_tolerance
            if within_tol:
                if self.program_stable_since is None:
                    self.program_stable_since = time.time()
                elif time.time() - self.program_stable_since >= self.program_stable_needed_s:
                    if step.get('end_program'):
                        # Ten krok konczy CALY program od razu po ustabilizowaniu
                        # sie na celu - bez przytrzymania, bez dalszych krokow
                        # (nawet jesli byly zdefiniowane po nim na liscie).
                        # NAPRAWIONE (ten sam bug co przy normalnym zakonczeniu
                        # powyzej): trzeba jawnie wyslac STOP do firmware i
                        # zaktualizowac przycisk RUN, inaczej PID nigdy sie
                        # nie zatrzyma i log cyklu nigdy sie nie zapisze. Przez
                        # wspolny helper, ktory dodatkowo zatrzymuje TEZ
                        # Keithleya jesli akurat mierzyl - bez tego Peltier
                        # sie wylaczal poprawnie, ale pomiar pradu zostawal
                        # wlaczony w nieskonczonosc, bezuzytecznie po fakcie.
                        self.program_running = False
                        self.program_state = 'done'
                        self._stop_peltier_and_keithley()
                        self.btn_prog_start.config(state='normal')
                        self.btn_prog_stop.config(state='disabled')
                        self._set_program_status(
                            "✓ Program complete (ended at target).", C['green'],
                            f"Reached {step['target']:.1f}°C - stopped as configured "
                            f"(step {self.program_step_idx+1}/{len(self.program_steps)}).")
                        self._refresh_program_list()
                        return
                    self.program_state = 'holding'
                    self.program_hold_start = time.time()
            else:
                self.program_stable_since = None

            ramp_rate = step['ramp'] if step['ramp'] is not None else self.sl_ru.get()
            ramp_rate = max(ramp_rate, 0.1)
            eta_s = abs(current_temp - step['target']) / ramp_rate * 60.0
            then_txt = "then END PROGRAM" if step.get('end_program') else \
                       f"then hold {self._fmt_mmss(step['hold_min']*60)}"
            self._set_program_status(
                text=f"STEP {self.program_step_idx+1}/{len(self.program_steps)}: "
                     f"approaching {step['target']:.1f}°C",
                color=C['cyan'],
                detail=f"Now: {current_temp:.2f}°C (±{active_tolerance:g}°C tol.) · "
                       f"ETA to target: ~{self._fmt_mmss(eta_s)} · {then_txt}")

        elif self.program_state == 'holding':
            elapsed = time.time() - self.program_hold_start
            remaining = step['hold_min'] * 60.0 - elapsed
            next_hint = ""
            if self.program_step_idx + 1 < len(self.program_steps):
                nxt = self.program_steps[self.program_step_idx + 1]
                next_hint = f" · next: → {nxt['target']:.1f}°C"
            else:
                next_hint = " · last step"
            self._set_program_status(
                text=f"STEP {self.program_step_idx+1}/{len(self.program_steps)}: "
                     f"holding at {step['target']:.1f}°C",
                color=C['green'],
                detail=f"Remaining: {self._fmt_mmss(remaining)} / "
                       f"{self._fmt_mmss(step['hold_min']*60)}{next_hint}")
            if remaining <= 0:
                self.program_step_idx += 1
                if self.program_step_idx >= len(self.program_steps):
                    # to byl ostatni krok - zakoncz program od razu, bez
                    # czekania na kolejny tick (ktory i tak by to wykryl na
                    # starcie funkcji, ale to niepotrzebne opoznienie o 250ms
                    # i dodatkowy, mylacy stan posredni)
                    # NAPRAWIONE: trzecie miejsce z tym samym bugiem co przy
                    # normalnym wejsciu do funkcji i przy end_program - brak
                    # wyslania STOP do firmware oznaczal ze PID nigdy sie
                    # nie zatrzymywal, a log cyklu nigdy sie nie zapisywal.
                    # Przez wspolny helper (zatrzymuje tez Keithleya).
                    self.program_running = False
                    self.program_state = 'done'
                    self._stop_peltier_and_keithley()
                    self.btn_prog_start.config(state='normal')
                    self.btn_prog_stop.config(state='disabled')
                    self._set_program_status("✓ Program complete.", C['green'], f"Completed {len(self.program_steps)} steps.")
                else:
                    self.program_state = 'ramping'
                    self.program_step_sent = False
                    self.program_stable_since = None
                self._refresh_program_list()

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
        self._make_collapsible_desc(sec2, "what is this?",
                 "HOLD = base power to maintain temperature. RAMP = extra power "
                 "for ramp dynamics. Symmetric: heating and cooling each have "
                 "their own HOLD/RAMP gains, so both track the set ramp rate "
                 "with predictive power instead of cooling relying only on "
                 "reactive PID correction.")
        tk.Label(sec2, text="HEATING", bg=C['bg2'], fg=C['orange'],
                 font=(FONT, fsz(8), 'bold')).pack(anchor='w', pady=(4, 2))
        self.sl_kffh = SliderField(sec2, "FF HOLD (KFFH)", 0, 8, 2.5, C['yellow'], "PWM/10°C", 2,
                                   on_change=lambda v: self.send(f"KFFH:{v:.2f}"))
        self.sl_kffr = SliderField(sec2, "FF RAMP (KFFR)", 0, 4, 1.0, C['yellow'], "PWM/(°C/min)", 2,
                                   on_change=lambda v: self.send(f"KFFR:{v:.2f}"))
        tk.Label(sec2, text="COOLING", bg=C['bg2'], fg=C['cyan'],
                 font=(FONT, fsz(8), 'bold')).pack(anchor='w', pady=(10, 2))
        self.sl_kffhc = SliderField(sec2, "FF HOLD COOL (KFFHC)", 0, 8, 2.5, C['cyan'], "PWM/10°C", 2,
                                    on_change=lambda v: self.send(f"KFFHC:{v:.2f}"))
        self.sl_kffrc = SliderField(sec2, "FF RAMP COOL (KFFRC)", 0, 4, 1.0, C['cyan'], "PWM/(°C/min)", 2,
                                    on_change=lambda v: self.send(f"KFFRC:{v:.2f}"))

        sec3 = self._adv_section(inner, "THERMOCOUPLE", C['purple'])
        self.sl_off = SliderField(sec3, "CAL OFFSET", -20, 20, 0.0,
                                  C['purple'], "°C", 1,
                                  on_change=lambda v: self.send(f"OFFSET:{v:.1f}"))

        sec_dir = self._adv_section(inner, "HEATING / COOLING DIRECTION", C['green'])
        self._make_collapsible_desc(sec_dir, "what does OFF/ON mean?",
                 "Direction is decided by the PID itself, not by comparing the "
                 "target to the current temperature: while heating, the system "
                 "keeps heating (with PWM naturally decaying toward 0 as the "
                 "ramp target drops) and only switches to cooling once the PID "
                 "would genuinely need negative power to keep up - not the "
                 "instant the target crosses below the current temperature. "
                 "This lets the system use passive cooling (heat loss to "
                 "ambient) before engaging active cooling, and vice versa. "
                 "OFF (default): once switched, PWM is locked to that single "
                 "direction (never goes negative while heating, or positive "
                 "while cooling) - fewer direction switches, less noise on a "
                 "sensitive measurement.\n\n"
                 "ON: the PID can use the opposite direction as an active brake "
                 "on overshoot (e.g. a brief heat pulse right after cooling "
                 "ends, if the temperature dropped too far) - faster, more "
                 "accurate correction near target, at the cost of possible "
                 "extra direction switches (and related noise).")
        self.oppdir_var = tk.BooleanVar(value=False)
        dir_row = tk.Frame(sec_dir, bg=C['bg2'])
        dir_row.pack(fill='x', pady=(4, 0))
        self.btn_oppdir_off = tk.Button(dir_row, text="OFF (lock)",
                                        command=lambda: self._set_oppdir(False),
                                        bg=C['green'], fg='#1a1c1f', font=(FONT, fsz(9), 'bold'),
                                        relief='flat', cursor='hand2', bd=0, padx=4, pady=8)
        self.btn_oppdir_off.pack(side='left', fill='x', expand=True, padx=(0, 4))
        self.btn_oppdir_on = tk.Button(dir_row, text="ON (brake)",
                                       command=lambda: self._set_oppdir(True),
                                       bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                       relief='flat', cursor='hand2', bd=0, padx=4, pady=8)
        self.btn_oppdir_on.pack(side='left', fill='x', expand=True)

        sec_at = self._adv_section(inner, "AUTO-TUNE (LIVE)", C['orange'])
        self._make_collapsible_desc(sec_at, "what does this do?",
                 "Runs for ~2 minutes (60 cycles, one every 2s) while the PID is "
                 "already running (AUTO). Watches for oscillation and slowly "
                 "nudges Kp/Ki/Kd toward more stable values - and, new: if it "
                 "sees oscillation WHILE HOLDING (not during an active ramp), it "
                 "also reduces the FEED-FORWARD HOLD gain (KFFH/KFFHC), since "
                 "that kind of steady, regular oscillation right at the target "
                 "often comes from a feed-forward value that's too strong for "
                 "the system's real thermal inertia, not from the PID gains "
                 "themselves - lowering Kp/Kd alone doesn't always fully fix it. "
                 "Adjustments happen live; nothing is saved to the device "
                 "permanently until you press SAVE elsewhere.")
        self.selftune_status_lbl = tk.Label(sec_at, text="Not running.",
                                            bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'))
        self.selftune_status_lbl.pack(anchor='w', pady=(4, 8))
        at_row = tk.Frame(sec_at, bg=C['bg2'])
        at_row.pack(fill='x')
        self.btn_selftune_start = tk.Button(at_row, text="▶ START AUTO-TUNE",
                                            command=self.start_selftune,
                                            bg=C['orange'], fg='#1a1c1f', font=(FONT, fsz(9), 'bold'),
                                            relief='flat', cursor='hand2', bd=0, padx=4, pady=8)
        self.btn_selftune_start.pack(side='left', fill='x', expand=True, padx=(0, 4))
        self.btn_selftune_stop = tk.Button(at_row, text="■ STOP AUTO-TUNE",
                                           command=self.stop_selftune,
                                           bg=C['bg2'], fg=C['red'], font=(FONT, fsz(9), 'bold'),
                                           relief='flat', cursor='hand2', bd=0, padx=4, pady=8,
                                           state='disabled')
        self.btn_selftune_stop.pack(side='left', fill='x', expand=True)

        sec4 = self._adv_section(inner, "RESET", C['red'])
        mk_btn_outline(sec4, "↺ RESET PID GAINS", self.do_reset, C['red']).pack(fill='x')

    def start_selftune(self):
        if not self.is_running:
            messagebox.showwarning("PID not running",
                "Auto-tune adjusts gains while the PID is actively holding/ramping - "
                "start the PID (CONTROL tab) first, then start auto-tune.")
            return
        self.selftune_running = True
        self.selftune_start_ts = time.time()
        self.btn_selftune_start.config(state='disabled')
        self.btn_selftune_stop.config(state='normal')
        self.selftune_status_lbl.config(text="Starting…", fg=C['orange'])
        self.send("SELFTUNE")

    def stop_selftune(self):
        self.selftune_running = False
        self.btn_selftune_start.config(state='normal')
        self.btn_selftune_stop.config(state='disabled')
        self.selftune_status_lbl.config(text="Stopped.", fg=C['dim'])
        self.send("SELFTUNESTOP")

    def _on_selftune_status(self, state_str, kp, ki, kd):
        """Wywolywane z tick() przy kazdym wierszu CSV ktorego 'state' zaczyna
        sie od 'ST' - to firmware co 2s raportuje postep auto-tune (patrz
        runST() w PeltierPID.ino). Rozne sufiksy: OK / OSC- / OSC-FF (feed-
        forward tez skorygowany) / SAT (nasycenie PWM) / SLOW++ / WORSE / Ki+."""
        if not getattr(self, 'selftune_running', False) or not hasattr(self, 'selftune_status_lbl'):
            return
        suffix = state_str[3:] if len(state_str) > 3 else ""
        elapsed = time.time() - getattr(self, 'selftune_start_ts', time.time())
        label_map = {
            'OK': ('tracking well', C['green']),
            'OSC-': ('oscillation detected - reducing Kp/Ki/Kd', C['yellow']),
            'OSC-FF': ('oscillation while holding - reducing Kp/Ki/Kd AND feed-forward HOLD', C['orange']),
            'SAT': ('PWM saturated - waiting', C['dim']),
            'SLOW++': ('response too slow - increasing Kp', C['cyan']),
            'WORSE': ('getting worse - increasing Kp', C['yellow']),
            'Ki+': ('steady-state error - increasing Ki', C['cyan']),
        }
        desc, col = label_map.get(suffix, (suffix, C['dim']))
        self.selftune_status_lbl.config(
            text=f"{elapsed:.0f}s: {desc}  (Kp={kp:.2f} Ki={ki:.3f} Kd={kd:.2f})",
            fg=col)
        # sync suwaki na biezaco, zeby bylo widac zmiany bez czekania na koniec
        if hasattr(self, 'sl_kp'): self.sl_kp.set(kp)
        if hasattr(self, 'sl_ki'): self.sl_ki.set(ki)
        if hasattr(self, 'sl_kd'): self.sl_kd.set(kd)

    def _set_oppdir(self, enabled, send=True):
        """Przelacza tryb hamowania przeciwnym kierunkiem (firmware: OPPDIR).
        Wysylane od razu (dziala tez w trakcie pracy PID, nie tylko przy STARCIE).
        send=False uzywane przy synchronizacji stanu odczytanego z firmware
        (CFG), zeby nie odsylac komendy z powrotem do urzadzenia bez potrzeby."""
        self.oppdir_var.set(enabled)
        if send:
            self.send(f"OPPDIR:{1 if enabled else 0}")
        if enabled:
            self.btn_oppdir_on.config(bg=C['green'], fg='#1a1c1f')
            self.btn_oppdir_off.config(bg=C['bg2'], fg=C['dim'])
        else:
            self.btn_oppdir_off.config(bg=C['green'], fg='#1a1c1f')
            self.btn_oppdir_on.config(bg=C['bg2'], fg=C['dim'])

    def _adv_section(self, parent, title, color):
        tk.Frame(parent, bg=color, height=2).pack(fill='x', pady=(12, 0))
        tk.Label(parent, text=title, bg=C['bg'], fg=color,
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(4, 6))
        box = tk.Frame(parent, bg=C['bg2'])
        box.pack(fill='x')
        inner = tk.Frame(box, bg=C['bg2'])
        inner.pack(fill='x', padx=12, pady=10)
        return inner

    def _make_collapsible_desc(self, parent, label, text, default_expanded=False):
        """Zwijany blok opisowy (przycisk ▶/▼ + ukryty/widoczny tekst) - ten
        sam wzorzec co 'field descriptions'/'units & hardware limits' w
        zakladce KEITHLEY, teraz wielokrotnego uzytku. Dlugie akapity
        wyjasnien zaciemnialy widok samych kontrolek (suwakow/przyciskow)
        ponizej - domyslnie zwiniete, rozwijane na zadanie."""
        state = {'expanded': default_expanded}
        btn = tk.Button(parent, text=("▼ " if default_expanded else "▶ ") + label,
                        bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                        relief='flat', cursor='hand2', bd=0, anchor='w')
        btn.pack(fill='x', pady=(0, 2))
        body = tk.Label(parent, text=text, bg=C['bg2'], fg=C['dim2'],
                        font=(FONT, fsz(8)), justify='left', wraplength=480)
        def _toggle():
            state['expanded'] = not state['expanded']
            if state['expanded']:
                body.pack(anchor='w', pady=(0, 10), after=btn)
                btn.config(text="▼ " + label)
            else:
                body.pack_forget()
                btn.config(text="▶ " + label)
        btn.config(command=_toggle)
        if default_expanded:
            body.pack(anchor='w', pady=(0, 10), after=btn)
        return btn, body

    def _set_panel_enabled(self, en):
        for sl in ['sl_sp', 'sl_ru', 'sl_kp', 'sl_ki', 'sl_kd', 'sl_kffh', 'sl_kffr',
                   'sl_kffhc', 'sl_kffrc', 'sl_off', 'sl_fan']:
            if hasattr(self, sl): getattr(self, sl).set_enabled(True)
        for b in ['btn_run', 'btn_estop', 'btn_fan']:
            if hasattr(self, b): getattr(self, b).config(state='normal')

    # ─── AKCJE ───────────────────────────────────────────
    def _refresh_control_program_list(self):
        """Skanuje katalog programow i wypelnia liste rozwijana w CONTROL.
        Wywolywane przy starcie aplikacji, po zapisie nowego programu (PROGRAM
        tab) i recznie przyciskiem odswiezania (⟳) obok listy."""
        if not hasattr(self, 'control_program_combo'):
            return
        names = sorted(p.name for p in self.programs_dir.glob("*.json"))
        values = [self.CONTROL_PROGRAM_MANUAL] + names
        current = self.control_program_var.get()
        self.control_program_combo.config(values=values)
        if current not in values:
            self.control_program_var.set(self.CONTROL_PROGRAM_MANUAL)
        self._on_control_program_pick()

    def _on_control_program_pick(self):
        """Aktualizuje podpowiedz pod lista zaleznie od wyboru - jasno mowi co
        zrobi przycisk START zanim uzytkownik go nacisnie, wraz z przyblizonym
        szacowanym czasem trwania calego programu."""
        if not hasattr(self, 'control_start_hint_lbl'):
            return
        sel = self.control_program_var.get()
        if sel == self.CONTROL_PROGRAM_MANUAL:
            self.control_program_hint.config(text="")
            self.control_start_hint_lbl.config(
                text="▶ START uses panel values", fg=C['green'])
        else:
            n_steps = None
            est_txt = ""
            try:
                steps = self._parse_program_json(self.programs_dir / sel)
                n_steps = len(steps)
                est_min = self._estimate_program_duration(steps, self.last_known_t1)
                est_txt = f", {self._fmt_duration_min(est_min)} estimated"
            except Exception:
                pass
            steps_txt = f"{n_steps} steps" if n_steps is not None else "?"
            self.control_program_hint.config(
                text=f"Runs the full step sequence from '{sel}' ({steps_txt}{est_txt}) "
                     f"instead of a single fixed target. Estimate assumes ideal "
                     f"ramp rates - actual time may vary with thermal load.")
            self.control_start_hint_lbl.config(
                text="▶ START runs the selected PROGRAM", fg=C['orange'])
        # gdy program aktualnie nie dziala, wyczysc podglad postepu (zeby nie
        # zostawal "przyklejony" stary status po zmianie wyboru na liscie)
        if not self.program_running and hasattr(self, 'control_prog_status_lbl'):
            self.control_prog_status_lbl.config(text="")
            self.control_prog_detail_lbl.config(text="")

    def _stop_active_program(self):
        """Zatrzymuje aktywna sekwencje programu (jesli byla wlaczona) - wolane
        przy STOP/E-STOP z CONTROL, zeby zatrzymanie PID zatrzymywalo tez
        automatyczne przejscia do kolejnych krokow, a nie tylko biezace grzanie."""
        if getattr(self, 'program_running', False):
            self.program_running = False
            self.program_state = 'idle'
            if hasattr(self, 'btn_prog_start'): self.btn_prog_start.config(state='normal')
            if hasattr(self, 'btn_prog_stop'): self.btn_prog_stop.config(state='disabled')
            if hasattr(self, '_refresh_program_list'): self._refresh_program_list()

    def toggle_run(self):
        if self.is_running: self.do_stop()
        else:
            sel = getattr(self, 'control_program_var', None)
            sel = sel.get() if sel is not None else self.CONTROL_PROGRAM_MANUAL
            if sel != getattr(self, 'CONTROL_PROGRAM_MANUAL', None):
                self._start_control_program(sel)
            else:
                self.do_start()

    def _start_control_program(self, name):
        """START na CONTROL, gdy wybrany jest zapisany program (nie 'Manual') -
        wczytuje kroki z pliku i uruchamia cala sekwencje (jak START PROGRAM
        w zakladce PROGRAM), zamiast pojedynczego, stalego celu z suwakow."""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        try:
            steps = self._parse_program_json(self.programs_dir / name)
        except Exception as e:
            messagebox.showerror("Program load error", f"Could not load program '{name}':\n{e}")
            return
        if not steps:
            messagebox.showinfo("Empty program", f"Program '{name}' has no steps.")
            return
        self.program_steps = steps
        if hasattr(self, '_refresh_program_list'):
            self._refresh_program_list()
        self.start_program()

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
        self.send(f"KFFHC:{self.sl_kffhc.get():.2f}")
        self.send(f"KFFRC:{self.sl_kffrc.get():.2f}")
        self.send(f"OFFSET:{self.sl_off.get():.1f}")
        self.send(f"OPPDIR:{1 if self.oppdir_var.get() else 0}")
        time.sleep(0.05)
        self.send("START")
        self._update_run_button(True)
        self.keithley_sweep_start()

    def do_stop(self):
        self.send("STOP")
        self._update_run_button(False)
        self.sweep_abort = True
        self.keithley_stop_measurement()
        self._stop_active_program()

    def do_stop_peltier_only(self):
        """Zatrzymuje TYLKO PID Peltiera, nie ruszajac sweepu/pomiaru Keithleya."""
        self.send("STOP")
        self._update_run_button(False)
        self._stop_active_program()

    def do_estop(self):
        self.send("STOP")
        self._update_run_button(False)
        self.sweep_abort = True
        self.keithley_stop_measurement()
        self._stop_active_program()

    def toggle_fan(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        self.fan_on = not self.fan_on
        if self.fan_on:
            spd = int(self.sl_fan.get())
            if spd == 0: spd = 100; self.sl_fan.set(100, silent=True)
            self.send(f"FAN:{spd}")
            self.btn_fan.config(text="● ON", bg=C['green'], fg='#0f1f14', highlightbackground=C['green'])
        else:
            self.send("FANOFF")
            self.btn_fan.config(text="○ OFF", bg=C['bg2'], fg=C['dim2'], highlightbackground=C['dim'])

    def set_fan_speed(self, v):
        spd = int(v)
        self.send(f"FAN:{spd}")
        if spd > 0:
            self.fan_on = True
            self.btn_fan.config(text="● ON", bg=C['green'], fg='#0f1f14', highlightbackground=C['green'])
        else:
            self.fan_on = False
            self.btn_fan.config(text="○ OFF", bg=C['bg2'], fg=C['dim2'], highlightbackground=C['dim'])

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
        tk.Label(hd, text="  live snapshot, 10 Hz",
                 bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8))).pack(side='left', padx=(8, 0))

        self.btn_raw_pause = tk.Button(hd, text="⏸ PAUSE", command=self._toggle_raw_pause,
                                       bg=C['bg2'], fg=C['yellow'], font=(FONT, fsz(9), 'bold'),
                                       relief='flat', cursor='hand2', bd=0, padx=12, pady=6,
                                       highlightthickness=1, highlightbackground=C['yellow'],
                                       activebackground=C['panel3'])
        self.btn_raw_pause.pack(side='right', padx=(6, 0))
        mk_btn_outline(hd, "⤓ EXPORT CSV", self.export_raw_csv, C['green']).pack(side='right', padx=(6, 0))
        mk_btn_outline(hd, "CLEAR", self.clear_raw, C['dim']).pack(side='right', padx=(6, 0))

        self.raw_count_lbl = tk.Label(hd, text="0 samples", bg=C['bg'], fg=C['dim'],
                                      font=(FONT, fsz(9)))
        self.raw_count_lbl.pack(side='right', padx=(6, 12))

        # ── JEDEN WIERSZ na zywo - zamiast rosnacej/przewijanej tabeli. Tanie
        # w utrzymaniu (kilka Label.config() zamiast Treeview.insert 10x/s),
        # wiec moze sie aktualizowac bez wzgledu na to czy zakladka jest
        # widoczna i bez zadnego throttlingu - nie ma tu nic co mogloby
        # spowolnic program, bo nie buduje sie zadna historia na ekranie.
        row_wrap = tk.Frame(wrap, bg=C['panel'])
        row_wrap.pack(fill='x')
        tk.Frame(row_wrap, bg=C['blue'], height=3).pack(fill='x')
        cells_frame = tk.Frame(row_wrap, bg=C['panel'])
        cells_frame.pack(fill='x', padx=8, pady=14)

        self.raw_live_cells = {}
        cols = [
            ('idx', '#', 50), ('czas_fw', 'FW TIME [s]', 90), ('pc_time', 'PC TIME', 130),
            ('t1', 'T1 [°C]', 80), ('t2', 'T2 [°C]', 80), ('sp', 'SP TARGET [°C]', 100),
            ('spa', 'SP ACTIVE [°C]', 100), ('pct', 'PELTIER %', 80), ('fan', 'FAN %', 70),
            ('dir', 'DIRECTION', 90), ('k_i', 'I KEITHLEY', 100), ('k_v', 'V KEITHLEY', 100),
            ('state', 'STATE', 70),
        ]
        for key, label, width in cols:
            cell = tk.Frame(cells_frame, bg=C['panel'])
            cell.pack(side='left', padx=(0, 10))
            tk.Label(cell, text=label, bg=C['panel'], fg=C['dim2'],
                     font=(FONT, fsz(7)), width=max(6, width//9), anchor='w').pack(anchor='w')
            val_lbl = tk.Label(cell, text="—", bg=C['panel'], fg=C['text'],
                               font=(FONT, fsz(11), 'bold'), anchor='w')
            val_lbl.pack(anchor='w')
            self.raw_live_cells[key] = val_lbl

        info = tk.Label(wrap, text="Live snapshot of the latest sample - updates in place, no scrolling "
                        f"history, so it never slows the program down. The buffer (max {self.raw_maxrows} "
                        "samples) still collects data in the background for EXPORT CSV. The full raw "
                        "record from START to STOP is in the ARCHIVE tab.",
                        bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8)), wraplength=900, justify='left')
        info.pack(anchor='w', pady=(14, 0))

    def _toggle_raw_pause(self):
        self.raw_paused = not self.raw_paused
        if self.raw_paused:
            self.btn_raw_pause.config(text="▶ RESUME", fg=C['green'], highlightbackground=C['green'])
        else:
            self.btn_raw_pause.config(text="⏸ PAUSE", fg=C['yellow'], highlightbackground=C['yellow'])

    def clear_raw(self):
        self.raw_rows = []
        if hasattr(self, 'raw_live_cells'):
            for lbl in self.raw_live_cells.values():
                lbl.config(text="—")
        if hasattr(self, 'raw_count_lbl'):
            self.raw_count_lbl.config(text="0 samples")

    def _raw_row_values(self, row, idx):
        """Formatuje jeden wiersz (krotke danych) na slownik wartosci do
        wyswietlenia w live-wierszu (klucze odpowiadaja self.raw_live_cells)."""
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
        return {'idx': str(idx), 'czas_fw': f"{czas_fw:.2f}", 'pc_time': pc_time,
                't1': t1s, 't2': t2s, 'sp': f"{sp:.2f}", 'spa': spas,
                'pct': f"{pct:.1f}", 'fan': f"{fan:.1f}", 'dir': dirs,
                'k_i': kis, 'k_v': kvs, 'state': state}

    def _raw_rebuild_tree(self):
        """Odswieza live-wiersz z ostatniej probki w buforze - wywolywane przy
        przelaczeniu na zakladke RAW DATA, zeby od razu widziec aktualny stan."""
        if not hasattr(self, 'raw_live_cells') or not self.raw_rows:
            return
        vals = self._raw_row_values(self.raw_rows[-1], len(self.raw_rows))
        for key, lbl in self.raw_live_cells.items():
            lbl.config(text=vals.get(key, "—"))
        if hasattr(self, 'raw_count_lbl'):
            self.raw_count_lbl.config(text=f"{len(self.raw_rows)} samples")
        self._raw_last_ui_ts = time.time()

    def _raw_append(self, row):
        # row = (czas_fw, pc_time, t1, t2, sp, spa, pct, fan, state, k_i, k_v, heat)
        # Bufor w pamieci rosnie zawsze (tani append+trim na liscie Pythona) -
        # zasila EXPORT CSV niezaleznie od tego czy live-wiersz jest widoczny.
        self.raw_rows.append(row)
        if len(self.raw_rows) > self.raw_maxrows:
            self.raw_rows = self.raw_rows[-self.raw_maxrows:]

        if self.raw_paused or not hasattr(self, 'raw_live_cells'):
            return

        # Aktualizacja jednego wiersza to zaledwie kilka Label.config() - na
        # tyle tanie, ze nie trzeba juz throttlowac ani sprawdzac widocznosci
        # zakladki (w przeciwienstwie do starej tabeli Treeview, ktora przy
        # 10 Hz insertow potrafila zauwazalnie spowalniac cala aplikacje).
        vals = self._raw_row_values(row, len(self.raw_rows))
        for key, lbl in self.raw_live_cells.items():
            lbl.config(text=vals.get(key, "—"))
        self.raw_count_lbl.config(text=f"{len(self.raw_rows)} samples")

    def export_raw_csv(self):
        if not self.raw_rows:
            messagebox.showinfo("No data", "No data to export.")
            return
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Export raw data", defaultextension=".csv",
            initialfile=f"raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            filetypes=[("CSV", "*.csv")])
        if not dest:
            return
        try:
            with open(dest, 'w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(['firmware_time_s', 'timestamp_pc', 'temperature1_C',
                           'temperature2_C', 'setpoint_target_C', 'setpoint_active_C',
                           'peltier_pct', 'fan_pct', 'direction', 'keithley_current_A',
                           'keithley_voltage_V', 'state'])
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
            messagebox.showinfo("Saved", f"Exported {len(self.raw_rows)} samples to:\n{dest}")
        except Exception as e:
            messagebox.showerror("Export error", str(e))

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
        tk.Label(linner, text="Step sweep of voltage or current\nwith measurement and I-V chart",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)),
                 wraplength=250, justify='left').pack(anchor='w', pady=(0, 8))

        # Jednostki sa teraz pokazane bezposrednio przy kazdym polu (V/A/ms/PLC/x/pts),
        # wiec ten blok to tylko DODATKOWY szczegol (twarde limity sprzetu) -
        # zwijany, domyslnie schowany, zeby nie zajmowal miejsca ktore realnie
        # potrzebne jest na same pola.
        self.units_expanded = tk.BooleanVar(value=False)
        self.btn_units_toggle = tk.Button(linner, text="▶ units & hardware limits",
                                          command=self._toggle_units_section,
                                          bg=C['panel'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                          relief='flat', cursor='hand2', bd=0, anchor='w')
        self.btn_units_toggle.pack(fill='x', pady=(0, 2))
        self.units_box = tk.Frame(linner, bg=C['bg2'], highlightthickness=1,
                             highlightbackground=C['border'])
        # nie pakujemy od razu - domyslnie zwiniete
        tk.Label(self.units_box,
                 text="VALUE/START/STOP: V (volts) or A (amps),\n"
                      "depends on MODE. LIMIT: opposite unit -\n"
                      "compliance (with a V source it limits current in A,\n"
                      "with an I source it limits voltage in V).\n"
                      "SETTLE TIME / INTERVAL: ms. NPLC: power line\n"
                      "cycles (no unit, 1.0=20ms\n"
                      "at 50Hz). AVERAGING: number of samples (x).\n"
                      "Measurement results: A and V, with SI prefix (n/µ/m).\n\n"
                      f"Hard hardware limits: voltage up to ±{SMU_V_MAX:g}V,\n"
                      f"current up to ±{SMU_I_MAX:g}A (DC continuous, not pulse),\n"
                      f"power up to {SMU_MAX_POWER:g}W/channel, min. compliance\n"
                      f"current {SMU_I_COMPLIANCE_MIN*1e9:g}nA. The program checks\n"
                      "this automatically before starting a sweep/measurement.",
                 bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(8)),
                 wraplength=250, justify='left').pack(anchor='w', padx=8, pady=8)

        # Wybor trybu
        mode_row = tk.Frame(linner, bg=C['panel'])
        mode_row.pack(fill='x', pady=(0, 10))
        tk.Label(mode_row, text="MODE", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9), 'bold')).pack(anchor='w', pady=(0, 4))
        self.sweep_mode_var = tk.StringVar(value="V")
        mrow = tk.Frame(mode_row, bg=C['panel'])
        mrow.pack(fill='x')
        self.btn_mode_v = tk.Button(mrow, text="SOURCE V", command=lambda: self._set_sweep_mode("V"),
                                    bg=C['orange'], fg='#1a1c1f', font=(FONT, fsz(9), 'bold'),
                                    relief='flat', cursor='hand2', bd=0, padx=4, pady=6)
        self.btn_mode_v.pack(side='left', fill='x', expand=True, padx=(0, 4))
        self.btn_mode_i = tk.Button(mrow, text="SOURCE I", command=lambda: self._set_sweep_mode("I"),
                                    bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                    relief='flat', cursor='hand2', bd=0, padx=4, pady=6)
        self.btn_mode_i.pack(side='left', fill='x', expand=True)

        # Jednoklikowy, gotowy preset dla pomiaru genuine short-circuit
        # current (np. prad piroelektryczny/fotoprad) - SOURCE V przy 0V to
        # WLASCIWY tryb do mierzenia prawdziwego pradu generowanego przez
        # probke. SOURCE I=0A robi cos innego (patrz ostrzezenie ponizej) i
        # bylo zrodlem realnego zamieszania w tej sesji.
        mk_btn_outline(mode_row, "⚡ SHORT-CIRCUIT CURRENT PRESET (recommended)",
                       self._apply_short_circuit_preset, C['green']).pack(fill='x', pady=(6, 0))

        self.sweep_mode_warning_lbl = tk.Label(
            mode_row, text="", bg=C['bg2'], fg=C['yellow'], font=(FONT, fsz(8), 'bold'),
            justify='left', wraplength=250)
        # nie pakujemy od razu - pokazuje sie tylko gdy wykryty zostanie
        # niewlasciwy tryb (patrz _update_sweep_mode_warning)

        # Przelacznik T2 - dodany po tym jak uzytkownik wlasnym testem
        # (odlaczenie przewodu -> skoki znikaly) zidentyfikowal DRUGA
        # termopare jako zrodlo zaklocen wstrzykujacych sie do czulego
        # pomiaru pradu pikoamperowego. Zamiast fizycznie odlaczac przewod
        # za kazdym razem, ten przelacznik po prostu wstrzymuje jej odczyt
        # w firmware (zero aktywnosci SPI/ADC na tym kanale), bez ruszania
        # samego kabla. T2 zostaje fizycznie podlaczona - to tylko pauza
        # odczytu, mozna wlaczyc z powrotem w kazdej chwili.
        tc2_row = tk.Frame(mode_row, bg=C['bg2'])
        tc2_row.pack(fill='x', pady=(8, 0))
        self.tc2_enabled_var = tk.BooleanVar(value=True)
        self.tc2_checkbox_row = mk_checkbox(tc2_row, "T2 thermocouple enabled", self.tc2_enabled_var,
                                            command=self._toggle_tc2, bg=C['bg2'])
        tk.Label(tc2_row, text="Identified as an EMI source affecting sensitive pA "
                 "current readings - disable while measuring, without unplugging the cable.",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8)),
                 justify='left', wraplength=250).pack(anchor='w', pady=(2, 0))

        # Tylko pomiar (bez sweepu) - pojedyncza stala wartosc, ciagly pomiar
        self.sweep_continuous_var = tk.BooleanVar(value=False)
        cont_row = tk.Frame(linner, bg=C['panel'])
        cont_row.pack(fill='x', pady=(10, 0))
        mk_checkbox(cont_row, "Measurement only (no sweep - constant value)",
                   self.sweep_continuous_var, command=lambda: self._toggle_continuous_mode())

        def _field(label, default, unit=""):
            row = tk.Frame(linner, bg=C['panel'])
            row.pack(fill='x', pady=4)
            lbl = tk.Label(row, text=label, bg=C['panel'], fg=C['dim'],
                     font=(FONT, fsz(9)), width=13, anchor='w')
            lbl.pack(side='left')
            # WAZNE: jednostka pakowana side='right' PRZED polem tekstowym -
            # to gwarantuje ze zawsze dostaje swoja naturalna szerokosc i jest
            # widoczna, niezaleznie od szerokosci panelu. Wczesniej etykieta
            # jednostki byla pakowana PO polu z expand=True/fill='x', ktore
            # przy waskim panelu potrafilo zabrac cala dostepna przestrzen i
            # wypchnac jednostke poza widoczny obszar (V/A/ms/x nigdy sie nie
            # pokazywaly).
            ulbl = tk.Label(row, text=unit, bg=C['panel'],
                            fg=C['orange'] if unit else C['dim2'],
                            font=(FONT, fsz(9), 'bold' if unit else 'normal'),
                            width=4, anchor='w')
            ulbl.pack(side='right', padx=(4, 0))
            e = tk.Entry(row, bg=C['bg2'], fg=C['text'], font=(FONT, fsz(10)),
                        relief='flat', bd=0, insertbackground=C['orange'],
                        highlightthickness=1, highlightbackground=C['border'])
            e.pack(side='left', fill='x', expand=True, ipady=4)
            e.insert(0, str(default))
            e.unit_lbl = ulbl
            e.field_lbl = lbl
            return e

        # ── Opis pol - ZWIJANY, domyslnie ZWINIETY (sama tresc pomocnicza,
        # nie zawiera pol - te sa zawsze widoczne nizej, bez wzgledu na stan) ──
        self.params_expanded = tk.BooleanVar(value=False)
        phd = tk.Frame(linner, bg=C['panel'])
        phd.pack(fill='x', pady=(12, 2))
        self.btn_params_toggle = tk.Button(phd, text="▶ field descriptions", command=self._toggle_params_section,
                                           bg=C['panel'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                           relief='flat', cursor='hand2', bd=0, anchor='w')
        self.btn_params_toggle.pack(fill='x')

        self.params_help_body = tk.Frame(linner, bg=C['panel'])
        # nie pakujemy od razu - domyslnie zwiniete (patrz params_expanded=False)

        self.params_desc_lbl = tk.Label(self.params_help_body,
                 text="START/STOP - range of the set value (V or A,\n"
                      "depends on the mode above). STEPS - how many\n"
                      "measurement points evenly spaced between\n"
                      "START and STOP. LIMIT - compliance (e.g. with a\n"
                      "V source, the max allowed current - protects\n"
                      "the sample/contacts). SETTLE TIME - how many ms to wait\n"
                      "after each change of the set value, before\n"
                      "the instrument measures the result (time for the\n"
                      "electrical signal to settle).",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)),
                 wraplength=250, justify='left')
        self.params_desc_lbl.pack(anchor='w', pady=(0, 8))

        # ── Pola - ZAWSZE WIDOCZNE, niezaleznie od stanu opisu powyzej ──
        self.params_body = tk.Frame(linner, bg=C['panel'])
        self.params_body.pack(fill='x')

        # Pola sweepu (zakres) - ukrywane w trybie "tylko pomiar"
        self._sweep_range_frame = tk.Frame(self.params_body, bg=C['panel'])
        self._sweep_range_frame.pack(fill='x')
        old_linner = linner
        linner = self._sweep_range_frame
        self.sweep_start_entry = _field("START", "0.000001", "V")
        self.sweep_stop_entry  = _field("STOP", "0.00005", "V")
        self.sweep_step_entry  = _field("STEPS", "50", "pts")
        linner = old_linner

        # Pole pojedynczej wartosci - widoczne TYLKO w trybie "tylko pomiar"
        self._single_value_frame = tk.Frame(self.params_body, bg=C['panel'])
        linner = self._single_value_frame
        self.sweep_value_entry = _field("VALUE", "0.000001", "V")
        linner = old_linner
        self._single_value_frame.pack_forget()  # ukryte domyslnie (tryb sweep aktywny)

        # LIMIT i SETTLE TIME zawsze widoczne (potrzebne w obu trybach)
        linner = self.params_body
        self.sweep_limit_entry = _field("LIMIT", "0.0001", "A")
        self.sweep_settle_entry = _field("SETTLE TIME", "50", "ms")
        self.sweep_nplc_entry = _field("NPLC", "1.0", "PLC")
        self.sweep_avg_entry = _field("AVERAGING", "1", "x")

        # AUTOZERO/AUTORANGE - wczesniej wpisane na sztywno (zawsze ON), bez
        # zadnej mozliwosci wylaczenia. Wedlug dokumentacji Keithleya i
        # zglaszanych na forum Keithley/NI realnych pomiarow, samo AUTOZERO
        # potrafi spowolnic pomiar nawet ~1.75x wzgledem czystego czasu NPLC
        # (np. NPLC=1 dajace teoretycznie ~19.7ms w praktyce mierzone jako
        # ~34.7ms z autozero wlaczonym) - to jeden z glownych, wczesniej
        # niewidocznych/niezmiennych czynnikow spowalniajacych pomiar ponizej
        # oficjalnej maksymalnej predkosci przyrzadu. AUTORANGE tez dodaje
        # narzut gdy sygnal "poluje" miedzy zakresami blisko ich granicy.
        az_row = tk.Frame(self.params_body, bg=C['panel'])
        az_row.pack(fill='x', pady=(8, 0))
        self.sweep_autozero_var = tk.BooleanVar(value=True)
        mk_checkbox(az_row, "AUTOZERO (more accurate, can be noticeably slower)",
                   self.sweep_autozero_var)
        self.sweep_autorange_var = tk.BooleanVar(value=True)
        mk_checkbox(az_row, "AUTORANGE (convenient, adds overhead near range edges)",
                   self.sweep_autorange_var)
        tk.Label(az_row, text="Turning either OFF trades some accuracy/range-safety for "
                 "speed - see field descriptions above for details.",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)),
                 justify='left', wraplength=250).pack(anchor='w', pady=(3, 0))

        linner = old_linner

        # Sweep dwukierunkowy (tam i z powrotem) - przydatne np. do histerezy
        self.sweep_bidir_var = tk.BooleanVar(value=False)
        self._bidir_row = tk.Frame(self.params_body, bg=C['panel'])
        self._bidir_row.pack(fill='x', pady=(6, 0))
        mk_checkbox(self._bidir_row, "Sweep back and forth", self.sweep_bidir_var)

        # Petla - powtarzaj sweep w kolko, np. przez caly czas rampy PID
        self.sweep_loop_var = tk.BooleanVar(value=False)
        self._loop_chk_row = tk.Frame(self.params_body, bg=C['panel'])
        self._loop_chk_row.pack(fill='x', pady=(4, 0))
        self.chk_loop = mk_checkbox(self._loop_chk_row, "Loop (repeat until STOP)",
                                    self.sweep_loop_var,
                                    command=lambda: self._toggle_loop_pause_field())

        linner = self.params_body
        self.sweep_loop_pause_entry = _field("PAUSE", "0", "ms")
        linner = old_linner
        self.sweep_loop_pause_entry.master.pack_forget()  # ukryte dopoki petla wylaczona
        self._loop_pause_row = self.sweep_loop_pause_entry.master

        # WAZNE: btn_row musi byc rodzenstwem _loop_pause_row (oba dzieci
        # self.params_body) - inaczej pack(before=self._sweep_btn_row) w
        # _toggle_continuous_mode() rzuca TclError ("can't pack X inside Y"),
        # bo Tk wymaga tego samego rodzica dla widgetu i jego "before=".
        # Wczesniej btn_row bylo parentowane do 'linner' (zewnetrzna ramka
        # sekcji), co rozjezdzalo sie z self.params_body przy pierwszym
        # zaznaczeniu "Measurement only" na swiezym uruchomieniu aplikacji.
        btn_row = tk.Frame(self.params_body, bg=C['panel'])
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
        mk_btn_outline(exp_row, "⤓ EXPORT CSV", self.export_sweep_csv, C['green']).pack(fill='x', pady=(0, 4))
        mk_btn_outline(exp_row, "CLEAR CHART", self.clear_sweep, C['dim']).pack(fill='x')

        # ── PANEL PRAWY: wykres (I-V / V(t) / I(t)) ──
        right = tk.Frame(wrap, bg=C['panel'])
        right.pack(side='left', fill='both', expand=True)
        tk.Frame(right, bg=C['border2'], height=3).pack(fill='x')

        rhd = tk.Frame(right, bg=C['panel'])
        rhd.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(rhd, text="CHART", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        self.sweep_pts_lbl = tk.Label(rhd, text="0 points", bg=C['panel'], fg=C['dim2'],
                                      font=(FONT, fsz(9)))
        self.sweep_pts_lbl.pack(side='right')

        # Przelacznik typu wykresu
        chart_sel = tk.Frame(right, bg=C['panel'])
        chart_sel.pack(fill='x', padx=14, pady=(0, 6))
        self.sweep_chart_view = "IV"  # "IV", "VT", "IT"
        # width=4 (jednakowa dla wszystkich trzech, font monospace) - bez tego
        # "I-V" (3 znaki) i "V(t)"/"I(t)" (4 znaki) dawaly rozjezdzajaca sie
        # szerokosc przyciskow, wiec caly rzad nie wygladal jak rowne "kwadraty".
        self.btn_chart_iv = tk.Button(chart_sel, text="I-V", command=lambda: self._set_chart_view("IV"),
                                      bg=C['orange'], fg='#1a1c1f', font=(FONT, fsz(9), 'bold'),
                                      relief='flat', cursor='hand2', bd=0, width=4, pady=5)
        self.btn_chart_iv.pack(side='left', padx=(0, 4))
        self.btn_chart_vt = tk.Button(chart_sel, text="V(t)", command=lambda: self._set_chart_view("VT"),
                                      bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                      relief='flat', cursor='hand2', bd=0, width=4, pady=5)
        self.btn_chart_vt.pack(side='left', padx=(0, 4))
        self.btn_chart_it = tk.Button(chart_sel, text="I(t)", command=lambda: self._set_chart_view("IT"),
                                      bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                      relief='flat', cursor='hand2', bd=0, width=4, pady=5)
        self.btn_chart_it.pack(side='left')

        self.sweep_autoscale_var = tk.BooleanVar(value=True)
        mk_checkbox(chart_sel, "Auto-scale", self.sweep_autoscale_var,
                   command=lambda: self._redraw_sweep_chart(), side='right')

        # Wygladzanie DISPLAY-ONLY (srednia krocząca) - nie dotyka surowych
        # danych ani pliku CSV, tylko dorysowuje dodatkowa, gladsza linie na
        # tym samym wykresie, zeby latwiej bylo dostrzec trend w bardzo
        # zaszumionym sygnale (np. prad rzedu pojedynczych fA/pA).
        tk.Label(chart_sel, text="window:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8))).pack(side='right', padx=(2, 4))
        self.sweep_smooth_window_entry = tk.Entry(
            chart_sel, bg=C['bg2'], fg=C['text'], insertbackground=C['text'],
            relief='flat', font=(FONT, fsz(9)), width=4,
            highlightthickness=1, highlightbackground=C['border'])
        self.sweep_smooth_window_entry.insert(0, "15")
        self.sweep_smooth_window_entry.pack(side='right', padx=(0, 2))
        self.sweep_smooth_window_entry.bind('<Return>', lambda e: self._redraw_sweep_chart())
        self.sweep_smooth_var = tk.BooleanVar(value=False)
        mk_checkbox(chart_sel, "Smooth (moving avg)", self.sweep_smooth_var,
                   command=lambda: self._redraw_sweep_chart(), side='right')

        # Duze karty na zywo: napiecie, prad, postep kroku
        scards_wrap = tk.Frame(right, bg=C['panel'])
        scards_wrap.pack(fill='x', padx=14, pady=(0, 10))
        self.sweep_cards = {}
        self.sweep_cards['v'] = self._stat_card(scards_wrap, "VOLTAGE", "V", C['orange'])
        self.sweep_cards['i'] = self._stat_card(scards_wrap, "CURRENT", "A", C['blue'])
        self.sweep_cards['step'] = self._stat_card(scards_wrap, "STEP", "", C['green'])
        self.sweep_progress_lbl = self.sweep_cards['step']['val']  # alias - karta pokazuje "X / Y"
        self.sweep_cards['step']['unit_lbl'].config(text="")

        self.sweep_fig = Figure(figsize=(7, 5.3), facecolor=C['panel'], dpi=100)
        self.sweep_ax = self.sweep_fig.add_subplot(111)
        self.sweep_ax.set_facecolor(C['panel2'])
        self.sweep_ax.set_xlabel("Voltage [V]", color=C['dim'], fontsize=8)
        self.sweep_ax.set_ylabel("Current [A]", color=C['dim'], fontsize=8)
        self.sweep_ax.tick_params(colors=C['dim'], labelsize=7)
        for spine in self.sweep_ax.spines.values():
            spine.set_color(C['border'])
        self.sweep_ax.grid(True, color=C['grid'], linewidth=0.5, alpha=0.5)
        self.sweep_line, = self.sweep_ax.plot([], [], color=C['orange'], marker='o',
                                               markersize=3, linewidth=1.2,
                                               zorder=5, label='raw')
        self.sweep_line_smooth, = self.sweep_ax.plot([], [], color=C['cyan'], marker='',
                                                      linewidth=2.0, zorder=6, label='smoothed')
        self.sweep_line_smooth.set_visible(False)
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
            self.mpl_toolbar_sweep = None
        self._setup_scroll_drag_zoom(self.sweep_cv, self.mpl_toolbar_sweep,
                                     '_sweep_chart_frozen', '_sweep_chart_dragging')

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
                self.sweep_ax.set_xlabel("Set voltage", color=C['dim'], fontsize=8)
                self.sweep_ax.set_ylabel("Measured current", color=C['dim'], fontsize=8)
                _fmt(self.sweep_ax, 'V', 'A'); mini_units = ('V', 'A')
            else:
                xs = [p[1] for p in pts]; ys = [p[3] for p in pts]
                self.sweep_ax.set_xlabel("Set current", color=C['dim'], fontsize=8)
                self.sweep_ax.set_ylabel("Measured voltage", color=C['dim'], fontsize=8)
                _fmt(self.sweep_ax, 'A', 'V'); mini_units = ('A', 'V')
            self.sweep_line.set_marker('o')
        elif self.sweep_chart_view == "VT":
            xs = [p[0] for p in pts]; ys = [p[3] for p in pts]
            self.sweep_ax.set_xlabel("Time [s]", color=C['dim'], fontsize=8)
            self.sweep_ax.set_ylabel("Measured voltage", color=C['dim'], fontsize=8)
            _fmt(self.sweep_ax, '', 'V'); mini_units = ('', 'V')
            self.sweep_line.set_marker('')
        else:  # IT
            xs = [p[0] for p in pts]; ys = [p[2] for p in pts]
            self.sweep_ax.set_xlabel("Time [s]", color=C['dim'], fontsize=8)
            self.sweep_ax.set_ylabel("Measured current", color=C['dim'], fontsize=8)
            _fmt(self.sweep_ax, '', 'A'); mini_units = ('', 'A')
            self.sweep_line.set_marker('')
        self.sweep_line.set_data(xs, ys)

        # Wygladzanie (srednia krocząca) - opcjonalna DRUGA linia narysowana
        # NA WIERZCHU surowych danych, zeby latwiej zobaczyc trend w mocno
        # zaszumionym sygnale. Nie zmienia self.sweep_points ani pliku CSV -
        # to czysto wizualna nakladka liczona na biezaco z tych samych danych.
        if getattr(self, 'sweep_smooth_var', None) is not None and self.sweep_smooth_var.get() and len(ys) >= 2:
            try:
                win = max(2, int(self.sweep_smooth_window_entry.get()))
            except ValueError:
                win = 15
            win = min(win, len(ys))
            ys_smooth = []
            run_sum = 0.0
            from collections import deque
            buf = deque()
            for y in ys:
                buf.append(y); run_sum += y
                if len(buf) > win:
                    run_sum -= buf.popleft()
                ys_smooth.append(run_sum / len(buf))
            self.sweep_line_smooth.set_data(xs, ys_smooth)
            self.sweep_line_smooth.set_visible(True)
            self.sweep_line.set_alpha(0.35)  # przygaszamy surowe dane, zeby wygladzona linia byla czytelna
            if not self.sweep_ax.get_legend():
                self.sweep_ax.legend(facecolor=C['panel'], edgecolor=C['border'],
                                     labelcolor=C['dim'], fontsize=7, loc='best')
        else:
            self.sweep_line_smooth.set_visible(False)
            self.sweep_line.set_alpha(1.0)
            leg = self.sweep_ax.get_legend()
            if leg is not None:
                leg.remove()

        manual_zoom = getattr(self, '_sweep_chart_frozen', False)
        if not manual_zoom and (getattr(self, 'sweep_autoscale_var', None) is None or self.sweep_autoscale_var.get()):
            self.sweep_ax.set_autoscale_on(True)
            self.sweep_ax.relim(); self.sweep_ax.autoscale_view()
        self.sweep_cv.draw_idle()
        if hasattr(self, 'sweep_line_mini'):
            _fmt(self.sweep_ax_mini, mini_units[0], mini_units[1])
            self.sweep_line_mini.set_data(xs, ys)
            self.sweep_ax_mini.relim(); self.sweep_ax_mini.autoscale_view()
            self.sweep_cv_mini.draw_idle()

    def _toggle_units_section(self):
        if self.units_expanded.get():
            self.units_box.pack_forget()
            self.btn_units_toggle.config(text="▶ units & hardware limits")
            self.units_expanded.set(False)
        else:
            self.units_box.pack(fill='x', pady=(0, 14), after=self.btn_units_toggle)
            self.btn_units_toggle.config(text="▼ units & hardware limits")
            self.units_expanded.set(True)

    def _toggle_params_section(self):
        if self.params_expanded.get():
            self.params_help_body.pack_forget()
            self.btn_params_toggle.config(text="▶ field descriptions")
            self.params_expanded.set(False)
        else:
            self.params_help_body.pack(fill='x', after=self.btn_params_toggle, before=self.params_body)
            self.btn_params_toggle.config(text="▼ field descriptions")
            self.params_expanded.set(True)

    def _toggle_continuous_mode(self):
        if self.sweep_continuous_var.get():
            self._sweep_range_frame.pack_forget()
            self._single_value_frame.pack(fill='x', before=self.sweep_limit_entry.master)
            self._bidir_row.pack_forget()
            self.sweep_loop_var.set(True)
            self.chk_loop.set_enabled(False)
            self.chk_loop.render()
            self._loop_pause_row.pack(fill='x', pady=4, before=self._sweep_btn_row)
            self.sweep_loop_pause_entry.field_lbl.config(text="INTERVAL")
            self.params_desc_lbl.config(
                text="VALUE - the constant set value (V or A,\n"
                     "depends on the mode above); the Keithley will\n"
                     "hold it and measure continuously. LIMIT - compliance\n"
                     "(protects the sample/contacts). These are TWO SEPARATE things:\n"
                     "SETTLE TIME - the physical settling time of the\n"
                     "signal AFTER a value change (not relevant here,\n"
                     "since the value is constant - can be set to 0). INTERVAL -\n"
                     "spacing between CONSECUTIVE samples [ms] - THIS controls\n"
                     "the data-collection rate (e.g. 100ms =\n"
                     "10 samples/s). NPLC/AVERAGING - see below.")
            self.sweep_value_entry.bind('<KeyRelease>', lambda e: self._update_sweep_mode_warning())
        else:
            self._single_value_frame.pack_forget()
            self._sweep_range_frame.pack(fill='x', before=self.sweep_limit_entry.master)
            self._bidir_row.pack(fill='x', pady=(6, 0), before=self._loop_chk_row)
            self.chk_loop.set_enabled(True)
            self.sweep_loop_var.set(False)
            self.chk_loop.render()
            self._loop_pause_row.pack_forget()
            self.sweep_loop_pause_entry.field_lbl.config(text="PAUSE")
            self.params_desc_lbl.config(
                text="START/STOP - range of the set value (V or A,\n"
                     "depends on the mode above). STEPS - how many\n"
                     "measurement points evenly spaced between\n"
                     "START and STOP. LIMIT - compliance (e.g. with a\n"
                     "V source, the max allowed current - protects\n"
                     "the sample/contacts). SETTLE TIME - how many ms to wait\n"
                     "after each change of the set value, before\n"
                     "the instrument measures the result (time for the\n"
                     "electrical signal to settle). NPLC/AVERAGING -\n"
                     "see below: control the noise of the current/voltage measurement.")

        self._update_sweep_mode_warning()

    def _toggle_loop_pause_field(self):
        if self.sweep_loop_var.get():
            self._loop_pause_row.pack(fill='x', pady=4, before=self._sweep_btn_row)
        else:
            self._loop_pause_row.pack_forget()

    def _update_sweep_mode_warning(self):
        """Pokazuje wyrazne ostrzezenie gdy wybrana kombinacja ustawien NIE
        jest wlasciwa do pomiaru prawdziwego pradu generowanego przez probke
        (np. prad piroelektryczny/fotoprad). Konkretnie: SOURCE I z wartoscia
        bliska zeru to NIE jest pomiar pradu - to Keithley aktywnie dociskajacy
        napiecie, zeby WYMUSIC prad bliski zeru (wirtualny "otwarty obwod
        pradowy"), wiec odczyt "current" w tym trybie to szum petli serwo,
        nie realny sygnal z probki. Bylo to realnym zrodlem zamieszania w tej
        sesji (probka dawala pozornie chaotyczny szum, bo mierzylismy w
        niewlasciwym trybie)."""
        if not hasattr(self, 'sweep_mode_warning_lbl'):
            return
        try:
            val = float(self.sweep_value_entry.get().replace(',', '.'))
        except (ValueError, AttributeError):
            val = None
        is_source_i_near_zero = (self.sweep_mode == "I" and val is not None and abs(val) < 1e-9)
        if is_source_i_near_zero:
            self.sweep_mode_warning_lbl.config(
                text="⚠ SOURCE I at ~0A does NOT measure real sample current - "
                     "it forces near-zero current and reports the servo-loop "
                     "voltage noise instead. For genuine short-circuit current "
                     "(e.g. pyroelectric/photocurrent), use SOURCE V at 0V - "
                     "see the preset button above.")
            self.sweep_mode_warning_lbl.pack(fill='x', pady=(6, 0))
        else:
            self.sweep_mode_warning_lbl.pack_forget()

    def _toggle_tc2(self):
        """Wysyla komende wlaczajaca/wylaczajaca odczyt DRUGIEJ termopary bez
        fizycznego odlaczania przewodu - patrz komentarz przy tworzeniu
        checkboxa. Uzytkownik zidentyfikowal T2 jako zrodlo zaklocen
        wplywajacych na pomiar pradu pikoamperowego na Keithleyu."""
        on = self.tc2_enabled_var.get()
        self.send(f"TC2:{1 if on else 0}")

    def _apply_program_tolerance(self):
        """Aktualizuje globalna tolerancje (± zakres) uzywana przez wszystkie
        kroki programu do rozpoznania 'cel osiagniety' - patrz pole
        TOLERANCE w formularzu, zastepuje wczesniej sztywna wartosc 0.5C."""
        try:
            val = float(self.prog_tolerance_entry.get().replace(',', '.'))
        except ValueError:
            self.prog_tolerance_entry.delete(0, 'end')
            self.prog_tolerance_entry.insert(0, str(self.program_tolerance))
            return
        val = max(0.05, min(val, 10.0))  # sensowne granice - od bardzo precyzyjnego po bardzo luzne
        self.program_tolerance = val
        self.prog_tolerance_entry.delete(0, 'end')
        self.prog_tolerance_entry.insert(0, f"{val:g}")

    def _apply_short_circuit_preset(self):
        """Jednoklikowe, gotowe ustawienie do pomiaru prawdziwego pradu
        zwarciowego probki (SOURCE V = 0V + ciagly pomiar) - wlasciwy tryb dla
        prądu piroelektrycznego/fotopradu. Eliminuje ryzyko przypadkowego
        pozostania w SOURCE I=0A (patrz _update_sweep_mode_warning), ktore
        NIE mierzy prawdziwego pradu probki."""
        if self.sweep_mode != "V":
            self._set_sweep_mode("V")
        if not self.sweep_continuous_var.get():
            self.sweep_continuous_var.set(True)
            self._toggle_continuous_mode()
        self.sweep_value_entry.delete(0, 'end'); self.sweep_value_entry.insert(0, "0.0")
        self.sweep_limit_entry.delete(0, 'end'); self.sweep_limit_entry.insert(0, "0.000001")  # 1uA compliance domyslnie
        self._update_sweep_mode_warning()
        messagebox.showinfo("Preset applied",
            "SOURCE V = 0V, measuring genuine short-circuit current.\n\n"
            "LIMIT (current compliance) set to a safe default of 1µA - "
            "adjust if your sample/setup needs a different protection level.")

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
        self._update_sweep_mode_warning()

    def clear_sweep(self):
        self.sweep_points = []
        self.sweep_line.set_data([], [])
        self.sweep_ax.relim(); self.sweep_ax.autoscale_view()
        self.sweep_cv.draw_idle()
        self.sweep_pts_lbl.config(text="0 points")
        self.sweep_cards['v']['val'].config(text="--")
        self.sweep_cards['i']['val'].config(text="--")
        self.sweep_progress_lbl.config(text="--", fg=C['dim2'])

    def keithley_sweep_start(self):
        if self.sweep_running:
            return
        if self.keithley_running:
            messagebox.showwarning("Keithley busy",
                "The PID continuous measurement is currently using the Keithley. Stop the PID (STOP) before sweeping.")
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
            messagebox.showerror("Error", "Check the numeric values in the sweep fields.")
            return
        if not continuous and n_steps < 2:
            messagebox.showerror("Error", "The number of steps must be at least 2.")
            return

        # ── Walidacja wzgledem twardych limitow sprzetowych Keithley 2611B ──
        # Zapobiega wpisaniu wartosci ktorej instrument fizycznie nie obsluzy
        # (np. pomylka rzedu wielkosci przy braku sufiksu n/u/m) zanim cokolwiek
        # zostanie wyslane do SMU.
        mode = self.sweep_mode
        src_max = SMU_V_MAX if mode == "V" else SMU_I_MAX
        lim_max = SMU_I_MAX if mode == "V" else SMU_V_MAX
        src_unit = "V" if mode == "V" else "A"
        lim_unit = "A" if mode == "V" else "V"
        for val, label in ((v0, "START/VALUE"), (v1, "STOP")):
            if abs(val) > src_max:
                messagebox.showerror("SMU limit exceeded",
                    f"{label} = {val:g} {src_unit} exceeds the Keithley 2611B maximum "
                    f"({src_max:g} {src_unit} {'voltage source' if mode=='V' else 'current source (DC continuous)'}).")
                return
        if abs(limit) > lim_max:
            messagebox.showerror("SMU limit exceeded",
                f"LIMIT (compliance) = {limit:g} {lim_unit} exceeds the "
                f"Keithley 2611B maximum ({lim_max:g} {lim_unit}).")
            return
        if mode == "V" and 0 < abs(limit) < SMU_I_COMPLIANCE_MIN:
            messagebox.showerror("Compliance too low",
                f"LIMIT (current compliance) = {limit:g} A is below the "
                f"Keithley 2611B hardware minimum ({SMU_I_COMPLIANCE_MIN:g} A = 10nA). "
                f"The instrument may not accept this.")
            return
        # Moc: P = V*I nie moze przekroczyc SMU_MAX_POWER na kanal (ograniczenie
        # termiczne) - sprawdzamy najgorszy przypadek (najwieksza wartosc zrodlowa
        # razem z limitem compliance, bo to iloczyn ktory faktycznie moze wystapic).
        worst_v = max(abs(v0), abs(v1)) if mode == "V" else limit
        worst_i = limit if mode == "V" else max(abs(v0), abs(v1))
        est_power = worst_v * worst_i
        if est_power > SMU_MAX_POWER:
            if not messagebox.askyesno("Possible power limit exceeded",
                f"Estimated power {est_power:.1f} W (at {worst_v:g} V x {worst_i:g} A) "
                f"exceeds the {SMU_MAX_POWER:g} W/channel maximum for the Keithley 2611B - "
                f"the instrument may thermally limit its output. Continue anyway?"):
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
            text=f"saving: {sweep_log_path.name}", fg=C['dim2']))

        def worker():
            try:
                if not self.keithley_connected:
                    idn = self.keithley.connect()
                    self.keithley_connected = True
                    self.root.after(0, lambda: self.keithley_status_lbl.config(
                        text=f"● connected: {idn[:40]}", fg=C['green']))

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
                    self.keithley.setup_source_v_measure_i("a", points[0], limit, nplc, avg_count,
                                                            autozero=self.sweep_autozero_var.get(),
                                                            autorange=self.sweep_autorange_var.get())
                else:
                    self.keithley.setup_source_i_measure_v("a", points[0], limit, nplc, avg_count,
                                                            autozero=self.sweep_autozero_var.get(),
                                                            autorange=self.sweep_autorange_var.get())
                self.keithley.output_on("a")

                first_pass = True
                first_pass_value_set = False  # gate dla measure_iv() - patrz petla for p in points
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
                        # W trybie ciaglym wartosc zrodlowa jest ustawiana RAZ
                        # (przy setup_source_*_measure_* powyzej + pierwsza
                        # probka ponizej) - kolejne probki NIE musza jej znow
                        # ustawiac ani czekac na settle, bo nic sie fizycznie
                        # nie zmienilo. To realnie przyspiesza tempo pomiaru
                        # (usuwa SETTLE_TIME+ponowny zapis z kazdej kolejnej
                        # probki, zostawiajac tylko czas integracji NPLC).
                        use_fast_path = continuous and first_pass_value_set
                        if use_fast_path:
                            i_meas, v_meas = self.keithley.measure_iv("a")
                        elif mode == "V":
                            i_meas, v_meas = self.keithley.set_voltage_and_measure("a", p, settle_s)
                        else:
                            i_meas, v_meas = self.keithley.set_current_and_measure("a", p, settle_s)
                        first_pass_value_set = True
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

                        # NOWY, NIEZALEZNY zapis rowniez do LOGU CYKLU (ten
                        # sam plik co ARCHIVE czyta) - przy KAZDEJ probce
                        # Keithleya, nie tylko przy tykniecach termopar co
                        # 100ms. Efekt: gdy Keithley mierzy szybciej (np. co
                        # ~30ms), jego probki zostaja WSTAWIONE MIEDZY wiersze
                        # temperaturowe wedlug rzeczywistego czasu, zamiast
                        # ginac (wczesniej tylko ostatnia probka przed danym
                        # tykniecem trafiala do tego pliku). Temperatura w
                        # tych "wstawionych" wierszach to OSTATNIA ZNANA
                        # wartosc (zero-order hold) ekstrapolowana w czasie -
                        # nie mamy swiezszego odczytu termopary dokladnie w
                        # tym momencie, ale przy typowej bezwladnosci cieplnej
                        # ukladu i odstepach rzedu dziesiatek ms, trzymanie
                        # ostatniej wartosci jest dokladniejsze niz czekanie
                        # do nastepnego tykniecia. Kolumna czasu ("t") jest
                        # ekstrapolowana z zegara systemowego od ostatniego
                        # znanego tykniecia, wiec wiersze faktycznie ukladaja
                        # sie chronologicznie MIEDZY wierszami z temperatura.
                        if self.cyc_on:
                            t_extrap = self.last_known_rel + \
                                (time.time() - self.last_known_rel_wallclock_ts)
                            self.cyc_log(t_extrap, self.last_known_t1, self.last_known_t2,
                                        self.last_known_sp, self.last_known_pct,
                                        self.last_known_fn, spa=self.last_known_spa,
                                        kp=self.last_known_kp, ki=self.last_known_ki,
                                        kd=self.last_known_kd, fw_ts=None,
                                        state=self.last_known_state,
                                        keithley_i=i_meas, keithley_v=v_meas,
                                        heat=self.last_known_heat_dir)

                    if not loop or self.sweep_abort:
                        break
                    if loop_pause_ms > 0:
                        time.sleep(loop_pause_ms / 1000.0)

                self.keithley.output_off("a")
            except Exception as e:
                self.root.after(0, lambda e=e: messagebox.showerror("Sweep error", str(e)))
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
            # Pomijamy na WYKRESIE pierwsze SWEEP_SETTLE_SKIP_S sekund po
            # starcie/nowej petli - SMU (i czesto sama probka/kontakty)
            # potrzebuje chwili zeby sie ustabilizowac po zalaczeniu wyjscia
            # (ladowanie pojemnosci kabli, autozero, mechaniczne osiadanie
            # kontaktow), wiec pierwsze odczyty potrafia byc bezsensowne
            # (skoki o rzedy wielkosci). Pelne surowe dane (WLACZNIE z tymi
            # pierwszymi probkami) nadal trafiaja bez zmian do pliku
            # sweep_*.csv w ~/BigPeltierPidLogi - filtrujemy TYLKO widok na
            # wykresie/karcie, zeby nic z surowego zapisu nie zgineło.
            if pt[0] < self.SWEEP_SETTLE_SKIP_S:
                got_new = True
                continue
            self.sweep_points.append(pt)
            last_pt = pt
            got_new = True

        if got_new:
            self._redraw_sweep_chart()
            self.sweep_pts_lbl.config(text=f"{len(self.sweep_points)} points")

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
                self.sweep_mini_status_lbl.config(text="done", fg=C['green'])
            else:
                self.sweep_mini_status_lbl.config(text="--", fg=C['dim2'])

        if self.sweep_running:
            self.sweep_progress_lbl.config(
                text=f"{loop_prefix}{self.sweep_done}/{self.sweep_total}", fg=C['green'])
        elif self.sweep_points:
            self.sweep_progress_lbl.config(text="Done", fg=C['green'])
        else:
            self.sweep_progress_lbl.config(text="--", fg=C['dim2'])

        self.root.after(150, self._sweep_tick)

    def export_sweep_csv(self):
        if not self.sweep_points:
            messagebox.showinfo("No data", "No sweep points to export.\n\n"
                "Note: a full, continuous record of ALL loops with time/temperature correlation\n"
                "is already saved automatically in ~/BigPeltierPidLogi/ during every sweep.")
            return
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Export current sweep view", defaultextension=".csv",
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
            messagebox.showinfo("Saved",
                f"Exported {len(self.sweep_points)} points (current loop) to:\n{dest}\n\n"
                f"A full record of all loops with time correlation is in ~/BigPeltierPidLogi/")
        except Exception as e:
            messagebox.showerror("Export error", str(e))

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

        info = tk.Frame(wrap, bg=C['panel'])
        info.pack(fill='x')
        tk.Frame(info, bg=C['dim2'], height=3).pack(fill='x')
        ii = tk.Frame(info, bg=C['panel'])
        ii.pack(fill='x', padx=20, pady=16)
        tk.Label(ii, text="INSTRUCTIONS", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(11), 'bold')).pack(anchor='w', pady=(0, 8))
        for line in [
            "1. Flash the PeltierPID.ino firmware to the ItsyBitsy M0 (Arduino IDE)",
            "2. Connect via USB, select the COM port, click CONNECT",
            "3. The sliders sync automatically after connecting",
            "4. Set TARGET and RATE, click START",
            "5. Live chart + CSV log in ~/BigPeltierPidLogi",
        ]:
            tk.Label(ii, text=line, bg=C['panel'], fg=C['dim'],
                     font=(FONT, fsz(9)), anchor='w').pack(anchor='w', pady=1)

        self.refresh_ports()
        self.root.after(1200, self._keithley_autoconnect_tick)

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
    def _keithley_autoconnect_tick(self):
        """Probuje polaczyc sie z Keithleyem automatycznie w tle, bez potrzeby
        klikania TEST CONNECTION - powtarza probe co ok. 12s dopoki polaczenie
        nie zostanie nawiazane. Uruchamiane raz przy starcie aplikacji, potem
        samo sie ponownie planuje. Respektuje reczny DISCONNECT (patrz
        keithley_disconnect) - jesli user swiadomie sie rozlaczyl, nie
        wskakujemy mu z powrotem co kilkanascie sekund; reczny TEST CONNECTION
        ponownie wlacza auto-retry."""
        interval_ms = 12000
        if self._keithley_autoconnect_enabled and not self.keithley_connected \
           and not self._keithley_autoconnect_busy:
            self._keithley_autoconnect_busy = True
            self.keithley_status_lbl.config(text="● searching for USB device...", fg=C['yellow'])
            def worker():
                try:
                    idn = self.keithley.connect()
                    self.keithley_connected = True
                    self.root.after(0, lambda: self.keithley_status_lbl.config(
                        text=f"● connected: {idn[:40]}", fg=C['green']))
                except Exception:
                    self.keithley_connected = False
                    self.root.after(0, lambda: self.keithley_status_lbl.config(
                        text="● not connected (retrying automatically…)", fg=C['dim2']))
                finally:
                    self._keithley_autoconnect_busy = False
            threading.Thread(target=worker, daemon=True).start()
        self.root.after(interval_ms, self._keithley_autoconnect_tick)

    def keithley_test_connect(self):
        self._keithley_autoconnect_enabled = True  # reczna proba tez zbroji auto-retry na przyszlosc
        self.keithley_status_lbl.config(text="● searching for USB device...", fg=C['yellow'])
        self.root.update_idletasks()

        def worker():
            try:
                idn = self.keithley.connect()
                self.keithley_connected = True
                self.root.after(0, lambda: self.keithley_status_lbl.config(
                    text=f"● connected: {idn[:40]}", fg=C['green']))
            except Exception as e:
                self.keithley_connected = False
                self.root.after(0, lambda: self.keithley_status_lbl.config(
                    text=f"● error: {e}", fg=C['red']))
        threading.Thread(target=worker, daemon=True).start()

    def keithley_disconnect(self):
        self.keithley_running = False
        self.keithley.disconnect()
        self.keithley_connected = False
        self._keithley_autoconnect_enabled = False  # respektuj swiadomy wybor uzytkownika
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
                    text="● connected (output OFF)", fg=C['dim']))
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
                    text=f"● measurement error: {e}", fg=C['red']))
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
        self.arch_checkboxes = {}

        cf = tk.Frame(body, bg=C['panel'])
        cf.pack(side='left', fill='both', expand=True)
        tk.Frame(cf, bg=C['border2'], height=3).pack(fill='x')
        self.fig_a = Figure(figsize=(8, 6), facecolor=C['panel'], dpi=100)
        # Osie tworzone dynamicznie w _redraw_arch: sama temperatura, albo
        # temperatura (gora) + prad Keithleya (dol) jeden pod drugim, wspolna os X.
        self.ax_a = self.fig_a.add_subplot(111)
        self.ax_a.set_facecolor(C['panel2'])
        self.ax_a_current = None
        self.ax_a_voltage = None
        # WAZNE: tworzymy obiekt FigureCanvasTkAgg TERAZ (potrzebny nizej przez
        # toolbar/hover), ale JEGO WIDGET PAKUJEMY NA SAMYM KONCU funkcji -
        # patrz komentarz przy .pack() ponizej. Wczesniej wykres byl pakowany
        # PIERWSZY z fill='both', expand=True i zabieral cala dostepna wysokosc
        # zanim paski przyciskow (w tym wiersz ALIGN) zdazyly zarezerwowac
        # sobie miejsce - przy nizszym oknie te paski byly wypychane calkowicie
        # poza widoczny obszar (brak scrollbara w tej czesci ekranu, wiec nie
        # dalo sie do nich dojechac).
        self.cv_a = FigureCanvasTkAgg(self.fig_a, master=cf)
        self.cv_a.draw()

        # ─── HOVER: kursor pokazujacy czas/temperature/prad w punkcie ───
        self._arch_hover_series = []
        self._arch_hover_mode = 'time'
        self._arch_annot_top = None
        self._arch_annot_bottom = None
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
            self.mpl_toolbar_a = None
        self._setup_scroll_drag_zoom(self.cv_a, self.mpl_toolbar_a,
                                     '_arch_chart_frozen', '_arch_chart_dragging')

        atb = tk.Frame(cf, bg=C['panel'])
        atb.pack(fill='x', padx=8, pady=(2, 8))
        mk_btn_outline(atb, "📁", self.open_log_folder, C['dim']).pack(side='right', padx=(4, 0))
        mk_btn_outline(atb, "⤓ PNG", self.save_arch_chart, C['cyan']).pack(side='right', padx=(4, 0))
        mk_btn(atb, "⤓ DOWNLOAD CSV (selected cycle)", self.export_selected_cycle_csv, C['green']).pack(
            side='right', padx=(4, 0))
        mk_btn_outline(atb, "❓ CSV in Excel", self._show_excel_help, C['yellow']).pack(
            side='right', padx=(4, 0))

        atb2 = tk.Frame(cf, bg=C['panel'])
        atb2.pack(fill='x', padx=8, pady=(0, 8))
        tk.Label(atb2, text="CHART:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(0, 6))
        self.arch_chart_mode = tk.StringVar(value='time')
        mk_segmented(atb2,
                     [("temperature / time", 'time'), ("Keithley current / temperature", 'iT')],
                     self.arch_chart_mode, command=self._redraw_arch)

        atb2b = tk.Frame(cf, bg=C['panel'])
        atb2b.pack(fill='x', padx=8, pady=(0, 8))
        self.arch_show_current_var = tk.BooleanVar(value=True)
        tk.Label(atb2b, text="KEITHLEY (below temperature):", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(0, 4))
        self.arch_show_current_chk = mk_checkbox(
            atb2b, "Current", self.arch_show_current_var,
            command=lambda: self._redraw_arch(), side='left')
        self.arch_show_voltage_var = tk.BooleanVar(value=False)
        self.arch_show_voltage_chk = mk_checkbox(
            atb2b, "Voltage (same panel, together)", self.arch_show_voltage_var,
            command=lambda: self._redraw_arch(), side='left')

        atb3 = tk.Frame(cf, bg=C['panel'])
        atb3.pack(fill='x', padx=8, pady=(0, 8))
        tk.Label(atb3, text="ALIGN:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(0, 6))
        self.arch_align_mode = tk.StringVar(value='none')
        _align_segments = mk_segmented(
            atb3,
            [("none (time since start)", 'none'), ("by temperature", 'temp'),
             ("by current", 'current'), ("auto: match cycle", 'auto')],
            self.arch_align_mode, command=self._on_align_mode_change, accent=C['orange'])
        self.arch_align_temp_btn = _align_segments['temp']
        self.arch_align_cur_btn = _align_segments['current']
        self.arch_align_auto_btn = _align_segments['auto']
        self.arch_align_entry = tk.Entry(atb3, bg=C['bg2'], fg=C['text'],
                                        insertbackground=C['text'], relief='flat',
                                        font=(FONT, fsz(9)), width=10,
                                        highlightthickness=1, highlightbackground=C['border'])
        self.arch_align_entry.insert(0, "25.0")
        self.arch_align_entry.pack(side='left', padx=(8, 2))
        self.arch_align_entry.bind('<Return>', lambda e: self._redraw_arch())
        self.arch_align_unit_lbl = tk.Label(atb3, text="°C", bg=C['panel'], fg=C['dim2'],
                                            font=(FONT, fsz(9)))
        self.arch_align_unit_lbl.pack(side='left', padx=(0, 6))
        mk_btn_outline(atb3, "APPLY", self._redraw_arch, C['cyan']).pack(side='left')
        self.arch_align_entry.config(state='disabled')

        atb4 = tk.Frame(cf, bg=C['panel'])
        atb4.pack(fill='x', padx=8, pady=(0, 8))
        tk.Label(atb4, text="  ↳ reference cycle (starting temperature is matched):",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8))).pack(side='left', padx=(0, 6))
        self.arch_align_ref_var = tk.StringVar(value="")
        self.arch_align_ref_combo = ttk.Combobox(
            atb4, textvariable=self.arch_align_ref_var, state='disabled',
            style='Dark.TCombobox', font=(FONT, fsz(9)), width=28)
        self.arch_align_ref_combo.pack(side='left')
        self.arch_align_ref_combo.bind('<<ComboboxSelected>>', lambda e: self._redraw_arch())

        # Wykres pakowany DOPIERO TERAZ, na samym koncu - wszystkie paski nad
        # nim (toolbar, pobieranie/PNG, tryb wykresu, ALIGN) maja juz
        # zarezerwowane miejsce, wiec sa zawsze widoczne. Wykres wypelnia
        # tylko to co zostalo (kurczy sie w razie potrzeby, zamiast wypychac
        # cokolwiek poza ekran).
        self.cv_a.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(8, 4))

        self._arch_colors = [C['blue'], C['orange'], C['green'], C['red'],
                            C['cyan'], C['purple'], C['yellow'], '#ff8fab']
        self.refresh_arch()
        self._redraw_arch()

    def _on_align_mode_change(self):
        mode = self.arch_align_mode.get()
        if mode == 'none':
            self.arch_align_entry.config(state='disabled')
            self.arch_align_ref_combo.config(state='disabled')
        elif mode == 'auto':
            self.arch_align_entry.config(state='disabled')
            self._refresh_align_ref_combo()
            self.arch_align_ref_combo.config(state='readonly')
        else:
            self.arch_align_entry.config(state='normal')
            self.arch_align_ref_combo.config(state='disabled')
            self.arch_align_unit_lbl.config(text="°C" if mode == 'temp' else "A (e.g. 50n, 2u, 0.001)")
        self._redraw_arch()

    def _refresh_align_ref_combo(self):
        """Wypelnia liste wyboru cyklu-referencji nazwami aktualnie ZAZNACZONYCH
        cykli (checkbox po lewej) - tylko sposrod nich ma sens wybierac, bo tylko
        te sa w ogole rysowane na wykresie."""
        names = [self._cycle_display_name(p) for p, v in self.arch_vars.items() if v.get()]
        self.arch_align_ref_combo.config(values=names)
        if self.arch_align_ref_var.get() not in names:
            self.arch_align_ref_var.set(names[0] if names else "")

    def _align_ref_name(self):
        return self.arch_align_ref_var.get() if hasattr(self, 'arch_align_ref_var') else None

    def _resolve_align_value(self, align_mode):
        """Zwraca liczbowa wartosc progu wyrownania dla biezacego trybu:
        - 'temp'/'current': wpisana recznie w pole (jak dotychczas)
        - 'auto': WYLICZONA automatycznie jako temperatura POCZATKOWA
          wybranego cyklu-referencji - reszta zaznaczonych cykli zostanie
          przesunieta tak, zeby zaczynac (przekraczac ta temperature) w tym
          samym punkcie co cykl referencyjny, bez recznego wpisywania liczby."""
        if align_mode in ('temp', 'current'):
            return self._parse_align_value(align_mode)
        if align_mode == 'auto':
            ref_name = self._align_ref_name()
            if not ref_name:
                return None
            for p, v in self.arch_vars.items():
                if self._cycle_display_name(p) == ref_name:
                    d = self._load_cycle(p)
                    if not d:
                        return None
                    t1 = d[1]
                    first = next((x for x in t1 if x is not None), None)
                    return first
            return None
        return None

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

    def _arch_list_files(self):
        """Lista plikow cykli w JEDNYM, spojnym porzadku (wg daty modyfikacji,
        najnowsze pierwsze) - uzywana zarowno przy budowaniu listy checkboxow
        (refresh_arch) jak i przy przypisywaniu kolorow na wykresie (_redraw_arch).
        Wczesniej obie funkcje sortowaly NIEZALEZNIE i RÓŻNYMI kluczami (mtime vs
        alfabetycznie), wiec ten sam plik mial inny indeks/kolor na liscie niz na
        wykresie. Jedno wspolne zrodlo prawdy eliminuje to na stale."""
        return sorted(
            [f for f in self.log_dir.glob("*.csv")
             if (f.name.startswith("cykl_") or f.name.startswith("c_"))
             and not f.name.startswith("_tmp")],
            key=lambda f: f.stat().st_mtime, reverse=True)

    def _arch_date_header_text(self, d):
        """Zwraca czytelny naglowek daty: DZISIAJ / WCZORAJ / pelna data dla starszych."""
        today = datetime.now().date()
        if d == today:
            return "TODAY"
        if d == today - timedelta(days=1):
            return "YESTERDAY"
        return d.strftime("%A, %d %B %Y").upper()

    def refresh_arch(self):
        for w in self.arch_items.winfo_children(): w.destroy()
        self.arch_vars = {}
        self.arch_checkboxes = {}
        files = self._arch_list_files()
        if not files:
            tk.Label(self.arch_items, text="No saved cycles yet.",
                     bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(9))).pack(
                     anchor='w', padx=12, pady=12)
            return
        # Grupowanie po dacie WYKONANIA (data modyfikacji pliku = koniec cyklu).
        # Lista jest juz posortowana od najnowszych (_arch_list_files), wiec
        # kolejne pliki o tej samej dacie tworza naturalnie ciagly blok - nie
        # trzeba nic dodatkowo sortowac, tylko wykryc zmiane dnia w przebiegu.
        last_date = None
        for i, f in enumerate(files):
            mtime = f.stat().st_mtime
            this_date = datetime.fromtimestamp(mtime).date()
            if this_date != last_date:
                last_date = this_date
                hdr = tk.Frame(self.arch_items, bg=C['panel3'])
                hdr.pack(fill='x', pady=(8 if i > 0 else 2, 2))
                tk.Label(hdr, text=self._arch_date_header_text(this_date),
                         bg=C['panel3'], fg=C['dim'], font=(FONT, fsz(8), 'bold')
                         ).pack(anchor='w', padx=10, pady=3)

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
            time_str = datetime.fromtimestamp(mtime).strftime("%H:%M")
            tk.Label(row, text=time_str, bg=C['bg2'], fg=C['dim2'],
                     font=(FONT, fsz(8)), width=6, anchor='w').pack(side='left', padx=(6, 0))
            name = self._cycle_display_name(f)
            disp = name if len(name) <= 22 else name[:20]+"…"
            cb_row = mk_checkbox(row, disp, var, command=self._redraw_arch,
                                 bg=C['bg2'], accent=col, side='left')
            cb_row.pack(fill='x', expand=True)  # nadpisujemy domyslny side='left' pack na fill+expand
            self.arch_checkboxes[str(f)] = cb_row

    def _delete_cycle(self, path):
        if messagebox.askyesno("Delete", f"Delete: {path.name}?"):
            try: path.unlink(); self.refresh_arch(); self._redraw_arch()
            except Exception as e: messagebox.showerror("Error", str(e))

    def _arch_clear_sel(self):
        for v in self.arch_vars.values(): v.set(False)
        for cb in self.arch_checkboxes.values(): cb.render()
        self._redraw_arch()

    def _load_cycle(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
            t,t1,t2,sp,pwm,ki,kv = [],[],[],[],[],[],[]
            for r in rows:
                try:
                    t.append(float(r.get('czas_od_startu_s',0)))
                    v1 = r.get('temperatura1_C','')
                    t1.append(float(v1) if v1 else None)
                    v2 = r.get('temperatura2_C','')
                    # Ta sama ochrona co w strumieniu live (_parse_csv_line):
                    # bug w starszym firmware potrafil zapisac literalne 0.0
                    # dla T2 przy pojedynczym, przejsciowym zlym odczycie
                    # termopary (naprawione w PeltierPID.ino - patrz komentarz
                    # przy "if(tc2OK)"). Traktujemy dokladne 0.0 jako brak
                    # odczytu (przerwa na wykresie), nie prawdziwa temperature -
                    # dzieki temu rowniez CYKLE ZAPISANE PRZED NAPRAWA nie
                    # pokazuja juz fantomowych skokow do zera.
                    t2v = float(v2) if v2 else None
                    t2.append(t2v if t2v != 0 else None)
                    sp.append(float(r.get('setpoint_cel_C',0)))
                    pwm.append(float(r.get('peltier_pct',0)))
                    vki = r.get('keithley_prad_A','')
                    ki.append(float(vki) if vki else None)
                    vkv = r.get('keithley_napiecie_V','')
                    kv.append(float(vkv) if vkv else None)
                except: continue
            if not t:
                return None
            # Sortuj wg czasu - dwa niezalezne watki (tyknienia PID co 100ms +
            # probki Keithleya w ich wlasnym tempie) pisza teraz do TEGO
            # SAMEGO pliku (patrz keithley_sweep_start). Blokada (cyc_log_lock)
            # chroni tylko przed uszkodzeniem pliku przy jednoczesnym zapisie,
            # NIE gwarantuje ze wiersze trafiaja w idealnej kolejnosci
            # czasowej (mozliwy niewielki jitter przy przelaczeniach watkow) -
            # sortujemy tutaj defensywnie, zeby wykres mial gwarantowanie
            # rosnaca os X niezaleznie od kolejnosci zapisu w pliku.
            order = sorted(range(len(t)), key=lambda i: t[i])
            t   = [t[i]   for i in order]
            t1  = [t1[i]  for i in order]
            t2  = [t2[i]  for i in order]
            sp  = [sp[i]  for i in order]
            pwm = [pwm[i] for i in order]
            ki  = [ki[i]  for i in order]
            kv  = [kv[i]  for i in order]
            return (t,t1,t2,sp,pwm,ki,kv)
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
        self._arch_annot_top = None
        self._arch_annot_bottom = None
        self._arch_vline_top = None
        self._arch_vline_bottom = None
        self._arch_marker_top = None
        self._arch_marker_bottom = None
        self._arch_marker_bottom_v = None
        self._arch_marker_main = None
        self._arch_hover_last_key = None

    def _arch_setup_hover(self, mode, hover_series):
        """Tworzy niewidoczne na starcie artysty (linia pionowa + kropka +
        dymek z tekstem) uzywane przez _on_arch_hover. Wywolywane po kazdym
        przebudowaniu wykresu (fig.clear() kasuje poprzednie artysty).

        WAZNE: kazdy panel (gorny=temperatura, dolny=prad) dostaje WLASNA
        adnotacje, bo matplotlib interpretuje xy adnotacji we wspolrzednych
        danych OSI do ktorej nalezy - jedna wspolna adnotacja przypisana do
        gornej osi wyswietlalaby sie w zlym miejscu (przy wartosciach pradu
        rzedu 1e-8 rzutowanych na skale temperatury 0-100 C) gdy kursor jest
        nad dolnym panelem."""
        self._arch_hover_mode = mode
        self._arch_hover_series = hover_series
        self._arch_hover_last_key = None
        annot_kw = dict(xy=(0, 0), xytext=(14, 14), textcoords='offset points',
                        bbox=dict(boxstyle='round,pad=0.4', fc=C['panel2'], ec=C['border'], alpha=0.95),
                        fontsize=fsz(8), color=C['text'], family=FONT, visible=False, zorder=10)
        if mode == 'time':
            self._arch_vline_top = self.ax_a.axvline(
                0, color=C['text'], lw=0.8, ls=':', alpha=0)
            self._arch_marker_top, = self.ax_a.plot(
                [], [], 'o', ms=5, alpha=0, zorder=9)
            self._arch_annot_top = self.ax_a.annotate('', **annot_kw)
            if self.ax_a_current is not None:
                self._arch_vline_bottom = self.ax_a_current.axvline(
                    0, color=C['text'], lw=0.8, ls=':', alpha=0)
                self._arch_marker_bottom, = self.ax_a_current.plot(
                    [], [], 'o', ms=5, alpha=0, zorder=9)
                self._arch_annot_bottom = self.ax_a_current.annotate('', **annot_kw)
                if getattr(self, 'ax_a_voltage', None) is not None:
                    self._arch_marker_bottom_v, = self.ax_a_voltage.plot(
                        [], [], 's', ms=5, alpha=0, zorder=9)
                else:
                    self._arch_marker_bottom_v = None
            else:
                self._arch_vline_bottom = None
                self._arch_marker_bottom = None
                self._arch_marker_bottom_v = None
                self._arch_annot_bottom = None
            self._arch_marker_main = None
        else:  # iT
            self._arch_marker_main, = self.ax_a.plot(
                [], [], 'o', ms=7, alpha=0, zorder=9)
            self._arch_annot_top = self.ax_a.annotate('', **annot_kw)
            self._arch_annot_bottom = None
            self._arch_vline_top = None
            self._arch_vline_bottom = None
            self._arch_marker_top = None
            self._arch_marker_bottom = None

    def _arch_hover_axes(self):
        if getattr(self, '_arch_hover_mode', 'time') == 'time':
            axes = [self.ax_a]
            if self.ax_a_current is not None:
                axes.append(self.ax_a_current)
            if getattr(self, 'ax_a_voltage', None) is not None:
                axes.append(self.ax_a_voltage)
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
        # Gdy uzytkownik aktywnie przybliza/przesuwa wykres (toolbar zoom/pan,
        # albo nasze wlasne przeciagniecie prostokata zoom) - NIE liczymy
        # hover ani nie rysujemy dymka, bo to konkurowalo o rysowanie przy
        # kazdym ruchu myszy podczas przeciagania i powodowalo zacinanie sie
        # calej interakcji. Hover wraca automatycznie po puszczeniu przycisku.
        if getattr(self, '_arch_chart_dragging', False):
            return
        tb = getattr(self, 'mpl_toolbar_a', None)
        if tb is not None and getattr(tb, 'mode', ''):
            return
        series = getattr(self, '_arch_hover_series', None)
        if not series or getattr(self, '_arch_annot_top', None) is None:
            return
        if event.inaxes not in self._arch_hover_axes() or event.xdata is None:
            self._arch_hide_hover()
            return

        mode = self._arch_hover_mode
        # Dopasowanie do panelu pod kursorem: gorny = temperatura, dolny =
        # prad/napiecie razem (oba moga byc na jednym, wspoldzielonym panelu
        # dzieki twin-axis). Kazdy panel ma WLASNA adnotacje (annot_top/bottom),
        # bo xy adnotacji jest interpretowane we wspolrzednych danych jej wlasnej osi.
        on_bottom = (mode == 'time' and event.inaxes in
                     (self.ax_a_current, getattr(self, 'ax_a_voltage', None)))
        # W dolnym panelu prad i napiecie moga miec zupelnie rozne skale (dwie
        # osie na jednym miejscu) - dopasowanie po Y do jednego z nich
        # faworyzowaloby arbitralnie ta krzywa, wiec tam liczy sie TYLKO
        # odleglosc w X (czas). Gorny panel (temperatura) nadal dopasowuje
        # po X i Y normalnie.
        field = 'temp' if not on_bottom and mode != 'iT' else ('current' if mode == 'iT' else None)
        active_annot = self._arch_annot_bottom if on_bottom else self._arch_annot_top
        other_annot = self._arch_annot_top if on_bottom else self._arch_annot_bottom

        xlo, xhi = event.inaxes.get_xlim()
        ylo, yhi = event.inaxes.get_ylim()
        xr = (xhi - xlo) or 1.0
        yr = (yhi - ylo) or 1.0

        best = None  # (score, series_dict, idx)
        for s in series:
            idx = self._arch_nearest_index(s['x'], event.xdata, s['sorted'])
            if idx is None:
                continue
            if field is None:
                dy = 0.0  # dolny panel w trybie 'time' - dopasowanie tylko po X
            else:
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
        key = (s['name'], idx, on_bottom)
        # Druga (nieaktywna) adnotacja chowana zawsze przy zmianie panelu,
        # nawet jesli najblizszy punkt danych sie nie zmienil.
        if other_annot is not None and other_annot.get_visible():
            other_annot.set_visible(False)
        if key == self._arch_hover_last_key:
            return
        self._arch_hover_last_key = key

        t_val = s['t'][idx]
        temp_val = s['temp'][idx] if idx < len(s['temp']) else None
        cur_val = s['current'][idx] if idx < len(s['current']) else None
        volt_val = s.get('voltage', [])
        volt_val = volt_val[idx] if idx < len(volt_val) else None

        lines = [s['name']]
        lines.append(f"time: {t_val:.2f} s")
        if temp_val is not None:
            lines.append(f"T1: {temp_val:.3f} °C")
        if cur_val is not None:
            v, p = fmt_si(cur_val, 3)
            lines.append(f"I: {v} {p}A")
        else:
            lines.append("I: --")
        if volt_val is not None:
            v, p = fmt_si(volt_val, 3)
            lines.append(f"V: {v} {p}V")
        text = "\n".join(lines)

        active_annot.set_text(text)
        active_annot.set_visible(True)
        active_annot.get_bbox_patch().set_edgecolor(s['color'])

        xv = s['x'][idx]
        if mode == 'time':
            panel_val = cur_val if on_bottom else temp_val
            active_annot.xy = (xv, panel_val if panel_val is not None else 0)
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
                if self._arch_marker_bottom_v is not None:
                    if volt_val is not None:
                        self._arch_marker_bottom_v.set_data([xv], [volt_val])
                        self._arch_marker_bottom_v.set_color(s['color']); self._arch_marker_bottom_v.set_alpha(1)
                    else:
                        self._arch_marker_bottom_v.set_alpha(0)
        else:
            active_annot.xy = (xv, cur_val if cur_val is not None else 0)
            if cur_val is not None:
                self._arch_marker_main.set_data([xv], [cur_val])
                self._arch_marker_main.set_color(s['color']); self._arch_marker_main.set_alpha(1)

        self.cv_a.draw_idle()

    def _arch_hide_hover(self):
        if getattr(self, '_arch_annot_top', None) is None:
            return
        if self._arch_hover_last_key is None:
            return
        self._arch_hover_last_key = None
        self._arch_annot_top.set_visible(False)
        if self._arch_annot_bottom is not None:
            self._arch_annot_bottom.set_visible(False)
        if self._arch_hover_mode == 'time':
            self._arch_vline_top.set_alpha(0)
            self._arch_marker_top.set_alpha(0)
            if self._arch_vline_bottom is not None:
                self._arch_vline_bottom.set_alpha(0)
                self._arch_marker_bottom.set_alpha(0)
                if getattr(self, '_arch_marker_bottom_v', None) is not None:
                    self._arch_marker_bottom_v.set_alpha(0)
        elif self._arch_marker_main is not None:
            self._arch_marker_main.set_alpha(0)
        self.cv_a.draw_idle()

    def _redraw_arch(self):
        sel = [(p,v) for p,v in self.arch_vars.items() if v.get()]
        show_current = getattr(self, 'arch_show_current_var', None) is not None \
                       and self.arch_show_current_var.get()
        show_voltage = getattr(self, 'arch_show_voltage_var', None) is not None \
                       and self.arch_show_voltage_var.get()
        mode = getattr(self, 'arch_chart_mode', None)
        mode = mode.get() if mode is not None else 'time'

        # Najpierw wczytaj dane - dopiero potem decyduj o ukladzie osi
        files = self._arch_list_files()
        forder = {str(f):i for i,f in enumerate(files)}
        loaded = []
        any_current_data = False
        any_voltage_data = False
        for path,_ in sel:
            d = self._load_cycle(path)
            if not d: continue
            t,t1,t2,sp,pwm,ki,kv = d
            # UWAGA: brak downsamplingu - rysujemy 100% surowych danych, bez
            # zadnej redukcji/wygladzania/upraszczania renderingu (patrz
            # poczatek pliku: path.simplify jest CELOWO wylaczone). Duze cykle
            # (dziesiatki tysiecy probek) moga rysowac sie wolniej, ale kazdy
            # punkt jest dokladnie taki, jaki jest w danych - zero kompromisu
            # na rzecz szybkosci kosztem wiernosci wykresu.
            has_i = any(v is not None for v in ki)
            has_v = any(v is not None for v in kv)
            any_current_data = any_current_data or has_i
            any_voltage_data = any_voltage_data or has_v
            loaded.append((path, t, t1, sp, ki, kv, has_i, has_v))

        # checkboxy prad/napiecie i wyrownanie maja sens tylko w widoku czasowym
        if hasattr(self, 'arch_show_current_chk'):
            enabled = (mode == 'time')
            self.arch_show_current_chk.set_enabled(enabled)
            self.arch_show_voltage_chk.set_enabled(enabled)
        if hasattr(self, 'arch_align_temp_btn'):
            align_state = 'normal' if mode == 'time' else 'disabled'
            self.arch_align_temp_btn.config(state=align_state)
            self.arch_align_cur_btn.config(state=align_state)
            self.arch_align_auto_btn.config(state=align_state)
            if mode != 'time':
                self.arch_align_entry.config(state='disabled')
                self.arch_align_ref_combo.config(state='disabled')
            elif self.arch_align_mode.get() == 'auto':
                self.arch_align_ref_combo.config(state='readonly')
            elif self.arch_align_mode.get() != 'none':
                self.arch_align_entry.config(state='normal')

        if mode == 'iT':
            self._redraw_arch_iT(sel, [(p,t,t1,s,ki,hi) for p,t,t1,s,ki,kv,hi,hv in loaded], forder)
            return

        # Przebuduj uklad figury od zera (jedna os lub dwie jedna pod druga).
        # Dolny panel (jesli pokazany) moze zawierac PRAD i NAPIECIE razem -
        # prad na lewej osi, napiecie na prawej (twin), oba widoczne naraz
        # zamiast osobnych, przelaczanych wykresow.
        self.fig_a.clear()
        want_bottom = (show_current and any_current_data) or (show_voltage and any_voltage_data)
        if want_bottom:
            gs = self.fig_a.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.12,
                                         left=0.1, right=0.9, top=0.96, bottom=0.09)
            self.ax_a = self.fig_a.add_subplot(gs[0])
            ax_i = self.fig_a.add_subplot(gs[1], sharex=self.ax_a)
            ax_v = ax_i.twinx() if (show_voltage and any_voltage_data) else None
        else:
            self.ax_a = self.fig_a.add_subplot(111)
            self.fig_a.subplots_adjust(left=0.1, right=0.97, top=0.96, bottom=0.09)
            ax_i = None
            ax_v = None
        self.ax_a_current = ax_i
        self.ax_a_voltage = ax_v
        self._style_arch_ax(self.ax_a)
        if ax_i is not None: self._style_arch_ax(ax_i)
        if ax_v is not None:
            ax_v.tick_params(colors=C['orange'], labelsize=8)
            for sp2 in ax_v.spines.values(): sp2.set_visible(False)

        if not sel:
            self.ax_a.text(0.5,0.5,"Tick a cycle to display",
                           ha='center',va='center',color=C['dim2'],
                           fontsize=11,transform=self.ax_a.transAxes)
            self._arch_reset_hover()
            self.cv_a.draw(); return

        hover_series = []
        align_mode = getattr(self, 'arch_align_mode', None)
        align_mode = align_mode.get() if align_mode is not None else 'none'
        if align_mode == 'auto' and hasattr(self, 'arch_align_ref_combo'):
            self._refresh_align_ref_combo()
        align_val = self._resolve_align_value(align_mode)
        align_missed = []  # nazwy cykli ktore nie osiagnely progu - uzyty naturalny start
        show_i_curve = show_current and any_current_data
        show_v_curve = show_voltage and any_voltage_data
        for path, t, t1, sp, ki, kv, has_i, has_v in loaded:
            ci = forder.get(path,0)%len(self._arch_colors)
            col = self._arch_colors[ci]
            nm = self._cycle_display_name(path)
            t_ref = t[0]
            if align_mode in ('temp', 'auto') and align_val is not None:
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
            if ax_i is not None and show_i_curve and has_i:
                # ten sam kolor co T1 danego cyklu - latwo skojarzyc pary krzywych.
                # Linia CIAGLA = prad, zeby odroznic od napiecia (przerywana) gdy
                # oba sa pokazane naraz na tym samym panelu.
                ax_i.plot(tx, ki, color=col, lw=1.4, ls='-')
            if ax_v is not None and show_v_curve and has_v:
                ax_v.plot(tx, kv, color=col, lw=1.2, ls=':', alpha=0.85)
            # dane do hover-kursora: tx jest chronologicznie rosnace (bisect OK)
            hover_series.append({'name': nm, 'color': col, 'x': tx, 't': tx,
                                  'temp': t1, 'current': ki, 'voltage': kv, 'sorted': True})

        if align_mode == 'temp' and align_val is not None:
            xlabel = f"time relative to T={align_val:g}°C [s]"
        elif align_mode == 'auto' and align_val is not None:
            ref_name = self._align_ref_name() or "reference"
            xlabel = f"time relative to T={align_val:g}°C (matched to '{ref_name}') [s]"
        elif align_mode == 'current' and align_val is not None:
            av, ap = fmt_si(align_val, 2)
            xlabel = f"time relative to I={av} {ap}A [s]"
        else:
            xlabel = "time [s]"

        if ax_i is not None:
            try:
                from matplotlib.ticker import EngFormatter
                ax_i.yaxis.set_major_formatter(EngFormatter(unit='A'))
                if ax_v is not None:
                    ax_v.yaxis.set_major_formatter(EngFormatter(unit='V'))
            except Exception:
                pass
            if show_i_curve and show_v_curve:
                ax_i.set_ylabel('Keithley current', color=C['dim'], fontsize=9)
                ax_v.set_ylabel('Keithley voltage', color=C['orange'], fontsize=9)
            elif show_v_curve:
                ax_i.set_ylabel('')
                ax_i.tick_params(labelleft=False, left=False)
                ax_v.set_ylabel('Keithley voltage', color=C['orange'], fontsize=9)
            else:
                ax_i.set_ylabel('Keithley current', color=C['dim'], fontsize=9)
                if ax_v is not None: ax_v.set_visible(False)
            ax_i.set_xlabel(xlabel, color=C['dim'], fontsize=9)
            # ukryj etykiety X gornego wykresu - wspolna os czasu na dole
            self.ax_a.tick_params(labelbottom=False)
        else:
            self.ax_a.set_xlabel(xlabel, color=C['dim'], fontsize=9)
            missing = []
            if show_current and not any_current_data: missing.append("current")
            if show_voltage and not any_voltage_data: missing.append("voltage")
            if missing:
                self.ax_a.text(0.02, 0.02, f"no Keithley {'/'.join(missing)} data in the selected cycles",
                               color=C['dim2'], fontsize=8, transform=self.ax_a.transAxes)
        if align_missed:
            names = ", ".join(align_missed[:3]) + ("…" if len(align_missed) > 3 else "")
            self.ax_a.text(0.02, 0.98,
                           f"⚠ threshold not reached: {names} (used natural start)",
                           color=C['yellow'], fontsize=7, transform=self.ax_a.transAxes,
                           va='top')

        self.ax_a.set_ylabel('temperature [°C]', color=C['dim'], fontsize=9)
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
        self.ax_a_voltage = None
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
            self.ax_a.text(0.5, 0.5, "no Keithley data in the selected cycles",
                           ha='center', va='center', color=C['dim2'],
                           fontsize=10, transform=self.ax_a.transAxes)
            self._arch_reset_hover()
            self.cv_a.draw(); return

        try:
            from matplotlib.ticker import EngFormatter
            self.ax_a.yaxis.set_major_formatter(EngFormatter(unit='A'))
        except Exception:
            pass
        self.ax_a.set_xlabel('temperature T1 [°C]', color=C['dim'], fontsize=9)
        self.ax_a.set_ylabel('Keithley current', color=C['dim'], fontsize=9)
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
               defaultextension=".png", initialfile="chart.png",
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
            messagebox.showinfo("Nothing selected", "Tick a cycle (checkbox) on the list to the left, "
                                "or use the ⤓ button directly next to the item.")
            return
        if len(selected) == 1:
            self._export_single_cycle_csv(selected[0])
            return
        from tkinter import filedialog
        dest_dir = filedialog.askdirectory(title="Choose destination folder for CSV")
        if not dest_dir:
            return
        ok = 0
        for src in selected:
            try:
                shutil.copy(src, _P(dest_dir) / src.name)
                ok += 1
            except Exception:
                pass
        messagebox.showinfo("Saved", f"Copied {ok}/{len(selected)} files to:\n{dest_dir}")

    def _export_single_cycle_csv(self, src):
        """Zapisuje jeden konkretny plik cyklu (przycisk ⤓ przy pozycji na liscie -
        dziala od razu, bez zaznaczania checkboxa)."""
        from pathlib import Path as _P
        import shutil
        src = _P(src)
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Save cycle raw data", defaultextension=".csv",
            initialfile=src.name, filetypes=[("CSV", "*.csv")])
        if not dest:
            return
        try:
            shutil.copy(src, dest)
            messagebox.showinfo("Saved", f"Cycle raw data saved to:\n{dest}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _show_excel_help(self):
        """Instrukcja: dlaczego Excel czesto psuje ten CSV przy zwyklym
        dwuklikni?ciu, i jak poprawnie zaimportowac dane (osobne kolumny,
        poprawny separator dziesietny), zeby dalsza obrobka byla latwa."""
        win = tk.Toplevel(self.root)
        win.title("How to open CSV in Excel")
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

        tk.Label(win, text="Importing CSV into Excel", bg=C['bg'], fg=C['text'],
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
            "PROBLEM: our CSV file separates columns with a COMMA, and decimal "
            "numbers use a PERIOD (e.g. 48.30, not 48,30). Some regional Excel "
            "settings (e.g. many European locales) expect the opposite by default: "
            "semicolon as the column separator and comma as the decimal separator. "
            "Because of this, simply double-clicking the CSV file can dump "
            "EVERYTHING into one column or turn numbers into text.\n\n"
            "BEST METHOD (Excel 2016+, always works correctly):\n"
            "1. Open a BLANK Excel workbook (don't double-click the CSV).\n"
            "2. DATA tab -> \"From Text/CSV\" (Get Data From Text/CSV).\n"
            "3. Point it at our cykl_....csv file.\n"
            "4. In the preview window set:\n"
            "     Delimiter: Comma\n"
            "     File origin: UTF-8\n"
            "5. Click \"Transform Data\" or just \"Load\".\n"
            "6. If numbers come out as text (left-aligned), select the numeric "
            "columns -> DATA -> Text to Columns -> Next -> Next -> in step 3 "
            "choose \"Period\" as the decimal separator (Advanced button) "
            "-> Finish.\n\n"
            "QUICKER METHOD (double-click the file, when you don't want to import):\n"
            "1. Open the file with a normal double-click - the data lands in one column.\n"
            "2. Select the entire column A.\n"
            "3. DATA -> Text to Columns.\n"
            "4. Choose \"Delimited\" -> Next.\n"
            "5. Check the \"Comma\" delimiter (uncheck others) -> Next.\n"
            "6. Click \"Advanced\" in the bottom-right corner -> set "
            "\"Decimal separator: period\", \"Thousands separator: (blank)\" -> OK.\n"
            "7. Finish.\n\n"
            "COLUMNS IN THE CYCLE FILE (cykl_*.csv / c_*.csv):\n"
            "  timestamp_pc          - computer timestamp (ISO 8601)\n"
            "  czas_firmware_s       - time from the microcontroller clock [s]\n"
            "  czas_od_startu_s      - time since the cycle started [s] (most convenient for charts)\n"
            "  temperatura1_C        - temperature from sensor 1 [°C]\n"
            "  temperatura2_C        - temperature from sensor 2 [°C]\n"
            "  setpoint_aktywny_C    - current setpoint (during the ramp) [°C]\n"
            "  setpoint_cel_C        - target setpoint [°C]\n"
            "  peltier_pct           - Peltier power [%]\n"
            "  fan_pct               - fan power [%]\n"
            "  kierunek              - HEAT / COOL\n"
            "  Kp, Ki, Kd            - PID controller gains\n"
            "  stan                  - state machine state (RUN / IDLE / etc.)\n"
            "  keithley_prad_A       - current measured by the SMU [A], scientific notation e.g. 4.83e-08\n"
            "  keithley_napiecie_V   - voltage measured by the SMU [V]\n\n"
            "NOTE on scientific notation (e.g. 4.83e-08): Excel correctly reads it as "
            "a number as long as the column format is \"General\" or \"Scientific\" - do NOT format "
            "these columns as \"Text\" before importing, or they'll be turned into strings "
            "and won't work in charts/formulas.\n\n"
            "QUICK CHART IN EXCEL (I vs T or I vs time):\n"
            "1. After a correct import, select two columns, e.g. temperatura1_C and "
            "keithley_prad_A (hold Ctrl to select non-adjacent columns).\n"
            "2. INSERT -> Scatter chart (XY Scatter) -> \"Scatter with Straight "
            "Lines\" if you want to preserve the time order (this shows the "
            "heating/cooling hysteresis loop), or plain \"Scatter\" for just the "
            "point cloud."
        )
        txt.insert('1.0', content)
        txt.config(state='disabled')

        mk_btn(win, "CLOSE", win.destroy, C['cyan']).pack(pady=(0, 14))

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
                heat_dir = d.get('heat', True)

                st_state = d.get('state', '')
                if st_state.startswith('ST'):
                    self._on_selftune_status(st_state, d.get('kp', 0), d.get('ki', 0), d.get('kd', 0))
                elif self.selftune_running and st_state == 'AUTO':
                    # firmware wrocilo do zwyklego stanu AUTO - auto-tune sam
                    # sie zakonczyl (60 cykli = ~2 min) albo ktos go zatrzymal
                    # z innego miejsca; zsynchronizuj przyciski/status w UI.
                    self.selftune_running = False
                    if hasattr(self, 'btn_selftune_start'):
                        self.btn_selftune_start.config(state='normal')
                        self.btn_selftune_stop.config(state='disabled')
                        self.selftune_status_lbl.config(text="Complete.", fg=C['green'])

                if self.t0 is None: self.t0 = tsr
                rel = tsr - self.t0

                self.t.append(rel); self.temp1.append(t1); self.temp2.append(t2)
                self.spt.append(sp); self.spa.append(spa)
                # PWM zapisywany ZE ZNAKIEM: dodatni = grzanie, ujemny =
                # chlodzenie - dzieki temu wykres pokazuje kierunek bez
                # potrzeby osobnego wskaznika HEAT/COOL obok (patrz _draw_chart:
                # dwukolorowe wypelnienie ponad/ponizej zera).
                self.pwm.append(pct if heat_dir else -pct)
                self.fanv.append(fn)

                # Ostatnia znana temp/czas - do odczytu przez watek sweep (korelacja
                # kazdego punktu I-V z temperatura w momencie pomiaru)
                self.last_known_rel = rel
                self.last_known_rel_wallclock_ts = time.time()
                self.last_known_t1 = t1
                self.last_known_t2 = t2
                self.last_known_sp = sp
                self.last_known_spa = spa
                self.last_known_pct = pct
                self.last_known_fn = fn
                self.last_known_heat_dir = heat_dir
                self.last_known_kp = d.get('kp')
                self.last_known_ki = d.get('ki')
                self.last_known_kd = d.get('kd')
                self.last_known_state = d.get('state')

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
                # (RAW DATA, karta na zywo), zeby wykluczyc jakakolwiek
                # niespojnosc miedzy oddzielnymi odczytami w tej samej iteracji.
                k_i, k_v = self._keithley_latest()

                # UWAGA: ten wiersz (tyknienie firmware, co 100ms) NIE
                # zapisuje juz Keithleya (zawsze None) - odpowiedzialnosc za
                # to przejal WLASNY, niezalezny zapis w watku Keithleya (patrz
                # keithley_sweep_start), ktory dopisuje swoj wlasny wiersz
                # PRZY KAZDEJ swojej probce (np. co ~30ms), a nie tylko przy
                # tyknieciach termopar. Dzieki temu ZADNA probka Keithleya nie
                # ginie (wczesniej: przy szybszym pomiarze niz 100ms, tylko
                # ostatnia probka przed danym tykniecem trafiala do pliku,
                # reszta byla bezpowrotnie tracona z tego pliku - mimo ze byla
                # w pelni zapisana w osobnym sweep_*.csv). Teraz oba strumienie
                # (temperatura co 100ms, Keithley co jego wlasne tempo) trafiaja
                # do TEGO SAMEGO pliku cyklu, przeplatane wg rzeczywistego czasu.
                if self.cyc_on:
                    self.cyc_log(rel, t1, t2, sp, pct, fn,
                                 spa=spa, kp=d.get('kp'), ki=d.get('ki'), kd=d.get('kd'),
                                 fw_ts=tsr, state=d.get('state'),
                                 keithley_i=None, keithley_v=None, heat=heat_dir)

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

            # Programator: maszyna stanow krokow (grzej/trzymaj/schlodz/trzymaj)
            # dziala na najswiezszej znanej temperaturze, niezaleznie czy w tym
            # ticku przyszly nowe dane z urzadzenia.
            if self.program_running:
                self._program_tick(self.last_known_t1)

            # Wykres aktualizuje dane zawsze (set_data, tanie) - ochrona przed
            # "wyrywaniem" recznego przyblizenia jest teraz wewnatrz _draw_chart
            # (pomija tam tylko autoscale, gdy aktywne narzedzie ZOOM/PAN).
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
                if 'kffhc' in d and hasattr(self,'sl_kffhc'): self.sl_kffhc.set(float(d['kffhc']))
                if 'kffrc' in d and hasattr(self,'sl_kffrc'): self.sl_kffrc.set(float(d['kffrc']))
                if 'offset' in d and hasattr(self,'sl_off'): self.sl_off.set(float(d['offset']))
                if 'oppdir' in d and hasattr(self,'oppdir_var'): self._set_oppdir(bool(d['oppdir']), send=False)
                self._cfg_synced = True
            # Stan wentylatora ORAZ T2 synchronizowane ZAWSZE (nie tylko raz
            # przy pierwszym polaczeniu) - w przeciwienstwie do Kp/Ki/SP itp.
            # to przelaczniki ktore uzytkownik bedzie zmienial WIELOKROTNIE w
            # trakcie sesji (np. wylacz T2 przed czulym pomiarem Keithleya,
            # wlacz z powrotem po). Jednorazowa synchronizacja zostawialaby
            # checkbox utkniety na stanie sprzed pierwszego polaczenia.
            if 'fan_on' in d and hasattr(self, 'btn_fan'):
                self._sync_fan_button(bool(d['fan_on']))
            if 'tc2_enabled' in d and hasattr(self, 'tc2_enabled_var'):
                self.tc2_enabled_var.set(bool(d['tc2_enabled']))
                if hasattr(self, 'tc2_checkbox_row'): self.tc2_checkbox_row.render()
        except Exception as e: print(f"cfg err: {e}")

    def _sync_fan_button(self, is_on):
        """Aktualizuje WYLACZNIE wyglad przycisku FAN wg stanu raportowanego
        przez firmware (CFG) - nie wysyla zadnej komendy zwrotnej. To jedyne
        wiarygodne zrodlo prawdy o tym czy wentylator faktycznie sie kreci,
        bo firmware moze go wlaczac/wylaczac sam (run-on po STOP, wymuszenie
        podczas chlodzenia), niezaleznie od tego co user ostatnio klikal."""
        if self.fan_on == is_on:
            return  # bez zmian - nie ma po co dotykac widgetu
        self.fan_on = is_on
        if is_on:
            self.btn_fan.config(text="● ON", bg=C['green'], fg='#0f1f14', highlightbackground=C['green'])
        else:
            self.btn_fan.config(text="○ OFF", bg=C['bg2'], fg=C['dim2'], highlightbackground=C['dim'])

    def _draw_chart(self):
        t=self.t; t1=self.temp1; t2=self.temp2; sp=self.spt; spa=self.spa; pw=self.pwm
        if self.chart_window > 0 and len(t)>1:
            cutoff = t[-1]-self.chart_window
            i0 = next((i for i in range(len(t)) if t[i]>=cutoff), 0)
            t=t[i0:]; t1=t1[i0:]; t2=t2[i0:]; sp=sp[i0:]; spa=spa[i0:]; pw=pw[i0:]

        def safe(lst): return [v if v is not None else float('nan') for v in lst]

        # Aktualizacja ISTNIEJACYCH linii (set_data) zamiast ax.clear()+replot -
        # duzo tansze (bez rekonstrukcji legendy/siatki/spines co 250ms) i NIE
        # resetuje aktualnego przyblizenia/przesuniecia widoku uzytkownika.
        self.ln_target.set_data(t, sp)
        self.ln_spa.set_data(t, spa)
        self.ln_t1.set_data(t, safe(t1))
        self.ln_t2.set_data(t, safe(t2))

        # PWM ze znakiem rozbity na dwie maskowane serie - tam gdzie grzanie
        # (dodatnie), krzywa chlodzenia ma NaN (przerwa) i odwrotnie. Dzieki
        # temu kolor przechodzi ostro na zerze bez zadnej dodatkowej logiki.
        pw_heat = [v if (v is not None and v > 0) else float('nan') for v in pw]
        pw_cool = [v if (v is not None and v < 0) else float('nan') for v in pw]
        self.ln_pwm_heat.set_data(t, pw_heat)
        self.ln_pwm_cool.set_data(t, pw_cool)

        # fill_between nie ma set_data - trzeba przebudowac te dwa artysty
        # (wciaz duzo tansze niz pelny ax.clear() calego wykresu)
        for attr in ('pwm_fill_heat', 'pwm_fill_cool'):
            old = getattr(self, attr, None)
            if old is not None:
                try: old.remove()
                except Exception: pass
        self.pwm_fill_heat = self.ax2.fill_between(t, 0, pw_heat, color=C['orange'], alpha=0.3)
        self.pwm_fill_cool = self.ax2.fill_between(t, 0, pw_cool, color=C['cyan'], alpha=0.3)

        # Autoscale TYLKO gdy uzytkownik nie ma aktywnego zoom/pan z toolbara -
        # inaczej co 250ms wyrywalibysmy mu widok z powrotem do pelnego zakresu.
        # UWAGA: toolbar zoom wewnetrznie wywoluje set_xlim/set_ylim, co w
        # matplotlib SAMO wylacza autoscale dla danej osi na stale - trzeba je
        # jawnie wlaczyc ponownie, inaczej po wylaczeniu narzedzia zoom widok
        # zostalby zamrozony na zawsze zamiast wrocic do sledzenia danych.
        if not self._live_toolbar_busy():
            self.ax1.set_autoscale_on(True); self.ax2.set_autoscale_on(True)
            self.ax1.relim(); self.ax1.autoscale_view()
            self.ax2.relim(); self.ax2.autoscale_view(scaley=False)  # Y stale -105..105 (symetryczne wzgledem zera)

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
                row = [
                    pc_ts, fwts, f"{t:.3f}",
                    t1s, t2s,
                    spas, f"{sp:.3f}",
                    f"{pct:.2f}", f"{fn:.2f}", dirs,
                    kps, kis, kds, state or "",
                    kis_a, kvs,
                ]
                # Lock: cyc_log() jest teraz wolane zarowno z watku glownego
                # (co 100ms, przy kazdym tyknieciu firmware) JAK I z watku
                # Keithleya (przy kazdej jego probce, np. co ~30ms - patrz
                # keithley_sweep_start) - bez blokady dwa jednoczesne
                # writerow() z roznych watkow moglyby przeplatac bajty i
                # uszkodzic plik CSV.
                with self.cyc_log_lock:
                    self.cyc_wr.writerow(row)
                    self.cyc_file.flush()
                self.cyc_rows += 1
            except Exception as e:
                # NIE gub bledow po cichu - licz je i pokaz w naglowku statusu,
                # zeby brak danych w CSV nie byl niespodzianka po eksperymencie
                self.cyc_write_errors = getattr(self, 'cyc_write_errors', 0) + 1
                print(f"cyc_log write err #{self.cyc_write_errors}: {e}")
                if self.cyc_write_errors == 1 and hasattr(self, 's_lbl'):
                    try:
                        self.s_lbl.config(text="WARNING: cycle CSV write error!", fg=C['red'])
                    except Exception:
                        pass

    def cyc_stop(self, reason=""):
        if self.cyc_file:
            try: self.cyc_file.close()
            except: pass
        had = self.cyc_on and self.cyc_rows > 0
        tmp = self.cyc_fn
        self.cyc_on=False; self.cyc_file=None; self.cyc_wr=None
        print(f"CYC STOP: {reason} ({self.cyc_rows} samples)")
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
            print(f"Saved: {dest.name}")
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
