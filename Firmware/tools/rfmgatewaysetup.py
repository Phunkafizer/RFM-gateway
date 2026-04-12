import json
import ctypes
import os
import threading
import time
from collections import deque
import requests
import msvcrt
from tkinter import *
from tkinter import ttk
from functools import partial

try:
    import numpy as np
except ImportError:
    np = None

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

#GATEWAY_IP = '4.3.2.1'
GATEWAY_IP = '192.168.178.96'
FSTEP = 32e6 / (1 << 19)

# Frequency bands: [0]=315MHz, [1]=433MHz, [2]=868MHz, [3]=915MHz
FREQ_BANDS = [
    315e6,        # 315 MHz
    433.92e6,     # 433 MHz
    868.3e6,      # 868 MHz
    915e6         # 915 MHz
]

_DLL_NAMES = ['rtlsdr.dll', 'librtlsdr.dll']
PLOT_Y_MIN_DB = 40.0
PLOT_Y_MAX_DB = 120.0
PLOT_SPAN_HZ = 50_000.0
SETTINGS_FILE = 'rfmgatewaysetup.settings.json'


def _find_rtlsdr_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    for folder in [script_dir, os.getcwd()]:
        for name in _DLL_NAMES:
            candidate = os.path.join(folder, name)
            if os.path.exists(candidate):
                return candidate

    for path_entry in os.environ.get('PATH', '').split(os.pathsep):
        if not path_entry:
            continue
        for name in _DLL_NAMES:
            candidate = os.path.join(path_entry, name)
            if os.path.exists(candidate):
                return candidate

    raise FileNotFoundError('Could not find rtlsdr.dll or librtlsdr.dll in current directory or PATH.')


def _load_rtlsdr_library():
    candidate = _find_rtlsdr_path()
    if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(os.path.dirname(candidate))
    dll = ctypes.WinDLL(candidate)

    def _register(name, restype, argtypes):
        func = getattr(dll, name)
        func.restype = restype
        func.argtypes = argtypes

    _register('rtlsdr_open', ctypes.c_int, [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint])
    _register('rtlsdr_close', ctypes.c_int, [ctypes.c_void_p])
    _register('rtlsdr_set_center_freq', ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    _register('rtlsdr_set_sample_rate', ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    _register('rtlsdr_set_tuner_gain_mode', ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    _register('rtlsdr_set_agc_mode', ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    _register('rtlsdr_set_tuner_gain', ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    _register('rtlsdr_reset_buffer', ctypes.c_int, [ctypes.c_void_p])
    _register('rtlsdr_read_sync', ctypes.c_int, [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint, ctypes.POINTER(ctypes.c_int)])
    return dll


class RtlSdr:
    def __init__(self, device_index=0):
        self._dll = _load_rtlsdr_library()
        self._dev = ctypes.c_void_p()
        result = self._dll.rtlsdr_open(ctypes.byref(self._dev), ctypes.c_uint(device_index))
        if result != 0:
            raise OSError(f'rtlsdr_open failed with error code {result}')

    def close(self):
        if self._dev and self._dev.value:
            self._dll.rtlsdr_close(self._dev)
            self._dev = ctypes.c_void_p()

    @property
    def sample_rate(self):
        raise AttributeError('sample_rate is write-only')

    @sample_rate.setter
    def sample_rate(self, value):
        result = self._dll.rtlsdr_set_sample_rate(self._dev, ctypes.c_uint(int(value)))
        if result != 0:
            raise OSError(f'rtlsdr_set_sample_rate failed with error code {result}')

    @property
    def center_freq(self):
        raise AttributeError('center_freq is write-only')

    @center_freq.setter
    def center_freq(self, value):
        result = self._dll.rtlsdr_set_center_freq(self._dev, ctypes.c_uint(int(value)))
        if result != 0:
            raise OSError(f'rtlsdr_set_center_freq failed with error code {result}')

    @property
    def gain(self):
        raise AttributeError('gain is write-only in integrated mode')

    @gain.setter
    def gain(self, value):
        result = self._dll.rtlsdr_set_agc_mode(self._dev, ctypes.c_int(0))
        if result != 0:
            raise OSError(f'rtlsdr_set_agc_mode failed with error code {result}')
        result = self._dll.rtlsdr_set_tuner_gain_mode(self._dev, ctypes.c_int(1))
        if result != 0:
            raise OSError(f'rtlsdr_set_tuner_gain_mode failed with error code {result}')
        raw_gain = int(round(float(value) * 10))
        result = self._dll.rtlsdr_set_tuner_gain(self._dev, ctypes.c_int(raw_gain))
        if result != 0:
            raise OSError(f'rtlsdr_set_tuner_gain failed with error code {result}')

    def read_samples(self, num_samples):
        self._dll.rtlsdr_reset_buffer(self._dev)
        buffer_len = num_samples * 2
        buffer = (ctypes.c_ubyte * buffer_len)()
        n_read = ctypes.c_int()
        result = self._dll.rtlsdr_read_sync(self._dev, buffer, ctypes.c_uint(buffer_len), ctypes.byref(n_read))
        if result != 0:
            raise OSError(f'rtlsdr_read_sync failed with error code {result}')
        raw = bytes(buffer[:n_read.value])
        samples = []
        for i in range(0, len(raw) - 1, 2):
            samples.append(complex(raw[i] - 128, raw[i + 1] - 128))
        return np.asarray(samples, dtype=np.complex64)


def compute_spectrum_db(samples, sample_rate, center_freq):
    window = np.hanning(len(samples))
    spectrum = np.fft.fftshift(np.fft.fft(samples * window))
    power = np.abs(spectrum) ** 2
    power_db = 10.0 * np.log10(power + 1e-12)
    freqs = np.fft.fftshift(np.fft.fftfreq(len(samples), d=1.0 / sample_rate))
    return center_freq + freqs, power_db


def find_peak_frequency(samples, sample_rate, center_freq, expected_offset_hz=None, search_span_hz=None):
    window = np.hanning(len(samples))
    spectrum = np.fft.fftshift(np.fft.fft(samples * window))
    power = np.abs(spectrum) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(len(samples), d=1.0 / sample_rate))

    if expected_offset_hz is not None and search_span_hz is not None and search_span_hz > 0:
        half_span = search_span_hz / 2.0
        mask = (freqs >= expected_offset_hz - half_span) & (freqs <= expected_offset_hz + half_span)
        if np.any(mask):
            candidate_indices = np.flatnonzero(mask)
            peak_index = int(candidate_indices[np.argmax(power[mask])])
        else:
            peak_index = int(np.argmax(power))
    else:
        peak_index = int(np.argmax(power))

    peak_offset = freqs[peak_index]
    peak_power = power[peak_index]
    return center_freq + peak_offset, peak_offset, peak_power


def mix_down(samples, sample_rate, shift_hz):
    n = np.arange(len(samples))
    osc = np.exp(-1j * 2 * np.pi * shift_hz * n / sample_rate)
    return samples * osc


def _fir_bandpass_coeffs(taps, lowcut_hz, highcut_hz, sample_rate):
    nyq = sample_rate / 2.0
    low = float(lowcut_hz) / nyq
    high = float(highcut_hz) / nyq
    if low <= 0.0 or high >= 1.0 or low >= high:
        raise ValueError('lowcut_hz and highcut_hz must define a valid passband')
    m = taps - 1
    n = np.arange(taps) - m / 2.0
    h_low = np.sinc(low * n)
    h_high = np.sinc(high * n)
    h = h_high - h_low
    window = np.hamming(taps)
    h *= window
    h /= np.sum(h)
    return h


def bandpass(samples, lowcut_hz, highcut_hz, sample_rate, taps=129):
    coeffs = _fir_bandpass_coeffs(taps, lowcut_hz, highcut_hz, sample_rate)
    return np.convolve(samples, coeffs, mode='same')


def estimate_tone_frequency(samples, sample_rate):
    window = np.hanning(len(samples))
    spectrum = np.fft.fftshift(np.fft.fft(samples * window))
    freqs = np.fft.fftshift(np.fft.fftfreq(len(samples), d=1.0 / sample_rate))
    peak_idx = np.argmax(np.abs(spectrum) ** 2)
    return freqs[peak_idx]


def measure_remote_carrier(
    samples,
    sample_rate,
    center_freq,
    target_offset_hz=455000.0,
    tone_band_hz=4000.0,
    expected_peak_offset_hz=0.0,
    peak_search_span_hz=30000.0,
):
    peak_freq, peak_offset, peak_power = find_peak_frequency(
        samples,
        sample_rate,
        center_freq,
        expected_offset_hz=expected_peak_offset_hz,
        search_span_hz=peak_search_span_hz,
    )
    shift_hz = -(peak_offset - target_offset_hz)
    mixed = mix_down(samples, sample_rate, shift_hz)
    half_bw = tone_band_hz / 2.0
    filtered = bandpass(
        mixed,
        target_offset_hz - half_bw,
        target_offset_hz + half_bw,
        sample_rate,
    )
    tone_hz = estimate_tone_frequency(filtered, sample_rate)
    return peak_freq, peak_offset, peak_power, tone_hz


class IntegratedSdrRunner:
    def __init__(self, center_freq):
        self.center_freq = center_freq
        self._thread = None
        self._stop_event = threading.Event()
        self._measurements = deque(maxlen=2048)
        self._measurements_lock = threading.Lock()
        self._plot_lock = threading.Lock()
        self._latest_freq_mhz = None
        self._latest_power_db = None
        self._latest_offset_now_khz = None
        self.running = False

    def get_measurements_after(self, timestamp):
        with self._measurements_lock:
            return [(ts, val) for ts, val in self._measurements if ts > timestamp]

    def get_latest_plot_data(self):
        with self._plot_lock:
            return self._latest_freq_mhz, self._latest_power_db, self._latest_offset_now_khz

    def start(self):
        if np is None or plt is None:
            print('SDR unavailable: please install numpy and matplotlib to enable spectrum view.')
            return False
        if self.running:
            return True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
        self.running = False

    def _run(self):
        self.running = True
        sdr = None
        try:
            sdr = RtlSdr()
            sr = 2.048e6
            sdr.sample_rate = sr
            sdr.center_freq = self.center_freq
            sdr.gain = 49.6

            read_size = 16384
            peak_search_span_hz = 300000.0
            power_threshold = 2e8
            step_hz = 32e6 / (1 << 19)
            expected_peak_offset_hz = 0.0
            last_offset_now_khz = None

            while not self._stop_event.is_set():
                now = time.monotonic()

                x = sdr.read_samples(read_size)

                freq_hz, power_db = compute_spectrum_db(x, sr, self.center_freq)
                half_plot_span_hz = PLOT_SPAN_HZ / 2.0
                span_mask = (freq_hz >= (self.center_freq - half_plot_span_hz)) & (freq_hz <= (self.center_freq + half_plot_span_hz))
                with self._plot_lock:
                    self._latest_freq_mhz = freq_hz[span_mask] / 1e6
                    self._latest_power_db = power_db[span_mask]
                    self._latest_offset_now_khz = last_offset_now_khz

                _, peak_offset, peak_power, tone_hz = measure_remote_carrier(
                    x,
                    sr,
                    self.center_freq,
                    target_offset_hz=455000.0,
                    tone_band_hz=4000.0,
                    expected_peak_offset_hz=expected_peak_offset_hz,
                    peak_search_span_hz=peak_search_span_hz,
                )
                if peak_power >= power_threshold:
                    expected_peak_offset_hz = peak_offset
                    if_error_hz = abs(tone_hz) - 455000.0
                    with self._measurements_lock:
                        self._measurements.append((now, if_error_hz))
                    last_offset_now_khz = if_error_hz / 1e3
                    offset_now_steps = int(round(if_error_hz / step_hz))
                    print(
                        f'IF {tone_hz / 1e3:+8.2f} kHz | '
                        f'error_now {int(round(if_error_hz)):+6d} Hz | '
                        f'steps {offset_now_steps:+5d} | '
                        f'offset_now {if_error_hz / 1e3:+6.1f} kHz'
                    )
        except Exception as exc:
            print(f'SDR unavailable: {exc}')
        finally:
            self.running = False
            if sdr is not None:
                try:
                    sdr.close()
                except Exception:
                    pass

class RFMTestApp:
    def __init__(self):
        self.ip = GATEWAY_IP
        self.sdr_runner = None
        self.sdr_fig = None
        self.sdr_ax = None
        self.sdr_line = None
        self.sdr_offset_text = None
        self.sdr_plot_after_id = None
        self.auto_calib_thread = None
        self.auto_calib_stop_event = threading.Event()
        self.root = Tk()
        self.root.title('RFM Gateway setup tool')
        self.root.geometry('640x720')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.setup_frame = ttk.LabelFrame(self.root, text='Setup / calibration', padding=10)
        self.setup_frame.pack(fill='x', padx=10, pady=8)

        ttk.Label(self.setup_frame, text='RFM type').grid(row=0, column=0, sticky='w')
        self.rfmtype = ttk.Combobox(self.setup_frame)
        self.rfmtype['values'] = ('RFM69CW', 'RFM69HCW')
        self.rfmtype.current(0)
        self.rfmtype.grid(row=1, column=0, padx=4, pady=2, sticky='w')
        self.rfmtype.bind('<<ComboboxSelected>>', self.update_txpower_for_rfmtype)

        ttk.Label(self.setup_frame, text='Freq band').grid(row=0, column=1, sticky='w')
        self.freqband = ttk.Combobox(self.setup_frame)
        self.freqband['values'] = ('315 MHz', '433 MHz', '868 MHz', '915 MHz')
        self.freqband.current(1)
        self.freqband.grid(row=1, column=1, padx=4, pady=2, sticky='w')

        ttk.Label(self.setup_frame, text='Freq correction').grid(row=2, column=0, columnspan=2, sticky='w')
        self.fcorr = Scale(self.setup_frame, from_=-500, to=500, orient=HORIZONTAL, length=460, command=self.update_fcorr_offset)
        self.fcorr.set(70)
        self.fcorr.grid(row=3, column=0, columnspan=2, padx=4, pady=2, sticky='w')
        self.fcorr.bind('<ButtonRelease-1>', self.trigger_tx_test_from_slider)
        self.fcorr.bind('<KeyRelease-Left>', self.trigger_tx_test_from_slider)
        self.fcorr.bind('<KeyRelease-Right>', self.trigger_tx_test_from_slider)
        self.fcorr_offset_label = ttk.Label(self.setup_frame, text=f'real offset: {(FSTEP * 70) / 1000:.2f} kHz')
        self.fcorr_offset_label.grid(row=4, column=0, columnspan=2, sticky='w', padx=4, pady=(0,6))

        ttk.Label(self.setup_frame, text='TX power (dBm)').grid(row=5, column=0, columnspan=2, sticky='w')
        self.txpower = Scale(self.setup_frame, from_=-18, to=20, orient=HORIZONTAL, length=460)
        self.txpower.grid(row=6, column=0, columnspan=2, padx=4, pady=2, sticky='w')
        self.update_txpower_for_rfmtype()

        self.sdrbtn = ttk.Button(self.setup_frame, text="Start SDR", width=12, command=self.start_sdr_with_freq)
        self.sdrbtn.grid(row=7, column=0, padx=4, pady=6, sticky='w')

        self.txtestbtn = ttk.Button(self.setup_frame, text="TX test", width=12, command=self.txtest)
        self.txtestbtn.grid(row=7, column=1, padx=4, pady=6, sticky='w')

        self.savebtn = ttk.Button(self.setup_frame, text="save radio setup", width=12, command=self.saveradiosetup)
        self.savebtn.grid(row=7, column=2, padx=4, pady=6, sticky='w')

        self.autocalibbtn = ttk.Button(self.setup_frame, text="Auto Calib", width=12, command=self.toggle_auto_calib)
        self.autocalibbtn.grid(row=8, column=0, padx=4, pady=6, sticky='w')

        self.configbtn = ttk.Button(self.root, text="send default config", width=18, command=self.saveconfig)
        self.configbtn.pack(padx=10, pady=4)

        self.ittristate_frame = ttk.LabelFrame(self.root, text='Tristate', padding=10)
        self.ittristate_frame.pack(fill='x', padx=10, pady=8)

        ttk.Label(self.ittristate_frame, text='House').grid(row=0, column=0, sticky='w')
        self.ittristate_house = ttk.Combobox(self.ittristate_frame, values=('A', 'B', 'C', 'D'), width=5)
        self.ittristate_house.current(2)
        self.ittristate_house.grid(row=1, column=0, padx=4, pady=2)

        ttk.Label(self.ittristate_frame, text='Group').grid(row=0, column=1, sticky='w')
        self.ittristate_group = ttk.Combobox(self.ittristate_frame, values=('1', '2', '3', '4'), width=5)
        self.ittristate_group.current(2)
        self.ittristate_group.grid(row=1, column=1, padx=4, pady=2)

        ttk.Label(self.ittristate_frame, text='Channel').grid(row=0, column=2, sticky='w')
        self.ittristate_channel = ttk.Combobox(self.ittristate_frame, values=('1', '2', '3', '4'), width=5)
        self.ittristate_channel.current(0)
        self.ittristate_channel.grid(row=1, column=2, padx=4, pady=2)

        self.ittristate_on = ttk.Button(self.ittristate_frame, text='ON', command=self.send_ittristate_on)
        self.ittristate_on.grid(row=1, column=3, padx=10)
        self.ittristate_off = ttk.Button(self.ittristate_frame, text='OFF', command=self.send_ittristate_off)
        self.ittristate_off.grid(row=1, column=4, padx=10)

        self.it32_frame = ttk.LabelFrame(self.root, text='Intertechno 32', padding=10)
        self.it32_frame.pack(fill='x', padx=10, pady=8)

        ttk.Label(self.it32_frame, text='ID').grid(row=0, column=0, sticky='w')
        self.it32_id = ttk.Entry(self.it32_frame, width=14)
        self.it32_id.insert(0, '25221242')
        self.it32_id.grid(row=1, column=0, padx=4, pady=2)

        ttk.Label(self.it32_frame, text='Channel').grid(row=0, column=1, sticky='w')
        self.it32_channel = ttk.Combobox(self.it32_frame, values=('1', '2', '3', '4'), width=5)
        self.it32_channel.current(0)
        self.it32_channel.grid(row=1, column=1, padx=4, pady=2)

        self.it32_on = ttk.Button(self.it32_frame, text='ON', command=self.send_intertechno32_on)
        self.it32_on.grid(row=1, column=2, padx=10)
        self.it32_off = ttk.Button(self.it32_frame, text='OFF', command=self.send_intertechno32_off)
        self.it32_off.grid(row=1, column=3, padx=10)

        self.emylo_frame = ttk.LabelFrame(self.root, text='Emylo', padding=10)
        self.emylo_frame.pack(fill='x', padx=10, pady=8)

        ttk.Label(self.emylo_frame, text='ID').grid(row=0, column=0, sticky='w')
        self.emylo_id = ttk.Entry(self.emylo_frame, width=14)
        self.emylo_id.insert(0, '12345')
        self.emylo_id.grid(row=1, column=0, padx=4, pady=2)

        self.emylo_a = ttk.Button(self.emylo_frame, text='A', width=6, command=lambda: self.send_emylo('A'))
        self.emylo_a.grid(row=1, column=1, padx=4)
        self.emylo_b = ttk.Button(self.emylo_frame, text='B', width=6, command=lambda: self.send_emylo('B'))
        self.emylo_b.grid(row=1, column=2, padx=4)
        self.emylo_c = ttk.Button(self.emylo_frame, text='C', width=6, command=lambda: self.send_emylo('C'))
        self.emylo_c.grid(row=1, column=3, padx=4)
        self.emylo_d = ttk.Button(self.emylo_frame, text='D', width=6, command=lambda: self.send_emylo('D'))
        self.emylo_d.grid(row=1, column=4, padx=4)

        btns = []

        for btnrow in btns:
            buttonframe = Frame(self.root)
            col = 0
    
            for key, value in btnrow.items():
                btn = ttk.Button(buttonframe, text=key, command=partial(self.send, value))
                btn.grid(row=0, column=col)
                col += 1
            
            buttonframe.pack()

        self.load_settings()

        self.root.mainloop()

    def _settings_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), SETTINGS_FILE)

    def save_settings(self):
        settings = {
            'geometry': self.root.geometry(),
            'rfmtype': self.rfmtype.current(),
            'freqband': self.freqband.current(),
            'fcorr': int(self.fcorr.get()),
            'txpower': int(self.txpower.get()),
            'ittristate_house': self.ittristate_house.current(),
            'ittristate_group': self.ittristate_group.current(),
            'ittristate_channel': self.ittristate_channel.current(),
            'it32_id': self.it32_id.get(),
            'it32_channel': self.it32_channel.current(),
            'emylo_id': self.emylo_id.get(),
        }
        try:
            with open(self._settings_path(), 'w', encoding='utf-8') as f:
                json.dump(settings, f, indent=2)
        except Exception as exc:
            print(f'Could not save settings: {exc}')

    def load_settings(self):
        path = self._settings_path()
        if not os.path.exists(path):
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                settings = json.load(f)
        except Exception as exc:
            print(f'Could not load settings: {exc}')
            return

        def _safe_set_combo(combo, idx):
            if isinstance(idx, int) and 0 <= idx < len(combo['values']):
                combo.current(idx)

        geometry = settings.get('geometry')
        if isinstance(geometry, str) and geometry:
            self.root.geometry(geometry)

        _safe_set_combo(self.rfmtype, settings.get('rfmtype'))
        _safe_set_combo(self.freqband, settings.get('freqband'))

        fcorr = settings.get('fcorr')
        if isinstance(fcorr, (int, float)):
            self.fcorr.set(int(fcorr))
            self.update_fcorr_offset(self.fcorr.get())

        txpower = settings.get('txpower')
        if isinstance(txpower, (int, float)):
            self.txpower.set(int(txpower))

        _safe_set_combo(self.ittristate_house, settings.get('ittristate_house'))
        _safe_set_combo(self.ittristate_group, settings.get('ittristate_group'))
        _safe_set_combo(self.ittristate_channel, settings.get('ittristate_channel'))

        it32_id = settings.get('it32_id')
        if isinstance(it32_id, str):
            self.it32_id.delete(0, END)
            self.it32_id.insert(0, it32_id)

        _safe_set_combo(self.it32_channel, settings.get('it32_channel'))

        emylo_id = settings.get('emylo_id')
        if isinstance(emylo_id, str):
            self.emylo_id.delete(0, END)
            self.emylo_id.insert(0, emylo_id)

    def start_sdr_with_freq(self):
        freq_band = self.freqband.current()
        if freq_band < len(FREQ_BANDS):
            center_freq = FREQ_BANDS[freq_band]
        else:
            center_freq = FREQ_BANDS[1]  # Default to 433 MHz

        if self.sdr_runner is not None and self.sdr_runner.running:
            self.sdr_runner.stop()
            self._stop_sdr_plot()
            self.sdrbtn.config(text='Start SDR')
            print('Stopped SDR')
            return

        self.sdr_runner = IntegratedSdrRunner(center_freq)
        if self.sdr_runner.start():
            self._start_sdr_plot(center_freq)
            self.sdrbtn.config(text='Stop SDR')
            print(f'Started SDR at {center_freq / 1e6:.2f} MHz')
        else:
            self.sdr_runner = None

    def _start_sdr_plot(self, center_freq):
        if plt is None:
            return
        self._stop_sdr_plot()
        plt.ion()
        self.sdr_fig, self.sdr_ax = plt.subplots(figsize=(10, 5))
        self.sdr_line, = self.sdr_ax.plot([], [], lw=1.0)
        self.sdr_offset_text = self.sdr_ax.text(
            0.02,
            0.96,
            'offset_now: n/a',
            transform=self.sdr_ax.transAxes,
            va='top',
            ha='left',
            fontsize=10,
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
        )
        self.sdr_ax.set_title('Live Spectrum')
        self.sdr_ax.set_xlabel('Frequency (MHz)')
        self.sdr_ax.set_ylabel('Power (dB)')
        half_plot_span_mhz = (PLOT_SPAN_HZ / 2.0) / 1e6
        self.sdr_ax.set_xlim((center_freq / 1e6) - half_plot_span_mhz, (center_freq / 1e6) + half_plot_span_mhz)
        self.sdr_ax.set_ylim(PLOT_Y_MIN_DB, PLOT_Y_MAX_DB)
        self.sdr_ax.grid(True, alpha=0.3)
        self.sdr_fig.tight_layout()
        self._update_sdr_plot()

    def _update_sdr_plot(self):
        if self.sdr_runner is None or not self.sdr_runner.running:
            return
        if self.sdr_fig is None or not plt.fignum_exists(self.sdr_fig.number):
            return

        freq_mhz, power_db, offset_now_khz = self.sdr_runner.get_latest_plot_data()
        if freq_mhz is not None and power_db is not None and len(freq_mhz) > 0:
            self.sdr_line.set_data(freq_mhz, power_db)

        if offset_now_khz is None:
            self.sdr_offset_text.set_text('offset_now: n/a')
        else:
            self.sdr_offset_text.set_text(f'offset_now: {offset_now_khz:+.1f} kHz')

        self.sdr_fig.canvas.draw_idle()
        self.sdr_fig.canvas.flush_events()
        self.sdr_plot_after_id = self.root.after(100, self._update_sdr_plot)

    def _stop_sdr_plot(self):
        if self.sdr_plot_after_id is not None:
            try:
                self.root.after_cancel(self.sdr_plot_after_id)
            except Exception:
                pass
            self.sdr_plot_after_id = None
        if self.sdr_fig is not None:
            try:
                plt.close(self.sdr_fig)
            except Exception:
                pass
        self.sdr_fig = None
        self.sdr_ax = None
        self.sdr_line = None
        self.sdr_offset_text = None

    def on_close(self):
        self.auto_calib_stop_event.set()
        if self.auto_calib_thread is not None and self.auto_calib_thread.is_alive():
            self.auto_calib_thread.join(timeout=1.5)
        self.save_settings()
        if self.sdr_runner is not None and self.sdr_runner.running:
            self.sdr_runner.stop()
        self._stop_sdr_plot()
        self.root.destroy()

    def update_fcorr_offset(self, value=None):
        try:
            fcorr = int(float(value)) if value is not None else int(self.fcorr.get())
        except ValueError:
            fcorr = 0
        self.fcorr_offset_label.config(text=f'real offset: {(fcorr * FSTEP) / 1000:.2f} kHz')

    def update_txpower_for_rfmtype(self, event=None):
        if self.rfmtype.current() == 0:
            self.txpower.set(-10)
        else:
            self.txpower.set(3)

    def trigger_tx_test_from_slider(self, event=None):
        if hasattr(self, 'txtestbtn'):
            self.txtestbtn.invoke()

    def _set_fcorr_threadsafe(self, value):
        done = threading.Event()

        def _apply():
            self.fcorr.set(int(value))
            self.update_fcorr_offset(self.fcorr.get())
            done.set()

        self.root.after(0, _apply)
        done.wait(timeout=1.0)

    def _get_fcorr_threadsafe(self):
        result = {'value': int(self.fcorr.get())}
        done = threading.Event()

        def _read():
            result['value'] = int(self.fcorr.get())
            done.set()

        self.root.after(0, _read)
        done.wait(timeout=1.0)
        return result['value']

    def _wait_for_next_offset_measurement(self, last_ts, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self.auto_calib_stop_event.is_set():
            if self.sdr_runner is None or not self.sdr_runner.running:
                return None
            measurements = self.sdr_runner.get_measurements_after(last_ts)
            if measurements:
                # Use the newest available sample to avoid acting on stale backlog data.
                return measurements[-1]
            time.sleep(0.05)
        return None

    def _set_autocalib_button(self, text):
        self.root.after(0, lambda: self.autocalibbtn.config(text=text))

    def toggle_auto_calib(self):
        if self.auto_calib_thread is not None and self.auto_calib_thread.is_alive():
            self.auto_calib_stop_event.set()
            self._set_autocalib_button('Stopping...')
            print('Auto calib: stopping...')
            return

        if self.sdr_runner is None or not self.sdr_runner.running:
            print('Auto calib: start SDR first.')
            return

        self.auto_calib_stop_event.clear()
        self._set_autocalib_button('Stop Calib')
        self.auto_calib_thread = threading.Thread(target=self._auto_calib_worker, daemon=True)
        self.auto_calib_thread.start()

    def _auto_calib_worker(self):
        print('Auto calib: started')
        last_ts = time.monotonic()
        iterations = 0
        max_iterations = 600
        kp_hz_per_step = 600.0
        max_step = 20
        deadband_hz = 250.0

        try:
            while not self.auto_calib_stop_event.is_set() and iterations < max_iterations:
                samples = []
                for _ in range(10):
                    if self.auto_calib_stop_event.is_set():
                        break
                    self.txtest()
                    measurement = self._wait_for_next_offset_measurement(last_ts, timeout_s=2.5)
                    if measurement is None:
                        print('Auto calib: no SDR measurements received.')
                        return
                    last_ts, if_error_hz = measurement
                    samples.append(if_error_hz)
                    time.sleep(0.05)

                if self.auto_calib_stop_event.is_set():
                    break

                if not samples:
                    print('Auto calib: no samples collected.')
                    return

                avg_if_error_hz = sum(samples) / len(samples)
                avg_rounded_hz = int(round(avg_if_error_hz))
                current_fcorr = self._get_fcorr_threadsafe()

                print(
                    f'Auto calib: avg offset {avg_if_error_hz / 1e3:+.2f} kHz '
                    f'({avg_rounded_hz:+d} Hz) with fCorr {current_fcorr:+d}'
                )

                if abs(avg_if_error_hz) <= deadband_hz:
                    print(
                        f'Auto calib: done (within deadband +/-{int(deadband_hz)} Hz, avg={avg_rounded_hz:+d} Hz).'
                    )
                    return

                step = int(round(abs(avg_if_error_hz) / kp_hz_per_step))
                step = max(1, min(max_step, step))

                if avg_if_error_hz > 0:
                    new_fcorr = current_fcorr + step
                elif avg_if_error_hz < 0:
                    new_fcorr = current_fcorr - step
                else:
                    new_fcorr = current_fcorr

                new_fcorr = max(-500, min(500, new_fcorr))
                if new_fcorr == current_fcorr:
                    print('Auto calib: fCorr limit reached.')
                    return

                self._set_fcorr_threadsafe(new_fcorr)
                print(
                    f'Auto calib: apply step {new_fcorr - current_fcorr:+d} -> fCorr {new_fcorr:+d}'
                )
                iterations += 1

            if iterations >= max_iterations:
                print('Auto calib: stopped after max iterations.')
        finally:
            self._set_autocalib_button('Auto Calib')

    def txtest(self):
        freq_band = self.freqband.current()
        if freq_band < len(FREQ_BANDS):
            freq = int(FREQ_BANDS[freq_band])
        else:
            freq = int(FREQ_BANDS[1])  # Default to 433 MHz

        data = {
            "rfmType": self.rfmtype.current(),
            "freq": freq,
            "fCorr": self.fcorr.get(),
            "pwr": int(self.txpower.get()),
            "baud": 15000
        }
        s = json.dumps(data)
        r = requests.post(f'http://{GATEWAY_IP}/txtest', data=s)
        print(f'TX test {s}: {r.status_code} {r.text}')

    def sendon(self):
        url = 'http://' + self.ip + '/send/intertechno/25221242/4/on'
        r = requests.get(url)
        print(f'Send ON {url}: {r.status_code} {r.text}')

    def sendoff(self):
        url = 'http://' + self.ip + '/send/intertechno/25221242/4/off'
        r = requests.get(url)
        print(f'Send OFF {url}: {r.status_code} {r.text}')

    def send_ittristate_on(self):
        house = self.ittristate_house.get().lower()
        group = self.ittristate_group.get()
        channel = self.ittristate_channel.get()
        url = f'http://{self.ip}/send/ittristate/{house}/{group}/{channel}/on'
        r = requests.get(url)
        print(f'Send Ittristate ON {url}: {r.status_code} {r.text}')

    def send_ittristate_off(self):
        house = self.ittristate_house.get().lower()
        group = self.ittristate_group.get()
        channel = self.ittristate_channel.get()
        url = f'http://{self.ip}/send/ittristate/{house}/{group}/{channel}/off'
        r = requests.get(url)
        print(f'Send Ittristate OFF {url}: {r.status_code} {r.text}')

    def send_intertechno32_on(self):
        it32_id = self.it32_id.get().strip()
        channel = self.it32_channel.get()
        url = f'http://{self.ip}/send/intertechno/{it32_id}/{channel}/on'
        r = requests.get(url)
        print(f'Send Intertechno 32 ON {url}: {r.status_code} {r.text}')

    def send_intertechno32_off(self):
        it32_id = self.it32_id.get().strip()
        channel = self.it32_channel.get()
        url = f'http://{self.ip}/send/intertechno/{it32_id}/{channel}/off'
        r = requests.get(url)
        print(f'Send Intertechno 32 OFF {url}: {r.status_code} {r.text}')

    def send_emylo(self, letter):
        emylo_id = self.emylo_id.get().strip()
        url = f'http://{self.ip}/send/emylo/{emylo_id}/{letter}'
        r = requests.get(url)
        print(f'Send Emylo {letter} {url}: {r.status_code} {r.text}')
    
    def saveradiosetup(self):
        data = {
            "radio": {
                "rfmType": self.rfmtype.current(),
                "freqBand": self.freqband.current(),
                "fCorr": self.fcorr.get()
            }
        }
        s = json.dumps(data)
        url = 'http://' + self.ip + '/config'
        r = requests.post(url, data=s)
        print(f'Save radio setup {s}: {r.status_code} {r.text}')

    def saveconfig(self):
        if self.freqband.current() == 1:
            config = {
                "config": {
                    "mqtt": {
                        "host": "",
                        "port": None,
                        "tls": False,
                        "user": "",
                        "pass": "",
                        "basetopic": ""
                    },
                    "application": 0,
                    "txPwr": 13,
                    "rxThresh": -85,
                    "appSettings": {
                        "codecs": [0, 1, 2, 3, 4, 5]
                    }
                }
            }
        elif self.freqband.current() == 2:
            config = {
                "config": {
                    "mqtt": {
                        "host": "",
                        "port": None,
                        "tls": False,
                        "user": "",
                        "pass": "",
                        "basetopic": ""
                    },
                    "application": 1,
                    "txPwr": 13,
                    "rxThresh": -85,
                    "appSettings": {
                        "rxmodes": 3,
                        "interval": 15
                    }
                }
            }
        
        c = json.dumps(config)
        url = 'http://' + self.ip + '/config'
        r = requests.post(url, data=c)
        print(f'Save config {c}: {r.status_code} {r.text}')

    def send(self, param):
        url = F"http://{self.ip}/send/{param}"
        r = requests.get(url)
        print(f'Send {url}: {r.status_code} {r.text}')

if __name__ == "__main__":
    app = RFMTestApp()