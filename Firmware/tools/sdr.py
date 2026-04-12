import argparse
import ctypes
import os
import pathlib
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

# Frequency bands: [0]=315MHz, [1]=433MHz, [2]=868MHz, [3]=915MHz
FREQ_BANDS = [
    315e6,        # 315 MHz
    433.92e6,     # 433 MHz
    868.3e6,      # 868 MHz
    915e6         # 915 MHz
]
DEFAULT_FREQ = FREQ_BANDS[1]  # 433 MHz

_DLL_NAMES = ['rtlsdr.dll', 'librtlsdr.dll']


def _find_rtlsdr_path():
    script_dir = pathlib.Path(os.path.abspath(__file__)).resolve().parent
    cwd_dir = pathlib.Path.cwd().resolve()

    for folder in [script_dir, cwd_dir]:
        for name in _DLL_NAMES:
            candidate = folder / name
            if candidate.exists():
                return candidate

    for path_entry in os.environ.get('PATH', '').split(os.pathsep):
        if not path_entry:
            continue
        folder = pathlib.Path(path_entry).resolve()
        for name in _DLL_NAMES:
            candidate = folder / name
            if candidate.exists():
                return candidate

    raise FileNotFoundError(
        'Could not find rtlsdr.dll or librtlsdr.dll in the current directory or PATH.'
    )


def _load_rtlsdr_library():
    candidate = _find_rtlsdr_path()
    if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(str(candidate.parent))
    dll = ctypes.WinDLL(str(candidate))

    def _register(name, restype, argtypes):
        func = getattr(dll, name)
        func.restype = restype
        func.argtypes = argtypes
        return func

    _register('rtlsdr_get_device_count', ctypes.c_uint, [])
    _register('rtlsdr_get_device_name', ctypes.c_char_p, [ctypes.c_uint])
    _register('rtlsdr_open', ctypes.c_int, [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint])
    _register('rtlsdr_close', ctypes.c_int, [ctypes.c_void_p])
    _register('rtlsdr_set_center_freq', ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    _register('rtlsdr_set_sample_rate', ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    _register('rtlsdr_set_freq_correction', ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
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
        self._gain_value = None
        self._agc_enabled = None
        result = self._dll.rtlsdr_open(ctypes.byref(self._dev), ctypes.c_uint(device_index))
        if result != 0:
            raise OSError(f'rtlsdr_open failed with error code {result}')

    def close(self):
        if self._dev and self._dev.value:
            self._dll.rtlsdr_close(self._dev)
            self._dev = ctypes.c_void_p()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def _ensure_open(self):
        if not self._dev or not self._dev.value:
            raise ValueError('RTL-SDR device is not open')

    @property
    def sample_rate(self):
        raise AttributeError('sample_rate is write-only')

    @sample_rate.setter
    def sample_rate(self, value):
        self._ensure_open()
        result = self._dll.rtlsdr_set_sample_rate(self._dev, ctypes.c_uint(int(value)))
        if result != 0:
            raise OSError(f'rtlsdr_set_sample_rate failed with error code {result}')

    @property
    def center_freq(self):
        raise AttributeError('center_freq is write-only')

    @center_freq.setter
    def center_freq(self, value):
        self._ensure_open()
        result = self._dll.rtlsdr_set_center_freq(self._dev, ctypes.c_uint(int(value)))
        if result != 0:
            raise OSError(f'rtlsdr_set_center_freq failed with error code {result}')

    @property
    def freq_correction(self):
        raise AttributeError('freq_correction is write-only')

    @freq_correction.setter
    def freq_correction(self, value):
        self._ensure_open()
        result = self._dll.rtlsdr_set_freq_correction(self._dev, ctypes.c_int(int(value)))
        if result != 0:
            raise OSError(f'rtlsdr_set_freq_correction failed with error code {result}')

    @property
    def gain(self):
        if self._gain_value is None:
            raise AttributeError('gain has not been set yet')
        return 'auto' if self._agc_enabled else self._gain_value

    @gain.setter
    def gain(self, value):
        self._ensure_open()
        if isinstance(value, str) and value.lower() == 'auto':
            result = self._dll.rtlsdr_set_tuner_gain_mode(self._dev, ctypes.c_int(0))
            if result != 0:
                raise OSError(f'rtlsdr_set_tuner_gain_mode failed with error code {result}')
            result = self._dll.rtlsdr_set_agc_mode(self._dev, ctypes.c_int(1))
            if result != 0:
                raise OSError(f'rtlsdr_set_agc_mode failed with error code {result}')
            self._gain_value = None
            self._agc_enabled = True
        else:
            result = self._dll.rtlsdr_set_agc_mode(self._dev, ctypes.c_int(0))
            if result != 0:
                raise OSError(f'rtlsdr_set_agc_mode failed with error code {result}')
            result = self._dll.rtlsdr_set_tuner_gain_mode(self._dev, ctypes.c_int(1))
            if result != 0:
                raise OSError(f'rtlsdr_set_tuner_gain_mode failed with error code {result}')
            gain_value = float(value)
            raw_gain = int(round(gain_value * 10))
            result = self._dll.rtlsdr_set_tuner_gain(self._dev, ctypes.c_int(raw_gain))
            if result != 0:
                raise OSError(f'rtlsdr_set_tuner_gain failed with error code {result}')
            self._gain_value = gain_value
            self._agc_enabled = False

    @property
    def valid_gains_db(self):
        return [
            0.0, 1.7, 3.4, 5.2, 7.0, 8.7, 10.4, 11.7, 12.8, 14.0,
            15.3, 16.6, 17.8, 19.1, 20.4, 21.7, 23.0, 24.4, 25.8,
            27.2, 28.7, 30.2, 31.8, 33.4, 35.0, 36.7, 38.4, 40.2,
            42.0, 43.8, 45.7, 47.6, 49.6,
        ]

    def read_samples(self, num_samples):
        self._ensure_open()
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


def find_peak_frequency(
    samples,
    sample_rate,
    center_freq,
    ignore_center=False,
    expected_offset_hz=None,
    search_span_hz=None,
):
    window = np.hanning(len(samples))
    spectrum = np.fft.fftshift(np.fft.fft(samples * window))
    power = np.abs(spectrum) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(len(samples), d=1.0 / sample_rate))
    if ignore_center:
        center_index = len(power) // 2
        power[center_index] = 0

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

    # Parabolic interpolation improves frequency precision below one FFT bin.
    peak_offset = freqs[peak_index]
    peak_power = power[peak_index]
    if 0 < peak_index < len(power) - 1:
        alpha = power[peak_index - 1]
        beta = power[peak_index]
        gamma = power[peak_index + 1]
        denom = alpha - (2.0 * beta) + gamma
        if denom != 0:
            delta = 0.5 * (alpha - gamma) / denom
            bin_hz = sample_rate / len(samples)
            peak_offset = freqs[peak_index] + (delta * bin_hz)

    return center_freq + peak_offset, peak_offset, peak_power


def mix_down(samples, sample_rate, shift_hz):
    n = np.arange(len(samples))
    osc = np.exp(-1j * 2 * np.pi * shift_hz * n / sample_rate)
    return samples * osc


def _fir_lowpass_coeffs(taps, cutoff_hz, sample_rate):
    nyq = sample_rate / 2.0
    cutoff = float(cutoff_hz) / nyq
    if cutoff <= 0.0 or cutoff >= 1.0:
        raise ValueError('cutoff_hz must be between 0 and Nyquist')
    m = taps - 1
    n = np.arange(taps) - m / 2.0
    h = np.sinc(cutoff * n)
    window = np.hamming(taps)
    h *= window
    h /= np.sum(h)
    return h


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


def lowpass(samples, cutoff_hz, sample_rate, taps=129):
    coeffs = _fir_lowpass_coeffs(taps, cutoff_hz, sample_rate)
    return np.convolve(samples, coeffs, mode='same')


def bandpass(samples, lowcut_hz, highcut_hz, sample_rate, taps=129):
    coeffs = _fir_bandpass_coeffs(taps, lowcut_hz, highcut_hz, sample_rate)
    return np.convolve(samples, coeffs, mode='same')


def estimate_tone_frequency(samples, sample_rate):
    """Measure IF frequency using FFT peak instead of phase unwrap for better stability."""
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
        ignore_center=False,
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


def main():
    parser = argparse.ArgumentParser(description='SDR carrier measurement')
    parser.add_argument('center_freq', type=float, nargs='?', default=DEFAULT_FREQ, help='Center frequency in Hz')
    args = parser.parse_args()
    
    sdr = RtlSdr()
    sr = 2.048e6  # Hz
    cf = args.center_freq
    sdr.sample_rate = sr
    sdr.center_freq = cf
    #sdr.freq_correction = 0  # PPM
    print('valid gains:', sdr.valid_gains_db)
    sdr.gain = 49.6
    print('gain set to', sdr.gain)
    print(f'Center frequency: {cf / 1e6:.2f} MHz')

    read_size = 16384
    capture = 0
    power_threshold = 2e8
    step_hz = 32e6 / (1 << 19)
    offset_sum_hz = 0.0
    offset_count = 0
    last_reception_time = None
    # Expected RF carrier offset from center frequency; narrow search avoids peak hopping.
    expected_peak_offset_hz = 0.0
    try:
        while True:
            capture += 1
            now = time.monotonic()
            if (
                last_reception_time is not None
                and (now - last_reception_time) > 1.0
                and offset_count > 0
            ):
                offset_sum_hz = 0.0
                offset_count = 0
            x = sdr.read_samples(read_size)
            peak_freq, peak_offset, peak_power, tone_hz = measure_remote_carrier(
                x,
                sr,
                cf,
                target_offset_hz=455000.0,
                tone_band_hz=4000.0,
                expected_peak_offset_hz=expected_peak_offset_hz,
                peak_search_span_hz=30000.0,
            )
            if peak_power >= power_threshold:
                expected_peak_offset_hz = peak_offset
                if_error_hz = abs(tone_hz) - 455000.0
                offset_sum_hz += if_error_hz
                offset_count += 1
                avg_if_error_hz = offset_sum_hz / offset_count
                offset_avg_steps = int(round(avg_if_error_hz / step_hz))
                last_reception_time = now
                print(
                    f'IF {tone_hz / 1e3:+8.2f} kHz | '
                    f'error_now {int(round(if_error_hz)):+6d} Hz | '
                    f'error_avg {int(round(avg_if_error_hz)):+6d} Hz | '
                    f'steps {offset_avg_steps:+5d} | '
                    f'offset_now {if_error_hz / 1e3:+6.1f} kHz | '
                    f'offset_avg {avg_if_error_hz / 1e3:+6.1f} kHz | '
                    f'peak {peak_freq / 1e6:7.3f} MHz'
                )
    except KeyboardInterrupt:
        print('Stopped by user')
    finally:
        sdr.close()

if __name__ == '__main__':
    main()