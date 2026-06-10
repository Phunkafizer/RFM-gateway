#!/usr/bin/env python3
"""
Capture RTL-SDR IQ data, demodulate OOK (envelope), and save as WAV.

Example:
  python tools/ook_demod_to_wav.py --freq 433920000 --seconds 15 --output out.wav
"""

import argparse
import ctypes
import os
import wave

import numpy as np


_DLL_NAMES = ["rtlsdr.dll", "librtlsdr.dll"]


def _find_rtlsdr_path() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))

    for folder in [script_dir, os.getcwd()]:
        for name in _DLL_NAMES:
            candidate = os.path.join(folder, name)
            if os.path.exists(candidate):
                return candidate

    for path_entry in os.environ.get("PATH", "").split(os.pathsep):
        if not path_entry:
            continue
        for name in _DLL_NAMES:
            candidate = os.path.join(path_entry, name)
            if os.path.exists(candidate):
                return candidate

    raise FileNotFoundError("Could not find rtlsdr.dll or librtlsdr.dll in script folder, cwd, or PATH.")


def _load_rtlsdr_library() -> ctypes.WinDLL:
    candidate = _find_rtlsdr_path()
    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        os.add_dll_directory(os.path.dirname(candidate))

    dll = ctypes.WinDLL(candidate)

    def _register(name, restype, argtypes):
        func = getattr(dll, name)
        func.restype = restype
        func.argtypes = argtypes

    _register("rtlsdr_open", ctypes.c_int, [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint])
    _register("rtlsdr_close", ctypes.c_int, [ctypes.c_void_p])
    _register("rtlsdr_set_center_freq", ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    _register("rtlsdr_set_sample_rate", ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    _register("rtlsdr_set_tuner_gain_mode", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    _register("rtlsdr_set_agc_mode", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    _register("rtlsdr_set_tuner_gain", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    _register("rtlsdr_set_freq_correction", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    _register("rtlsdr_reset_buffer", ctypes.c_int, [ctypes.c_void_p])
    _register(
        "rtlsdr_read_sync",
        ctypes.c_int,
        [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint, ctypes.POINTER(ctypes.c_int)],
    )
    return dll


class RtlSdr:
    def __init__(self, device_index: int = 0):
        self._dll = _load_rtlsdr_library()
        self._dev = ctypes.c_void_p()
        self._read_buffer = None
        self._read_buffer_len = 0
        self._n_read = ctypes.c_int()
        self._tmp_iq = np.empty(0, dtype=np.float32)
        self._tmp_complex = np.empty(0, dtype=np.complex64)

        rc = self._dll.rtlsdr_open(ctypes.byref(self._dev), ctypes.c_uint(device_index))
        if rc != 0:
            raise OSError(f"rtlsdr_open failed with error code {rc}")

    def close(self):
        if self._dev and self._dev.value:
            self._dll.rtlsdr_close(self._dev)
            self._dev = ctypes.c_void_p()

    def set_sample_rate(self, sample_rate_hz: int):
        rc = self._dll.rtlsdr_set_sample_rate(self._dev, ctypes.c_uint(int(sample_rate_hz)))
        if rc != 0:
            raise OSError(f"rtlsdr_set_sample_rate failed with error code {rc}")

    def set_center_freq(self, center_freq_hz: int):
        rc = self._dll.rtlsdr_set_center_freq(self._dev, ctypes.c_uint(int(center_freq_hz)))
        if rc != 0:
            raise OSError(f"rtlsdr_set_center_freq failed with error code {rc}")

    def set_freq_correction(self, ppm: int):
        rc = self._dll.rtlsdr_set_freq_correction(self._dev, ctypes.c_int(int(ppm)))
        if rc not in (0, -2):
            raise OSError(f"rtlsdr_set_freq_correction failed with error code {rc}")

    def set_agc(self, enabled: bool):
        rc = self._dll.rtlsdr_set_agc_mode(self._dev, ctypes.c_int(1 if enabled else 0))
        if rc != 0:
            raise OSError(f"rtlsdr_set_agc_mode failed with error code {rc}")

    def set_gain(self, gain_db: float):
        rc = self._dll.rtlsdr_set_tuner_gain_mode(self._dev, ctypes.c_int(1))
        if rc != 0:
            raise OSError(f"rtlsdr_set_tuner_gain_mode failed with error code {rc}")
        rc = self._dll.rtlsdr_set_tuner_gain(self._dev, ctypes.c_int(int(round(gain_db * 10.0))))
        if rc != 0:
            raise OSError(f"rtlsdr_set_tuner_gain failed with error code {rc}")

    def flush_buffer(self):
        rc = self._dll.rtlsdr_reset_buffer(self._dev)
        if rc != 0:
            raise OSError(f"rtlsdr_reset_buffer failed with error code {rc}")

    def read_samples(self, num_samples: int) -> np.ndarray:
        buffer_len = num_samples * 2
        if self._read_buffer is None or self._read_buffer_len != buffer_len:
            self._read_buffer = (ctypes.c_ubyte * buffer_len)()
            self._read_buffer_len = buffer_len

        rc = self._dll.rtlsdr_read_sync(
            self._dev,
            self._read_buffer,
            ctypes.c_uint(buffer_len),
            ctypes.byref(self._n_read),
        )
        if rc != 0:
            raise OSError(f"rtlsdr_read_sync failed with error code {rc}")

        count = self._n_read.value // 2
        if count <= 0:
            return np.empty(0, dtype=np.complex64)

        if self._tmp_iq.size < count * 2:
            self._tmp_iq = np.empty(count * 2, dtype=np.float32)
        if self._tmp_complex.size < count:
            self._tmp_complex = np.empty(count, dtype=np.complex64)

        raw_u8 = np.frombuffer(self._read_buffer, dtype=np.uint8, count=count * 2)
        iq = self._tmp_iq[: count * 2]
        np.subtract(raw_u8, 128.0, out=iq)

        samples = self._tmp_complex[:count]
        samples.real = iq[0::2]
        samples.imag = iq[1::2]
        return samples


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return x
    kernel = np.ones(window, dtype=np.float32) / float(window)
    return np.convolve(x, kernel, mode="same")


def resample_linear(x: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    if src_rate == dst_rate:
        return x
    duration = len(x) / float(src_rate)
    out_len = max(1, int(round(duration * dst_rate)))
    src_t = np.linspace(0.0, duration, num=len(x), endpoint=False)
    dst_t = np.linspace(0.0, duration, num=out_len, endpoint=False)
    return np.interp(dst_t, src_t, x).astype(np.float32)


def demod_ook(iq: np.ndarray, sample_rate: int, cutoff_hz: float, binary: bool) -> np.ndarray:
    if iq.size == 0:
        return np.empty(0, dtype=np.float32)

    envelope = np.abs(iq).astype(np.float32)

    # Smooth envelope to suppress high-frequency noise before thresholding/export.
    window = max(1, int(sample_rate / max(cutoff_hz, 1.0)))
    env_smooth = moving_average(envelope, window)

    env_smooth -= np.min(env_smooth)
    peak = float(np.max(env_smooth))
    if peak > 0:
        env_smooth /= peak

    if binary:
        threshold = float(np.mean(env_smooth) + 0.5 * np.std(env_smooth))
        return (env_smooth >= threshold).astype(np.float32)
    return env_smooth


def save_wav_mono_16(path: str, samples: np.ndarray, sample_rate: int):
    clipped = np.clip(samples, 0.0, 1.0)
    pcm = np.round((clipped * 2.0 - 1.0) * 32767.0).astype(np.int16)

    with wave.open(path, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm.tobytes())


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Capture and demodulate OOK from RTL-SDR to WAV.")
    p.add_argument("--freq", type=float, default=433_920_000, help="Center frequency in Hz (default: 433920000)")
    p.add_argument("--sample-rate", type=int, default=250_000, help="SDR sample rate in Hz (default: 250000)")
    p.add_argument("--wav-rate", type=int, default=48_000, help="Output WAV sample rate in Hz (default: 48000)")
    p.add_argument("--seconds", type=float, default=10.0, help="Capture length in seconds (default: 10)")
    p.add_argument("--block-size", type=int, default=262_144, help="Samples per USB read (default: 262144)")
    p.add_argument("--gain", type=float, default=38.0, help="Manual tuner gain in dB (default: 38.0)")
    p.add_argument("--agc", action="store_true", help="Enable tuner AGC instead of manual gain")
    p.add_argument("--ppm", type=int, default=0, help="Frequency correction in ppm (default: 0)")
    p.add_argument("--cutoff", type=float, default=20_000.0, help="OOK envelope smoothing cutoff (Hz)")
    p.add_argument("--binary", action="store_true", help="Export hard-sliced 0/1 OOK instead of analog envelope")
    p.add_argument("--output", default="sample.wav", help="Output WAV file path (default: sample.wav)")
    p.add_argument("--device", type=int, default=0, help="RTL-SDR device index (default: 0)")
    return p.parse_args()


def main():
    args = parse_args()

    total_samples = int(round(args.seconds * args.sample_rate))
    total_samples = max(total_samples, 1)

    sdr = RtlSdr(device_index=args.device)
    try:
        sdr.set_sample_rate(args.sample_rate)
        sdr.set_center_freq(int(args.freq))
        sdr.set_freq_correction(args.ppm)

        if args.agc:
            sdr.set_agc(True)
        else:
            sdr.set_agc(False)
            sdr.set_gain(args.gain)

        sdr.flush_buffer()

        chunks = []
        captured = 0
        while captured < total_samples:
            want = min(args.block_size, total_samples - captured)
            iq = sdr.read_samples(want)
            if iq.size == 0:
                continue
            chunks.append(iq.copy())
            captured += iq.size

            pct = 100.0 * captured / total_samples
            print(f"\rCapturing: {pct:5.1f}%", end="", flush=True)
        print()

        iq_all = np.concatenate(chunks) if chunks else np.empty(0, dtype=np.complex64)
        ook = demod_ook(iq_all, sample_rate=args.sample_rate, cutoff_hz=args.cutoff, binary=args.binary)
        wav_sig = resample_linear(ook, src_rate=args.sample_rate, dst_rate=args.wav_rate)

        save_wav_mono_16(args.output, wav_sig, sample_rate=args.wav_rate)
        print(f"Saved demodulated OOK WAV: {args.output}")
        print(f"Captured IQ samples: {iq_all.size}")
        print(f"WAV samples: {wav_sig.size} @ {args.wav_rate} Hz")

    finally:
        sdr.close()


if __name__ == "__main__":
    main()
