#!/usr/bin/env python3
"""Scan LTE base stations and report detected downlink frequency peaks."""

import argparse
import ctypes
import os
import pathlib
import time

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

LTE_BANDS = [
    {"name": "B1", "dl_low": 2110.0, "dl_high": 2170.0, "n_off": 0},
    {"name": "B3", "dl_low": 1805.0, "dl_high": 1880.0, "n_off": 1200},
    {"name": "B7", "dl_low": 2620.0, "dl_high": 2690.0, "n_off": 2750},
    {"name": "B8", "dl_low": 925.0, "dl_high": 960.0, "n_off": 0},
    {"name": "B20", "dl_low": 791.0, "dl_high": 821.0, "n_off": 6150},
    {"name": "B28", "dl_low": 758.0, "dl_high": 803.0, "n_off": 9210},
    {"name": "B38", "dl_low": 2570.0, "dl_high": 2620.0, "n_off": 37750},
]

_DLL_NAMES = ["librtlsdr.dll", "rtlsdr.dll"]


def find_rtlsdr_library():
    search_dirs = [
        pathlib.Path(__file__).resolve().parent,
        pathlib.Path.cwd().resolve(),
    ]
    search_dirs.extend(
        pathlib.Path(p)
        for p in os.environ.get("PATH", "").split(os.pathsep)
        if p
    )

    for folder in search_dirs:
        try:
            folder = folder.resolve()
        except OSError:
            continue
        for name in _DLL_NAMES:
            candidate = folder / name
            if candidate.exists():
                return candidate

    raise FileNotFoundError(
        "Could not find rtlsdr.dll or librtlsdr.dll in the current directory or PATH."
    )


def load_rtlsdr():
    path = find_rtlsdr_library()
    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(path.parent))
    dll = ctypes.WinDLL(str(path))

    def register(name, restype, argtypes):
        func = getattr(dll, name)
        func.restype = restype
        func.argtypes = argtypes
        return func

    register("rtlsdr_open", ctypes.c_int, [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint])
    register("rtlsdr_close", ctypes.c_int, [ctypes.c_void_p])
    register("rtlsdr_set_center_freq", ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    register("rtlsdr_set_sample_rate", ctypes.c_int, [ctypes.c_void_p, ctypes.c_uint])
    register("rtlsdr_set_freq_correction", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    register("rtlsdr_set_tuner_gain_mode", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    register("rtlsdr_set_agc_mode", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    register("rtlsdr_set_tuner_gain", ctypes.c_int, [ctypes.c_void_p, ctypes.c_int])
    register("rtlsdr_reset_buffer", ctypes.c_int, [ctypes.c_void_p])
    register(
        "rtlsdr_read_sync",
        ctypes.c_int,
        [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint, ctypes.POINTER(ctypes.c_int)],
    )

    return dll


class RtlSdrDevice:
    def __init__(self, device_index=0):
        self._dll = load_rtlsdr()
        self._dev = ctypes.c_void_p()
        result = self._dll.rtlsdr_open(ctypes.byref(self._dev), ctypes.c_uint(device_index))
        if result != 0:
            raise OSError(f"rtlsdr_open failed with error code {result}")

    def close(self):
        if self._dev and self._dev.value:
            self._dll.rtlsdr_close(self._dev)
            self._dev = ctypes.c_void_p()

    def set_sample_rate(self, sample_rate_hz):
        result = self._dll.rtlsdr_set_sample_rate(self._dev, ctypes.c_uint(int(sample_rate_hz)))
        if result != 0:
            raise OSError(f"rtlsdr_set_sample_rate failed with error code {result}")

    def set_center_freq(self, freq_hz):
        result = self._dll.rtlsdr_set_center_freq(self._dev, ctypes.c_uint(int(freq_hz)))
        if result != 0:
            raise OSError(f"rtlsdr_set_center_freq failed with error code {result}")

    def set_freq_correction(self, ppm):
        result = self._dll.rtlsdr_set_freq_correction(self._dev, ctypes.c_int(int(ppm)))
        if result != 0:
            if result == -2:
                print(
                    "Warning: rtlsdr_set_freq_correction is not supported by this device/driver. "
                    "Continuing without frequency correction."
                )
                return
            raise OSError(f"rtlsdr_set_freq_correction failed with error code {result}")

    def set_gain(self, gain_db):
        if isinstance(gain_db, str) and gain_db.lower() == "auto":
            result = self._dll.rtlsdr_set_tuner_gain_mode(self._dev, ctypes.c_int(0))
            if result != 0:
                raise OSError(f"rtlsdr_set_tuner_gain_mode failed with error code {result}")
            result = self._dll.rtlsdr_set_agc_mode(self._dev, ctypes.c_int(1))
            if result != 0:
                raise OSError(f"rtlsdr_set_agc_mode failed with error code {result}")
        else:
            result = self._dll.rtlsdr_set_agc_mode(self._dev, ctypes.c_int(0))
            if result != 0:
                raise OSError(f"rtlsdr_set_agc_mode failed with error code {result}")
            result = self._dll.rtlsdr_set_tuner_gain_mode(self._dev, ctypes.c_int(1))
            if result != 0:
                raise OSError(f"rtlsdr_set_tuner_gain_mode failed with error code {result}")
            raw_gain = int(round(float(gain_db) * 10.0))
            result = self._dll.rtlsdr_set_tuner_gain(self._dev, ctypes.c_int(raw_gain))
            if result != 0:
                raise OSError(f"rtlsdr_set_tuner_gain failed with error code {result}")

    def read_samples(self, num_samples):
        self._dll.rtlsdr_reset_buffer(self._dev)
        buffer_len = num_samples * 2
        buffer = (ctypes.c_ubyte * buffer_len)()
        n_read = ctypes.c_int()
        result = self._dll.rtlsdr_read_sync(self._dev, buffer, ctypes.c_uint(buffer_len), ctypes.byref(n_read))
        if result != 0:
            raise OSError(f"rtlsdr_read_sync failed with error code {result}")
        raw = bytes(buffer[: n_read.value])
        iq = np.empty(n_read.value // 2, dtype=np.complex64)
        iq.real = np.frombuffer(raw[0::2], dtype=np.uint8).astype(np.float32) - 128.0
        iq.imag = np.frombuffer(raw[1::2], dtype=np.uint8).astype(np.float32) - 128.0
        return iq


def find_peak_frequency(samples, sample_rate, center_freq):
    window = np.hanning(len(samples))
    spectrum = np.fft.fftshift(np.fft.fft(samples * window))
    power = np.abs(spectrum) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(len(samples), d=1.0 / sample_rate))
    peak_index = int(np.argmax(power))
    return center_freq + freqs[peak_index], freqs[peak_index], power[peak_index]


def lte_channel_to_freq(band_name, earfcn):
    band = next((b for b in LTE_BANDS if b["name"] == band_name), None)
    if band is None:
        raise ValueError(f"Unknown LTE band: {band_name}")
    return band["dl_low"] + (earfcn - band["n_off"]) * 0.1


def calculate_ppm(expected_freq_mhz, measured_freq_mhz):
    return (expected_freq_mhz - measured_freq_mhz) / expected_freq_mhz * 1e6


def nearest_lte_channel(freq_mhz):
    best = None
    for band in LTE_BANDS:
        edge_freq = min(max(freq_mhz, band["dl_low"]), band["dl_high"])
        earfcn = int(round((edge_freq - band["dl_low"]) * 10.0)) + band["n_off"]
        expected_freq = lte_channel_to_freq(band["name"], earfcn)
        delta_khz = abs(freq_mhz - expected_freq) * 1000.0
        candidate = {
            "band": band["name"],
            "earfcn": earfcn,
            "expected_freq_mhz": expected_freq,
            "measured_freq_mhz": freq_mhz,
            "delta_khz": delta_khz,
        }
        if band["dl_low"] <= freq_mhz <= band["dl_high"]:
            return candidate
        if best is None or candidate["delta_khz"] < best["delta_khz"]:
            best = candidate
    return best


def plot_carriers(candidates):
    if plt is None:
        print("matplotlib not installed; cannot generate plot.")
        return
    if not candidates:
        print("No carrier candidates to plot.")
        return
    best_per_carrier = {}
    for c in candidates:
        key = (c["band_name"], c["earfcn"])
        if key not in best_per_carrier or c["power"] > best_per_carrier[key]["power"]:
            best_per_carrier[key] = c
    reduced = list(best_per_carrier.values())
    freqs = [c["peak_freq"] for c in reduced]
    power = [c["power"] for c in reduced]
    labels = [f"{c['band_name']}/{c['earfcn']}" for c in reduced]
    plt.figure(figsize=(10, 5))
    plt.scatter(freqs, power, c="blue", s=30)
    plt.yscale("log")
    for i, txt in enumerate(labels):
        plt.annotate(txt, (freqs[i], power[i]), fontsize=8, alpha=0.7)
    plt.xlabel("Frequency (MHz)")
    plt.ylabel("Received power (log scale)")
    plt.title("Detected LTE carriers")
    plt.grid(True, which='both', axis='y')
    plt.tight_layout()
    plt.show()


def generate_center_frequencies(start_mhz, stop_mhz, sample_rate_hz, overlap=0.85):
    sample_rate_mhz = sample_rate_hz / 1e6
    step = sample_rate_mhz * overlap
    if step <= 0:
        return [start_mhz]
    centers = []
    current = start_mhz + sample_rate_mhz / 2.0
    while current < stop_mhz:
        centers.append(current)
        current += step
    if not centers:
        centers = [(start_mhz + stop_mhz) / 2.0]
    return centers


def filter_centers_by_range(centers, min_mhz, max_mhz):
    return [c for c in centers if min_mhz <= c <= max_mhz]


def scan_lte(device, bands, sample_rate, gain, threshold, min_freq=50.0, max_freq=2200.0, calibrate=False, plot=True):
    device.set_sample_rate(sample_rate)
    device.set_gain(gain)
    print(
        f"scan start: bands={bands}, sample_rate={sample_rate}, gain={gain}, threshold={threshold}, "
        f"min_freq={min_freq}, max_freq={max_freq}, calibrate={calibrate}"
    )
    if calibrate:
        print(
            "Calibration mode maps detected carrier frequencies to LTE band/EARFCN values. "
            "This is a channel number and frequency mapping, not MCC/MNC/LAC/CID identity info."
        )

    ranges = []
    if bands == ["all"]:
        for band in LTE_BANDS:
            ranges.append((band["dl_low"], band["dl_high"], band["name"]))
    else:
        for band_name in bands:
            band = next((b for b in LTE_BANDS if b["name"] == band_name), None)
            if band is None:
                raise ValueError(f"Unknown LTE band: {band_name}")
            ranges.append((band["dl_low"], band["dl_high"], band["name"]))

    capture = 0
    best_candidate = None
    candidates = []
    try:
        print("starting scan cycle")
        for start_mhz, stop_mhz, band_name in ranges:
            centers = generate_center_frequencies(start_mhz, stop_mhz, sample_rate)
            centers = filter_centers_by_range(centers, min_freq, max_freq)
            if not centers:
                print(f"Skipping band {band_name} because no centers fit {min_freq}-{max_freq} MHz")
            for center in centers:
                print(f"scanning band {band_name} center {center:.3f} MHz")
                try:
                    device.set_center_freq(int(center * 1e6))
                except OSError as exc:
                    print(f"Skipping unsupported center {center:.3f} MHz: {exc}")
                    continue
                time.sleep(0.05)
                try:
                    samples = device.read_samples(16384)
                except OSError as exc:
                    print(f"Read failed at {center:.3f} MHz: {exc}")
                    continue
                peak_freq, peak_offset, peak_power = find_peak_frequency(samples, sample_rate, center * 1e6)
                capture += 1
                if peak_power < threshold:
                    continue
                channel = nearest_lte_channel(peak_freq / 1e6)
                candidate = {
                    "capture": capture,
                    "band": band_name,
                    "peak_freq": peak_freq / 1e6,
                    "offset_khz": peak_offset / 1e3,
                    "power": peak_power,
                    "expected_freq_mhz": channel["expected_freq_mhz"],
                    "band_name": channel["band"],
                    "earfcn": channel["earfcn"],
                    "delta_khz": channel["delta_khz"],
                }
                candidates.append(candidate)
                if calibrate:
                    if best_candidate is None or peak_power > best_candidate["power"]:
                        best_candidate = candidate
                        print(
                            f"new best candidate: capture {capture}, band={candidate['band_name']}, "
                            f"peak={candidate['peak_freq']:.6f} MHz, power={int(candidate['power'])}, "
                            f"LTE {candidate['band_name']} EARFCN {candidate['earfcn']}"
                        )
                else:
                    print(
                        f"capture {capture}: band={band_name} peak={candidate['peak_freq']:.6f} MHz "
                        f"(offset {candidate['offset_khz']:+.2f} kHz) power={int(candidate['power'])} "
                        f"LTE {candidate['band_name']} EARFCN {candidate['earfcn']} delta={candidate['delta_khz']:.2f} kHz"
                    )
    except KeyboardInterrupt:
        print("scan stopped by user")

    if calibrate and best_candidate is not None:
        print("Verifying three strongest carriers with repeated measurements...")
        best_per_carrier = {}
        for candidate in candidates:
            key = (candidate["band_name"], candidate["earfcn"])
            if key not in best_per_carrier or candidate["power"] > best_per_carrier[key]["power"]:
                best_per_carrier[key] = candidate
        top_candidates = sorted(best_per_carrier.values(), key=lambda candidate: candidate["power"], reverse=True)[:3]

        for candidate_index, candidate in enumerate(top_candidates, start=1):
            print(
                f"verifying carrier {candidate_index}/{len(top_candidates)}: "
                f"LTE {candidate['band_name']} EARFCN {candidate['earfcn']} at {candidate['peak_freq']:.6f} MHz"
            )
            try:
                device.set_center_freq(int(candidate["peak_freq"] * 1e6))
                time.sleep(0.05)
                verification_freqs = []
                verification_offsets = []
                verification_powers = []
                verify_loops = 20
                for verify_idx in range(verify_loops):
                    try:
                        verify_samples = device.read_samples(16384)
                    except OSError as exc:
                        print(
                            f"Verification read {verify_idx + 1}/{verify_loops} failed for "
                            f"LTE {candidate['band_name']} EARFCN {candidate['earfcn']}: {exc}"
                        )
                        continue
                    v_peak_freq, v_peak_offset, v_peak_power = find_peak_frequency(
                        verify_samples, sample_rate, int(candidate["peak_freq"] * 1e6)
                    )
                    verification_freqs.append(v_peak_freq / 1e6)
                    verification_offsets.append(v_peak_offset / 1e3)
                    verification_powers.append(v_peak_power)
                    print(
                        f"verify {candidate_index}/{len(top_candidates)} {verify_idx + 1}/{verify_loops}: "
                        f"{v_peak_freq / 1e6:.6f} MHz, offset {v_peak_offset / 1e3:+.2f} kHz, "
                        f"power={int(v_peak_power)}"
                    )
                if verification_freqs:
                    avg_freq_mhz = float(np.mean(verification_freqs))
                    avg_offset_khz = float(np.mean(verification_offsets))
                    avg_power = float(np.mean(verification_powers))
                    avg_ppm = calculate_ppm(candidate["expected_freq_mhz"], avg_freq_mhz)
                    candidate["verified_freq_mhz"] = avg_freq_mhz
                    candidate["verified_offset_khz"] = avg_offset_khz
                    candidate["verified_power"] = avg_power
                    candidate["verified_ppm"] = avg_ppm
                    print(
                        f"verification summary {candidate_index}/{len(top_candidates)}: "
                        f"avg_peak={avg_freq_mhz:.6f} MHz, avg_offset={avg_offset_khz:+.2f} kHz, "
                        f"avg_power={int(avg_power)}, avg_ppm={avg_ppm:.2f}, "
                        f"LTE {candidate['band_name']} EARFCN {candidate['earfcn']}"
                    )
            except OSError as exc:
                print(
                    f"Carrier verification failed for LTE {candidate['band_name']} "
                    f"EARFCN {candidate['earfcn']}: {exc}"
                )

    if plot:
        plot_carriers(candidates)

    if calibrate:
        if best_candidate is None:
            print("No valid LTE carrier found for calibration.")
            return None
        result_freq_mhz = best_candidate.get("verified_freq_mhz", best_candidate["peak_freq"])
        result_power = best_candidate.get("verified_power", best_candidate["power"])
        ppm = best_candidate.get(
            "verified_ppm",
            calculate_ppm(best_candidate["expected_freq_mhz"], result_freq_mhz),
        )
        print(
            f"best calibration result: peak={result_freq_mhz:.6f} MHz "
            f"expected={best_candidate['expected_freq_mhz']:.6f} MHz "
            f"power={int(result_power)} ppm={ppm:.2f} "
            f"LTE {best_candidate['band_name']} EARFCN {best_candidate['earfcn']} delta={best_candidate['delta_khz']:.1f} kHz"
        )
        return ppm


def parse_args():
    parser = argparse.ArgumentParser(
        description="Scan LTE downlink bands on an RTL-SDR device and report peak frequency detections."
    )
    parser.add_argument(
        "--band",
        type=str,
        nargs="*",
        default=["B1", "B3", "B8", "B20", "B28"],
        help="LTE band(s) to scan, e.g. B1 B3 B7 B20, or all",
    )
    parser.add_argument(
        "--gain",
        type=str,
        default="49.6",
        help="Tuner gain in dB or 'auto'",
    )
    parser.add_argument(
        "--ppm",
        type=float,
        default=0.0,
        help="Frequency correction in ppm",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=2e8,
        help="Minimum peak power to report",
    )
    parser.add_argument(
        "--min-freq",
        type=float,
        default=50.0,
        help="Minimum center frequency to scan in MHz",
    )
    parser.add_argument(
        "--max-freq",
        type=float,
        default=2200.0,
        help="Maximum center frequency to scan in MHz",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        default=True,
        help="Compute receiver ppm frequency correction from detected LTE channel",
    )
    parser.add_argument(
        "--no-calibrate",
        action="store_false",
        dest="calibrate",
        help="Do not run calibration mode",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        default=True,
        help="Show a frequency vs power plot after scanning",
    )
    parser.add_argument(
        "--no-plot",
        action="store_false",
        dest="plot",
        help="Do not show the frequency vs power plot",
    )
    parser.add_argument(
        "--device",
        type=int,
        default=0,
        help="RTL-SDR device index",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    device = RtlSdrDevice(device_index=args.device)
    try:
        device.set_freq_correction(args.ppm)
        scan_lte(
            device,
            args.band,
            sample_rate=2_048_000,
            gain=args.gain,
            threshold=args.threshold,
            min_freq=args.min_freq,
            max_freq=args.max_freq,
            calibrate=args.calibrate,
            plot=args.plot,
        )
    finally:
        device.close()


if __name__ == "__main__":
    main()
