import serial, sys, time, os, subprocess, requests, socket
from pathlib import Path
from serial.tools import list_ports
import esptool


DEVICE_IP = "4.3.2.1"
PROJECT_ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = PROJECT_ROOT / ".pio" / "build" / "release"
FIRMWARE_BIN = BUILD_DIR / "firmware.bin"

# ANSI colour output
if sys.platform == "win32":
    os.system("")  # enable VT processing on Windows
_RED    = "\033[91m"
_GREEN  = "\033[92m"
_YELLOW = "\033[93m"
_RESET  = "\033[0m"

def _ok(msg):   print(f"{_GREEN}{msg}{_RESET}")
def _err(msg):  print(f"{_RED}{msg}{_RESET}")
def _warn(msg): print(f"{_YELLOW}{msg}{_RESET}")


def show_progress(prefix: str, current: int, total: int, width: int = 28) -> None:
    total = max(total, 1)
    ratio = min(max(current / total, 0), 1)
    filled = int(width * ratio)
    bar = "#" * filled + "-" * (width - filled)
    print(f"\r{prefix} [{bar}] {ratio * 100:5.1f}%", end="", flush=True)
    if current >= total:
        print()

def wait_for_target_port(pid, vid):
    """
    Wait for USB device to appear, then return immediately.
    Returns the port name as soon as it is present.
    """
    last_port = None

    print(
        f"Waiting for USB device VID:PID {vid:04X}:{pid:04X}... (Ctrl+C to stop)"
    )

    oldports = list_ports.comports()

    while True:
        newports = list_ports.comports()
        added = [p for p in newports if p not in oldports]

        if len(added) > 0:
            if (added[0].vid == vid) and (added[0].pid == pid):
                print("Found", added[0])
                return added[0].device
            
        oldports = newports
        time.sleep(0.5)

def flash_firmware(esp) -> None:
    if not FIRMWARE_BIN.exists():
        raise FileNotFoundError(f"Firmware not found: {FIRMWARE_BIN}")

    print("Flashing firmware...")
    image = FIRMWARE_BIN.read_bytes()
    esp.flash_begin(len(image), 0x0)

    total_blocks = max(1, (len(image) + esp.FLASH_WRITE_SIZE - 1) // esp.FLASH_WRITE_SIZE)
    show_progress("Flashing", 0, total_blocks)

    seq = 0
    for offset in range(0, len(image), esp.FLASH_WRITE_SIZE):
        block = image[offset:offset + esp.FLASH_WRITE_SIZE]
        if len(block) < esp.FLASH_WRITE_SIZE:
            block += b"\xFF" * (esp.FLASH_WRITE_SIZE - len(block))
        esp.flash_block(block, seq)
        seq += 1
        show_progress("Flashing", seq, total_blocks)

    esp.flash_finish(True)
    print("\033[92mFirmware upload finished.\033[0m")

def connect_to_wifi(profile, timeout=20, retries=3):
    def _netsh_connect():
        print(f"Connecting to OTthing WiFi (profile: {profile})...")
        cmd = f'netsh wlan connect name="{profile}"'
        result = subprocess.run(["cmd", "/c", cmd], capture_output=True, text=True)
        if result.stdout.strip():
            print(result.stdout.strip())
        if result.returncode != 0:
            print(result.stderr.strip())
            return False
        return True

    for attempt in range(1, retries + 1):
        if not _netsh_connect():
            return False

        wait = 8 if attempt == 1 else 5
        disconnected_early = False
        for remaining in range(wait, 0, -1):
            # Query current WiFi SSID every second during the wait
            check = subprocess.run(
                ["cmd", "/c", "netsh wlan show interfaces"],
                capture_output=True, text=True, encoding="cp850", errors="replace"
            )
            ssid_line = next((l.strip() for l in check.stdout.splitlines() if "SSID" in l and "BSSID" not in l), "SSID: ?")
            state_line = next((l.strip() for l in check.stdout.splitlines() if "Status" in l or "tatus" in l or "Status" in l), "State: ?")
            print(f"  [{remaining:2d}s] {state_line} | {ssid_line}")
            output_lower = check.stdout.lower()
            # Break early if already associated
            if ("verbunden" in output_lower or "connected" in output_lower) and profile.lower() in output_lower:
                print(f"  WiFi associated after {wait - remaining + 1}s")
                break
            # If fully disconnected (not just associating), retry connect immediately
            if "getrennt" in output_lower or "disconnected" in output_lower:
                _warn("  Disconnected — retrying netsh connect immediately...")
                disconnected_early = True
                break
            time.sleep(1)

        if disconnected_early:
            continue  # skip IP probe, go straight to next netsh connect attempt

        # Probe the device IP to confirm the WiFi link is up
        deadline = time.time() + timeout
        connected = False
        while time.time() < deadline:
            try:
                with socket.create_connection((DEVICE_IP, 80), timeout=5):
                    _ok(f"✓ Connected to WiFi profile {profile} (device reachable at {DEVICE_IP})")
                    connected = True
                    break
            except OSError:
                time.sleep(1)

        if connected:
            return True

        _warn(f"⚠ Device not reachable at {DEVICE_IP} after attempt {attempt}/{retries}, retrying netsh...")

    _err(f"✗ OTthing WiFi did not become connected after {retries} attempt(s)")
    return False


while True:
    progport = wait_for_target_port(vid=0x1a86, pid=0x7523)
    project_dir = os.path.dirname(os.path.abspath(__file__))
    esp = esptool.cmds.detect_chip(port=progport)
    esp = esp.run_stub()
    esp.change_baud(921600)
    esp.flash_set_parameters(esptool.util.flash_size_bytes("4MB"))
    flash_firmware(esp)
    esp.hard_reset()
    
    time.sleep(3)
    connect_to_wifi(profile="RFM-Gateway")
