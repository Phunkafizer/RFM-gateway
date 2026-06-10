import shutil
import gzip
import os
import sys
import subprocess
import time
import webbrowser
from serial.tools import list_ports
import requests
import socket
subprocess.check_call([sys.executable, "-m", "pip", "install", "minify_html"])
import minify_html

Import("env")

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


def copy_html(source, target, varname, env):
    print(f"Creating {target} from {source}");
    with open(os.path.join(env["PROJECT_DATA_DIR"], source), "r", encoding="utf-8") as fin:
        content = fin.read()
        if (env["PIOENV"] == "release"):
            print("minify html");
            content = minify_html.minify(
                content,
                # --- JS / CSS ---
                minify_js=True,               # minify inline <script> content
                minify_css=True,              # minify inline <style> and style= attributes
                minify_doctype=True,         # shorten <!DOCTYPE html> to <!doctype html>
                # --- Attributes ---
                keep_input_type_text_attr=True,              # keep type="text" on <input> (default is removed as redundant)
                allow_noncompliant_unquoted_attribute_values=False,  # allow unquoted attribute values (faster, but non-standard)
                allow_removing_spaces_between_attributes=False,      # remove spaces between attributes (non-standard)
                # --- Tags ---
                keep_closing_tags=False,                # keep optional closing tags e.g. </li>, </td>
                keep_html_and_head_opening_tags=False,  # keep <html> and <head> opening tags (removed when optional)
                # --- Comments ---
                keep_comments=False,          # keep regular HTML comments
                keep_ssi_comments=False,      # keep SSI comments <!--# ... -->
                remove_bangs=False,           # remove <!...> declarations (e.g. <!DOCTYPE>)
                remove_processing_instructions=False,  # remove <?...?> processing instructions
                # --- Entities ---
                allow_optimal_entities=False, # use shortest entity representation (may change semantics in edge cases)
                # --- Template syntax preservation ---
                preserve_brace_template_syntax=False,          # preserve {{ }}, {% %}, {# #} (Jinja, Handlebars, etc.)
                preserve_chevron_percent_template_syntax=False, # preserve <% %> (EJS, ERB, JSP, etc.)
            )
        with open(os.path.join(env["PROJECT_DIR"], "include", target), "w", encoding="utf-8") as fout:
            fout.write(f'const char {varname}[] PROGMEM = R"html(')
            fout.write(content)
            fout.write('\n)html";')
    
def post_build(source, target, env):
    print("Version: " + env.GetProjectOption("custom_version"))
    print("project dir: " + env["PROJECT_DIR"])
    print("build: " + env["BUILD_DIR"])

def before_upload(source, target, env):
    if env.get("UPLOAD_PORT") == None:
        devices = list_ports.comports()
        for d in devices:
            if (d.vid == 0x1A86) and (d.pid == 0x7523):
                print("Auto-detected upload port:", d[0])
                env.Replace(UPLOAD_PORT=d[0])
                break


DEVICE_IP = "4.3.2.1"

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

    _err(f"✗ WiFi did not become connected after {retries} attempt(s)")
    return False

def after_upload(source, target, env):
    return
    time.sleep(3)
    connect_to_wifi(profile="RFM-Gateway")
    webbrowser.open('http://4.3.2.1')
    r = requests.get('http://4.3.2.1/config')
    print(f'Current config: {r.status_code} {r.text}')

env.AddPostAction("buildprog", post_build)
env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)

copy_html("index.html", "html.h", "html", env)
copy_html("rc433.html", "rc433html.h", "rc433html", env)