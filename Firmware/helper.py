import shutil
import gzip
import os
import sys
import subprocess
import time
import webbrowser
from serial.tools import list_ports
import requests

subprocess.check_call([sys.executable, "-m", "pip", "install", "minify_html"])
import minify_html

Import("env")

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

def after_upload(source, target, env):
    time.sleep(3)
    os.system('cmd /c netsh wlan connect name = "RFM-Gateway"')
    time.sleep(5)
    webbrowser.open('http://4.3.2.1')
    r = requests.get('http://4.3.2.1/config')
    print(f'Current config: {r.status_code} {r.text}')

env.AddPostAction("buildprog", post_build)
env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)

copy_html("index.html", "html.h", "html", env)
copy_html("rc433.html", "rc433html.h", "rc433html", env)