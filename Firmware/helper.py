import shutil
import gzip
import os
import time
import webbrowser
from serial.tools import list_ports
import requests
from htmlmin import minify as html_minify
from jsmin import jsmin
from bs4 import BeautifulSoup
import cssmin
Import("env")

def copy_html(source, target, env):
    print("Building html.h from index.html")
    with open(os.path.join(env["PROJECT_DATA_DIR"], "index.html"), "r") as fin:
        content = fin.read()
        if (env["PIOENV"] == "release"):
            print("minify html");
            # Parse HTML and minify inline JS and CSS
            soup = BeautifulSoup(content, 'html.parser')
            for script in soup.find_all('script'):
                if script.string:
                    script.string = jsmin(script.string)
            for style in soup.find_all('style'):
                if style.string:
                    style.string = cssmin.cssmin(style.string)
            content = html_minify(str(soup), remove_comments=True, remove_empty_space=True)
        with open(os.path.join(env["PROJECT_DIR"], "include/html.h"), "w") as fout:
            fout.write('const char html[] PROGMEM = R"html(')
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

copy_html(None, None, env)