import shutil
import gzip
import os
import time
import webbrowser
Import("env")

def copy_html(source, target, env):
    with open(os.path.join(env["PROJECT_DATA_DIR"], "index.html"), "r") as fin:
        with open(os.path.join(env["PROJECT_DIR"], "include/html.h"), "w") as fout:
            fout.write('const char html[] PROGMEM = R"html(')
            for line in fin:
                fout.write(line)
            fout.write('\n)html";')
    
def post_build(source, target, env):
    print("Version: " + env.GetProjectOption("custom_version"))
    print("project dir: " + env["PROJECT_DIR"])
    print("build: " + env["BUILD_DIR"])



def after_upload(source, target, env):
    #upload_port = env.get("UPLOAD_PORT", None)
    #if upload_port == None:
    #    env.AutodetectUploadPort()
    #    upload_port = env.get("UPLOAD_PORT", "none")

    time.sleep(3)
    os.system('cmd /c netsh wlan connect name = "RFM-Gateway"')
    time.sleep(5)
    webbrowser.open('http://4.3.2.1')

env.AddPreAction("$BUILD_DIR/src/main.cpp.o", copy_html)
env.AddPostAction("buildprog", post_build)
env.AddPostAction("upload", after_upload)