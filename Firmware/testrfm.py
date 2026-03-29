import json
import requests
import msvcrt
from tkinter import *
from tkinter import ttk
from functools import partial

def testrfm():
    f_corr = 60
    rfmType = 0 # 0=CW, 1=HCW
    freqBand = 1 # 0=300MHz, 1=433MHz, 2=868 MHz, 3=915 MHz
    print("t: txtest, +: inc f_corr, -: dec f_corr, 0: RFM69CW, 1: RFM69HCW")
    while (True):
        ch = msvcrt.getch()

        if ch == b't':
            if freqBand == 1:
                freq = 433900000
            elif freqBand == 2:
                freq = 868300000

            data = {
                "rfmType": rfmType,
                "freq": freq,
                "fCorr": f_corr,
                "pwr": 3,
                "baud": 12000
            }
            s = json.dumps(data)
            print("Send TX test:", s)
            r = requests.post('http://4.3.2.1/txtest', data=s)

        elif ch == b'+':
            f_corr += 1
        elif ch == b'-':
            f_corr -= 1
        elif ch == b'0':
            rfmtype = 0
        elif ch == b'1':
            rfmtype = 1

        elif ch == b'q':
            requests.get('http://4.3.2.1/send/intertechno/25221242/4/on')

        elif ch == b'w':
            requests.get('http://4.3.2.1/send/intertechno/25221242/4/off')

        elif ch == b's':
            data = {
                "radio": {
                    "rfmType": rfmType,
                    "freqBand": freqBand,
                    "fCorr": f_corr
                }
            }
            s = json.dumps(data)
            print("Send config:", s)
            r = requests.post('http://4.3.2.1/config', data=s)
            print(r.status_code, r.text)
            
        else:
            print("??:", ch)


class RFMTestApp:
    def __init__(self):
        self.ip = '4.3.2.1'
        self.root = Tk()
        self.root.wm_title = 'RFM-Gateway test'
        self.root.geometry('640x720')

        self.rfmtype = ttk.Combobox(self.root)
        self.rfmtype['values'] = ('RFM69CW', 'RFM69HCW')
        self.rfmtype.current(0)
        self.rfmtype.pack()

        self.freqband = ttk.Combobox(self.root)
        self.freqband['values'] = ('315 MHz', '433 MHz', '868 MHz', '915 MHz')
        self.freqband.current(1)
        self.freqband.pack()

        self.fcorr = Scale(self.root, from_=-500, to=500, orient=HORIZONTAL, length=460)
        self.fcorr.set(70)
        self.fcorr.pack()

        self.txtestbtn = ttk.Button(self.root, text="TX test", padding=20, command=self.txtest)
        self.txtestbtn.pack()

        self.savebtn = ttk.Button(self.root, text="save radio setup", padding=20, command=self.saveradiosetup)
        self.savebtn.pack()

        self.configbtn = ttk.Button(self.root, text="send default config", padding=20, command=self.saveconfig)
        self.configbtn.pack()

        self.onbutton = ttk.Button(self.root, text="ON", command=self.sendon)
        self.onbutton.pack()

        self.offbutton = ttk.Button(self.root, text="OFF", command=self.sendoff)
        self.offbutton.pack()

        btns = [
            {
                "Büro 1 on": 'intertechno/25221242/1/on',
                "Büro 1 off": 'intertechno/25221242/1/off',

                "Büro 2 on": 'intertechno/25221242/2/on',
                "Büro 2 off": 'intertechno/25221242/2/off',

                "Büro 3 on": 'intertechno/25221242/3/on',
                "Büro 3 off": 'intertechno/25221242/3/off',
            },
            {
                "Tris C 3 1 on": 'ittristate/c/3/1/on',
                "Tris C 3 1 off": 'ittristate/c/3/1/off',
                "Tris C 3 2 on": 'ittristate/c/3/2/on',
                "Tris C 3 2 off": 'ittristate/c/3/2/off',
                "Tris C 3 3 on": 'ittristate/c/3/3/on',
                "Tris C 3 3 off": 'ittristate/c/3/3/off',
                "Tris C 3 4 on": 'ittristate/c/3/4/on',
                "Tris C 3 4 off": 'ittristate/c/3/4/off',
            },
            {
                "IT32 1 on": 'intertechno/123456/1/on',
                "IT32 1 off": 'intertechno/123456/1/off',
                "IT32 2 on": 'intertechno/123456/2/on',
                "IT32 2 off": 'intertechno/123456/2/off',
                "IT32 3 on": 'intertechno/123456/3/on',
                "IT32 3 off": 'intertechno/123456/3/off',
                "IT32 4 on": 'intertechno/123456/4/on',
                "IT32 4 off": 'intertechno/123456/4/off',
            },
            {
                "Emylo A": 'emylo/12345/A',
                "Emylo B": 'emylo/12345/B',
                "Emylo C": 'emylo/12345/C',
                "Emylo D": 'emylo/12345/D'
            }
        ]

        for btnrow in btns:
            buttonframe = Frame(self.root)
            col = 0
    
            for key, value in btnrow.items():
                btn = ttk.Button(buttonframe, text=key, command=partial(self.send, value))
                btn.grid(row=0, column=col)
                col += 1
            
            buttonframe.pack()


        self.root.mainloop()

    def txtest(self):
        if self.freqband.current() == 1:
            freq = 433920000
        elif self.freqband.current() == 2:
            freq = 868300000

        data = {
            "rfmType": self.rfmtype.current(),
            "freq": freq,
            "fCorr": self.fcorr.get(),
            "pwr": 3,
            "baud": 20000
        }
        s = json.dumps(data)
        r = requests.post('http://4.3.2.1/txtest', data=s)
        print(f'TX test {s}: {r.status_code} {r.text}')

    def sendon(self):
        url = 'http://' + self.ip + '/send/intertechno/25221242/4/on'
        r = requests.get(url)
        print(f'Send ON {url}: {r.status_code} {r.text}')

    def sendoff(self):
        url = 'http://' + self.ip + '/send/intertechno/25221242/4/off'
        r = requests.get(url)
        print(f'Send OFF {url}: {r.status_code} {r.text}')
    
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