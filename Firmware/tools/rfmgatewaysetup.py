import json
import pathlib
import subprocess
import sys
import requests
import msvcrt
from tkinter import *
from tkinter import ttk
from functools import partial

#GATEWAY_IP = '4.3.2.1'
GATEWAY_IP = '192.168.178.97'
FSTEP = 32e6 / (1 << 19)

# Frequency bands: [0]=315MHz, [1]=433MHz, [2]=868MHz, [3]=915MHz
FREQ_BANDS = [
    315e6,        # 315 MHz
    433.92e6,     # 433 MHz
    868.3e6,      # 868 MHz
    915e6         # 915 MHz
]

def testrfm():
    f_corr = 60
    rfmType = 0 # 0=CW, 1=HCW
    freqBand = 1 # 0=300MHz, 1=433MHz, 2=868 MHz, 3=915 MHz
    print("t: txtest, +: inc f_corr, -: dec f_corr, 0: RFM69CW, 1: RFM69HCW")
    while (True):
        ch = msvcrt.getch()

        if ch == b't':
            if freqBand < len(FREQ_BANDS):
                freq = int(FREQ_BANDS[freqBand])

            data = {
                "rfmType": rfmType,
                "freq": freq,
                "fCorr": f_corr,
                "pwr": 3,
                "baud": 12000
            }
            s = json.dumps(data)
            print("Send TX test:", s)
            r = requests.post(f'http://{GATEWAY_IP}/txtest', data=s)

        elif ch == b'+':
            f_corr += 1
        elif ch == b'-':
            f_corr -= 1
        elif ch == b'0':
            rfmtype = 0
        elif ch == b'1':
            rfmtype = 1

        elif ch == b'q':
            requests.get(f'http://{GATEWAY_IP}/send/intertechno/25221242/4/on')

        elif ch == b'w':
            requests.get(f'http://{GATEWAY_IP}/send/intertechno/25221242/4/off')

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
            r = requests.post(f'http://{GATEWAY_IP}/config', data=s)
            print(r.status_code, r.text)
            
        else:
            print("??:", ch)


class RFMTestApp:
    def __init__(self):
        self.ip = GATEWAY_IP
        self.sdr_process = None
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

        ttk.Label(self.setup_frame, text='Freq band').grid(row=0, column=1, sticky='w')
        self.freqband = ttk.Combobox(self.setup_frame)
        self.freqband['values'] = ('315 MHz', '433 MHz', '868 MHz', '915 MHz')
        self.freqband.current(1)
        self.freqband.grid(row=1, column=1, padx=4, pady=2, sticky='w')

        ttk.Label(self.setup_frame, text='Freq correction').grid(row=2, column=0, columnspan=2, sticky='w')
        self.fcorr = Scale(self.setup_frame, from_=-500, to=500, orient=HORIZONTAL, length=460, command=self.update_fcorr_offset)
        self.fcorr.set(70)
        self.fcorr.grid(row=3, column=0, columnspan=2, padx=4, pady=2, sticky='w')
        self.fcorr_offset_label = ttk.Label(self.setup_frame, text=f'real offset: {(FSTEP * 70) / 1000:.2f} kHz')
        self.fcorr_offset_label.grid(row=4, column=0, columnspan=2, sticky='w', padx=4, pady=(0,6))

        ttk.Label(self.setup_frame, text='TX power (dBm)').grid(row=5, column=0, columnspan=2, sticky='w')
        self.txpower = Scale(self.setup_frame, from_=-18, to=20, orient=HORIZONTAL, length=460)
        self.txpower.set(3)
        self.txpower.grid(row=6, column=0, columnspan=2, padx=4, pady=2, sticky='w')

        self.txtestbtn = ttk.Button(self.setup_frame, text="TX test", width=18, command=self.txtest)
        self.txtestbtn.grid(row=7, column=0, padx=4, pady=6, sticky='w')

        self.savebtn = ttk.Button(self.setup_frame, text="save radio setup", width=18, command=self.saveradiosetup)
        self.savebtn.grid(row=7, column=1, padx=4, pady=6, sticky='w')

        self.configbtn = ttk.Button(self.root, text="send default config", width=18, command=self.saveconfig)
        self.configbtn.pack(padx=10, pady=4)

        self.sdrbtn = ttk.Button(self.root, text="Start SDR measurement", width=18, command=self.start_sdr_with_freq)
        self.sdrbtn.pack(padx=10, pady=4)

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


        self.root.mainloop()

    def start_sdr_with_freq(self):
        freq_band = self.freqband.current()
        if freq_band < len(FREQ_BANDS):
            center_freq = FREQ_BANDS[freq_band]
        else:
            center_freq = FREQ_BANDS[1]  # Default to 433 MHz
        
        script_path = pathlib.Path(__file__).resolve().with_name('sdr.py')
        if not script_path.exists():
            print(f'SDR script not found: {script_path}')
            return
        
        if self.sdr_process is not None and self.sdr_process.poll() is None:
            print('SDR is already running')
            return
        
        self.sdr_process = subprocess.Popen(
            [sys.executable, str(script_path), str(center_freq)],
            cwd=str(script_path.parent)
        )
        print(f'Started SDR at {center_freq / 1e6:.2f} MHz')

    def start_sdr(self):
        script_path = pathlib.Path(__file__).resolve().with_name('sdr.py')
        if not script_path.exists():
            print(f'SDR script not found: {script_path}')
            return
        self.sdr_process = subprocess.Popen([sys.executable, str(script_path)], cwd=str(script_path.parent))

    def on_close(self):
        if self.sdr_process is not None and self.sdr_process.poll() is None:
            self.sdr_process.terminate()
        self.root.destroy()

    def update_fcorr_offset(self, value=None):
        try:
            fcorr = int(float(value)) if value is not None else int(self.fcorr.get())
        except ValueError:
            fcorr = 0
        self.fcorr_offset_label.config(text=f'real offset: {(fcorr * FSTEP) / 1000:.2f} kHz')

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
            "baud": 20000
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