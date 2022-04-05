from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.pyplot import Figure

from main import CSV_PATH

import matplotlib.animation as anim
import tkinter as tk
import tkinter.ttk as ttk

import copy
import csv
import os
import time
import threading

import device
import main
import log

FRAME_PADX = main.FRAME_PADX
FRAME_PADY = main.FRAME_PADY

LED_OFF_PATH = main.LED_OFF_PATH
LED_ON_PATH = main.LED_ON_PATH

# assets/img/led_off.png
LED_WIDTH = main.LED_WIDTH
LED_HEIGHT = main.LED_HEIGHT

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper


class SerialComboBox(ttk.Combobox):
    def __init__(self, parent):
        ttk.Combobox.__init__(self, parent, state="readonly",
                              postcommand=self.update_devices)
        devices = device.get_devices()
        self['values'] = devices

        if(len(devices) > 0):
            self.current(0)

    def update_devices(self):
        self['values'] = device.get_devices()


class GraphMonitor(tk.Tk):
    def __init__(self, parent, logger, arduino, filename, fieldnames, baudrate, sampling_size):
        tk.Tk.__init__(self, parent)
        self.geometry("900x400")
        self.resizable(0, 0)
        self.title(parent)
        self.destroyed = False

        self.arduino = arduino
        self.baudrate = baudrate
        self.filename = filename
        self.sampling_size = sampling_size
        self.fieldnames = fieldnames
        # [ [Time], [U1], [U2], [U3], [IR1], [IR2], [IR3], [EN_A], [EN_B] ]
        self.xy_list = [[], [], [], [], [], [], [], [], []]
        self.xy_offset = 0

        # self.devices_combo = SerialComboBox(self)
        # self.devices_combo.pack()
        # self.devices_combo.bind("<<ComboboxSelected>>", self.on_port_select)

        self.fig = Figure(figsize=(12, 9))
        self.canvas = FigureCanvasTkAgg(self.fig, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=tk.YES)

        self.fig.subplots_adjust(bottom=0.25, wspace=0.5)
        self.ultrasonic = self.fig.add_subplot(121)
        self.ultrasonic.set_title("Ultrasonic")
        self.ultrasonic.set_xlabel("Time")
        self.ultrasonic.set_ylabel("Distance")
        self.ultrasonic.set(xlim=(0, sampling_size), ylim=(0, sampling_size))
        self.ultrasonic.grid()

        self.encoder = self.fig.add_subplot(122)
        self.encoder.set_title("Encoder")
        self.encoder.set_xlabel("Time")
        self.encoder.set_ylabel("PWM")
        self.encoder.set(xlim=(0, sampling_size), ylim=(0, sampling_size))
        self.encoder.grid()

        self.line = anim.FuncAnimation(
            self.fig, self.update_line, interval=10, blit=False)
        
        # self.ir_frame = main.IRFrame(self)
        # self.ir_frame.pack(fill=tk.X, side=tk.TOP)

    #def on_port_select(self, event):
    #    com = event.widget.get()
    #    try:
    #        self.arduino.connect_to(com, self.baudrate)
    #    except SerialException as e:
    #        print(f"Cannot connect to {com}: {e}")

    def writeRow(self, array, writer):
        row = {}
        for i, y in enumerate(array):
            row[self.fieldnames[i]] = y.pop(0)
        writer.writerow(row)

    @threaded
    def collect_data(self):
        path = os.path.join(CSV_PATH, f"{self.filename}.csv")
        ref = time.time()
        timer = log.Timer(ref)
        with open(path, 'a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            while not self.destroyed:
                try:
                    sensors = self.arduino.read_sensors()
                    array = copy.deepcopy(self.xy_list)
                    array[0].append(timer.seconds())

                    for i, y in enumerate(array[1:]):
                        y.append(sensors[i])

                    if(len(array[0]) > self.sampling_size):
                        self.writeRow(array, writer)

                    data = copy.deepcopy(array)
                    self.xy_list = data
                except Exception as e:
                    pass
            
            # saves data samples under sampling size
            for i in range(len(self.xy_list[0])):
                self.writeRow(self.xy_list, writer)

    def update_line(self, i):
        time_list = self.xy_list[0]
        u1_list = self.xy_list[1]
        u2_list = self.xy_list[2]
        u3_list = self.xy_list[3]
        en_a_list = self.xy_list[7]
        en_b_list = self.xy_list[8]

        if(len(time_list) > 0):
            x_min = time_list[0]
            x_max = time_list[len(time_list) - 1]

            self.ultrasonic.set(xlim=(x_min, x_max + 0.1), ylim=(0, 300))
            self.ultrasonic.plot(time_list, u1_list, label='U1', color='orange')
            self.ultrasonic.plot(time_list, u2_list, label='U2', color='yellow')
            self.ultrasonic.plot(time_list, u3_list, label='U3', color='green')
            
            self.encoder.set(xlim=(x_min, x_max + 0.1), ylim=(0, 255))
            self.encoder.plot(time_list, en_a_list, label='EN_A', color='blue')
            self.encoder.plot(time_list, en_b_list, label='EN_B', color='red')