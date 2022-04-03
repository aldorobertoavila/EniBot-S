from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.pyplot import Figure
from tkinter import messagebox

import matplotlib.animation as anim
import numpy as np
import tkinter as tk
import tkinter.ttk as ttk

import copy
import csv
import os
import serial
import time
import threading

from datetime import datetime
from openpyxl import Workbook
from serial import SerialException
from serial.tools import list_ports

import device
import excel
import log

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

class SerialComboBox(ttk.Combobox):
    def __init__(self, parent):
        ttk.Combobox.__init__(self, parent, state = "readonly", postcommand = self.update_available_devices)        
        self.values = device.get_devices()
        self.bind("<<ComboboxSelected>>", self.on_port_select)
        
    def on_port_select(self, event):
        val = event.widget.get()
        com = val.split(' ')[0]
        global arduino
        if(arduino != None and arduino.is_open):
            arduino.close()
        try:
            arduino = serial.Serial(port = com, baudrate = 115200, timeout = 1)
        except SerialException as e:
            print(f"Cannot connect to {com}: {e}")

    def update_available_devices(self):
        self.values = device.get_devices()

class GraphMonitor(tk.Tk):
    def __init__(self, parent, logger, sampling_size):
        tk.Tk.__init__(self, parent)
        self.geometry("700x500")
        self.resizable(0, 0)
        self.title(parent)
        self.destroyed = False

        self.sampling_size = sampling_size
        self.xy_list = [ [ ], [ ] ]
        self.xy_offset = 0

        self.arduino = serial.Serial(port = device.get_devices()[0], baudrate = 115200)

        self.combo = SerialComboBox(self)
        self.combo.pack()
        
        self.fig = Figure(figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, self)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(bottom = 0.25)
        self.ax.set(xlim = (0, sampling_size), ylim = (0, sampling_size))
        self.ax.grid()

        self.line = anim.FuncAnimation(self.fig, self.update_line, interval=100, blit=False)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    @threaded
    def collect_data(self):
        ref = log.get_current_millis()
        timer = log.Timer(ref) 
        index = 0
        with open(f"{csv_folder}/{filename}.csv", 'a', newline='') as file:
            fieldnames = [ 'Time', 'U1' ]
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            
            while not self.destroyed:
                try:
                    line = self.arduino.readline().decode("utf-8")
                    array = copy.deepcopy(self.xy_list)
                    array[0].append(timer.millis())
                    array[1].append(float(line))
                    index+=1
                    
                    if(len(array[0]) > self.sampling_size):
                        x = array[0].pop(0)
                        y = array[1].pop(0)
                        writer.writerow({ 'Time': x, 'U1': y })
                    
                    data = copy.deepcopy(array)
                    self.xy_list = data
                except:
                    pass

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.destroyed = True
            self.arduino.close()
            self.destroy()
            excel.convert_csv(f"{excel_folder}/{filename}.xlsx", "Ultrasonic 1", f"{csv_folder}/{filename}.csv")
                
    def update_line(self, i):
        x_list = self.xy_list[0]
        y_list = self.xy_list[1]    
            
        if(len(x_list) > 0):
            # x_min = i - 10 if i - 10 > 0 else 0
            # ax.set(xlim = (x_min, i + 50), ylim = (0, 300))
            self.ax.set(xlim = (0, 10000), ylim = (0, 300))
            self.ax.plot(x_list, y_list, color = 'orange')

csv_folder = "../../run/datagen/csv"
excel_folder = "../../run/datagen/excel"
logs_folder = "../../run/logs"

filename = log.get_filename()
logger = log.get_logger(logs_folder, filename)

monitor = GraphMonitor("Ultrasonic 1", logger = logger, sampling_size = 100)
thread = monitor.collect_data()
monitor.mainloop()
thread.join()