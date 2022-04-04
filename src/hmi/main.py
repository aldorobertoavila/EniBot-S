import tkinter as tk
import os

from PIL import ImageTk, Image
from serial import SerialException
from tkinter import StringVar, messagebox

import device
import graph
import log

CWD = os.getcwd()
IMG_PATH = os.path.join(CWD, '../assets/img')
RUN_PATH = os.path.join(CWD, '../../run')

DEBUG_PATH = os.path.join(RUN_PATH, 'logs')
DATAGEN_PATH = os.path.join(RUN_PATH, 'datagen')
CSV_PATH = os.path.join(DATAGEN_PATH, 'csv')
EXCEL_PATH = os.path.join(DATAGEN_PATH, 'excel')

LED_OFF_PATH = os.path.join(IMG_PATH, 'led_off.png')
LED_ON_PATH = os.path.join(IMG_PATH, 'led_on.png')

LOGO_PATH = os.path.join(IMG_PATH, 'logo.png')
SCHEM_PATH = os.path.join(IMG_PATH, 'schematic.png')

FRAME_PADX = 15
FRAME_PADY = 15
# assets/img/logo.png
LOGO_WIDTH = 195
LOGO_HEIGHT = 100
# assets/img/schematic.png
SCHEM_WIDTH = 400
SCHEM_HEIGHT = 300
# assets/img/led_off.png
LED_WIDTH = 25
LED_HEIGHT = 25

CONTROL_HEIGHT = 3
CONTROL_WIDTH = 6
# Serial Config
BAUDRATE = 57600


def get_photo_image(path, size):
    image = Image.open(path).resize(size)
    return ImageTk.PhotoImage(image)


class SettingsFrame(tk.Frame):
    def __init__(self, parent, **kw):
        tk.Frame.__init__(self, parent, **kw)

        self.header_lb = tk.Label(
            self, font='Roboto 32', text='Settings')
        self.header_lb.pack(anchor=tk.CENTER, expand=True, side=tk.TOP)

        self.forward_velocity_var = StringVar(self, value='100')
        self.leftward_velocity_var = StringVar(self, value='100')
        self.rightward_velocity_var = StringVar(self, value='100')
        self.reverse_velocity_var = StringVar(self, value='100')

        def validate_number(input):
            if input.isdigit():
                return True
            else:
                self.bell()
                return False

        self.validate_number_vcmd = (self.register(validate_number), '%S')

        self.forward_velocity_label = tk.Label(
            self, text="Forward Velocity", font="console 10", pady=5)
        self.forward_velocity_label.pack(fill=tk.X)

        self.forward_velocity_textbox = tk.Entry(
            self, validate='key', textvariable=self.forward_velocity_var, vcmd=self.validate_number_vcmd)
        self.forward_velocity_textbox.pack()

        self.leftward_velocity_label = tk.Label(
            self, text="Leftward Velocity", font="console 10", pady=5)
        self.leftward_velocity_label.pack(fill=tk.X)

        self.leftward_velocity_textbox = tk.Entry(
            self, validate='key', textvariable=self.leftward_velocity_var, vcmd=self.validate_number_vcmd)
        self.leftward_velocity_textbox.pack()

        self.rightward_velocity_label = tk.Label(
            self, text="Rightward Velocity", font="console 10", pady=5)
        self.rightward_velocity_label.pack(fill=tk.X)

        self.rightward_velocity_textbox = tk.Entry(
            self, validate='key', textvariable=self.rightward_velocity_var, vcmd=self.validate_number_vcmd)
        self.rightward_velocity_textbox.pack()

        self.reverse_velocity_label = tk.Label(
            self, text="Reverse Velocity", font="console 10", pady=5)
        self.reverse_velocity_label.pack(fill=tk.X)

        self.reverse_velocity_textbox = tk.Entry(
            self, validate='key', textvariable=self.reverse_velocity_var, vcmd=self.validate_number_vcmd)
        self.reverse_velocity_textbox.pack()

        self.forward_velocity_var.trace("w", self.forward_limit_byte)
        self.leftward_velocity_var.trace("w", self.leftward_limit_byte)
        self.rightward_velocity_var.trace("w", self.rightward_limit_byte)
        self.reverse_velocity_var.trace("w", self.reverse_limit_byte)

        self.submit_button = tk.Button(self, text='Submit')
        self.submit_button.pack()

    def forward_limit_byte(self, *args):
        val = self.forward_velocity_var.get()
        if len(val) > 3:
            self.forward_velocity_var.set(val[:3])

    def leftward_limit_byte(self, *args):
        val = self.leftward_velocity_var.get()
        if len(val) > 3:
            self.leftward_velocity_var.set(val[:3])

    def rightward_limit_byte(self, *args):
        val = self.rightward_velocity_var.get()
        if len(val) > 3:
            self.rightward_velocity_var.set(val[:3])

    def reverse_limit_byte(self, *args):
        val = self.reverse_velocity_var.get()
        if len(val) > 3:
            self.reverse_velocity_var.set(val[:3])


class ControlsFrame(tk.Frame):
    def __init__(self, parent, **kw):
        tk.Frame.__init__(self, parent, **kw)

        self.header_lb = tk.Label(
            self, font='Roboto 32', text='Controls')
        self.header_lb.pack(anchor='center', expand=True, side='top')

        self.directions_frame = tk.Frame(self)
        self.directions_frame.pack()

        self.forward_button = tk.Button(
            self.directions_frame, height=CONTROL_HEIGHT, width=CONTROL_WIDTH, text="f")
        self.forward_button.grid(row=0, column=1)

        self.leftward_button = tk.Button(
            self.directions_frame, height=CONTROL_HEIGHT, width=CONTROL_WIDTH, text="l")
        self.leftward_button.grid(row=1, column=0)

        self.rightward_button = tk.Button(
            self.directions_frame, height=CONTROL_HEIGHT, width=CONTROL_WIDTH, text="r")
        self.rightward_button.grid(row=1, column=2)

        self.backward_button = tk.Button(
            self.directions_frame, height=CONTROL_HEIGHT, width=CONTROL_WIDTH, text="b")
        self.backward_button.grid(row=2, column=1)

        self.directions_frame.grid_rowconfigure(1, weight=1)
        self.directions_frame.grid_columnconfigure(1, weight=1)


class GraphFrame(tk.Frame):
    def __init__(self, parent, **kw):
        tk.Frame.__init__(self, parent, **kw)

        self.header_lb = tk.Label(
            self, font='Roboto 32', text='Sensors')
        self.header_lb.grid(row=1, column=1, sticky="ew")

        self.sensors_frame = tk.Frame(self)
        self.sensors_frame.grid(row=2, column=1, sticky="w")

        self.led_off_photo = get_photo_image(
            LED_OFF_PATH, (LED_WIDTH, LED_HEIGHT))
        self.led_on_photo = get_photo_image(
            LED_ON_PATH, (LED_WIDTH, LED_HEIGHT))

        self.infrared1 = tk.Label(self.sensors_frame, height=LED_HEIGHT, width=LED_WIDTH,
                                  padx=FRAME_PADX, pady=FRAME_PADY, image=self.led_off_photo)
        self.infrared1.pack(side="left", fill=None, expand=False)

        self.infrared2 = tk.Label(self.sensors_frame, height=LED_HEIGHT, width=LED_WIDTH,
                                  padx=FRAME_PADX, pady=FRAME_PADY, image=self.led_off_photo)
        self.infrared2.pack(side="left", fill=None, expand=False)

        self.infrared3 = tk.Label(self.sensors_frame, height=LED_HEIGHT, width=LED_WIDTH,
                                  padx=FRAME_PADX, pady=FRAME_PADY, image=self.led_off_photo)
        self.infrared3.pack(side="left", fill=None, expand=False)

        self.ultrasonic1_button = tk.Button(self.sensors_frame, text='U1')
        self.ultrasonic1_button.pack(side="left", fill=None, expand=False)

        self.ultrasonic2_button = tk.Button(self.sensors_frame, text='U2')
        self.ultrasonic2_button.pack(side="left", fill=None, expand=False)

        self.ultrasonic3_button = tk.Button(self.sensors_frame, text='U3')
        self.ultrasonic3_button.pack(side="left", fill=None, expand=False)

        self.grid_columnconfigure(1, weight=1)


class HMI(tk.Tk):
    def __init__(self, parent, **kw):
        tk.Tk.__init__(self, parent, **kw)
        self.title(parent)
        self.resizable(0, 0)
        self.geometry("900x480")

        self.filename = log.get_filename()
        self.logger = log.get_logger(DEBUG_PATH, self.filename)

        self.arduino = device.EniBot()
        self.arduino.connect_to(device.get_devices()[0], BAUDRATE)

        self.header_frame = tk.Frame(
            self, padx=FRAME_PADX, pady=FRAME_PADY)
        self.header_frame.pack(fill=tk.X, side=tk.TOP)
        self.logo_photo = get_photo_image(LOGO_PATH, (LOGO_WIDTH, LOGO_HEIGHT))

        self.logo = tk.Label(self.header_frame, width=LOGO_WIDTH, height=LOGO_HEIGHT,
                             padx=FRAME_PADX, pady=FRAME_PADY, image=self.logo_photo)
        self.logo.pack(fill=tk.Y, side=tk.LEFT)

        self.header_lb = tk.Label(
            self.header_frame, font='Roboto 32', text='Monitor View for EniBot-S')
        self.header_lb.pack(anchor=tk.CENTER, expand=True, side=tk.TOP)

        self.body_frame = tk.Frame(
            self, padx=FRAME_PADX, pady=FRAME_PADY)
        self.body_frame.pack(fill=tk.X, side=tk.TOP)

        self.debug_frame = log.ConsoleFrame(
            self.body_frame, self.logger, self.arduino, height=550, width=280, padx=FRAME_PADX, pady=FRAME_PADY, bg='#66142a')
        self.debug_frame.pack(fill=tk.Y, side=tk.LEFT)

        self.schem_frame = tk.Frame(self.body_frame, height=550, width=480)
        self.schem_frame.pack(fill=tk.Y, side=tk.RIGHT)

        def open_win():
            self.attributes('-disabled', True)
            self.debug_frame.disable()
            self.monitor_button['state'] = 'disabled'
            monitor = graph.GraphMonitor("Monitor", logger=self.logger, filename=self.filename, arduino=self.arduino, baudrate=BAUDRATE, sampling_size=400)
            thread = monitor.collect_data()

            def delete():
                self.attributes('-disabled', False)
                self.debug_frame.enable()
                self.monitor_button.configure(state='normal')
                monitor.destroyed = True
                monitor.destroy()
                thread.join()
            
            monitor.protocol("WM_DELETE_WINDOW", delete)

        self.monitor_button = tk.Button(
            self.debug_frame, text="Open Monitor", command=open_win)
        self.monitor_button.pack(fill=tk.X, side=tk.TOP)

        self.photo = get_photo_image(
            SCHEM_PATH, (SCHEM_WIDTH, SCHEM_HEIGHT))
        self.logo = tk.Label(self.schem_frame, width=SCHEM_WIDTH, height=SCHEM_HEIGHT,
                             padx=FRAME_PADX, pady=FRAME_PADY, image=self.photo)
        self.logo.pack(fill=tk.X, side=tk.TOP)

        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.eval('tk::PlaceWindow . center')
    
    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.destroy()


if __name__ == '__main__':
    root = HMI("EniBot-S")
    root.mainloop()
