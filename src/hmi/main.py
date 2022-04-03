import tkinter as tk
import tkinter.ttk as tkk

import os

from PIL import ImageTk, Image
from tkinter import StringVar, messagebox

LOGO = os.path.join(os.getcwd(), '../assets/img/logo.png')
SCHEM = os.path.join(os.getcwd(), '../assets/img/schematic.png')

FRAME_PADX = 15
FRAME_PADY = 15
# assets/img/logo.png
LOGO_WIDTH = 175
LOGO_HEIGHT = 100
# assets/img/schematic.png
SCHEM_WIDTH = 380
SCHEM_HEIGHT = 300


def get_photo_image(path, size):
    image = Image.open(path).resize(size)
    return ImageTk.PhotoImage(image)


class ConfigFrame(tk.Frame):
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


class HMI(tk.Tk):
    def __init__(self, parent, **kw):
        tk.Tk.__init__(self, parent, **kw)
        self.title(parent)
        self.resizable(0, 0)
        self.geometry("900x700")

        self.header_frame = tk.Frame(
            self, padx=FRAME_PADX, pady=FRAME_PADY)
        self.header_frame.pack(fill=tk.X, side=tk.TOP)
        self.logo_photo = get_photo_image(LOGO, (LOGO_WIDTH, LOGO_HEIGHT))

        self.logo = tk.Label(self.header_frame, width=LOGO_WIDTH, height=LOGO_HEIGHT,
                             padx=FRAME_PADX, pady=FRAME_PADY, image=self.logo_photo)
        self.logo.pack(fill=tk.Y, side=tk.LEFT)

        self.header_lb = tk.Label(
            self.header_frame, font='Roboto 32', text='Monitor View for EniBot-S')
        self.header_lb.pack(anchor=tk.CENTER, expand=True, side=tk.TOP)

        self.body_frame = tk.Frame(
            self, padx=FRAME_PADX, pady=FRAME_PADY, bg='#64a0dd')
        self.body_frame.pack(fill=tk.X, side=tk.TOP)

        self.config_frame = ConfigFrame(
            self.body_frame, height=400, width=380, bg='#dde061')
        self.config_frame.pack(side=tk.LEFT)

        self.schem_frame = tk.Frame(self.body_frame, height=200, width=380)
        self.schem_frame.pack(fill=tk.Y, side=tk.RIGHT)

        self.photo = get_photo_image(
            SCHEM, (SCHEM_WIDTH + 35, SCHEM_HEIGHT + 35))
        self.logo = tk.Label(self.schem_frame, width=SCHEM_WIDTH, height=SCHEM_HEIGHT,
                             padx=FRAME_PADX, pady=FRAME_PADY, image=self.photo)
        self.logo.pack(fill=tk.X, side=tk.TOP)


if __name__ == '__main__':
    root = HMI("EniBot-S")
    root.mainloop()
