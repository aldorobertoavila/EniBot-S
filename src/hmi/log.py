import tkinter as tk
import tkinter.ttk as ttk

from tkinter import scrolledtext
from datetime import datetime

import logging
import queue
import serial
import time


class Timer:
    def __init__(self, ref):
        self.ref = ref

    def millis(self):
        return get_current_millis() - self.ref


def get_current_millis():
    return round(time.time() * 1000)


def get_filename():
    return datetime.today().strftime("%Y-%m-%d-%H.%M.%S")


def get_logger(folder, filename):
    logging.basicConfig(filename=f"{folder}/{filename}.log",
                        format="%(asctime)s %(levelname)s %(message)s", filemode="w")
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    return logger


class Arduino():
    def __init__(self, device):
        self.port = device

    def sendCommand(self, command):
        self.port.write(command)
        echo = self.read_string()
        res = self.read_string()
        print(echo)
        print(res)
        return [echo, res]

    def read_string(self):
        return self.port.readline().decode("utf-8")


class QueueHandler(logging.Handler):
    def __init__(self, log_queue):
        super().__init__()
        self.log_queue = log_queue

    def emit(self, record):
        self.log_queue.put(record)


class ConsoleFrame(tk.Frame):
    def __init__(self, parent, logger, **kw):
        tk.Frame.__init__(self, parent, **kw)
        self.logger = logger

        self.vertical_pane = ttk.PanedWindow(self, orient=tk.VERTICAL)
        self.vertical_pane.grid(row=0, column=0, sticky="nsew")
        self.horizontal_pane = ttk.PanedWindow(
            self.vertical_pane, orient=tk.HORIZONTAL)
        self.vertical_pane.add(self.horizontal_pane)
        self.form_frame = ttk.Labelframe(self.horizontal_pane, text="MyForm")
        self.form_frame.columnconfigure(1, weight=1)
        self.horizontal_pane.add(self.form_frame, weight=1)
        self.console_frame = ttk.Labelframe(
            self.horizontal_pane, text="Console")
        self.console_frame.columnconfigure(0, weight=1)
        self.console_frame.rowconfigure(0, weight=1)
        self.horizontal_pane.add(self.console_frame, weight=1)
        self.third_frame = ttk.Labelframe(
            self.vertical_pane, text="Third Frame")
        self.vertical_pane.add(self.third_frame, weight=1)

        self.form = FormUi(self.form_frame, self.logger)
        self.console = ConsoleUi(self.console_frame, self.logger)


class ConsoleUi:
    def __init__(self, frame, logger):
        self.frame = frame
        self.logger = logger

        self.scrolled_text = scrolledtext.ScrolledText(
            frame, state='disabled', height=12)
        self.scrolled_text.grid(
            row=0, column=0, sticky=(tk.N, tk.S, tk.W, tk.E))
        self.scrolled_text.configure(font='TkFixedFont')
        self.scrolled_text.tag_config('INFO', foreground='black')
        self.scrolled_text.tag_config('DEBUG', foreground='gray')
        self.scrolled_text.tag_config('WARNING', foreground='orange')
        self.scrolled_text.tag_config('ERROR', foreground='red')
        self.scrolled_text.tag_config(
            'CRITICAL', foreground='red', underline=1)

        self.log_queue = queue.Queue()
        self.queue_handler = QueueHandler(self.log_queue)
        formatter = logging.Formatter('%(asctime)s: %(message)s')

        self.queue_handler.setFormatter(formatter)
        self.logger.addHandler(self.queue_handler)

        self.frame.after(100, self.poll_log_queue)

    def display(self, record):
        msg = self.queue_handler.format(record)
        self.scrolled_text.configure(state='normal')
        self.scrolled_text.insert(tk.END, msg + '\n', record.levelname)
        self.scrolled_text.configure(state='disabled')

        self.scrolled_text.yview(tk.END)

    def poll_log_queue(self):
        while True:
            try:
                record = self.log_queue.get(block=False)
            except queue.Empty:
                break
            else:
                self.display(record)
        self.frame.after(100, self.poll_log_queue)


class FormUi:
    def __init__(self, frame, logger):
        self.logger = logger
        self.frame = frame
        values = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        self.level = tk.StringVar()
        ttk.Label(self.frame, text='Level:').grid(column=0, row=0, sticky=tk.W)
        self.combobox = ttk.Combobox(
            self.frame,
            textvariable=self.level,
            width=25,
            state='readonly',
            values=values
        )
        self.combobox.current(0)
        self.combobox.grid(column=1, row=0, sticky=(tk.W, tk.E))
        # Create a text field to enter a message
        self.message = tk.StringVar()
        ttk.Label(self.frame, text='Message:').grid(
            column=0, row=1, sticky=tk.W)
        ttk.Entry(self.frame, textvariable=self.message, width=25).grid(
            column=1, row=1, sticky=(tk.W, tk.E))
        # Add a button to log the message
        self.button = ttk.Button(
            self.frame, text='Submit', command=self.submit_message)
        self.button.grid(column=1, row=2, sticky=tk.W)

    def submit_message(self):
        lvl = getattr(logging, self.level.get())
        self.logger.log(lvl, self.message.get())
