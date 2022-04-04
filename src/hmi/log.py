import tkinter as tk
import tkinter.ttk as ttk

from tkinter import scrolledtext
from datetime import datetime

import logging
import queue
import time


class Timer:
    def __init__(self, ref=0):
        self.ref = ref

    def seconds(self):
        return time.time() - self.ref


def get_filename():
    return datetime.today().strftime("%Y-%m-%d-%H.%M.%S")


def get_logger(folder, filename):
    logging.basicConfig(filename=f"{folder}/{filename}.log",
                        format="%(asctime)s %(levelname)s %(message)s", filemode="w")
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    return logger


class QueueHandler(logging.Handler):
    def __init__(self, log_queue):
        super().__init__()
        self.log_queue = log_queue

    def emit(self, record):
        self.log_queue.put(record)


class ConsoleFrame(tk.Frame):
    def __init__(self, parent, logger, arduino, **kw):
        tk.Frame.__init__(self, parent, **kw)
        self.arduino = arduino
        self.logger = logger
        self.log_queue = queue.Queue()
        self.handler = QueueHandler(self.log_queue)

        # set format
        self.handler.setFormatter(
            logging.Formatter('%(asctime)s: %(message)s', '%H:%M:%S'))
        self.logger.addHandler(self.handler)

        self.scrolled_text = scrolledtext.ScrolledText(self, state='disabled', height=12, width=30)
        self.scrolled_text.pack(fill=tk.X, side=tk.TOP)

        self.scrolled_text.configure(font='TkFixedFont')
        self.scrolled_text.tag_config('INFO', foreground='black')
        self.scrolled_text.tag_config('DEBUG', foreground='gray')
        self.scrolled_text.tag_config('WARNING', foreground='orange')
        self.scrolled_text.tag_config('ERROR', foreground='red')
        self.scrolled_text.tag_config('CRITICAL', foreground='red', underline=1)

        self.form_frame = tk.Frame(self, height=15, width=60)
        self.form_frame.pack(fill=tk.X, side=tk.TOP)
        
        self.message = tk.StringVar()

        self.entry = ttk.Entry(
            self.form_frame, textvariable=self.message, font="Roboto 12", width=30)
        self.entry.pack(fill=tk.Y, side=tk.LEFT)

        self.button = ttk.Button(self.form_frame, text='Send', command=self.submit, width=20)
        self.button.pack(fill=tk.Y, side=tk.LEFT)
        
        self.logger.setLevel(logging.DEBUG)
        self.logger.debug('EniBot-S ready for service!')
        self.logger.setLevel(logging.INFO)

        self.after(100, self.poll)

    def submit(self):
        msg = self.message.get()
        self.logger.setLevel(logging.DEBUG)
        self.logger.debug(msg)
        self.message.set('')

        res = self.arduino.send_command(bytes(msg, encoding='utf8'))
        self.logger.debug(res)
        
        self.after(100, self.poll)
        
    def disable(self):
        self.button['state'] = 'disable'
    
    def enable(self):
        self.button['state'] = 'normal'

    def display(self, record):
        msg = self.handler.format(record)
        self.scrolled_text.configure(state='normal')
        self.scrolled_text.insert(tk.END, msg + '\n', 'DEBUG')
        self.scrolled_text.configure(state='disabled')
        self.scrolled_text.yview(tk.END)

    def poll(self):
        while True:
            try:
                record = self.log_queue.get(block=False)
            except queue.Empty:
                break
            else:
                self.display(record)
        self.after(100, self.poll)
