from serial.tools import list_ports

import time
import serial

def get_devices():
    ports = []
    for port in list_ports.comports():
        if("Arduino" in port.description):
            ports.append(port.device)
    return ports

class EniBot:
    def __init__(self):
        self.serial = serial.Serial()

    def connect_to(self, port, baudrate):
        self.disconnect()
        time.sleep(2)
        # opens new port
        self.serial.baudrate = baudrate
        self.serial.port = port
        self.serial.open()
        time.sleep(2)
    
    def disconnect(self):
        self.serial.close()
            
    def is_open(self):
        return self.serial.is_open
    
    def read_sensors(self):
        res = self.send_command(b"sensor -get\n")
        arr = res.split(',')
        return list(map(float, arr))
    
    def read_string(self):
        return self.serial.readline().decode("utf-8")
        
    def send_command(self, command):
        self.serial.write(command)
        return self.read_string()