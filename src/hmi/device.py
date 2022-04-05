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
        
    def autoconnect(self, baudrate):
        if(self.is_connected()):
            return True
        
        ports = get_devices()
        
        if(len(ports) > 0):
            self.connect_to(ports[0], baudrate)
            return True
        else:
            return False

    def connect_to(self, port, baudrate):
        self.disconnect()
        time.sleep(2)
        # opens new port
        self.serial = serial.Serial(port, baudrate)
        time.sleep(2)
        self.open()
        self.set_manual()
        
    def open(self):
        if(not self.is_open()):
            self.serial.open()
            time.sleep(2)
        
    def set_manual(self):
        self.send_command(b"mode -set MANUAL\n")
        self.send_command(b"state -set STOP\n")
    
    def disconnect(self):
        self.serial.close()
        
    def is_connected(self):
        return self.serial.port in get_devices()
            
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