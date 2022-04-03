from serial.tools import list_ports

import serial

def get_devices():
    ports = []
    for port in list_ports.comports():
        if("Arduino" in port.description):
            ports.append(port.device)
    return ports

class Arduino:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

    def connect(self, port, baudrate):
        self.com = serial.Serial(port, baudrate)
        print("Connect!")
    
    def disconnect(self):
        print("Disconnect!")

    def read_string(self):
        return self.com.readline().decode("utf-8")

    def move_forward(self):
        self.send_command(b"state -set FORWARD -get")
        
    def move_leftward(self):
        self.send_command(b"state -set LEFTWARD -get")
        
    def move_rightward(self):
        self.send_command(b"state -set RIGHTWARD -get")
        
    def move_reverse(self):
        self.send_command(b"state -set REVERSE -get")
        
    def send_command(self, str):
        # time.sleep(1)
        echo = self.read_string()
        res = self.read_string()
        print(echo)
        print(res)
        return [ echo, res ]