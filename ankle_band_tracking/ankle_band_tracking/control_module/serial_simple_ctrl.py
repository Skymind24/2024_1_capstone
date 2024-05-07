#!/home/nvidia/env/bin/python
import serial
import argparse
import threading

class UGVControl:
    def __init__(self):
        self.input = ""

    def set_input(self, _input):
        self.input = _input

    def read_serial(self):
        while True:
            data = ser.readline().decode('utf-8')
            if data:
                print(f"Received: {data}", end='')

    def connect(self, _portName):
        global ser
        #parser = argparse.ArgumentParser(description='Serial JSON Communication')
        # parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')

        # args = parser.parse_args()

        ser = serial.Serial(_portName, baudrate=115200, dsrdtr=None)
        ser.setRTS(False)
        ser.setDTR(False)

        serial_recv_thread = threading.Thread(target=self.read_serial)
        serial_recv_thread.daemon = True
        serial_recv_thread.start()

        try:
            while True:
                command = input(self.input)
                ser.write(command.encode() + b'\n')
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
