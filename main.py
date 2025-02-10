import serial
import time

class Arduino_Writer:
    arduino_port = '/dev/tty.usbmodemXXXX'
    baud_rate = 9600

    arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    arduino.write(b'Mac/Arduino connected!\n')

    while True:
        if arduino.in_waiting > 0:
            response = arduino.readline().decode('utf-8').strip()
            print(f"Arduino says: {response}")

    arduino.close()
