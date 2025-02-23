# main.py
import time
import serial
from kinematics import Kinematics
from vision import QRScanner
import numpy as np
import cv2

def compute_center(bbox):
    """
    Given a list of (x, y) tuples, compute the center point.
    """
    if bbox and len(bbox) > 0:
        center_x = sum([pt[0] for pt in bbox]) / len(bbox)
        center_y = sum([pt[1] for pt in bbox]) / len(bbox)
        return center_x, center_y
    return None

def main():
    # Initialize kinematics with example arm lengths.
    # You may need to adjust L1 and L2 based on your actual robotic arm configuration.
    kin = Kinematics(L1=100, L2=100)  # Example values in your chosen unit

    # Initialize the serial connection to the Arduino.
    # Update the arduino_port with your actual device identifier.
    arduino_port = '/dev/tty.usbmodemXXXX'
    baud_rate = 9600
    try:
        arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
    except serial.SerialException as e:
        print("Failed to connect to Arduino:", e)
        return

    time.sleep(2)  # Give Arduino time to reset
    arduino.write(b'Mac/Arduino connected!\n')
    print("Arduino connected. Beginning QR scanning...")

    try:
        # Run an infinite loop to continuously scan for QR codes.
        while True:
            # Call the QR scanner function (which opens the camera, reads a frame,
            # and returns as soon as a QR is detected)
            qr_data, qr_bbox = QRScanner.scan_qr_code()
            if qr_bbox:
                center = compute_center(qr_bbox)
                if center:
                    x, y = center
                    print(f"QR center: ({x:.2f}, {y:.2f})")
                    try:
                        # Use the kinematics solver to compute angles.
                        theta2 = kin.forward(x, y)
                        theta1 = kin.inverse(x, y, theta2)
                        print(f"Computed angles: theta1 = {theta1:.2f}, theta2 = {theta2:.2f}")
                        
                        # Format the command string to send to Arduino.
                        # Here we send the center coordinates and the computed angles.
                        command = f"{x:.2f},{y:.2f},{theta1:.2f},{theta2:.2f}\n"
                        arduino.write(command.encode())
                        print(f"Sent command: {command.strip()}")
                    except Exception as e:
                        print("Kinematics computation error:", e)
            else:
                print("No QR detected.")

            # A short delay to avoid overwhelming the camera and Arduino.
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        arduino.close()
        print("Arduino connection closed.")

if __name__ == "__main__":
    main()
