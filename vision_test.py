import cv2
from pyzbar.pyzbar import decode
from vision import QRScanner
import numpy as np

if __name__ == "__main__":
    print("Starting QRScanner test. Please present a QR code to the webcam...")
    data, bbox = QRScanner.scan_qr_code()
    if data:
        print("QR Code Detected:")
        print("Data:", data)
        print("Bounding Box:", bbox)
    else:
        print("No QR Code detected.")
