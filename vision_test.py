import cv2
from pyzbar.pyzbar import decode
from vision import QRScanner
import numpy as np

# if __name__ == "__main__":
#     print("Starting QRScanner test. Please present a QR code to the webcam...")
#     data, bbox = QRScanner.scan_qr_code()
#     if data:
#         print("QR Code Detected:")
#         print("Data:", data)
#         print("Bounding Box:", bbox)
#     else:
#         print("No QR Code detected.")

if __name__ == "__main__":
    tracker = QRScanner(camera_id=0, width=1920, height=1080, position_history_size=15)
    results = tracker.scan_and_track(display=True, min_frames=5)
    if results["qr_data"]:
        print(f"QR Data: {results['qr_data']}")
        print(f"Speed: {results['speed']} {results['unit']}")
        if results["direction"]:
            print(f"Direction: ({results['direction'][0]:.2f}, {results['direction'][1]:.2f})")
    else:
        print("No QR code detected")