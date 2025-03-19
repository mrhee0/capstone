import cv2
from pyzbar.pyzbar import decode
from vision import QRScanner
import numpy as np

if __name__ == "__main__":
    scanner = QRScanner(camera_id=0, width=1920, height=1080, position_history_size=15)

    # Calibrate the QR height/size
    calibration = None
    calibration = scanner.calibrate(known_qr_size_cm=7, height_difference_cm=12)

    results = scanner.scan_and_track(
        display=True,
        min_frames=5,
        reliable_frames=10,
        min_speed=0.5,
        max_variance=0.2
    )
    
    if results and results["qr_data"]:
        print(f"QR Data: {results['qr_data']}")
        print(f"Speed: {results['speed']} {results['unit']}")
        if results["direction"]:
            print(f"Direction: ({results['direction'][0]:.2f}, {results['direction'][1]:.2f})")
        if results["final_position"]:
            print(f"Final Position: ({results['final_position'][0]:.2f}, {results['final_position'][1]:.2f})")
        if results["box_size"]:
            print(f"Box Size: {results['box_size']['width']:.2f} × {results['box_size']['height']:.2f} px")
            print(f"Box Area: {results['box_size']['area']:.2f} px²")
            
            # If calibrated, show real-world measurements
            if calibration and calibration["pixels_per_cm"]:
                width_cm = results["box_size"]["width"] / calibration["pixels_per_cm"]
                height_cm = results["box_size"]["height"] / calibration["pixels_per_cm"]
                print(f"Real-world size: {width_cm:.2f} × {height_cm:.2f} cm")
    else:
        print("No QR code detected")