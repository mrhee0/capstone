# main.py
import time
import serial
import math
from kinematics import Kinematics
from vision import QRScanner
import numpy as np
import cv2

def predict_future_position(results, camera_to_robot_distance=30):
    """
    Predict where the QR code will be after a specified time.
    
    Args:
        results: Dictionary from scan_and_track containing position, speed, and direction
        camera_to_robot_distance: Distance in cm from camera to robot arm base
        
    Returns:
        (x_time, y, z) tuple in robot's reference frame
        x_time equals number of seconds it takes for the box to reach the robot's idle position
    """
    if "real_world_speed" in results and results["real_world_speed"] is not None and results["height_above_belt"] is not None:
        _, pos_y = results["final_position"]
        robot_x_time = camera_to_robot_distance / results["real_world_speed"]
        robot_y = pos_y
        robot_z = results["height_above_belt"]
        return (robot_x_time, robot_y, robot_z)
    print("Cannot predict real-world position: calibration data missing")
    return None

# Send 5 comma-separated commands (turret angle, lift1 angle, lift2 angle, servo (never changed), vacuum)
# Changes: don't send servo value, curently 0-180 degrees corresponds to 0-4 command
def main():
    kin = Kinematics(L1=30, L2=25) # Initialize kinematics class with arm lengths (in cm)
    arduino_port = '/dev/tty.usbmodemXXXX' # Update with the actual port
    baud_rate = 9600
    arduino_connected = False
    try:
        arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
        time.sleep(2)
        arduino.write(b'PC connected to Arduino!\n')
        arduino_connected = True
        print("PC connected to Arduino connected!")
    except serial.SerialException as e:
        print("Failed to connect to Arduino:", e)
        return
    
    scanner = QRScanner(camera_id=0, width=1280, height=720, position_history_size=15)
    print("Starting calibration...")
    print("Place QR code on conveyor belt, then raise it when prompted")
    calibration = scanner.calibrate(known_qr_size_cm=7, height_difference_cm=10, display=True)
    if not calibration:
        print("Calibration failed.")
        return
    print("Calibration successful.")
    
    try:
        # Main operational loop
        while True:
            print("\nScanning for QR codes...")
            results = scanner.scan_and_track(
                display=True,
                min_frames=5,
                reliable_frames=10,
                min_speed=0.5,
                max_variance=0.3,
                max_frames_without_detection=3,
                calibration=calibration
            )
            if not results or not results["qr_data"]:
                print("No QR code detected. Trying again...")
                continue
            print(f"QR Data: {results['qr_data']}")
            print(f"Speed: {results['speed']} {results['unit']}")
            if "real_world_speed" in results and results["real_world_speed"] is not None:
                print(f"Real-world speed: {results['real_world_speed']} cm/frame")
            if results["final_position"]:
                pos_x, pos_y = results["final_position"]
                print(f"Current Position (pixels): ({pos_x:.2f}, {pos_y:.2f})")
            if "height_above_belt" in results and results["height_above_belt"] is not None:
                print(f"Height above belt: {results['height_above_belt']:.2f} cm")
    
            future_pos = predict_future_position(results, camera_to_robot_distance=30)
            if future_pos:
                robot_x_time, robot_y, robot_z = future_pos
                try:
                    # Calculate arm angles using kinematics
                    raised_hover_cm = 5
                    theta2 = kin.forward(robot_y, robot_z+raised_hover_cm)
                    theta1 = kin.inverse(robot_y, robot_z+raised_hover_cm, theta2)
                    theta1_deg = math.degrees(theta1)
                    theta2_deg = math.degrees(theta2)
                    print(f"Robot arm angles: θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°")

                    theta2_lower = kin.forward(robot_y, robot_z)
                    theta1_lower = kin.inverse(robot_y, robot_z, theta2)
                    theta1_deg_lower = math.degrees(theta1)
                    theta2_deg_lower = math.degrees(theta2)
                    
                    # Send command to Arduino (turret_angle, lift1_angle, lift2_angle, servo angle, vacuum on/off)
                    command = f"0,{theta1_deg:.2f},{theta2_deg:.2f},0,0\n"
                    arduino.write(command.encode())
                    print(f"Sent hover command to Arduino: {command.strip()}")
                    time.sleep(robot_x_time)
                    
                    command = f"0,{theta1_deg_lower:.2f},{theta2_deg_lower:.2f},0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent pickup command to Arduino: {command.strip()}")
                except Exception as e:
                    print(f"Kinematics calculation error: {e}")
            else:
                print("Unable to predict future position.")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")
    
    finally:
        if arduino_connected:
            arduino.close()
            print("Arduino connection closed.")

if __name__ == "__main__":
    main()
