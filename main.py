# main.py
import time
import serial
import math
from kinematics import Kinematics
from vision import QRScanner
import numpy as np
import cv2

def predict_future_position(results, camera_to_robot_distance_x=3, robot_y_offset=13, robot_z_offset=10, calibration=None):
    """
    Predict where the QR code will be after a specified time.
    
    Args:
        results: Dictionary from scan_and_track containing position, speed, and direction
        camera_to_robot_distance: Distance in cm from camera to robot arm base
        calibration: Webcam calibration results
        
    Returns:
        (x_time, y, z) tuple in robot's reference frame
        x_time equals number of seconds it takes for the box to reach the robot's idle position
    """
    if "real_world_speed" in results and results["real_world_speed"] is not None and results["height_above_belt"] is not None:
        pos_x, pos_y = results["final_position"]
        robot_x_time = (camera_to_robot_distance_x+pos_x/(calibration["pixels_per_cm_belt"]+calibration["change_in_pixels_per_cm"]*results["height_above_belt"])) / results["real_world_speed"]
        robot_y_cm = (pos_y-calibration["top_edge_y"])/calibration["pixels_per_cm_belt"]+robot_y_offset
        robot_z = results["height_above_belt"] - robot_z_offset
        return (robot_x_time, robot_y_cm, robot_z)
    print("Cannot predict real-world position: calibration data missing")
    return None

def l1_deg_to_voltage(degree):
    # Convert from degrees (0-360) to voltage (0-5) for lift1
    return round(((degree+12) / 360) * 5,2)

def l2_deg_to_voltage(degree):
    # Convert from degrees (0-360) to voltage (0-5) for lift2
    return round(((170-degree) / 360) * 5,2)

def safe_sleep(arduino, seconds):
    """Sleep without closing Arduino connection"""
    start_time = time.time()
    while time.time() - start_time < seconds:
        if arduino.in_waiting:
            data = arduino.readline().decode().strip()
            print("Arduino:", data)
        time.sleep(0.01)

# Send 5 comma-separated commands (turret angle, lift1 angle, lift2 angle, servo (never changed), vacuum)
# Changes: don't send servo value, curently 0-180 degrees corresponds to 0-4 command
def main():
    L1 = 20
    L2 = 20
    kin = Kinematics(L1=L1, L2=L2) # Initialize kinematics class with arm lengths (in cm)
    arduino_port = '/dev/tty.usbmodem2101'
    baud_rate = 115200
    
    scanner = QRScanner(camera_id=0, width=1280, height=720, position_history_size=15)
    print("Starting calibration...")
    print("Place QR code on conveyor belt, then raise it when prompted")
    calibration = scanner.calibrate(known_qr_size_cm=7, height_difference_cm=11, display=True)
    if not calibration:
        print("Calibration failed.")
        return
    print("Calibration successful.")
    
    # default position (arbitrary)
    turret_voltage_straight = 0.42
    l1_voltage_default = l1_deg_to_voltage(108)
    l2_voltage_default = l2_deg_to_voltage(78)
    default_command = f"{l1_voltage_default},{l2_voltage_default},{turret_voltage_straight},0,0\n"
    try:
        arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
        safe_sleep(arduino,2)
        arduino.flushInput()
        arduino.write('enable\n'.encode())
        print("PC connected to Arduino!")
        safe_sleep(arduino,2)
        arduino.write(default_command.encode())
        print(f"Sent default command to Arduino: {default_command.strip()}")
    except serial.SerialException as e:
        print("Failed to connect to Arduino:", e)
        return

    try:
        # Main operational loop
        while True:
            print("\nScanning for QR codes...")
            results = scanner.scan_and_track(
                display=True,
                min_frames=5,
                reliable_frames=10,
                min_speed=5,
                max_variance=0.3,
                max_frames_without_detection=3,
                fps=60,
                calibration=calibration
            )
            if not results or not results["qr_data"]:
                print("No QR code detected. Trying again...")
                continue
            print(f"QR Data: {results['qr_data']}")
            print(f"Speed: {results['speed']} {results['unit']}")
            if "real_world_speed" in results and results["real_world_speed"] is not None:
                print(f"Real-world speed: {results['real_world_speed']} cm/s")
            if results["final_position"]:
                pos_x, pos_y = results["final_position"]
                print(f"Current Position (pixels): ({pos_x:.2f}, {pos_y:.2f})")
            if "height_above_belt" in results and results["height_above_belt"] is not None:
                print(f"Height above belt: {results['height_above_belt']:.2f} cm")
    
            future_pos = predict_future_position(results, camera_to_robot_distance_x=5, robot_y_offset=13, calibration=calibration)
            if future_pos:
                robot_x_time, robot_y, robot_z = future_pos
                try:
                    gripper_cm = 8
                    robot_z += gripper_cm

                    # Out of reach target
                    d = math.sqrt(robot_y**2 + robot_z**2)
                    if d > (L1 + L2):
                        print("Target is far")
                        robot_z += .2
                        robot_y = math.sqrt((L1 + L2)**2 - robot_z**2) - 1
                        print(robot_y,robot_z)
        
                    raised_hover_cm = 5
                    robot_z_lift = min(robot_z + raised_hover_cm*4, 20)
                    print(robot_x_time)

                    # Wait time normalization
                    # if robot_x_time>1:
                    #     robot_x_time *= robot_x_time/(robot_x_time+.4)
                    # elif robot_x_time>.7:
                    #     robot_x_time *= robot_x_time/(robot_x_time+.2)

                    # Calculate arm angles using kinematics
                    theta1_deg, theta2_deg = kin.inverse_kinematics(robot_y, robot_z+raised_hover_cm)
                    theta1_deg_lower, theta2_deg_lower = kin.inverse_kinematics(robot_y, robot_z)
                    theta1_deg_raised, theta2_deg_raised = kin.inverse_kinematics(robot_y, robot_z_lift)
                    print(f"Robot arm hover angles: θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°")
                    print(f"Robot arm grab angles: θ1={theta1_deg_lower:.2f}°, θ2={theta2_deg_lower:.2f}°")
                    print(f"Robot arm lift angles: θ1={theta1_deg_raised:.2f}°, θ2={theta2_deg_raised:.2f}°")
                    print()

                    # Convert from degrees to voltage (0-5), lift2 actuator is inverted and goes from 0-4
                    theta1_command = l1_deg_to_voltage(theta1_deg)
                    theta2_command = l2_deg_to_voltage(theta2_deg)
                    theta1_command_lower = l1_deg_to_voltage(theta1_deg_lower)
                    theta2_command_lower = l2_deg_to_voltage(theta2_deg_lower)
                    theta1_command_raised = l1_deg_to_voltage(theta1_deg_raised)
                    theta2_command_raised = l2_deg_to_voltage(theta2_deg_raised)
                    
                    bin_x = 25
                    bin_deg1, bin_deg2 = kin.inverse_kinematics(bin_x, robot_z_lift)
                    turret_voltage_bin = None
                    if int(results["qr_data"])%2 == 0:
                        turret_voltage_bin = 1.5
                    else:
                        turret_voltage_bin = 2.3
                    theta1_command_bin = l1_deg_to_voltage(bin_deg1)
                    theta2_command_bin = l2_deg_to_voltage(bin_deg2)
                    arduino.flushInput()

                    # Send command to Arduino (lift1_angle, lift2_angle, turret, servo angle, vacuum on/off)
                    command = f"{theta1_command:.2f},{theta2_command:.2f},{turret_voltage_straight},0,1\n"
                    # command = f"4.1,2,1,0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent hover command to Arduino: {command.strip()}")
                    safe_sleep(arduino,.7)
                    # safe_sleep(arduino,.7)

                    command = f"{theta1_command_lower:.2f},{theta2_command_lower:.2f},{turret_voltage_straight},0,1\n"
                    # command = f"4.2,1.8,1,0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent pickup command to Arduino: {command.strip()}")
                    safe_sleep(arduino,.3)

                    command = f"{theta1_command_raised:.2f},{theta2_command_raised:.2f},{turret_voltage_straight},0,1\n"
                    # command = f"4,2.1,1,0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent lift command to Arduino: {command.strip()}")
                    safe_sleep(arduino,1)

                    command = f"{theta1_command_bin:.2f},{theta2_command_bin:.2f},{turret_voltage_bin},0,1\n"
                    arduino.write(command.encode())
                    print(f"Sent bin command to Arduino: {command.strip()}")
                    safe_sleep(arduino,2)

                    command = f"{theta1_command_bin:.2f},{theta2_command_bin:.2f},{turret_voltage_bin},0,0\n"
                    arduino.write(command.encode())
                    arduino.write("disable\n".encode())
                    arduino.write("enable\n".encode())
                    print(f"Sent bin drop command to Arduino: {command.strip()}")
                    safe_sleep(arduino,1)

                    command = default_command
                    arduino.write(command.encode())
                    print(f"Sent default command to Arduino: {command.strip()}")

                except Exception as e:
                    print(f"Kinematics calculation error: {e}")
            else:
                print("Unable to predict future position.")
            safe_sleep(arduino,2)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")
    
    finally:
        arduino.close()
        print("Arduino connection closed.")

if __name__ == "__main__":
    main()
