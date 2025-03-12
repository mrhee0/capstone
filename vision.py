import cv2
from pyzbar.pyzbar import decode
import numpy as np
import time
from collections import deque

class QRScanner:
    def __init__(self, camera_id=0, width=1920, height=1080, position_history_size=10):
        """
        Initialize the QR code speed tracker.
        
        Args:
            camera_id: Camera device ID (default 0)
            width: Frame width
            height: Frame height
            position_history_size: Number of positions to store for speed calculation
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.position_history = deque(maxlen=position_history_size)
        self.timestamp_history = deque(maxlen=position_history_size)
        self.last_qr_data = None
        
    def calculate_center(self, bbox):
        """Calculate the center point of a bounding box."""
        if not bbox or len(bbox) != 4:
            return None
        
        x_coords = [p[0] for p in bbox]
        y_coords = [p[1] for p in bbox]
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)
        
        return (center_x, center_y)
    
    def calculate_speed(self):
        """Calculate speed from position history in pixels/frame."""
        if len(self.position_history) < 2:
            return 0.0, None, "px/frame"
            
        distances = []
        for i in range(1, len(self.position_history)):
            prev_pos = self.position_history[i-1]
            curr_pos = self.position_history[i]
            # Calculate Euclidean distance
            dx = curr_pos[0] - prev_pos[0]
            dy = curr_pos[1] - prev_pos[1]
            distance = np.sqrt(dx*dx + dy*dy)
            distances.append(distance)
        
        if not distances:
            return 0.0, None, "px/frame"
            
        # Calculate average speed in pixels per frame
        avg_speed_px = sum(distances) / len(distances)
            
        # Direction vector from first to last point (for overall movement direction)
        if len(self.position_history) >= 2:
            first_pos = self.position_history[0]
            last_pos = self.position_history[-1]
            dir_x = last_pos[0] - first_pos[0]
            dir_y = last_pos[1] - first_pos[1]
            
            # Normalize vector
            magnitude = np.sqrt(dir_x*dir_x + dir_y*dir_y)
            if magnitude > 0:
                dir_x /= magnitude
                dir_y /= magnitude
            direction = (dir_x, dir_y)
        else:
            direction = None
        return avg_speed_px, direction, "px/frame"
    
    def scan_and_track(self, display=True, exit_key='q', min_frames=10):
        """
        Continuously scan for QR codes and track their speed.
        
        Args:
            display: Whether to display the video feed
            exit_key: Key to press to exit the loop
            min_frames: Minimum frames to collect before calculating speed
            
        Returns:
            Dictionary with QR data, speed, and direction information
        """
        # Open the webcam
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_AVFOUNDATION)
        cap.set(3, self.width)
        cap.set(4, self.height)
        if not cap.isOpened():
            print("Failed to open camera")
            return None
            
        results = {
            "qr_data": None,
            "speed": 0.0,
            "direction": None,
            "unit": "px/s"
        }
        self.position_history.clear()
        self.timestamp_history.clear()
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break
                current_time = time.time()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                qr_codes = decode(gray)
                
                # Process QR codes
                for qr in qr_codes:
                    qr_data = qr.data.decode('utf-8')
                    results["qr_data"] = qr_data
                    self.last_qr_data = qr_data
                    pts = qr.polygon
                    if len(pts) == 4:
                        qr_bbox = [(p.x, p.y) for p in pts]
                        center = self.calculate_center(qr_bbox)
                        
                        if center:
                            # Add to position history
                            self.position_history.append(center)
                            self.timestamp_history.append(current_time)
                            
                            # Calculate speed if we have enough data points
                            if len(self.position_history) >= min_frames:
                                speed, direction, unit = self.calculate_speed()
                                results["speed"] = round(speed, 2)
                                results["direction"] = direction
                                results["unit"] = unit
                        
                        # Draw QR code boundary
                        if display:
                            cv2.polylines(frame, [np.array(qr_bbox, dtype=np.int32)], True, (0, 255, 0), 3)
                            if center and "speed" in results:
                                # Draw speed information
                                speed_text = f"Speed: {results['speed']} {results['unit']}"
                                cv2.putText(frame, speed_text, (int(center[0]), int(center[1]) - 20), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                
                                # Draw QR data
                                data_text = f"QR: {qr_data}"
                                cv2.putText(frame, data_text, (int(center[0]), int(center[1]) - 50), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                
                                # Draw movement trail
                                if len(self.position_history) >= 2:
                                    points = [np.array([int(x), int(y)]) for x, y in self.position_history]
                                    for i in range(1, len(points)):
                                        cv2.line(frame, tuple(points[i-1]), tuple(points[i]), (255, 0, 0), 2)
                # Display the frame
                if display:
                    cv2.imshow('QR Code Speed Tracker', frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(exit_key):
                        break
        finally:
            cap.release()
            if display:
                cv2.destroyAllWindows()
        return results
