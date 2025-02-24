import cv2
from pyzbar.pyzbar import decode
import numpy as np

class QRScanner:
    def scan_qr_code():
        # Open the webcam (change 0 if you have multiple cameras)
        cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
        cap.set(3, 1920) # Set width
        cap.set(4, 1080) # Set height

        qr_data = None
        qr_bbox = None

        while True:
            ret, frame = cap.read() # Read frame from the camera
            if not ret:
                print("Failed to grab frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_codes = decode(gray)

            for qr in qr_codes:
                qr_data = qr.data.decode('utf-8')
                pts = qr.polygon
                if len(pts) == 4: # If the QR code is fully detected
                    qr_bbox = [(p.x, p.y) for p in pts]
                    cv2.polylines(frame, [np.array(qr_bbox, dtype=np.int32)], True, (0, 255, 0), 3)
                else:
                    qr_bbox = None

                cv2.imshow('QR Code Scanner', frame)
                cv2.waitKey(500)

                cap.release()
                cv2.destroyAllWindows()
                return qr_data, qr_bbox

            cv2.imshow('QR Code Scanner', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        return qr_data, qr_bbox # Could be None if no QR was detected
