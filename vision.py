import cv2
from pyzbar.pyzbar import decode

# Open the webcam (Change '0' if multiple cameras are connected)
cap = cv2.VideoCapture(0)

# Set resolution (optional)
cap.set(3, 1920)  # Width
cap.set(4, 1080)  # Height

while True:
    ret, frame = cap.read()  # Read frame from the webcam
    if not ret:
        print("Failed to grab frame")
        break

    # Convert frame to grayscale for better QR detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect QR codes in the frame
    qr_codes = decode(gray)

    for qr in qr_codes:
        data = qr.data.decode('utf-8')  # Extract QR code data
        print(f"QR Code Detected: {data}")

        # Draw bounding box around QR code
        pts = qr.polygon
        if len(pts) == 4:  # If the QR code is fully detected
            pts = [(p.x, p.y) for p in pts]
            cv2.polylines(frame, [np.array(pts, dtype=np.int32)], True, (0, 255, 0), 3)

        # Display the decoded text
        x, y, w, h = qr.rect
        # Here, we can case on the coordinates and decide when we want the robotic arm to make a move!!!
        
        cv2.putText(frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    # Show the frame
    cv2.imshow('QR Code Scanner', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
