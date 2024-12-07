import requests  # For HTTP requests to ESP32-CAM and Arduino
import cv2  # For video frame processing
import mediapipe as mp  # For face and landmark detection
from scipy.spatial import distance as dist  # For calculating Euclidean distances

# Initialize Mediapipe FaceMesh with detection and tracking confidence
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Eye landmark indices (as per Mediapipe's FaceMesh model)
LEFT_EYE = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]

# EAR (Eye Aspect Ratio) calculation function
def eye_aspect_ratio(eye_points, frame_width, frame_height):
    """
    Calculates the Eye Aspect Ratio (EAR) for detecting eye state (open/closed).

    Args:
        eye_points: List of landmark points for the eye.
        frame_width: Width of the video frame.
        frame_height: Height of the video frame.

    Returns:
        ear: Computed Eye Aspect Ratio.
    """
    points = [(int(p.x * frame_width), int(p.y * frame_height)) for p in eye_points]
    A = dist.euclidean(points[1], points[5])  # Vertical distance 1
    B = dist.euclidean(points[2], points[4])  # Vertical distance 2
    C = dist.euclidean(points[0], points[3])  # Horizontal distance
    ear = (A + B) / (2.0 * C)  # EAR formula
    return ear

# ESP32-CAM and Arduino R4 board URLs
ESP32_CAM_URL = "http://192.168.8.210:81/stream"  # Replace with your ESP32-CAM URL
ARDUINO_URL = "http://192.168.8.183"  # Replace with your Arduino's IP address

# Start video capture from ESP32-CAM
cap = cv2.VideoCapture(ESP32_CAM_URL)

# Set resolution for faster processing (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Check if the connection to the ESP32-CAM is successful
if not cap.isOpened():
    print("Failed to connect to ESP32-CAM stream.")
    exit()

frame_count = 0  # Counter to process frames at intervals

# Process video stream
while cap.isOpened():
    ret, frame = cap.read()  # Read a frame from the video stream
    if not ret:
        print("Failed to grab frame from ESP32-CAM.")
        break

    frame_count += 1
    if frame_count % 5 == 0:  # Process every 5th frame for efficiency
        # Prepare the frame for Mediapipe processing
        frame_height, frame_width, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb_frame)  # Detect face landmarks

        # If face landmarks are detected
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # Get landmarks for the left and right eyes
                left_eye_points = [face_landmarks.landmark[i] for i in LEFT_EYE]
                right_eye_points = [face_landmarks.landmark[i] for i in RIGHT_EYE]

                # Calculate EAR for both eyes
                left_ear = eye_aspect_ratio(left_eye_points, frame_width, frame_height)
                right_ear = eye_aspect_ratio(right_eye_points, frame_width, frame_height)
                ear = (left_ear + right_ear) / 2.0  # Average EAR

                # Send EAR value to Arduino for further action
                try:
                    response = requests.get(f"{ARDUINO_URL}/?EAR={ear}")
                except Exception as e:
                    print(f"Failed to send EAR: {e}")

    # Add a small delay to control frame rate
    cv2.waitKey(1)

# Release video capture resource when done
cap.release()
