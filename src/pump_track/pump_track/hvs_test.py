import cv2
import numpy as np

# Function to calculate HVS values
def calculate_hvs_color(frame, x, y):
    # Convert the BGR color space to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get the HSV values at the specified (x, y) coordinates
    h, s, v = hsv_frame[y, x]

    return h, s, v

# Open the camera (you can specify the camera index, usually 0 for the default camera)
cap = cv2.VideoCapture(4)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

try:
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Get the dimensions of the frame
        height, width, _ = frame.shape

        # Calculate the coordinates of the middle area
        middle_x = width // 2
        middle_y = height // 2

        # Calculate the HVS values of the color in the middle area
        h, s, v = calculate_hvs_color(frame, middle_x, middle_y)

        # Draw a rectangle around the middle area
        rect_color = (0, 255, 0)  # Green color
        rect_thickness = 2
        rect_width = 100
        rect_height = 100
        cv2.rectangle(frame, (middle_x - rect_width // 2, middle_y - rect_height // 2),
                      (middle_x + rect_width // 2, middle_y + rect_height // 2), rect_color, rect_thickness)

        # Display the HVS values on the frame
        text = f"H: {h}, S: {s}, V: {v}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow("Camera Feed", frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
