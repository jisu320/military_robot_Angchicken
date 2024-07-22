import cv2

# Create a VideoCapture object to access the camera (0 is usually the default camera)
cap = cv2.VideoCapture(4)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Loop to continuously capture and display frames from the camera
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the captured frame in a window
    cv2.imshow('Camera', frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close the OpenCV window
cap.release()
cv2.destroyAllWindows()
