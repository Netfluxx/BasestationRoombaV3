import cv2

print(cv2.getBuildInformation())

def open_video_stream():
    # Load the pre-trained Haar Cascade for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Define the GStreamer pipeline for receiving a JPEG stream over RTP
    gst_pipeline = (
        'udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! '
        'rtpjpegdepay ! jpegdec ! videoconvert ! appsink'
    )

    # Create a video capture object with the defined pipeline
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open video stream!")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame from the video stream")
                break

            # Convert frame to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces in the image
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
                flags=cv2.CASCADE_SCALE_IMAGE
            )

            # Draw rectangles around the faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Display the resulting frame with detections
            cv2.imshow('Received Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    open_video_stream()
