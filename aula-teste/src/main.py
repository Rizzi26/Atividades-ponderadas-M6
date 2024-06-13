import os
from flask import Flask, request, send_file, render_template
import cv2
import numpy as np
import tempfile

app = Flask(__name__, template_folder='templates', static_folder='static')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/input', methods=['POST'])
def input():
    if 'video' not in request.files:
        return "No video file", 400

    video_file = request.files['video']
    video_bytes = video_file.read()

    # Process the video and detect faces
    processed_video = identify_faces(video_bytes)

    # Save the processed video to a temporary file
    temp_output_path = tempfile.mktemp(suffix='.mp4')
    with open(temp_output_path, 'wb') as f:
        f.write(processed_video)

    return send_file(
        temp_output_path,
        mimetype='video/mp4',
        as_attachment=True,
        download_name='processed_video.mp4'
    )

def identify_faces(data):
    classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # Create a temporary file to store the video
    with tempfile.NamedTemporaryFile(delete=False, suffix='.mp4') as temp_video:
        temp_video.write(data)
        temp_video_path = temp_video.name

    video_capture = cv2.VideoCapture(temp_video_path)

    # Prepare output video
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    output_path = 'output.mp4'
    out = cv2.VideoWriter(output_path, fourcc, 20.0, (int(video_capture.get(3)), int(video_capture.get(4))))

    while video_capture.isOpened():
        ret, frame = video_capture.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = classifier.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

        out.write(frame)

    video_capture.release()
    out.release()

    # Read the output video file
    with open(output_path, 'rb') as f:
        processed_video = f.read()

    # Clean up temporary files
    os.remove(temp_video_path)

    return processed_video

if __name__ == '__main__':
    app.run(debug=True)
