import cv2
import numpy
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from PIL import Image, ImageOps
import io
import base64
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
from std_srvs.srv import Empty
from concurrent.futures import ThreadPoolExecutor
import time

app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode='threading')

executor = ThreadPoolExecutor(max_workers=4)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('message')
def handle_message(data):
    print(f'received message: {data}')
    emit('response', {'data': f'Server received: {data}'}, broadcast=True)

@socketio.on('move')
def handle_move(data):
    direction = data['direction']
    print('Received move command:', direction)
    turtle_bot.handle_move(direction)

@socketio.on('frame')
def handle_frame(data):
    print("Received frame data")
    
    start_time = time.time()
    received_timestamp = data['timestamp']

    def process_image(image_data):
        decode_start_time = time.time()
        image_data = base64.b64decode(image_data.split(',')[1])
        image = Image.open(io.BytesIO(image_data))
        decode_end_time = time.time()

        image = ImageOps.fit(image, (image.width // 2, image.height // 2))

        face_detection_start_time = time.time()
        image_base64 = identify_faces(image)
        face_detection_end_time = time.time()
        
        return image_base64, decode_end_time - decode_start_time, face_detection_end_time - face_detection_start_time

    image_data = data['image']
    future = executor.submit(process_image, image_data)
    image_base64, decode_time, face_detection_time = future.result()
    
    total_time = time.time() - start_time

    emit('image_response', {
        'image': f'data:image/png;base64,{image_base64}',
        'received_timestamp': received_timestamp,
        'received_time': start_time,
        'sent_time': time.time(),
        'total_time': total_time,
        'decode_time': decode_time,
        'face_detection_time': face_detection_time
    }, broadcast=True)

def identify_faces(image):
    classificador = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    classificador2 = cv2.CascadeClassifier('haarcascade_profileface.xml')

    image = numpy.array(image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = classificador.detectMultiScale(gray, 1.3, 5)
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_face = Image.fromarray(image)

    image_face = numpy.array(image_face)
    gray = cv2.cvtColor(image_face, cv2.COLOR_BGR2GRAY)
    faces = classificador2.detectMultiScale(gray, 1.3, 5)
    for (x, y, w, h) in faces:
        cv2.rectangle(image_face, (x, y), (x + w, y + h), (0, 255, 0), 2)
    image_face = cv2.cvtColor(image_face, cv2.COLOR_BGR2RGB)
    image_face = Image.fromarray(image_face)

    buffered = io.BytesIO()
    image_face.save(buffered, format="PNG", optimize=True)
    image_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
    return image_base64

class TurtleBot(Node):
    def __init__(self):
        super().__init__('TurtleBot')
        self.connected = True
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 2.0
        self.angular_speed = 2.0
        self.stopped = False

    def handle_move(self, direction):
        if self.stopped:
            print("O robô está parado e não aceitará mais comandos.")
            return

        msg = Twist()
        match direction:
            case 'forward':
                msg.linear.x = -self.linear_speed
            case 'backward':
                msg.linear.x = self.linear_speed
            case 'left':
                msg.angular.z = self.angular_speed
            case 'right':
                msg.angular.z = -self.angular_speed
            case 'stop':
                self.stop()
        self.publisher_.publish(msg)
        print(f'Published message: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')

    def stop(self):
        print("Chamando o serviço para parar o robô...")
        client = self.create_client(Empty, 'stop_robot')
        if client.wait_for_service(timeout_sec=0.1):
            request = Empty.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('Serviço chamado com sucesso.')
        else:
            self.get_logger().info('Serviço não disponível.')

        # Set the flag to stopped
        self.stopped = True
        print("O robô agora está incomunicável.")

if __name__ == '__main__':
    rclpy.init()
    print("Initializing TurtleBot node")
    turtle_bot = TurtleBot()
    try:
        socketio.run(app)
    except KeyboardInterrupt:
        pass
    finally:
        turtle_bot.destroy_node()
        rclpy.shutdown()
