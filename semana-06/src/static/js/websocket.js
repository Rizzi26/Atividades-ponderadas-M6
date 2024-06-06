var socket = io.connect('http://' + document.domain + ':' + location.port);
var video = document.getElementById('video');
var canvas = document.createElement('canvas');
var context = canvas.getContext('2d');

socket.on('image_response', function(data) {
    console.log('Image received from server');
    var imageElement = document.getElementById('receivedImage');
    imageElement.src = data.image;

    // Calculate latencies
    var sentTimestamp = data.received_timestamp;
    var receivedByServerTime = data.received_time * 1000; // convert to ms
    var sentBackTime = data.sent_time * 1000; // convert to ms
    var now = new Date().getTime(); // current time in ms

    var totalLatency = now - sentTimestamp;
    var serverProcessingTime = sentBackTime - receivedByServerTime;
    var webcamLatency = receivedByServerTime - sentTimestamp;

    document.getElementById('latency').innerText = `Total Latency: ${totalLatency.toFixed(2)} ms (Server Processing: ${serverProcessingTime.toFixed(2)} ms)`;
    document.getElementById('webcamLatency').innerText = `Webcam Latency: ${webcamLatency.toFixed(2)} ms`;
});

navigator.mediaDevices.getUserMedia({ video: true })
    .then(function(stream) {
        video.srcObject = stream;
        video.play();
    })
    .catch(function(err) {
        console.error('Error accessing webcam: ' + err);
    });

function sendFrame() {
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    context.drawImage(video, 0, 0, canvas.width, canvas.height);
    var imageData = canvas.toDataURL('image/png');
    socket.emit('frame', { 'image': imageData, 'timestamp': new Date().getTime() });
}
setInterval(sendFrame, 100);  // Send a frame every 100ms

function moveFoward() {
    socket.emit('move', { 'direction': 'forward' });
}

function moveBackward() {
    socket.emit('move', { 'direction': 'backward' });
}

function moveLeft() {
    socket.emit('move', { 'direction': 'left' });
}   

function moveRight() {
    socket.emit('move', { 'direction': 'right' });
}

function stop() {
    socket.emit('move', { 'direction': 'stop' });
}
