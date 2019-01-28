import io
import picamera
import logging
import socketserver
# from threading import Condition
from http import server

from threading import Condition, Event, Thread

import os
import requests

from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO, emit



app = Flask(__name__)
app.config["SECRET_KEY"] = os.getenv("SECRET_KEY")
socketio = SocketIO(app)

@app.route("/")
def index():
    return render_template("index.html")

controller = []
from driver.stepper_driver import *
pan = Stepper(PINS_PAN)
tilt = Stepper(PINS_TILT)
steppers = (pan,tilt)
pantilt = PanTilt(steppers)
@socketio.on("submit gamepad")
def gamepad(data):
    # Array: 0-3 axis 4-n buttons
    controller = data
    gamepad_active = False
    for idx,value in enumerate(controller):
        if value != 0 and idx!=2:
            print("Value {0} = {1}".format(idx,value))
            gamepad_active = True
    if gamepad_active is True:
        forcex = -controller[0]
        forcey = controller[1]
        pantilt.gamepad_move(forcex,forcey)



class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


if (__name__ == "__main__"):
    import code
    with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
        output = StreamingOutput()
        camera.start_recording(output, format='mjpeg')
        try:
            address = ('', 5001)
            server = StreamingServer(address, StreamingHandler)
            streamserver = Thread(target = server.serve_forever)
            streamserver.start()
            logging.info("Started stream server for picamera")
            socketio.run(app,host='0.0.0.0') # runs on port 5000
            # code.interact(local=dict(globals(), **locals()))
        finally:
            camera.stop_recording()
            server.shutdown()
            streamserver.join()
            pantilt.shutdown()
    # import code
    # code.interact(local=dict(globals(), **locals()))

