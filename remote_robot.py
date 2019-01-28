#!/usr/bin/python3
# Dexter Industries GoPiGo3 Remote Camera robot
# With this project you can control your Raspberry Pi Robot, the GoPiGo3, with a phone, tablet, or browser.
# Remotely view your robot as first person in your browser.
#
# You MUST run this with python3
# To Run:  python3 flask_server.py

# imports needed for stream server
import io
import logging
import signal
import socketserver
import subprocess
import sys
import time
from http import server
from threading import Condition, Event, Thread
from time import sleep

# imports needed for web server
from flask import (Flask, Response, jsonify, render_template, request,
                   send_from_directory, url_for)
from werkzeug.serving import make_server

import picamera
from driver.stepper_driver import *

# check if it's ran with Python3
assert sys.version_info[0:1] == (3,)

# from gopigo3 import FirmwareVersionError
# from easygopigo3 import EasyGoPiGo3

logging.basicConfig(level = logging.DEBUG)

# for triggering the shutdown procedure when a signal is detected
# keyboard_trigger = Event()
# def signal_handler(signal, frame):
#     logging.info('Signal detected. Stopping threads.')
#     keyboard_trigger.set()

#######################
### Web Server Stuff ##
#######################

# Directory Path can change depending on where you install this file.  Non-standard installations
# may require you to change this directory.
#directory_path = '/media/manolosolalinde/DATOS/BITBUCKET/moving-camera/static'
directory_path = '/home/pi/BITBUCKET/moving-camera/static'

MAX_FORCE = 5.0
MIN_SPEED = 100.
MAX_SPEED = 300

pan = Stepper(PINS_PAN)
tilt = Stepper(PINS_TILT)
steppers = (pan,tilt)
pantilt = PanTilt(steppers)

HOST = "0.0.0.0"
WEB_PORT = 5000

app = Flask(__name__, static_url_path='')

# socket io stuff
from flask_socketio import SocketIO, emit
socketio = SocketIO(app)

@socketio.on("submit gamepad")
def gamepad(data):
    # 0-3 axis 4-n buttons
    controller = data
    for idx,value in enumerate(controller):
        if value != 0 and idx!=2:
            print("Value {0} = {1}".format(idx,value))

class WebServerThread(Thread):
    '''
    Class to make the launch of the flask server non-blocking.
    Also adds shutdown functionality to it.
    '''
    def __init__(self, app, host, port):
        Thread.__init__(self)
        self.srv = make_server(host, port, app)
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        logging.info('Starting Flask server')
        self.srv.serve_forever()

    def shutdown(self):
        logging.info('Stopping Flask server')
        self.srv.shutdown()

@app.route("/robot", methods = ["POST"])
def robot_commands():

    # get the query
    args = request.args
    state = args['state']
    angle_degrees = int(float(args['angle_degrees']))
    angle_dir = args['angle_dir']
    force = float(args['force'])
    determined_speed = MIN_SPEED + force * (MAX_SPEED - MIN_SPEED) / MAX_FORCE
    if determined_speed > MAX_SPEED:
        determined_speed = MAX_SPEED

    print("hello movement")
    pantilt.move(force,angle_degrees)

    # TODO averiguar que es esto
    resp = Response()
    resp.mimetype = "application/json"
    resp.status = "OK"
    resp.status_code = 200

    return resp

@app.route("/")
def index():
    return page("index.html")

@app.route("/<string:page_name>")
def page(page_name):
    return render_template("{}".format(page_name))

@app.route("/static/<path:path>")
def send_static(path):
    return send_from_directory(directory_path, path)

#############################
### Video Streaming Stuff ###
#############################

class StreamingOutput(object):
    '''
    Class to which the video output is written to.
    The buffer of this class is then read by StreamingHandler continuously.
    '''
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
    '''
    Implementing GET request for the video stream.
    '''
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

##############################
### Youtube Broadcast Stuff ##
##############################

# import io
YOUTUBE="rtmp://a.rtmp.youtube.com/live2/" 
KEY= "mtx7-wuwc-0fs0-47pa"
# stream_cmd = 'ffmpeg -f h264 -r 25 -i - -itsoffset 5.5 -f alsa -ac 1 -i hw:1,0 -vcodec copy -acodec aac -ac 1 -ar 8000 -ab 32k -filter:a "volume=+5dB" -map 0:0 -map 1:0 -strict experimental -f flv ' + YOUTUBE + KEY 
stream_cmd = 'ffmpeg -loglevel -8 -f h264 -r 25 -y -i - -itsoffset 5.5 -fflags nobuffer -use_wallclock_as_timestamps 1 -f alsa -ac 1 -i hw:1,0 -vcodec copy -acodec aac -ac 1 -ar 8000 -ab 32k -filter:a "volume=+15dB" -map 0:0 -map 1:0 -strict experimental out.mkv'


#############################
### Aggregating all calls ###
#############################

if __name__ == "__main__":
    # registering both types of signals
    # signal.signal(signal.SIGINT, signal_handler)
    # signal.signal(signal.SIGTERM, signal_handler)

    # firing up the video camera (pi camera)
    with picamera.PiCamera(resolution='640x480', framerate=25) as camera:
        camera.vflip = True 
        camera.hflip = True 

        stream_pipe = subprocess.Popen(stream_cmd, shell=True, stdin=subprocess.PIPE) 
        output = StreamingOutput()
        # camera.start_recording('grabacion.h264',splitter_port=2)
        camera.start_recording(stream_pipe.stdin, format='h264', bitrate = 2000000) 
        camera.start_recording(output,resize=(320,240), format='mjpeg',splitter_port=2)
        logging.info("Started recording with picamera")

        STREAM_PORT = 5001
        stream = StreamingServer((HOST, STREAM_PORT), StreamingHandler)

        # # starting the video streaming server
        streamserver = Thread(target = stream.serve_forever)
        streamserver.start()
        logging.info("Started stream server for picamera")

        # # starting the web server
        webserver = WebServerThread(app, HOST, WEB_PORT)
        webserver.start()
        logging.info("Started Flask web server")

        # and run it indefinitely
        try:
            while True:
                sleep(0.5)
                # print("Sleep") 
        except KeyboardInterrupt:
            print("Keyboard interrupt") 

        # until some keyboard event is detected
        logging.info("Keyboard event detected")
        camera.stop_recording(splitter_port=2)
        camera.stop_recording()

        logging.info("Camera Stopped recording")

        # trigger shutdown procedure
        webserver.shutdown()
        stream.shutdown()

        # and finalize shutting them down
        webserver.join()
        streamserver.join()
        pantilt.shutdown()
        logging.info("Stopped all threads")
        
        # finalize youtube streaming
        stream_pipe.stdin.close() 
        stream_pipe.wait() 
    
    sys.exit(0)
