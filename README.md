# Remote Controlled Camera

In this project we remotely control a pan tilt moving camera. The movement is controlled by stepper motors.

## Requirements

We need the following components for this project:

* A Raspberry Pi.
* A Pi camera.
* 2 stepper motors attached to the camera for pan and tilt motion
* A laptop or a mobile device (aka smartphone).

## Setting Up

Install requirements
 ```
 sudo pip3 install -r requirements.txt
 ```
 
## Running it

Start the server by typing the following command:
```
python3 remote_robot.py
```

The web app is set to port `5000` whereas the video stream is found at port `5001`.

TODO: make a better presentation
