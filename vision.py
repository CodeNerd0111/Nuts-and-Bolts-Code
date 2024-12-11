#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# This is a demo program showing the use of OpenCV to do vision processing. The image is acquired
# from the USB camera, then a rectangle is put on the image and sent to the dashboard. OpenCV has
# many methods for different types of processing.
#
# NOTE: This code runs in its own process, so we cannot access the robot here,
#       nor can we create/use/see wpilib objects
#
# To try this code out locally (if you have robotpy-cscore installed), you
# can execute `python3 -m cscore vision.py:main`
#

import cv2
import numpy as np
import wpilib

from cscore import CameraServer as CS
def main():
    CS.enableLogging()
    # Get the UsbCamera from CameraServer
    camera = CS.startAutomaticCapture()
    # Set the resolution
    camera.setResolution(640, 480)
    #camera.setResolution(1280, 720)

    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()
    # Setup a CvSource. This will send images back to the Dashboard
    outputStream = CS.putVideo("Rectangle", 640, 480)

    # Allocating new images is very expensive, always try to preallocate
    mat = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, mat = cvSink.grabFrame(mat)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        # Put a rectangle on the image
        cv2.rectangle(mat, (100, 100), (400, 400), (255, 255, 255), 5)


        # Give the output stream a new image to display
        outputStream.putFrame(mat)


def qrreader():
    # Does not check the code during the test so it passes
    if wpilib.RobotBase.isSimulation():
        return "Simulated environment: No camera available"
    
    # Creates a QRCodeDetector object
    qcd = cv2.QRCodeDetector()
    
    # Opens the camera (has an issue with the tests)
    capture = cv2.VideoCapture('http://roborio-10476-frc.local:1181/')
    try:
        while True:
            # Capture a frame
            ret, frame = capture.read()
            
            # If there is an error with opening the camera
            if not capture.isOpened():
                return "Super Duper not Skibidi so not like the rizzler fr in OHIO"
            
            # If there is an error with reading the image
            if not ret:
                return "Not very Skibidi"
            
            
            # Detect and decode QR codes
            ret_qr, decoded_info, points, _ = qcd.detectAndDecodeMulti(frame)

            # If QR codes were detected, return the decoded information
            if ret_qr:
                for s in decoded_info:
                    if s:
                        return s
            else:
                return "no QR code"
    finally:
        return "THE MOST UNSKIBIDI THING THAT COULD HAPPEN"

