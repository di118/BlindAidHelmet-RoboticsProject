import speech_recognition as sr
import pyaudio
import wave
import RPi.GPIO as GPIO #Import Raspberry Pi GPIO library
from gpiozero import Servo
from time import sleep
from picamera import PiCamera
from time import sleep
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import sys
from time import sleep
import RPi.GPIO as GPIO
import pyttsx3



def setServoAngle(servo, angle):
    assert angle >= 30 and angle <= 150
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    sleep(0.3)
    pwm.stop()





targetObject = 0
def getText():
    '''
    This function records for 5 seconds after the pushbutton is pressed. The recorded file is then converted into text.
    :return: string from audio
    '''
    CHUNK = 1024
    FORMAT = pyaudio.paInt16  # paInt8
    CHANNELS = 2
    RATE = 44100  # sample rate
    RECORD_SECONDS = 5
    WAVE_OUTPUT_FILENAME = "output.wav"

    p = pyaudio.PyAudio()

    GPIO.setwarnings(False)  # Ignore warning for now
    GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
    GPIO.setup(10, GPIO.IN,
               pull_up_down=GPIO.PUD_DOWN)  # Set pin 10 to be an input pin and set initial value to be pulled $
    while True:  # Run forever
        if GPIO.input(10) == GPIO.HIGH:
            print("Button was pushed!")
            stream = p.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=RATE,
                            input=True,
                            frames_per_buffer=CHUNK)  # buffer

            print("* recording")

            frames = []

            for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK)
                frames.append(data)  # 2 bytes(16 bits) per channel

            print("* done recording")

            stream.stop_stream()
            stream.close()
            p.terminate()

            wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()
            break
    audioFile = sr.AudioFile('output.wav')
    r = sr.Recognizer()
    with audioFile as source:
        audio = r.record(source)

    # r.recognize_google(audio)
    print("You just said: ", r.recognize_google(audio))
    fullText = r.recognize_google(audio)
    fullTextList = fullText.split()
    targetObject = fullTextList[len(fullTextList) - 1]
    print("Target object: ", targetObject)
    if(targetObject == "fridge"):
        global targetObjectID
        targetObjectID = 1
    elif targetObject == "sink":
        global targetObjectID
        targetObjectID = 2
    elif targetObject == "sofa":
        global targetObjectID
        targetObjectID = 3
    elif targetObject == "door":
        global targetObjectID
        targetObjectID = 4
    GPIO.cleanup()


getText()

def takePictures():
    '''
    This function takes 3 pictures
    :return: 3 jpg files
    '''
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    pan = 17
    GPIO.setup(pan, GPIO.OUT)  # gray ==> PAN
    setServoAngle(pan, 40)
    sleep(2)
    print("Taking picture here")
    camera = PiCamera()
    camera.start_preview()
    sleep(2)
    camera.capture('/home/pi/V2.0/BlindAidHelmet-RoboticsProject/image1.jpg')
    print("Setting servo to middle position")
    setServoAngle(pan, 80)
    print("Taking picture here")
    sleep(2)
    camera.capture('/home/pi/V2.0/BlindAidHelmet-RoboticsProject/image2.jpg')
    print("Setting servo to left position")
    setServoAngle(pan, 120)
    sleep(2)
    print("Taking picture here")
    camera.capture('/home/pi/V2.0/BlindAidHelmet-RoboticsProject/image3.jpg')
    camera.stop_preview()
    GPIO.cleanup()
takePictures()
rightImage = ('/home/pi/V2.0/BlindAidHelmet-RoboticsProject/image1.jpg')
midImage = ('/home/pi/V2.0/BlindAidHelmet-RoboticsProject/image2.jpg')
leftImage = ('/home/pi/V2.0/BlindAidHelmet-RoboticsProject/image3.jpg')


def analyseImages(targetImage):
    objects = []
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('calib_images/*.jpg')
    images2 = glob.glob('images/image*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    image3 = targetImage

    frame = cv2.imread(image3)
    # operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


    if np.all(ids is not None):

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx,
                                                        dist)  # Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
        # (rvec-tvec).any() # get rid of that nasty numpy value array error

        for i in range(0, ids.size):
            # aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)  # Draw Axis
            x = str(int(
                (corners[i - 1][0][0][0] + corners[i - 1][0][1][0] + corners[i - 1][0][2][0] + corners[i - 1][0][3][
                    0]) / 4))
            # print("X coordinates: ", x)
            y = str(int(
                (corners[i - 1][0][0][1] + corners[i - 1][0][1][1] + corners[i - 1][0][2][1] + corners[i - 1][0][3][
                    1]) / 4))
            # print("Y coordinates: ", y)
            objectArr = [ids[i][0], x, y]
            objects.append(objectArr)
        rotM = np.zeros(shape=(3, 3))
        angle = str(cv2.Rodrigues(rvec[i - 1], rotM, jacobian=0))

        # aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers

        ###### DRAW ID #####
        # strg = ''
        # for i in range(0, ids.size):
        #     if ids[i] == 2:
        #         strg += 'glasses, '
        #         print("glasses")
        #     elif ids[i] == 1:
        #         strg += 'lamp, '
        #         print("lamp")
        #
        #     elif ids[i] == 3:
        #         strg += 'calender, '
        #         print("calender")
        #     else:
        #         strg += str(ids[i][0]) + ', '

    else:
        ##### DRAW "NO IDS" #####
        print("No id's found")
    return objects




def getObjLocation(target1, position):
    print(target1)
    imageObj = analyseImages(position)
    for i in range(0, len(imageObj)):
        # print("ob", imageObj[i][0])
        if(imageObj[i][0] == target1):

                    return [imageObj[i][1], imageObj[i][2]]


def runAll():
    # engine = pyttsx3.init()
    analyseImages(leftImage)
    if getObjLocation(targetObjectID, leftImage) is not None :
        coordinates = getObjLocation(targetObjectID, leftImage)
        if(int(coordinates[0]) < 213):
            print("your object is 30 degrees to the left")
            # engine.say("your object is 30 degrees to the left")
            sys.exit('object found ')
        elif (int(coordinates[0]) >= 213 and int(coordinates[0]) <= 416):
            print("your object is 20 degrees to the left")
            sys.exit('object found ')
        else:
            print("your object is 10 degrees to the left")
            sys.exit('object found ')
    analyseImages(leftImage)
    if getObjLocation(targetObjectID, rightImage) is not None :
        coordinates = getObjLocation(targetObjectID, rightImage)
        if (int(coordinates[0]) < 213):
            print("your object is 30 degrees to the right")
            sys.exit('object found ')
        elif (int(coordinates[0]) >= 213 and int(coordinates[0]) <= 416):
            print("your object is 20 degrees to the right")
            sys.exit('object found ')
        else:
            print("your object is 10 degrees to the right")
            sys.exit('object found ')
    analyseImages(midImage)
    if getObjLocation(targetObjectID, midImage) is not None :
        coordinates = getObjLocation(targetObjectID, midImage)
        print("your object is in the middle")
        sys.exit('object found ')



runAll()