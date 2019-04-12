import speech_recognition as sr
import pyaudio
import wave
import RPi.GPIO as GPIO #Import Raspberry Pi GPIO library
from gpiozero import Servo
from time import sleep
from picamera import PiCamera
from time import sleep


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

def snapshot(n):

    camera.capture('/home/pi/image.jpg')
    camera.stop_preview()

def takePictures():
    '''
    This function takes 3 pictures
    :return: 3 jpg files
    '''

    myGPIO = 17

    # Min and Max pulse widths converted into milliseconds
    # To increase range of movement:
    #   increase maxPW from default of 2.0
    #   decrease minPW from default of 1.0
    # Change myCorrection using increments of 0.05 and
    # check the value works with your servo.
    myCorrection = 0.45
    maxPW = (2.0 + myCorrection) / 1200
    minPW = (1.0 - myCorrection) / 1200

    myServo = Servo(myGPIO, min_pulse_width=minPW, max_pulse_width=maxPW)

    print("Using GPIO17")
    print("Max pulse width is set to 2.45 ms")
    print("Min pulse width is set to 0.55 ms")
    print("Setting servo to default position")
    myServo.mid()
    sleep(2)
    print("Setting servo to right position")
    myServo.min()
    sleep(2)
    print("Taking picture here")
    camera = PiCamera()
    camera.start_preview()
    sleep(2)
    camera.capture('/home/pi/image1.jpg')
    print("Setting servo to middle position")
    myServo.mid()
    print("Taking picture here")
    sleep(2)
    camera.capture('/home/pi/image2.jpg')
    print("Setting servo to left position")
    myServo.max()
    sleep(2)
    print("Taking picture here")
    camera.capture('/home/pi/image3.jpg')
    camera.stop_preview()


takePictures()