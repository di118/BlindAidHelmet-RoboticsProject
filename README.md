BlindAidHelmet-RoboticsProject
A robot that can recognise objects in a room and specify their location ( maybe in angles)
This project aims to create a device that allows users with vision difficulties to find objects in the room.

This project will use a python environment running on RaspberryPi.
The final product will work as follows:
1. The user presses a button to record a request. The request can be questions such as : 
    - "Where is the door?"
    - "Where is the ball?"
    - "Where is the book?"
    - "What objects are around me?"
2. The recorded file is sent to a speech to text python function. 
3. The function then sends the string to an object detection function.
4. Then we compute the object (x,y) pixel coordinates to determine an angle.
5. Reply to the user using text to speech where is the object located (i.e "The door is 90 degrees to your right").

** NOTE **
In case the user asks: "What objects are around me?" The camera will scan the room.
We will be using a motor to make spin 360 degrees. Therefore we will meet the movement requirements from the marking sheet.

** HARDWARE REQUIREMENTS **
- RaspberryPi
- Camera
- Motor ( to move the camera )
- Microphone
- Push button (maybe for press to record request )

** SOFTWARE REQUIREMENTS ** 
- Python 3.0 or higher
- OpenCV
- Speech_recognition
- Object Detection 
