# run pip install requirements.txt -- ** NOTE ** not all requirements needed
# run python speechText.py

import speech_recognition as sr
audioFile = sr.AudioFile('output.wav')
r = sr.Recognizer()
with audioFile as source:
    audio = r.record(source)

# r.recognize_google(audio)
print(r.recognize_google(audio))

