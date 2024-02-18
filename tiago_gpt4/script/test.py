import speech_recognition as sr


for index, name in enumerate(sr.Microphone.list_microphone_names()):
	print("Microphone with index {} is named \"{}\"".format(index, name))

import pyaudio

def list_audio_devices():
    p = pyaudio.PyAudio()
    print("Available audio devices:")
    for i in range(p.get_device_count()):
        dev = p.get_device_info_by_index(i)
        print(f"{i}: {dev['name']} (Input Channels: {dev['maxInputChannels']} Output Channels: {dev['maxOutputChannels']})")
    p.terminate()

list_audio_devices()