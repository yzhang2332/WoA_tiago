#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# import openai
# import pyaudio
# from pydub import AudioSegment
# from pydub.playback import play
# import io
# from text_to_speech_gpt4 import TTSFunction
from woa_tiago.tiago_gpt4.script.sim_script.sim_gpt_generation_b import GenerationFuncion
import time

# Configure your OpenAI API key here
# openai.api_key = ''

class GPT4Client:
    def __init__(self):
        rospy.init_node('gpt4_client', anonymous=True)
        
        # Subscriber to get data from voice recognition server
        self.subscriber = rospy.Subscriber('/tiago/recognized_text', String, self.callback)
        
        # Publisher to publish GPT-4's response
        self.publisher = rospy.Publisher('/tiago/gpt4_response', String, queue_size=10)
        
        # Publisher to publish a flag and timestamp
        self.flag_publisher = rospy.Publisher('/tiago/conversation_cont', String, queue_size=10)
        
        # self.stream = None
        self.generate = GenerationFuncion()


    def callback(self, data):
        recognized_text = data.data
        rospy.loginfo(f"Received recognized text: {recognized_text}")
        
        # Process the text with GPT-4
        response = self.generate.process_with_gpt4(recognized_text)

        # Publish a timestamp to indicate the conversation time
        timestamp = rospy.get_time() 
        flag_message = str(timestamp)
        self.flag_publisher.publish(flag_message)

        # Publish GPT-4's response
        self.publisher.publish(response)
    

def main():
    gpt4_client = GPT4Client()
    rospy.spin()

if __name__ == '__main__':
    main()