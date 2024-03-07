#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
import openai
# from text_to_speech_gpt4 import TTSFunction
import yaml
import os
from create_calendar import create_event_calendar
from Showing_Events_Caleder import Showing_Events_Calender
from breathing import BreathingExercise
from handover_snacks import GetSnack
from strech_ball import CatchBall
from def_actions import play_action
from customized_gesture import FollowMe, ShowAround

import sys
sys.path.append("/home/pal/tiago_ws/src/woa_tiago/tiago_nav/scripts")
# sys.path.append("/home/pal/tiago_ws/src/woa_tiago/tiago_follow_person/scripts")

from nav_function import NavigationClient

from half_turn import HalfTurn

# from ....woa_tiago.tiago_nav.scripts.nav_function import NavigationClient

# from tiago_nav.scripts.nav_function import NavigationClient

# from tiago_follow_person.scripts.half_turn import HalfTurn

from pal_interaction_msgs.msg import TtsAction, TtsGoal
import actionlib


# Configure your OpenAI API key here
current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
config_path = os.path.join(current_dir, '..', 'config', 'gpt_api.yaml')  # Navigate to the config.yaml file
with open(config_path, 'r') as file:
    config = yaml.safe_load(file)
openai.api_key = config['api_key']
snake_flag = True
ball_flag = True
breathing_flag = True
schedule_flag = True

class GenerationFuncion():
    def __init__(self):
        # self.speak = TTSFunction()
        self.navigation = NavigationClient("ux_start")
        self.follow_me = FollowMe()
        self.show_around = ShowAround()
        self.turn_around = HalfTurn()

        self.follow_person_pub = rospy.Publisher('/follow_person_flag', Bool, queue_size=1)
        self.bool_msg = Bool()
        

        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.tts_client.wait_for_server()
        rospy.loginfo("Tts server connected.")

    def tts(self, text):
        rospy.loginfo("Inside the tts function!!!")
        # Create a goal to say our sentence
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        # Send the goal and wait
        self.tts_client.send_goal_and_wait(goal)
    
    def process_with_gpt4(self, text):
        global snake_flag, ball_flag, breathing_flag, schedule_flag
        try:
            rospy.loginfo("Start generate by gpt")
            # keyword_list= ['no_action', 'stress_ball', 'breathing_exercise', 'provide_snack', 'schedule_meeting', 'navigate_to_meeting_room_A', 'navigate_to_meeting_room_B', 'navigate_to_kitchen', 'say_hi_wave_hand']
            keyword_list= ['no_action', 'stress_ball', 'breathing_exercise', 'provide_snack', 'schedule_meeting', 'navigate_to_meeting_room', 'navigate_to_kitchen', 'say_hi_wave_hand']
            gpt_response = openai.chat.completions.create(
                model="gpt-4",  # Use the model identifier for GPT-4. Adjust if you're using a specific variant.
                messages=[{"role": "system", "content": 
                           "You are a helpful office assistant robot. Your name is Tiago and you are Australian. You can assist office work and maintain a relax vibe. \
                           Now you are tested in a robot competion fo a human-robot conference. \
                           Info on Human-Robot-Interaction conference: 19th Annual ACM/IEEE International Conference on Human Robot Interaction (HRI). HRI 2024 is the 19th annual conference for basic and applied HRI research. Researchers from across the world present their best work to HRI to exchange ideas about the theory, technology, data, and science furthering the state-of-the-art in the field. The conference theme for HRI 2024 is 'HRI in the real world' and will focus on key HRI theories, designs, studies, systems, and technical advances that aim to bring HRI out of the lab and into everyday life. We encourage the community to consider what it means to do HRI in practice, and ways to bring it into the mainstream. The HRI 2024 conference is taking place in Boulder, Colorado, in the US.\
                           Info on the robot competition: The interaction is taking place during the HRI 2024 Robot Competition. It is the first competition at the HRI conference and the task is to develop human-robot applications designed to provide invaluable assistance and companionship to workers in the office environment. The competition serves as a platform for participants to demonstrate the capabilities of their robots and to highlight the ways in which these intelligent machines can enhance the productivity and well-being of office professionals. Participants are invited to present their human-robot applications based on the Tiago Platform, each with its unique add-ons design, features, and functionalities. These robots should behave as humanly as possible, fostering a sense of familiarity and promoting natural interaction with humans. They should understand human emotions, displaying expressive facial features and gestures that enable effective communication and empathetic engagement.The person the robot is interacting with during the competition is from a jury. The jury will score the interaction. For instance: The robot’s Conversational Ability, i.e. How well does the robot engage in conversations with users? Does it understand and respond appropriately to user queries and requests? Is the conversation natural and fluid? Besides, the robot has to convey emotional intelligence, i.e. How well does the robot understand and respond to human emotions? Does it display empathy and provide emotional support when needed?\
                           "
                           }, 
                          {"role": "user", "content": 
                           f"{text}. \
                           Note: Please response with proper natural language and provide a keyword after a * sign, without a period mark.\
                           The keyword should be choostexte from this keyword list: [{keyword_list}]\
                           If the keyword is 'no_action', please give some natural response.\
                           If the keyword isn't 'no_action', only propose to do the action, don't introduce the detail of the action in the natural language response.\
                           For example: Sure, come with me. *navigate_to_meeting_room_a\
                           Another example: I can help you to relax, let's do a breathing exercise. *breathing_exercise\
                           "}],
            )
            rospy.loginfo("GPT responsed")
            # Sometimes you can propose to do an action from the keyword list, but still put no_action after the * sign.

            gpt_response = gpt_response.choices[0].message.content
            rospy.loginfo("GPT responsed")
            robot_response, action_keyword = gpt_response.split("*")
            rospy.loginfo(f"Robot response is: {robot_response}")
            rospy.loginfo(f"Action is: {action_keyword}")
            self.tts(robot_response)
            # self.speak.text_to_speech(robot_response, 1.0)
            if action_keyword != "no_action":
                
                self.bool_msg.data = False
                self.follow_person_pub.publish(self.bool_msg)

                if action_keyword == "breathing_exercise":
                    if breathing_flag == True:
                        rospy.loginfo("Doing Breathing Exercises")
                        breathing = BreathingExercise()
                        breathing.start_exercise()
                        breathing_flag = False
                    else:
                        rospy.loginfo("no more breathing")
                        text = "But we had enough breathing exercise, aren't we?"
                        self.tts(text)

                elif action_keyword == "provide_snack":
                    if snake_flag == True:
                        rospy.loginfo("Doing Get a Snack")
                        snake = GetSnack()
                        snake.run()
                        snake_flag = False
                    else:
                        rospy.loginfo("No more snakes")
                        text = "I'm sorry. There's no more snack. I can bring you more next time."
                        self.tts(text)

                elif action_keyword == "stress_ball":
                    if ball_flag ==True:
                        rospy.loginfo("Doing Stress Ball")
                        ball = CatchBall()
                        ball.run()
                        ball_flag = False
                    else:
                        rospy.loginfo("no more ball")
                        text = "Oh, the stress ball is already in your hand"
                        self.tts(text)

                elif action_keyword == "say_hi_wave_hand":
                    rospy.loginfo("Doing a wave")
                    play_action('wave')
                    play_action('home')

                elif action_keyword == "schedule_meeting":
                    if schedule_flag == True:
                        rospy.loginfo("Doing schedule a meeting")
                        self.turn_around.run()
                        create_event_calendar()
                        self.turn_around.run()
                        schedule = Showing_Events_Calender()
                        rospy.loginfo(schedule)
                        self.tts(schedule)
                        # self.speak.text_to_speech(schedule, 1.2)
                        schedule_flag = False

                    else:
                        rospy.loginfo("no more schedule")
                        text = "But, relax. You had enough meeting today."
                        self.tts(text)

                
                elif action_keyword == "navigate_to_meeting_room":
                    rospy.loginfo("Show meeting room")
                    self.follow_me.run()
                    self.navigation.run("ux_room_a_inside")
                    text = "This is the meeting room. You can have a meeting here."
                    self.show_around.run(text)

                # elif action_keyword == "navigate_to_meeting_room_A":
                #     rospy.loginfo("Show meeting room A")
                #     self.follow_me.run()
                #     self.navigation.run("ux_room_a_inside")
                #     text = "This is the meeting room A. You can have a meeting here."
                #     self.show_around.run(text)

                # elif action_keyword == "navigate_to_meeting_room_B":
                #     rospy.loginfo("Show meeting room B")
                #     self.follow_me.run()
                #     self.navigation.run("ux_room_b_inside")
                #     text = "This is the meeting room B. You can have a meeting here."
                #     self.show_around.run(text)

                elif action_keyword == "navigate_to_kitchen":
                    rospy.loginfo("Show kitchen")
                    self.follow_me.run()
                    self.navigation.run("ux_kitchen")
                    text = "This is the kitchen. Help yourself to a cup of coffee."
                    self.show_around.run(text)
                else:
                    rospy.loginfo("Wrong keyword.")

                self.bool_msg.data = True
                self.follow_person_pub.publish(self.bool_msg)

            return robot_response
        except Exception as e:
            rospy.logerr(f"Failed to process text with GPT-4: {e}")
            return "Error processing text with GPT-4."