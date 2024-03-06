#!/usr/bin/env python

import rospy
# from sound_play.libsoundplay import SoundClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time
# from text_to_speech_gpt4 import TTSFunction
# import openai
# import pyaudio
# from pydub import AudioSegment
# from pydub.playback import play
# import io
import time
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

from pal_interaction_msgs.msg import TtsAction, TtsGoal


class BreathingExercise:
    def __init__(self):
        # 

        # self.soundhandle = SoundClient()

        # Publisher for controlling Tiago's torso height
        self.height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

        # Subscribe to the torso sensor height
        self.current_height = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
        self.current_torso_height = 0.0
        # Wait for the sound client to properly initialize
        # time.sleep(1)

        # self.speak = TTSFunction()

        self.unfold_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.lower_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.unfold_client.wait_for_server()
        self.lower_client.wait_for_server()
        rospy.loginfo("Arm server connected.")

        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.tts_client.wait_for_server()
        rospy.loginfo("Tts server connected.")

        self.home_client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        self.home_client.wait_for_server()

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(1.0)

        rospy.loginfo("Connected to server")

    def joint_states_callback(self, msg):
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            self.current_torso_height = msg.position[index]

    def adjust_height(self, target_height):
        rate = rospy.Rate(10)
        duration = 1.0  # Duration for height adjustment

        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]


        target_height = float(target_height)

        # Initial position
        start_point = JointTrajectoryPoint()
        start_point.positions = [self.current_torso_height]
        start_point.time_from_start = rospy.Duration(0)
        traj.points.append(start_point)

        # Target position
        target_point = JointTrajectoryPoint()
        target_point.positions = [target_height]
        target_point.time_from_start = rospy.Duration(duration)
        traj.points.append(target_point)

        # Publish trajectory
        self.height_pub.publish(traj)
        time.sleep(duration)

    def tts(self, text):
        rospy.loginfo("Inside the tts function!!!")
        # Create a goal to say our sentence
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        # Send the goal and wait
        self.tts_client.send_goal_and_wait(goal)


    def start_exercise(self):
        def speak_and_move(text, arm_action=None, height=None):
            speak_thread = threading.Thread(target=self.tts, args=(text,))
            speak_thread.start()

            if arm_action == 'unfold':
                arm_thread = threading.Thread(target=self.go_unfold_arm)
                arm_thread.start()
            elif arm_action == 'lower':
                arm_thread = threading.Thread(target=self.go_lower_arm)
                arm_thread.start()
            elif arm_action == 'home':
                arm_thread = threading.Thread(target=self.go_home_position)
                arm_thread.start()

            if height is not None:
                height_thread = threading.Thread(target=self.adjust_height, args=(height,))
                height_thread.start()

            speak_thread.join()
            if arm_action is not None:
                arm_thread.join()
            if height is not None:
                height_thread.join()


        text = "Let's take a moment to recharge and refocus. Join me in a brief breathing exercise to relieve any tension, and to come back to your tasks with renewed energy and focus. We'll do this together for three rounds, syncing our breaths and movements. Find a comfortable standing position, with your feet hip-width apart and your spine straight."
        # speak_and_move(text, "initial", 0.2)
        speak_and_move(text)
        time.sleep(2)

        for i in range(2):
            # Inhale
            
            text = "Inhale deeply through your nose, feeling your lungs expand fully. Hold your breath for a moment at the top of your inhale."
            speak_and_move(text, 'unfold', 0.35)
           
            time.sleep(1)  # Wait for user to hold breath

            # Exhale
            
            text = "Exhale slowly and completely through your mouth, releasing any remaining tension."
            speak_and_move(text, 'lower', 0.1)

            time.sleep(1)  # Wait for user to exhale

        
        text = "Let's do one more round."
        speak_and_move(text)

        # Inhale
        
        text = "Inhaling deeply, filling your lungs with fresh, revitalizing air. Hold for a moment. "
        speak_and_move(text, 'unfold', 0.35)

        time.sleep(1)  # Wait for user to hold breath

        # Exhale
        text = "And exhale slowly, feeling any tightness or stress dissolve with each breath. Feel the tension melting away with each breath, leaving you refreshed and ready to tackle your tasks with renewed focus."
        speak_and_move(text, 'lower', 0.1)

        text = "How do you feel?"
        speak_and_move(text, 'home', 0.2)

        time.sleep(1)  # Wait for user to exhale

    def go_unfold_arm(self):
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = [
            0.21, -0.2, -2.2, 1.15, -1.57, 0.2, 0.0  # Positions for the arm joints
        ]
        point.time_from_start = rospy.Duration(4.0)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        self.unfold_client.send_goal(goal)
        if self.unfold_client.wait_for_result(rospy.Duration(7.0)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Action completed successfully.")
        else:
            rospy.loginfo("Action did not complete before the timeout.")

    
    def go_lower_arm(self):
        
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = [
            0.21, -0.37, -1.08, 1.18, -2.07, 1.06, -1.58  # Positions for the arm joints
        ]
        point.time_from_start = rospy.Duration(3.0)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        self.lower_client.send_goal(goal)
        if self.lower_client.wait_for_result(rospy.Duration(7.0)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Action completed successfully.")
        else:
            rospy.loginfo("Action did not complete before the timeout.")

    def go_home_position(self):
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False

        self.home_client.send_goal(goal)
        self.home_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm tucked.")

        
if __name__ == '__main__':
    try:
        rospy.init_node('breathing_exercise')
        exercise = BreathingExercise()
        exercise.start_exercise()
    except rospy.ROSInterruptException:
        pass
