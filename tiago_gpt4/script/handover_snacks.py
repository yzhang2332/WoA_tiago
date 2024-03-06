#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
# from text_to_speech_gpt4 import TTSFunction
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

from pal_interaction_msgs.msg import TtsAction, TtsGoal


class GetSnack:
    def __init__(self):
        

        # self.speak = TTSFunction()

        # Publisher for controlling Tiago's torso height
        self.height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)

        # Subscribe to the torso sensor height
        self.current_height = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
        self.current_torso_height = 0.0

        self.arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("arm server connected.")
        self.gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("gripper server connected.")

        self.home_client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        self.home_client.wait_for_server()

        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.tts_client.wait_for_server()
        rospy.loginfo("Tts server connected.")

        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(3.0)

        # rospy.loginfo("Connected to server")

    def joint_states_callback(self, msg):
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            self.current_torso_height = msg.position[index]

    def adjust_height(self, target_height):
        rate = rospy.Rate(10)
        duration = 1.5  # Duration for height adjustment

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

    
    def move_arm(self, joint_angles, t):
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
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        self.arm_client.send_goal(goal)
        if self.arm_client.wait_for_result(rospy.Duration(t+1)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Arm completed successfully.")
        else:
            rospy.loginfo("Arm did not complete before the timeout.")
    
    
    def move_gripper(self, width, t):
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        # goal.command.position = width
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = width
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory
        
        self.gripper_client.send_goal(goal)
        if self.gripper_client.wait_for_result():
            rospy.loginfo("Gripper completed successfully.")
        else:
            rospy.loginfo("Gripper did not complete before the timeout.")
    
    def speak_and_move(self, text, joint_angles, t):
        speak_thread = threading.Thread(target=self.tts, args=(text,))
        speak_thread.start()

        arm_thread = threading.Thread(target=self.move_arm(joint_angles, t))
        arm_thread.start()

        speak_thread.join()
        arm_thread.join()
    
    def go_home_position(self):
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = False

        self.home_client.send_goal(goal)
        self.home_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("Arm home.")
    
    def run(self):
        try:
            self.adjust_height(0.296)
            strech_joint_angles = [0.21, 0.19, -0.2, 1.32, -1.57, 1.37, 0.0]
            text = "Relax time now. Let's have some snacks."
            self.speak_and_move(text, strech_joint_angles, 4)

            time.sleep(0.5)
            # Open 
            width_open = [0.2, 0.2]
            self.move_gripper(width_open, 1)  # Replace with actual width needed to grasp the box
            
            # self.move_arm(strech_joint_angles, 6)
            # Move arm to pick position
            pre_grasp_angles = [0.116, -0.42,-0.0163, 1.367, -1.436, 0.775, 0.003]
            self.move_arm(pre_grasp_angles, 3)
            rospy.sleep(0.5)

            pick_joint_angles = [0.116, -0.635,-0.0163, 1.367, -1.436, 0.775, 0.003] 
            self.move_arm(pick_joint_angles, 1)
            # Close gripper to grasp the box
            # width_close = [0.044, 0.044]
            width_close = [0.02, 0.02]
            self.move_gripper(width_close, 1)
            
            # Move arm to handover position
            # self.move_arm(strech_joint_angles, 6)

            self.adjust_height(0.35)
            
            offer_angles = [0.44, -0.63, -1.88, 1.37, -1.12, -0.7, 0.41]
            text = "Here you go. They are all yours. One chocolate a day keeps the doctor away."
            self.speak_and_move(text, offer_angles, 5)
            # self.move_arm(offer_angles, 6)

            time.sleep(5)
            self.adjust_height(0.296)
            back_angles = [0.116, -0.34,-0.0163, 1.71, -1.436, 0.775, 0.003]
            self.move_arm(back_angles, 4)
            rospy.sleep(0.5)

            pick_joint_angles = [0.116, -0.635,-0.0163, 1.367, -1.436, 0.775, 0.003]
            self.move_arm(pick_joint_angles, 1)

            # Open 
            width_open = [0.2, 0.2]
            self.move_gripper(width_open, 1)  # Replace with actual width needed to grasp the box

            self.move_arm(pre_grasp_angles, 1)
            
            strech_joint_angles = [0.21, 0.19, -0.2, 1.32, -1.57, 1.37, 0.0]
            self.move_arm(strech_joint_angles, 4)

            self.go_home_position()

            # # Open gripper to release the box
            # GetSnack.move_gipper(0.1)
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    rospy.init_node('get_snack')
    snack = GetSnack()
    snack.run()