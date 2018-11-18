#!/usr/bin/env python

##################################################
## {Description}
##################################################
## {License_info}
##################################################
## Author: {Luka Kicinbaci}
## Copyright: Copyright {2018}, {rur}
## Credits: [{Aldebaran}]
## License: {}
## Version: {V1}.{V2}.{2}
## Mmaintainer: {Luka Kicinbaci}
## Email: {luka.fua2@gmail.com}
## Status: {dev_status}
##################################################


import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
from smach_ros import *
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PickupActionGoal
import time
from naoqi_bridge_msgs.srv import (
    SetFloatResponse,
    SetFloat,
    SetStringResponse,
    SetString,
    GetFloatResponse,
    GetFloat)
from std_srvs.srv import (
    SetBool,
    SetBoolResponse)

from naoqi_bridge_msgs.msg import BodyPoseWithSpeedActionGoal, JointTrajectoryActionGoal, SetSpeechVocabularyActionGoal, RunBehaviorActionGoal
"""
roslaunch pepper_interaction pepper.launch 

roslaunch pepper_interaction servers.launch 

rosrun pepper_interaction sulla_scena1.py
"""
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['time'])
    def execute(self, userdata):
        time.sleep(userdata.time) 
        return 'succeeded'

class Fill_Vocabulary(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['vocabulary_data'])
        self.vocabulary_pub = rospy.Publisher('/speech_vocabulary_action/goal', SetSpeechVocabularyActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        vocalubary_goal = SetSpeechVocabularyActionGoal()
        vocalubary_goal.goal.words = userdata.vocabulary_data
        self.vocabulary_pub.publish(vocalubary_goal)
        return 'succeeded'



class GoToPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['pose', 'time'])
        self.pose_pub = rospy.Publisher('/body_pose_naoqi/goal', BodyPoseWithSpeedActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        pose_goal = BodyPoseWithSpeedActionGoal()
        pose_goal.goal.posture_name = userdata.pose
        pose_goal.goal.speed = 0.5
        self.pose_pub.publish(pose_goal)
        time.sleep(userdata.time) 
        return 'succeeded'

class Fill_Vocabulary(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['vocabulary_data'])
        self.vocabulary_pub = rospy.Publisher('/speech_vocabulary_action/goal', SetSpeechVocabularyActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        vocalubary_goal = SetSpeechVocabularyActionGoal()
        vocalubary_goal.goal.words = userdata.vocabulary_data
        self.vocabulary_pub.publish(vocalubary_goal)
        return 'succeeded'


class Run_Behavior(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['behavior', 'time'])
        self.behavior_pub = rospy.Publisher('/run_behavior/goal', RunBehaviorActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        behavior_goal = RunBehaviorActionGoal()
        behavior_goal.goal.behavior = userdata.behavior
        self.behavior_pub.publish(behavior_goal)
        time.sleep(userdata.time) 
        return 'succeeded'



class JointTrajectory(smach.State):
    #TODO: Omoguciti mogucnost manipulacije i mogucnost postavljanja stiffnesa
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['joint_names', 'joint_points', 'time'])
        self.joint_pub = rospy.Publisher('/joint_trajectory/goal', JointTrajectoryActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        joints_goal = JointTrajectoryActionGoal()
        #joints_goal.header.frame_id = "torso"
        #joints_goal.header.stamp = rospy.Time.now()
        joints_goal.goal.trajectory.joint_names = "RShoulderPitch" # userdata.joint_names
        joints_goal.goal.trajectory.points = [0.5, 0.5, 0.5, 0.5] #userdata.joint_points
        #joints_goal.goal.trajectory.points.positions = 0.5 
        #joints_goal.goal.trajectory.points.velocities = 0.5
        #joints_goal.goal.trajectory.points.accelerations = 0.5
        #joints_goal.goal.trajectory.points.effort = 0.5
        self.joint_pub.publish(joints_goal)
        time.sleep(userdata.time) 
        return 'succeeded'


def main():

    rospy.init_node('Behaviours_emotional')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    sm.userdata.time = 3.0
    sm.userdata.body_pose = "Stand"
    sm.userdata.vocabulary_data = 'yes, please, no, love, kill'
    sm.userdata.behavior = 'animations/Stand/Gestures/Stretch_2'
    sm.userdata.joint_names = ''
    sm.userdata.joint_points = [0]
    sm.userdata.word = ""
    sm.userdata.word_conf = 0

    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('wake_up', ServiceState('/wakeup', Empty),
                                transitions={'succeeded':'wait_0',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})
        sm.userdata.time = 7
        smach.StateMachine.add('wait_0', Wait(),
                                transitions={'succeeded':'vocabulary_0',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})


        # Fill in the vocabulary
        sm.userdata.vocabulary_data = 'happy, angry, fear, sorrow, suprise'
        smach.StateMachine.add('vocabulary_0', Fill_Vocabulary(),
                                transitions={'succeeded':'asr_start_1',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'vocabulary_data':'vocabulary_data'})

        # start asr                                 
        smach.StateMachine.add('asr_start_1', ServiceState('/start_asr', SetBool, True),
                                transitions={'succeeded':'tts_happy',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})


     

        # happy         1                                           
        smach.StateMachine.add('tts_happy', ServiceState('/tts_speech', SetString, 'Now I will show you my Happiness'),
                                transitions={'succeeded':'run_behavior_1',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



       
        sm.userdata.behavior = 'animations/Stand/Emotions/Positive/Happy_2'
        smach.StateMachine.add('run_behavior_1', Run_Behavior(),
                                transitions={'succeeded':'wait_1',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior'})


        sm.userdata.time = 5
        smach.StateMachine.add('wait_1', Wait(),
                                transitions={'succeeded':'tts_angry',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})


        # angry     2                                               
        smach.StateMachine.add('tts_angry', ServiceState('/tts_speech', SetString, 'Now I will show you my Anger'),
                                transitions={'succeeded':'run_behavior_2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        sm.userdata.behavior = 'animations/Stand/Emotions/Negative/Angry_2'
        smach.StateMachine.add('run_behavior_2', Run_Behavior(),
                                transitions={'succeeded':'wait_2',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior'})


        sm.userdata.time = 5
        smach.StateMachine.add('wait_2', Wait(),
                                transitions={'succeeded':'tts_fear',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})





        # fear        3                                            
        smach.StateMachine.add('tts_fear', ServiceState('/tts_speech', SetString, 'Now I will show you my fear'),
                                transitions={'succeeded':'run_behavior_3',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        sm.userdata.behavior = 'animations/Stand/Emotions/Negative/Fear_2'
        smach.StateMachine.add('run_behavior_3', Run_Behavior(),
                                transitions={'succeeded':'wait_3',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior'})


        sm.userdata.time = 5
        smach.StateMachine.add('wait_3', Wait(),
                                transitions={'succeeded':'tts_sad',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})



        # sad         4                                           
        smach.StateMachine.add('tts_sad', ServiceState('/tts_speech', SetString, 'Now I will show you my sadness'),
                                transitions={'succeeded':'run_behavior_4',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        sm.userdata.behavior = 'animations/Stand/Emotions/Negative/Sad_2'
        smach.StateMachine.add('run_behavior_4', Run_Behavior(),
                                transitions={'succeeded':'wait_4',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior'})


        sm.userdata.time = 5
        smach.StateMachine.add('wait_4', Wait(),
                                transitions={'succeeded':'tts_suprise',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})





        # suprise 5                                        
        smach.StateMachine.add('tts_suprise', ServiceState('/tts_speech', SetString, 'Now I will show you my suprise'),
                                transitions={'succeeded':'run_behavior_5',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})



        sm.userdata.behavior = 'animations/Stand/Emotions/Negative/Surprise_2'
        smach.StateMachine.add('run_behavior_5', Run_Behavior(),
                                transitions={'succeeded':'wait_5',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'behavior':'behavior'})


        sm.userdata.time = 5
        smach.StateMachine.add('wait_5', Wait(),
                                transitions={'succeeded':'rest',
                                             'aborted':'aborted',
                                             'preempted':'preempted'},
                                remapping={'time':'time'})


    #########################################################################
    # Create and start the introspection server
    sis = IntrospectionServer('my_smach_introspection_server', sm, '/Sulla')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
