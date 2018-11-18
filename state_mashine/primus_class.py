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
from naoqi_bridge_msgs.msg import (BodyPoseWithSpeedActionGoal, JointTrajectoryActionGoal, SetSpeechVocabularyActionGoal, RunBehaviorActionGoal, WordRecognized)

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded'],
                                    input_keys=['time'])
    def execute(self, userdata):
        time.sleep(userdata.time) 
        return 'succeeded'


class GoToPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded'],
                                    input_keys=['pose', 'time'])
        self.pose_pub = rospy.Publisher('/primus/body_pose/goal', BodyPoseWithSpeedActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        pose_goal = BodyPoseWithSpeedActionGoal()
        pose_goal.goal.posture_name = userdata.pose
        pose_goal.goal.speed = 0.5
        self.pose_pub.publish(pose_goal)
        time.sleep(userdata.time) 
        return 'succeeded'


class Run_Behavior(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded'],
                                    input_keys=['behavior', 'time'])
        self.behavior_pub = rospy.Publisher('/primus/behaviour/run/goal', RunBehaviorActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        behavior_goal = RunBehaviorActionGoal()
        behavior_goal.goal.behavior = userdata.behavior
        self.behavior_pub.publish(behavior_goal)
        time.sleep(userdata.time) 
        return 'succeeded'


class Fill_Vocabulary(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded'],
                                    input_keys=['vocabulary_data'])
        self.vocabulary_pub = rospy.Publisher('/primus/asr/vocabulary/set/goal', SetSpeechVocabularyActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        vocalubary_goal = SetSpeechVocabularyActionGoal()
        vocalubary_goal.goal.words = userdata.vocabulary_data
        self.vocabulary_pub.publish(vocalubary_goal)

        rospy.set_param('/primus/vocabulary_data', userdata.vocabulary_data)
        return 'succeeded'

class Empty_Vocabulary(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded'],
                                    input_keys=['vocabulary_data'])
        self.vocabulary_pub = rospy.Publisher('/primus/asr/vocabulary/set/goal', SetSpeechVocabularyActionGoal, queue_size=10)
        rate = rospy.Rate(10) # 10hz
       
    def execute(self, userdata):
        vocalubary_goal = SetSpeechVocabularyActionGoal()
        vocalubary_goal.goal.words = userdata.vocabulary_data
        self.vocabulary_pub.publish(vocalubary_goal)

        rospy.set_param('/primus/vocabulary_data', userdata.vocabulary_data)
        try:
            rospy.set_param('/primus/word_recognized', userdata.vocabulary_data)
        except KeyError:
            print "Nepostoji parametar /primus/word_recognized"
        return 'succeeded'

class ASR_Word(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                    outcomes=['succeeded'],
                                    input_keys=['vocabulary_data', 'word_recognized'],
                                    output_keys=['word_recognized'])
       
    def execute(self, userdata):
        msg = rospy.wait_for_message('/primus/asr/word_recognized', WordRecognized)
        print msg.words[0]  # ovdje je ispis
        rospy.set_param('/primus/word_recognized', msg.words[0])
        time.sleep(0.5)
        self.data = userdata.vocabulary_data.split(",") 
        for x in self.data:
            if x == msg.words[0]:
                print x
                userdata.word_recognized = x  
        #print userdata.word_recognized      ovo ne moze jer je ovaj userdata samo output

        return 'succeeded'
            



