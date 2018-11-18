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
from smach import CBState
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
from primus_class import *


def main():

    rospy.init_node('Stateovi_asr')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])

    sm.userdata.time = 3.0
    sm.userdata.body_pose = "Stand"
    sm.userdata.vocabulary_data = 'Hello, primus'
    sm.userdata.behavior = 'animations/Stand/Gestures/Stretch_2'
    sm.userdata.word_recognized =''
    sm.userdata.word_to_say = ''
    sm.userdata.word_conf = 0

    # Open the container
    with sm:
        
        # Wake up robot 
        smach.StateMachine.add('primus/wakeup', ServiceState('primus/wakeup', Empty),
                                transitions={'succeeded':'/primus/asr/vocabulary/set/goal1'})


################################ ASR TTS #########################################

        # Fill in the vocabulary and parameter server
        sm.userdata.vocabulary_data = 'yes, please, no, love, kill'
        smach.StateMachine.add('/primus/asr/vocabulary/set/goal1', Fill_Vocabulary(),
                                transitions={'succeeded':'/primus/tts/animation1'},
                                remapping={'vocabulary_data':'vocabulary_data'})


        # Say word via tts service  and animate while talking     
        sm.userdata.word_to_say = 'Hello, you'                                              
        smach.StateMachine.add('/primus/tts/animation1', ServiceState('/primus/tts/motionless', SetString, sm.userdata.word_to_say),
                                transitions={'succeeded':'/primus/asr/enable1'})

        # enable asr                                 
        smach.StateMachine.add('/primus/asr/enable1', ServiceState('/primus/asr/enable', Empty),
                                transitions={'succeeded':'/primus/asr/word_recognized1'})

        #Get detected word                        
        smach.StateMachine.add('/primus/asr/word_recognized1', ASR_Word(),
                                                  transitions={'succeeded':'/primus/asr/disable1'},
                                                remapping={'vocabulary_data':'vocabulary_data','word_recognized':'word_recognized'}) 
        # disable asr
        smach.StateMachine.add('/primus/asr/disable1', ServiceState('/primus/asr/disable', Empty),
                                transitions={'succeeded':'/primus/tts/animation2'})


        # Say word via tts service  and animate while talking    
        #sm.userdata.word_to_say = rospy.get_param('/primus/word_recognized')                                               
        smach.StateMachine.add('/primus/tts/animation2', ServiceState('/primus/tts/motionless', SetString, rospy.get_param('/primus/word_recognized')),
                                transitions={'succeeded':'/primus/asr/vocabulary/empty1'})


        # Empty  the vocabulary and parameter server
        sm.userdata.vocabulary_data = ''
        smach.StateMachine.add('/primus/asr/vocabulary/empty1', Empty_Vocabulary(),
                                transitions={'succeeded':'succeeded'},
                                remapping={'vocabulary_data':'vocabulary_data'})


############# REST
        # Go to rest pose
        smach.StateMachine.add('/primus/rest', ServiceState('/primus/rest', Empty),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted',
                                             'preempted':'preempted'})

    #########################################################################
    # Create and start the introspection server
    sis = IntrospectionServer('my_smach_introspection_server', sm, '/Test ASR')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
