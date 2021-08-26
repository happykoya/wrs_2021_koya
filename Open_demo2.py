#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
#Title: ゴミをゴミ箱まで運ぶプログラム
#Author: Koya Okuse
#Data: 2021/08/24
#Memo:WRS大会用プログラム
#-------------------------------------------------------------------
import time
import sys

import rospy
from std_msgs.msg import String
import smach
import smach_ros

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import *
from common_function import *
from mimi_common_pkg.srv import ManipulateSrv, RecognizeCount


class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_Ltyr'])

    def execute(self, userdata):
        rospy.loginfo('Enter The Room')
        speak('start open demonstration 2')
        enterTheRoomAC(0.8)
        return 'to_Ltyr'

class Listen_to_your_request(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_pap'])
        self.stt_pub = rospy.ServiceProxy('/speech_recog', SpeechRecog)
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)

    def execute(self,userdata):
        location_list = searchLocationName('Tall table')
        navigationAC(location_list)
        while 1:
            print "Listen_to_your_request"
            speak("Choose a cup or bottle.")
            result = self.stt_pub().result
            if result == 'cup':
                speak("Bring a cup")
                sm_puserdata.sm_name = 'cup'
                break

            elif result == 'bottle':
                speak("Bring a bottle")
                userdata.sm_name = 'bottle'
                break

            else:
                speak("I couldn't hear it well")

        return 'to_pap'


class MoveAndPick(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['success', 'failed'],
                            input_keys=['object_name_in'],
                            output_keys=['object_name_out'])
        #Service
        self.grab = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        #Publisher
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)


    def execute(self, userdata):

        location_list = searchLocationName('Bookshelf')
        navigationAC(location_list)

        rospy.wait_for_service('/object/recognize')
        recog = rospy.ServiceProxy('/object/recognize', RecognizeCount)
        res = recog('any')

        #if len(res.data) >= 2:
        #    object_name = res.data[1]

        #elif len(res.data) == 1:
        #    object_name = res.data[0]

        #else:
        #    object_name = 'any'
        userdata.object_name_out = object_name
        self.pub_location.publish('Bookshelf')

        result = self.grab(object_name).result  #object_nameによってif等で条件分岐
        if result == True:
            return 'success'
        else:
            return 'failed'


class MoveAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomeいs=['completed'],
                            input_keys=['object_name_in'])

        self.object_list = ['cup','bottle','snack','dish','chips',
                            'bag','toy','smartphone','book','pen']

        #Service
        self.stt_pub = rospy.ServiceProxy('/speech_recog', SpeechRecog)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', ManipulateSrv)
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)

    def execute(self, userdata):
        if userdata.object_name_in  in self.object_list:
            location_list = searchLocationName('Tall table')
            self.pub_location.publish('Tall table')

        navigationAC(location_list)
        self.arm_srv('place')

        speak("Do you still have what you need?")
        result = self.stt_pub().result
        #if result == 'yes':
        #    return 'need'

        #elif result == 'no':
        #    return 'completed'

        return 'completed'


def main():
    sm_top = smach.StateMachine(outcomes=['FINISH'])
    with sm_top:
        ### EnterRoom
        smach.StateMachine.add('ENTER', EnterRoom(),
                            transitions={'to_Ltyr':'Listen_to_your_request'})

        ### Listen_to_your_request
        smach.StateMachine.add('to_Ltyr',Listen_to_your_request(),
                            transitions={'to_pap':'PICH_AND_PLACE'})

        ### Pick and place
        sm_pap = smach.StateMachine(outcomes=['to_exit'])
        #sm_pap.userdata.sm_name = 'cup'
        with sm_pap:
            smach.StateMachine.add('pick', MoveAndPick(),
                            transitions={'success':'place',
                                         'failed':'to_exit'},
                            remapping={'object_name_out':'sm_name',
                                       'object_name_in':'sm_name'})
            smach.StateMachine.add('place', MoveAndPlace(),
                            transitions={'completed':'to_exit',
                                         'again':'to_Ltyr'},
                            remapping={'object_name_in':'sm_name'})

        smach.StateMachine.add('PICH_AND_PLACE', sm_pap,
                            transitions={'to_exit':'EXIT'})

        ### Avoid that
        #smach.StateMachine.add('AVOID_THAT', AvoidThat(),
        #                    transitions={'to_WDYS':'WHAT_DID_YOU_SAY'})

        ### what did you say
        #sm_wdys = smach.StateMachine(outcomes=['to_exit'])
        #sm_wdys.userdata.sm_time = time.time()
        #sm_wdys.userdata.sm_success

        #with sm_wdys:
        #    smach.StateMachine.add('STARTWDYS', TimeCount(),
        #                    transitions={'to_PS':'PersonSearch'},
        #                    remapping={'start_time_out':'sm_time',
        #                               'success_count_out':'sm_success'})
        #    smach.StateMachine.add('PersonSearch', PersonSearch(),
        #                    transitions={'found':'QUESTION'})
        #                    transitions={'continues':'QUESTION',
        #                                'give_up':'to_exit',
        #                                'completed':'to_exit'},
        #                    remapping={'success_count_in':'sm_success',
        #                               'success_count_out':'sm_success',
        #                               'start_time_in':'sm_time',
        #                               'start_time_out':'sm_time'})
        #smach.StateMachine.add('WHAT_DID_YOU_SAY', sm_wdys,
        #                    transitions={'to_exit':'EXIT'})

        ### Go to the exit
        smach.StateMachine.add('EXIT', ExitRoom(),
                            transitions={'to_finish':'FINISH'})

    outcome = sm_top.execute()

if __name__ == '__main__':
    rospy.init_node('wrs_opendemo2')
    main()
