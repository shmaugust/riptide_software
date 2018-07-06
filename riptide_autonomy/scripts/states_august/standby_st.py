#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs.msg import Constants, ResetControls, SwitchState

class Standby(State):


    def __init__(self):
        '''
            Outcomes:       execute_mission     (start riptide_concurrence)
                            standby             (stay in current standby state)
                            test                (go into vehicle test state)

            Input Keys:     test_status         (topside test success status)

            Output Keys:    mission_protocol    (which mission we are running)
        '''
        State.__init__(self, outcome=['execute_mission', 'standby', 'test'],
                        input_keys=['test_status'],
                        output_keys=['mission_protocol'])
        rospy.init_node('standby')

        switch_sub = rospy.Subscriber("/state/switches", SwitchState, callback)

    def execute(self, userdata):
        #Act on master_switch_status


    def callback(data):
        # Get kill switch status
