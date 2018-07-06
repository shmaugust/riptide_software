#!/usr/bin/env python

import rospy
from smach import State, StateMachine
import smach_ros
from riptide_msgs.msg import Constants, ResetControls, SwitchState
import auv_concurrence_sm
import subprocess
