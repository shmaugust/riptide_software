#!/usr/bin/env python

import serial
import rospy
from riptide_msgs.msg import Pnuematics

COM_PORT = 'dev/pnuematics'
ser = serial.Serial(COM_PORT, baudrate=9600, timeout=None)
def pnueCB(pnueMsg):
    pnueStart = "++++"
    pnueEnd = "@@@@"
    pin1 = "!nnnn"
    pin2 = "!nnnn"
    pin3 = "!nnnn"
    pin4 = "!nnnn"
    #conditionals to send to
    if pnueMsg.torpedo_stbd:
        pin2 = "!1000"
    if pnueMsg.torpedo_port:
        pin1 = "!1000"
    if pnueMsg.markerdropper:
        pin3 = "!1000"
    if pnueMsg.manipulator:
        pin4 = "!1000"
    final_pnue = pnueStart + pin1 + pin2 + pin3 + pin4 + pnueEnd
    ser.write(final_pnue)

def main():
    rospy.init_node('pnuematics')

    pnuePub = rospy.Subsciber("/command/pnuematics", Pnuematics, pnueCB, queue_size=1)
    rate = rospy.Rate(100)
    pnueMsg = Pneumatics()
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__": main()
