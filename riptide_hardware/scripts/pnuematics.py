#!/usr/bin/env python

import serial
import rospy
from riptide_msgs.msg import Pneumatics
#set up the symlink
COM_PORT = '/dev/ttyS1'
ser = serial.Serial(COM_PORT, baudrate=9600, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
def pneuCB(pneuMsg):
    pneuStart = "++++"
    pneuEnd = "@@@@\n"
    #blank nothing messages
    pin1 = "!nnnn"
    pin2 = "!nnnn"
    pin3 = "!nnnn"
    pin4 = "!nnnn"
    #conditionals to send to
    if pneuMsg.torpedo_stbd:
        pin2 = "!1000"
    if pneuMsg.torpedo_port:
        pin3 = "!1000"
    if pneuMsg.markerdropper:
        pin1 = "!1000"
    if pneuMsg.manipulator:
        pin4 = "!1000"
    final_pneu = pneuStart + pin1 + pin2 + pin3 + pin4 + pneuEnd
    print "sent: ", final_pneu
    ser.write(final_pneu)

def main():
    rospy.init_node('pneumatics')
    pneuSub = rospy.Subscriber("/command/pneumatics", Pneumatics, pneuCB, queue_size=1)
    pneuPub = rospy.Publish("/state/pnuematics", PnueStatus, queue_size=1)
    rate = rospy.Rate(100)
    pneuMsg = Pneumatics()
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__": main()
