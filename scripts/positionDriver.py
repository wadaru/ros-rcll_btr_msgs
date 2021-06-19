#!/usr/bin/env python
#
import rospy
import sys
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity

sendSeq = 10

def goToDestination(point):
    rospy.wait_for_service('rvw2/positionDriver')
    try:
        set_Pose = rospy.ServiceProxy('rvw2/positionDriver', SetPosition)
        sendDataHeader = Header()
        sendDataPose = Pose2D()
        sendDataHeader.stamp = rospy.Time.now()
        sendDataHeader.frame_id = "positionDriver"
        sendDataPose = point
        resp = set_Pose(sendDataHeader, sendDataPose)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    rospy.init_node("babyTigers")

    point = Pose2D()
    point.x = 1000
    point.y = 0
    point.theta = 0
    goToDestination(point)

    point.x = 0
    point.y = 0
    point.theta = 180
    goToDestination(point)

    point.x = 0
    point.y = -1000
    point.theta = 90
    goToDestination(point)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
