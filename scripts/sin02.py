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

def setVelocity(velocity):
    global sendSeq
    rospy.wait_for_service('rvw2/setVelocity')
    try:
        sendSeq = sendSeq + 1
        print "Seq: ", sendSeq
        set_Velocity = rospy.ServiceProxy('rvw2/setVelocity', SetVelocity)
        sendDataHeader = Header()
        sendDataPose = Pose2D()
        sendDataHeader.seq = sendSeq
        sendDataHeader.stamp = rospy.Time.now()
        sendDataHeader.frame_id = "sin02"
        sendDataPose = velocity
        resp = set_Velocity(sendDataHeader, sendDataPose)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    rospy.init_node("babyTigers")
    # pub = rospy.Publisher('robotino/velocity', Float32MultiArray, queue_size = 10)
    # rate = rospy.Rate(10)

    startDeg = 0
    endDeg = 360
    stepDeg = 1
    counter = 0
    velocity = Pose2D()
    for theta in range(startDeg, endDeg, stepDeg):
        x = math.sin(math.radians(theta)) * 300
        z = math.cos(math.radians(theta)) * 100
        # velocity = Float32MultiArray()
        velocity.x = x
        velocity.y = 0
        velocity.theta = z
        # rospy.loginfo(velocity)
        # pub.publish(velocity)
        # rate.sleep()
        setVelocity(velocity)
    velocity.x = 0
    velocity.y = 0
    velocity.theta = 0
    # rospy.loginfo(velocity)
    # pub.publish(velocity)
    setVelocity(velocity)
    # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
