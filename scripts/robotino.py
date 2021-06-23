#!/usr/bin/python
import struct
import time
import sys
import math
import rospy
import udpcomm
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Float32, Float32MultiArray, Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty
from nav_msgs.msg import Odometry
import rcll_ros_msgs
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity
#
# ROS for robotino
# 
RPLIDAR = True

def getCenterPoint(data):
    global centerPoint
    centerPoint = data

def getLeftPoint(data):
    global leftPoint
    leftPoint = data

def getRightPoint(data):
    global rightPoint
    rightPoint = data

def changeViewData(point):
    x = int(point.x * 100)
    y = int(point.y * 100)
    viewData = math.abs(x) * 10000 + math.abs(y)
    if ( point.x < 0):
      viewData += 10000 * 10000
    if ( point.y < 0):
      viewData += 10000 * 10000 * 2
    return viewData

def updateUDP():
    if (robViewMode == 2):
      if (RPLIDAR == True):
        point = Point()
        point.x = centerPoint.x * 1000
        point.y = center
        udp.view3Send[5] = changeViewData(centerPoint)
        udp.view3Send[6] = changeViewData(leftPoint)
        udp.view3Send[7] = changeViewData(rightPoint)

def getResponse(value):
    if (value == 0):
        while float(udp.view3Recv[1]) == value:
            updateUDP()
            udp.receiver()
            udp.sender()
            rate.sleep()
    else:
        while float(udp.view3Recv[1]) != 0:
            updateUDP()
            udp.receiver()
            udp.sender()
            rate.sleep()
    # print(value, udp.view3Recv[1])
    return

def sendRobView():
    global robViewMode
    robViewMode = udp.view3Send[1]
    
    # set mode = 0 and wait for ack(!=0) from RobView.
    udp.view3Send[1] = 0
    getResponse(0)

    # send command with mode
    udp.view3Send[1] = robViewMode
    getResponse(0)
    getResponse(1)

    # command finished
    udp.view3Send[1] = 0
    getResponse(0)

    robViewMode = 0
    return udp.view3Recv[1]

def setVelocity(data):
    global velocityData, robViewMode
    print "setVelocity"
    resp = SetVelocity()
    velocityData = data
    robViewMode = 1
    udp.view3Send[1] = robViewMode # mode number
    udp.view3Send[2] = int(velocityData.pose.x)
    udp.view3Send[3] = int(velocityData.pose.y)
    udp.view3Send[4] = int(velocityData.pose.theta)

    print("header:", data.header)
    print("velocity data:", data.pose.x, data.pose.y, data.pose.theta)
    resp.success = (sendRobView() == 1)
    # return resp
    return [resp.success, ""]

def goPosition(data):
    global positionDriver, robViewMode
    resp = SetPosition()
    positionDriver = data
    robViewMode = 2
    udp.view3Send[1] = robViewMode # mode number
    udp.view3Send[2] = int(positionDriver.pose.x)
    udp.view3Send[3] = int(positionDriver.pose.y)
    udp.view3Send[4] = int(positionDriver.pose.theta)

    print("goToPosition:", positionDriver.pose)
    print(udp.view3Send[2])

    resp.success = (sendRobView() == 1)
    # stop for RPLidar
    if (RPLIDAR == True):
        rospy.wait_for_service('/btr/scan_stop')
        scan_stop = rospy.ServiceProxy('/btr/scan_stop', Empty)
        resp = scan_stop()

    return [resp.success, ""]
    # print("setPosition:", positionDriver.position.x)

def setOdometry(data):
    global odometryData, robViewMode
    resp = SetBoolResponse()
    odometryData = data
    robViewMode = 3
    udp.view3Send[1] = robViewMode # mode number
    udp.view3Send[2] = int(odometryData.pose.x)
    udp.view3Send[3] = int(odometryData.pose.y)
    udp.view3Send[4] = int(odometryData.pose.theta)

    print("setOdometry:", odometryData.pose.x, odometryData.pose.y, odometryData.pose.theta)

    resp.success = (sendRobView() == 1)
    # print("OK")
    return [resp.success, ""]

#
# main
#
if __name__ == '__main__':
  args = sys.argv
  if (len(args) == 5):
    sendADDRESS = args[1]
    sendPORT    = args[2]
    recvADDRESS = args[3]
    recvPORT    = args[4] 
  else:
    sendPORT = 9180
    recvPORT = 9182
    sendADDRESS = "127.0.1.1"
    recvADDRESS = "127.0.1.1"

  print("sendADD:", sendADDRESS, ", sendPORT:", sendPORT)
  print("recvADD:", recvADDRESS, ", recvPORT:", recvPORT)
  udp = udpcomm.Udpcomm(sendADDRESS, sendPORT, recvADDRESS, recvPORT)

  rospy.init_node('robotino')
  srv01 = rospy.Service('rvw2/setVelocity', SetVelocity, setVelocity)
  srv02 = rospy.Service('rvw2/positionDriver', SetPosition, goPosition)
  srv03 = rospy.Service('rvw2/setOdometry', SetOdometry, setOdometry)
  # pub01 = rospy.Publisher('odometry', Float32MultiArray, queue_size = 10)
  pub01 = rospy.Publisher('robotino/odometry', Odometry, queue_size = 10)
  pub02 = rospy.Publisher('robotino/checkFlag', Bool, queue_size = 10)
  pub03 = rospy.Publisher('robotino/getVelocity', Float32MultiArray, queue_size = 10)
  rate = rospy.Rate(10)

  velocityData = SetVelocity()
  velocityData.pose = [0, 0, 0]
  positionDriver = Pose()
  positionDriver.position.x = 0
  positionDriver.position.y = 0
  positionDriver.position.z = 0
  positionDriver.orientation = 0
  robViewMode = 0
  oldMode = 0
  checkFlag = 0

  centerPoint = Point()
  leftPoint = Point()
  rightPoint = Point()

  # setup for RPLidar
  if (RPLIDAR == True):
    rospy.wait_for_service('/btr/scan_start')
    scan_start = rospy.ServiceProxy('/btr/scan_start', Empty)
    resp = scan_start()
    rospy.Subscriber("/btr/centerPoint", Point, getCenterPoint)
    rospy.Subscriber("/btr/leftPoint", Point, getLeftPoint)
    rospy.Subscriber("/btr/rightPoint", Point, getRightPoint)

  udp.view3Send[1] = robViewMode
  udp.sender()

  # while True:
  while not rospy.is_shutdown():
    udp.receiver()
    # set publish data
    # odometry = Float32MultiArray()
    # odometry.data = (float(udp.view3Recv[1]) / 10, float(udp.view3Recv[2]) / 10, float(udp.view3Recv[3]) / 10)
    checkFlag = float(udp.view3Recv[1])
    getOdometry = Odometry()
    poseWithCovariance = PoseWithCovariance()
    point = Point()
    quaternion = Quaternion()
    pose = Pose()
    header = Header()
    theta = float(udp.view3Recv[4]) / 10
    point.x = float(udp.view3Recv[2]) / 10
    point.y = float(udp.view3Recv[3]) / 10
    point.z = 0 # float(udp.view3Recv[4]) / 10
    quaternion.x = math.cos(theta / 2.0) # 0
    quaternion.y = math.sin(theta / 2.0) # 0
    quaternion.z = math.sin(theta / 2.0) # 0
    quaternion.w = 0
    pose.position = point
    pose.orientation = quaternion
    poseWithCovariance.pose = pose
    poseWithCovariance.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    header.seq = 1
    header.stamp = rospy.Time.now()
    header.frame_id = "odometry"
    getOdometry.header = header
    getOdometry.pose = poseWithCovariance
    velocity = Float32MultiArray()
    velocity.data = (float(udp.view3Recv[5]) / 10, float(udp.view3Recv[6]) / 10, float(udp.view3Recv[7]) / 10)

    # rospy.loginfo(getOdometry)
    # rospy.loginfo(checkFlag)
    # rospy.loginfo(velocity)
    pub01.publish(getOdometry)
    pub02.publish(checkFlag)
    pub03.publish(velocity)

    if (robViewMode != oldMode):
      # print("mode change from ", oldMode, " to ", robViewMode)
      oldMode = robViewMode

    udp.sender()
    # time.sleep(0.1)
    rate.sleep()
    # if (checkFlag == 1):
    #   robViewMode = 0
    #   # velocity.data = (0, 0, 0)
    #   velocityData.pose = [0, 0, 0]

  udp.closer()


