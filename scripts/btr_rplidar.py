#!/usr/bin/env python
#!/usr/bin/python

START_ANGLE = -180 # -90
END_ANGLE = 0 # 90
START_EDGE_ANGLE = -120
END_EDGE_ANGLE = -60
THRESHOLD_ANGLE = 20

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

#
def scanDistance(deg):
  return scanData.ranges[len(scanData.ranges) / 360 * ((deg + 360) % 360)]

#
def polarToPoint(distance, angle):
  point = Point()
  radian = math.radians(angle + START_ANGLE)
  # point.x = distance * math.cos(radian)
  # point.y = distance * math.sin(radian)
  point.x = distance * math.sin(radian)
  point.y = distance * -math.cos(radian)
  point.z = 0
  return point

#
def calcAngle(pointA, pointB):
  point = Point()
  point.x = pointA.x - pointB.x
  point.y = pointA.y - pointB.y
  return math.degrees(math.atan2(point.y, point.x))

#
def findEdge(startAngle, angleStep):
  oldPoint = polarToPoint(scanDistance(startAngle - angleStep), startAngle - angleStep)
  i = startAngle
  oldAngle = -360

  while True:
    nowPoint = polarToPoint(scanDistance(i), i)
    angle = calcAngle(oldPoint, nowPoint)
    if (oldAngle == -360):
      oldAngle = angle
    elif (abs(oldAngle - angle) > THRESHOLD_ANGLE):
      break
    if (math.isinf(scanDistance(i))):
      break
    i = i + angleStep
    if (i < -180 or i > 180):
      break
  
  # print("findEdge: ", i - angleStep, scanDistance(i - angleStep))
  return polarToPoint(scanDistance(i - angleStep), i - angleStep)

#
def calcPoint():
  global centerPoint, closePoint, leftPoint, rightPoint
  minDistance = scanDistance((START_ANGLE + END_ANGLE) / 2)
  minAngle = (START_ANGLE + END_ANGLE) / 2
  centerPoint = polarToPoint(minDistance, minAngle)

  for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
    if (minDistance > scanDistance(i)):
      minDistance = scanDistance(i)
      minAngle = i
  # print("minAngle:", minAngle, ", minDistance:", minDistance)
  closePoint = polarToPoint(minDistance, minAngle)

  # find the left edge and right edge
  # leftPoint = findEdge(minAngle - 1, -1)
  # rightPoint = findEdge(minAngle + 1, 1)
  leftPoint  = findEdge(minAngle + 1,  1)
  rightPoint = findEdge(minAngle - 1, -1)

#
def laserScan(data):
  global scanData
  scanData = data
  # scanNumber = len(scanData.ranges)
  if (scanFlag == True):
    calcPoint()
  # else:
  #   print   "0:", scanDistance(  0), \
  #          "90:", scanDistance( 90), \
  #         "180:", scanDistance(180), \
  #         "270:", scanDistance(-90)

#
def btrScanStart(self):
  global scanFlag
  scanFlag = True
  print("start publishing")
  return EmptyResponse()
#
def btrScanStop(self):
  global scanFlag
  scanFlag = False
  print("stop publishing")
  return EmptyResponse()

# main
#
if __name__ == '__main__':
  # args = sys.argv

  scanFlag = False
  centerPoint = Point()
  closePoint = Point()
  leftPoint = Point()
  rightPoint = Point()

  rospy.init_node('btr_scan')
  sub01 = rospy.Subscriber("/scan", LaserScan, laserScan)
  srv01 = rospy.Service("/btr/scan_start", Empty, btrScanStart)
  srv02 = rospy.Service("/btr/scan_stop", Empty, btrScanStop)
  pub00 = rospy.Publisher("/btr/centerPoint", Point, queue_size = 10)
  pub01 = rospy.Publisher("/btr/closePoint", Point, queue_size = 10)
  pub02 = rospy.Publisher("/btr/leftPoint", Point, queue_size = 10)
  pub03 = rospy.Publisher("/btr/rightPoint", Point, queue_size = 10)
  rate = rospy.Rate(10)

  # rospy.spin()

  scanData = LaserScan
  
  while not rospy.is_shutdown():
    if (scanFlag == True):
      pub00.publish(centerPoint)
      pub01.publish(closePoint)
      pub02.publish(leftPoint)
      pub03.publish(rightPoint)
    rate.sleep()

