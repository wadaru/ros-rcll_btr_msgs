#!/usr/bin/python
TEAMNAME = "BabyTigers"

import struct
import time
import math
import sys
import rospy
import udpcomm
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, UInt32, String, Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
import rcll_btr_msgs
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportEntryBTR, \
                              MachineReportInfo, OrderInfo, Order, \
                              ProductColor, RingInfo, Ring, Team, Time
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendMachineReportBTR, SendPrepareMachine
#
# ROS for robotino
# 

def getResponse(value):
    while float(udp.view3Recv[1]) != 0:
        udp.view3Send[2] = refboxGameState
        udp.view3Send[3] = refboxGamePhase
        udp.view3Send[4] = refboxTime.sec
        udp.receiver()
        udp.sender()
        sendBeacon()
        print("wait for receiving 0")
        rate.sleep()
    return

def waitResponse():
    # set mode = 0 and wait for ack(!=0) from RobView.
    udp.view3Send[1] = 1
    getResponse(1)

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
    resp.ok = (sendRobView() == 1)
    # return resp
    return [resp.ok, ""]

def setPosition(data):
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
    resp.ok = (sendRobView() == 1)
    return [resp.ok, ""]
    # print("setPosition:", positionDriver.position.x)

def setOdometry(data):
    global odometryData, robViewMode
    resp = SetBoolResponse()
    odometryData = data
    robViewMode = 3
    udp.view3Send[1] = robViewMode # mode number
    udp.view3Send[2] = int(odometryData.position.x)
    udp.view3Send[3] = int(odometryData.position.y)
    udp.view3Send[4] = int(odometryData.orientation.z)

    resp.ok = (sendRobView() == 1)
    return [resp.ok, ""]

#
# receive information from RefBox
#

def beaconSignal(data):
    global refboxBeaconSignal
    refboxBeaconSignal = data
    # print("BeaconSignal: ", data)

def explorationInfo(data):
    global refboxExplorationInfo
    refboxExplorationInfo = data
    # print("ExplorationInfo: ", data)

def gameState(data):
    global refboxTime, refboxGameState, refboxGamePhase, \
           refboxPointsMagenta, refboxTeamMagenta, \
           refboxPointCyan, refboxTeamCyan
    refboxTime = data.game_time
    refboxGameState = data.state
    refboxGamePhase = data.phase
    refboxPointsMagenta = data.points_magenta
    refboxTeamMagenta = data.team_magenta
    refboxPointsCyan = data.points_cyan
    refboxTeamCyan = data.team_cyan
    # print("GameState: ", data)

def machineInfo(data):
    global refboxMachineInfo
    refboxMachineInfo = data
    # print("MachineInfo: ", data)

def machineReportInfo(data):
    global refboxMachineReportInfo
    refboxMachineReportInfo = data
    # print("MachineReportInfo: ", data)

def orderInfo(data):
    global refboxOrderInfo
    refboxOrderInfo = data
    # print("OrderInfo: ", data)

def ringInfo(data):
    global refboxRingInfo
    refboxRingInfo = data
    # print("RingInfo: ", data)

#
# send information to RefBox
#
def sendBeacon():
    beacon = SendBeaconSignal()
    header1 = Header()
    header2 = Header()
    poseStamped = PoseStamped()
    pose = Pose()
    
    # pose.position = point
    # set Pose
    pose.position.x = float(udp.view3Recv[2]) / 10
    pose.position.y = float(udp.view3Recv[3]) / 10
    pose.position.z = 0
    # set quaternion
    theta = math.radians(float(udp.view3Recv[4]) / 10)
    pose.orientation.x = math.cos(theta / 2.0)
    pose.orientation.y = math.sin(theta / 2.0)
    pose.orientation.z = math.sin(theta / 2.0)
    pose.orientation.w = 0
    header1.seq = 1
    header1.stamp = rospy.Time.now()
    header1.frame_id = TEAMNAME
    header2.seq = 1
    header2.stamp = rospy.Time.now()
    header2.frame_id = "robot1"
    poseStamped.header = header2
    poseStamped.pose = pose
    beacon.header = header1
    beacon.pose  = poseStamped

    rospy.wait_for_service('/rcll/send_beacon')
    try:
        refboxSendBeacon = rospy.ServiceProxy('/rcll/send_beacon', SendBeaconSignal)
        resp1 = refboxSendBeacon(beacon.header, beacon.pose)
        # print("sendBeacon: ", beacon.header, beacon.pose)
        # print("resp: ", resp1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendMachineReport(report):
    sendReport = SendMachineReport()
    machineReport = MachineReportEntryBTR()
    machineReport.name = report.name
    machineReport.type = report.type
    machineReport.zone = report.zone
    machineReport.rotation = report.rotation
    if (refboxTeamCyan == TEAMNAME):
        sendReport.team_color = 1
    else:
        sendReport.team_color = 2
    machineReportEntryBTR = [machineReport]
    sendReport.machines = machineReportEntryBTR
    print("machineReport: ", machineReport)

    rospy.wait_for_service('/rcll/send_machine_report')
    try:
        refboxMachineReport = rospy.ServiceProxy('/rcll/send_machine_report', SendMachineReportBTR)
        resp1 = refboxMachineReport(sendReport.team_color, sendReport.machines)
        # print("sendBeacon: ", beacon.header, beacon.pose)
        # print("resp: ", resp1)
        waitResponse()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendPrepareMachine(data):
    prepare = SendPrepareMachine()
    # prepare.machine = data.machine
    prepare.machine = data.machine
    prepare.bs_side = 0
    prepare.bs_base_color = 0
    prepare.ds_order_id = 0
    prepare.cs_operation = 0
    prepare.rs_ring_color =0
    
    machineType = prepare.machine[2:4]
    print("sendPrepareMachine: ", machineType)
    if (machineType == "BS"):
        prepare.bs_side = data.bs_side
        prepare.bs_base_color =data.bs_base_color
    if (machineType == "DS"):
        prepare.ds_order_id = data.ds_order_id
    if (machineType == "CS"):
        prepare.cs_operation = data.cs_operation
    if (machineType == "RS"):
        prepare.rs_ring_color = data.rs_ring_color
    prepare.wait = False # data.wait
    rospy.wait_for_service('/rcll/send_prepare_machine')
    try:
        refboxPrepareMachine = rospy.ServiceProxy('/rcll/send_prepare_machine', SendPrepareMachine)
        resp1 = refboxPrepareMachine(prepare.machine, prepare.wait, prepare.bs_side, prepare.bs_base_color, prepare.ds_order_id, prepare.rs_ring_color, prepare.cs_operation)
        waitResponse()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

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

  # valiables for refbox
  refboxBeaconSignal = BeaconSignal()
  refboxExplorationInfo = ExplorationInfo()
  refboxExplorationSignal = ExplorationSignal()
  refboxExplorationZone = ExplorationZone()
  refboxGameState = 0 # Int8()
  refboxGamePhase = 0 # Int8()
  refboxPointsMagenta = UInt32()
  refboxTeamMagenta = String()
  refboxPointsCyan = UInt32()
  refboxTeamCyan = String()
  refboxLightSpec = LightSpec()
  refboxMachineInfo = MachineInfo()
  refboxMachine = Machine()
  refboxMachineReportEntry = MachineReportEntryBTR()
  refboxMachineReportInfo = MachineReportInfo()
  refboxOrderInfo = OrderInfo()
  refboxOrder = Order()
  refboxProductColor = ProductColor()
  refboxRingInfo = RingInfo()
  refboxRing = Ring()
  refboxTeam = Team()
  refboxTime = Time()

  rospy.init_node('robotino')
  rospy.Subscriber("rcll/beacon", BeaconSignal, beaconSignal)
  rospy.Subscriber("rcll/exploration_info", ExplorationInfo, explorationInfo)
  rospy.Subscriber("rcll/game_state", GameState, gameState)
  rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
  rospy.Subscriber("rcll/machine_report_info", MachineReportInfo, machineReportInfo)
  rospy.Subscriber("rcll/order_info", OrderInfo, orderInfo)
  rospy.Subscriber("rcll/ring_info", RingInfo, ringInfo)
  
  rate = rospy.Rate(10)

  robViewMode = 1
  machineReport = MachineReportEntryBTR()
  prepareMachine = SendPrepareMachine() 
  machineName = [
                  "C-CS1", "C-CS2", "C-BS", "C-RS1", "C-RS2", "C-DS", "C-SS",
                  "M-CS1", "M-CS2", "M-BS", "M-RS1", "M-RS2", "M-DS", "M-SS"
                ]

  udp.view3Send[1] = robViewMode
  udp.sender()

  # while True:
  while not rospy.is_shutdown():
    udp.receiver()
    sendBeacon()

    # get GameState
    # if (udp.view3Recv[1] == 10):
    robViewMode = 1
    udp.view3Send[1] = robViewMode
    udp.view3Send[2] = refboxGameState
    udp.view3Send[3] = refboxGamePhase
    udp.view3Send[4] = refboxTime.sec

    # Exploration Phase
    # machineRecognition
    if (udp.view3Recv[1] == 10):
        machineReport.name = machineName[udp.view3Recv[5]]
        machineReport.type = machineReport.name[2:4]
        machineReport.zone = udp.view3Recv[6]
        machineReport.rotation = udp.view3Recv[7]
        print(machineReport)
        sendMachineReport(machineReport)

    # Production Phase
    ## make C0
    ## which requires get base with cap from shelf at C-CS1, 
    ##                Retrieve cap at C-CS1,
    ##                bring base without cap to C-RS1,
    ##                get base at C-BS,
    ##                bring base to C-CS1,
    ##                Mount cap at C-CS1,
    ##                bring it to C-DS corresponded by order it.

    # planning == 1 || 3
    if (udp.view3Recv[1] == 20):
        prepareMachine.machine = machineName[udp.view3Recv[5]]
        prepareMachine.cs_operation = udp.view3Recv[6] # CS_OP_RETRIEVE_CAP
        prepareMachine.wait = True
        sendPrepareMachine(prepareMachine)
    
    # planning == 2
    if (udp.view3Recv[1] == 30):
        prepareMachine.machine = machineName[udp.view3Recv[5]]
        prepareMachine.bs_side = udp.view3Recv[6]  # INPUT or OUTPUT side
        prepareMachine.bs_base_color = udp.view3Recv[7] # BASE COLOR
        prepareMachine.wait = True
        sendPrepareMachine(prepareMachine)
    
    # planning == 4
    if (udp.view3Recv[1] == 40):
        prepareMachine.machine = machineName[udp.view3Recv[5]]
        prepareMachine.ds_order_id = udp.view3Recv[6] # ORDER ID
        prepareMachine.wait = True
        sendPrepareMachine(prepareMachine)

    # planning == 01
    if (udp.view3Recv[1] == 50):
        prepareMachine.machine = machineName[udp.view3Recv[5]]
        prepareMachine.rs_ring_color = udp.view3Recv[6] # RING COLOR
        prepareMachine.wait = True
        sendPrepareMachine(prepareMachine)

    udp.sender()
    rate.sleep()

  udp.closer()


