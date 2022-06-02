#!/usr/bin/python
import struct
import time
import sys
import rospy
from geometry_msgs.msg import Pose
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Float32, Float32MultiArray, Bool

class Udpcomm():
  def __init__(self, sADDRESS = "127.0.1.1", sPORT = 9180, rADDRESS = "127.0.1.1", rPORT = 9182):

    self.HOST = ''
    self.sendPORT = int(sPORT) # 9180
    self.recvPORT = int(rPORT) # 9182
    self.sendADDRESS = sADDRESS # "127.0.1.1"
    self.recvADDRESS = rADDRESS # "127.0.1.1"

    self.send = socket(AF_INET, SOCK_DGRAM)
    # self.view3Send = struct.Struct("!BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
    self.view3Recv = struct.Struct("!BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
    #                           [00][01][02][03][04][05][06][07][08]
    #self.view3Send = [00,00,00,00,255,22,23,24, 31,32,33,34, 41,42,43,44,
    #                   51,52,53,54, 61,62,63,64, 71,72,73,74, 81,82,83,84]
    self.view3Send = [ 0, 1, 2, 3,  4, 5, 6, 7]
    self.view3Info = [00,00,00,00, 00,00,00,00, 00,00,00,00, 00,00,00,00,
                      00,00,00,00, 00,00,00,00, 00,00,00,00, 00,00,00,00]
    self.view3Recv = [00,00,00,00, 00,00,00,00, 00,00,00,00, 00,00,00,00,
                      00,00,00,00, 00,00,00,00, 00,00,00,00, 00,00,00,00]
    self.data = []


    # print("recvADDRESS: ", self.recvADDRESS, ", recvPORT: ", self.recvPORT)
    self.BUFSIZE = 1024
    self.recv = socket(AF_INET, SOCK_DGRAM)
    self.recv.bind((self.recvADDRESS, self.recvPORT))

  def sender(self):
    s = struct.Struct("!BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
    self.data =[00, 36, 00, 00];

    for i in range(4, 9 * 4, 4):
      # print(type(i), i)
      self.view3Info[i - 4] = self.view3Send[int((i - 4) / 4)]
      if (self.view3Info[i - 4] < 0):
        self.view3Info[i - 4] = ((-self.view3Send[int((i - 4) / 4)]) ^ 0xffffffff) + 1
      # if (self.view3Send[i - 4] > 255):
      self.view3Info[i - 1] = (self.view3Send[int((i - 4) / 4)] & 0xff000000) >> 24
      self.view3Info[i - 2] = (self.view3Send[int((i - 4) / 4)] & 0x00ff0000) >> 16
      self.view3Info[i - 3] = (self.view3Send[int((i - 4) / 4)] & 0x0000ff00) >>  8
      self.view3Info[i - 4] = (self.view3Send[int((i - 4) / 4)] & 0x000000ff) 

    # print(self.view3Info[11], self.view3Info[10], self.view3Info[9], self.view3Info[8])
    for i in range(4, 9 * 4):
      # print(i)
      self.data.append(self.view3Info[i - 4]);

    self.data[3] = 0;
    checkSum = 0;
    for i in range(36):
      checkSum += self.data[i];
      if (checkSum > 0xff):
        checkSum -= 0x100;
    self.data[3] = (0xff - checkSum);

    # print(data)
    # print(len(data))
    tmpData = []
    for i in range(0, len(self.data), 4):
      tmpData.append(self.data[i + 0])
      tmpData.append(self.data[i + 1])
      tmpData.append(self.data[i + 2])
      tmpData.append(self.data[i + 3])
    self.data = tmpData
    if (len(self.data) != 36):
        print("range", len(self.data))
    sendData =s.pack(*self.data)
    if (len(self.data) == 36):
        self.send.sendto(sendData, (self.sendADDRESS, self.sendPORT))
        #                           [00][01][02][03][04][05][06][07][08]
    # for i in range(4, 36):
    #     self.view3Send[i - 4] = 0
    # print("send:", len(self.data), "(" , self.sendADDRESS, ",", self.sendPORT, ")", end="")
    # for i in range(len(self.data)):
    #   print(format(self.data[i], '02x'), "")
    # print()

  def receiver(self):
    s = struct.Struct("!B")
    data, addr = self.recv.recvfrom(self.BUFSIZE)
    for i in range(8):
      self.view3Recv[i] = ((int(data[i * 4 + 4 + 0])      )
                        + (int(data[i * 4 + 4 + 1]) <<  8)
                        + (int(data[i * 4 + 4 + 2]) << 16)
                        + (int(data[i * 4 + 4 + 3]) << 24))
      if (self.view3Recv[i] >= (1 << 31)):
        self.view3Recv[i]  -= (1 <<32)
      # print (self.view3Recv[i])

    # self.view3Send[0] = data[4] + 1;
    self.view3Send[0] = self.view3Recv[0] + 1;
    if (self.view3Send[0] > 255):
      self.view3Send[0] = 0


  def closer(self):
    self.send.close()

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
  udp = udpcomm(sendADDRESS, sendPORT, recvADDRESS, recvPORT)
  while True:
    udp.receiver()
    udp.sender()
    time.sleep(0.1)
  udp.closer()


