#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# image publisher
# 2021.05.20 version 2.0
# by T.Nishimura @AiRRC
#
#<usage>
#$ ros2 run ros2robo cam           ## terminal1
#<reference>
#https://zido.co.jp/tech/ros/pubsub/
#
import cv2
#
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
import sensor_msgs.msg
from cv_bridge import CvBridge
from std_msgs.msg import String
#
class UsbCam(Node):
    def __init__(self):
        super().__init__("camera")
        qos = QoSProfile(
             depth=1,
             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
             durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.pub_img = self.create_publisher(sensor_msgs.msg.Image, "/img_topic",qos)
        self.cvb = CvBridge()
        print ('start camera')
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(3,320)
        self.cap.set(4,240)
        self.count = 0
        self.time_period = 0.1
        self.tmr = self.create_timer(self.time_period, self.callback)

    def callback(self):
        self.count += 1
        ret, data = self.cap.read()
        frameHeight = data.shape[0]
        frameWidth = data.shape[1]
        print(frameWidth,frameHeight)
        image=cv2.cvtColor(data,cv2.COLOR_BGR2RGB)
        self.pub_img.publish(self.cvb.cv2_to_imgmsg(image,"rgb8"))

def main():
    rclpy.init()
    node = UsbCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

