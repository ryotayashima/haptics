#!/usr/bin/python
from core_tool import *
import rospy
from geometry_msgs.msg import PoseStamped
import time

def Help():

  return '''Position contorl script by haptics.
  Usage: haptics.position'''

def Run(ct, *args):
  flag = 0
  x = ct.robot.FK()

  def move(msg):
    tmp = []
    tmp.append(msg.pose.position.x)
    tmp.append(msg.pose.position.y)
    tmp.append(msg.pose.position.z)
    time.sleep(0.25)
    tmp[0] = msg.pose.position.x - tmp[0]
    tmp[1] = msg.pose.position.y - tmp[1]
    tmp[2] = msg.pose.position.z - tmp[2]
    x[0] += tmp[0]
    x[1] += tmp[1]
    x[2] += tmp[2]
    ct.robot.MoveToX(x, 0.25)

  sub = rospy.Subscriber('/phantom/pose', PoseStamped, move)

  r = rospy.Rate(2)
  while not rospy.is_shutdown():
    r.sleep()