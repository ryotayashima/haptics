#!/usr/bin/python
from core_tool import *
import rospy
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniState
import time

def Help():

  return '''Position contorl script by haptics.
  Usage: haptics.position'''

def Run(ct, *args):
  x = ct.robot.FK()

  def move(msg):
    rate = 1e-3 * 2
    rospy.loginfo(msg.velocity)
    # x[0] += msg.velocity.x * rate
    # x[1] += msg.velocity.y * rate
    # x[2] += msg.velocity.z * rate
    x[0] += rate if msg.velocity.x>0 else -rate
    x[1] += rate if msg.velocity.y>0 else -rate
    x[2] += rate if msg.velocity.z>0 else -rate
    ct.robot.MoveToX(x, 0.1)

  sub = rospy.Subscriber('/phantom/state', OmniState, move)

  rospy.spin()
  # r = rospy.Rate(0.5)
  # while not rospy.is_shutdown():
  #   r.sleep()