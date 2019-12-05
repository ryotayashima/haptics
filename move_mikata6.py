#!/usr/bin/python
from core_tool import *
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniState, OmniButtonEvent
import time
import math
import numpy as np

def Help():

  return '''Mikata6's position contorl script by haptics.
  Usage: haptics.position'''

def Run(ct, *args):
      
  x = ct.robot.FK()
  if len(args) == 1:
    rate = 1e-4 * args[0]
  elif len(args) == 0:
    rate = 1e-4 * 1.2

  def cont(vel, i, rate):
    if abs(vel) < 1.0:
      pass
    else:
      x[i] += vel*rate

  def move(msg):

    if msg.locked==False:
      #### Position ###
      rospy.loginfo(msg.velocity)
      cont(msg.velocity.x, 1, -rate)
      cont(msg.velocity.y, 0, rate)
      cont(msg.velocity.z, 2, rate)
      ### Quaternion ###
      haps = [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w  
      ]
      tmpx = haps[0]
      tmpy = haps[1]
      haps[0] = -haps[2]
      haps[1] = -tmpx
      haps[2] = tmpy
      haps[3] = haps[3]
      new = haps
      ### Griper ###
      if msg.close_gripper==True and ct.robot.GripperPos()<0.075:
        ct.robot.MoveGripper(0.1,blocking=True)
      elif msg.close_gripper==False and ct.robot.GripperPos()>0.075:
        ct.robot.MoveGripper(0.05,blocking=True)
      else:
        pass
      ### Move ### 
      for i, component in enumerate(new):
        x[i+3] = component
      ct.robot.MoveToX(x, 0.1)
    else:
      pass

  sub = rospy.Subscriber('/phantom/state', OmniState, move)
  rospy.spin()
  # r = rospy.Rate(0.5)
  # while not rospy.is_shutdown():
  #   r.sleep()