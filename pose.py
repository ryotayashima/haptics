#!/usr/bin/python
from core_tool import *
import rospy
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniState

def Help():
  return '''Read haptics states script.
  ---subscribe---
  /phantom/pose
  ---------------
  Usage: haptics.states'''

def Run(ct, *args):

  def pose(msg):
    out = []
    out.append(msg.pose.position)			# ex) position.x (float64)
    out.append(msg.pose.orientation)	# ex) orientation.x (float64)
    print('---position---')
    print(out[0])
    print('---orientation---')
    print(out[1])
    print('')
  # def velocity(msg):
  #   out = msg.velocity                # ex) velocity.x (float64)
  #   print('---velocity---')
  #   print(out)    

  subPose = rospy.Subscriber("/phantom/pose", PoseStamped, pose)
  # subVelo = rospy.Subscriber("/phantom/state", OmniState, velocity)

  rospy.spin()