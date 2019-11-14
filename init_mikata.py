#!/usr/bin/python
from core_tool import *
import rospy

def Help():
    
  return '''Mikata6's initial position for haptics.
  Usage: haptics.position'''

def Run(ct, *args):
  q_init = [0]*6
  ct.robot.MoveToQ(q_init, 2.0)