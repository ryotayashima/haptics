#!/usr/bin/python
from core_tool import *
import rospy

def Help():
    
  return '''Mikata's initial position for haptics.
  Usage: haptics.init_mikata'''

def Run(ct, *args):
  q_init = [0]*4
  ct.robot.MoveToQ(q_init, 2.0)