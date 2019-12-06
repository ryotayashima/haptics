#!/usr/bin/python
from core_tool import *
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniState
import time
import math
import numpy as np

def Help():

  return '''Mikata6's default position for haptics.
  Usage: haptics.position'''

def Run(ct, *args):
  
  #ct.Run('grip', 0.05)
  # print ct.robot.GripperPos()
  ct.robot.MoveGripper(0.05,blocking=True)
  x_def = [0.25, -0.01, 0.06, 0.0, 0.0, 0.0, 1.0]
  ct.robot.MoveToX(x_def, 2.0)
  