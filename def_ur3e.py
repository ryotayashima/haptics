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

  return '''Mikata's default position for haptics.
  Usage: haptics.def_mikata'''

def Run(ct, *args):
  
  q_def = [-0.014975364510756902, -0.666861366134744, 1.616824224959834, 2.178139491333841, -1.554499574162408, 1.6118486136961405]
  ct.robot.MoveToQ(q_def, 2.0)
  