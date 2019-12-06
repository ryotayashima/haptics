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
  
  x_def = [0.059006140732741963,
           -0.0016296713292546724,
           0.12926543907595478,
           0.00037055650369856058,
           0.026838881738960561,
           -0.013800414496973199,
           0.99954443906941504]
  ct.robot.MoveToX(x_def, 2.0)
  