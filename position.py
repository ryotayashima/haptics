#!/usr/bin/python
from core_tool import *
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniState
import time

def Help():

  return '''Position contorl script by haptics.
  Usage: haptics.position'''

def Run(ct, *args):
      
  x = ct.robot.FK()

  def cont(vel, i, rate):
    if abs(vel) < 1.0:
      pass
    else:
      x[i] += vel*rate

  def move(msg):
    rate = 1e-4*1.5
    rospy.loginfo(msg.velocity)
    cont(msg.velocity.x, 1, -rate)
    cont(msg.velocity.y, 0, rate)
    cont(msg.velocity.z, 2, rate)
    
    origin = [x[i+3] for i in range(4)]
    haps = [
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z,
      msg.pose.orientation.w  
    ]

    rotationY90 = tf.transformations.quaternion_about_axis(math.pi/2, [0,1,0])
    rotationY90 = rotationY90.tolist()

    new = tf.transformations.quaternion_multiply(origin, haps)
    new = tf.transformations.quaternion_multiply(haps, rotationY90)

    for i, component in enumerate(new):
      x[i+3] = component
    # x[3] += msg.pose.orientation.x
    # x[4] += msg.pose.orientation.y
    # x[5] += msg.pose.orientation.z
    # x[6] += msg.pose.orientation.w
    ct.robot.MoveToX(x, 0.1)

  sub = rospy.Subscriber('/phantom/state', OmniState, move)
  rospy.spin()
  # r = rospy.Rate(0.5)
  # while not rospy.is_shutdown():
  #   r.sleep()

  # movex [0.45000740211100976, 9.2708276568646815e-07, 0.49999834857096287, 0.025, 0.02571235660000027, -0.7, 0.7]