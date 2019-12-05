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

  return '''real Mikata's position contorl script by haptics.
  Usage: haptics.move_mikata_real'''


def Callback(rate, steps, wsteps, state, msg):
  steps[:] = [0.0]*3
  wsteps[:] = [0.0]*3
  state[:] = ['run', 0]

  state[0] = 'stop' if msg.locked==True else 'run'

  steps[0] = rate * msg.velocity.y if abs(msg.velocity.y)>1.0 else 0.0
  steps[1] = -rate * msg.velocity.x if abs(msg.velocity.x)>1.0 else 0.0
  steps[2] = rate * msg.velocity.z if abs(msg.velocity.z)>1.0 else 0.0


def Run(ct, *args):
  if len(args) == 1:
    rate = 1e-4 * args[0]
  elif len(args) == 0:
    rate = 1e-4 * 1.2

  steps = [0.0, 0.0, 0.0]
  wsteps = [0.0, 0.0, 0.0]
  state = ['run', 0]
  speed_gain= 10.0

  arm= ct.robot.Arm
  is_dxlg= [ct.robot.EndEff(a) is not None and ct.robot.EndEff(a).Is('DxlGripper') for a in range(ct.robot.NumArms)]
  if any(is_dxlg):
    active_holding= [False]*ct.robot.NumArms

  ct.AddSub('/phantom/state', '/phantom/state', OmniState, lambda msg: Callback(rate, steps, wsteps, state, msg))
  velctrl= [ct.Run('velctrl',a) for a in range(ct.robot.NumArms)]
  suppress_velctrl= False

  kbhit= TKBHit()
  try:
    while True:
      if state[0] is 'run':
        if kbhit.IsActive():
          key= kbhit.KBHit()
          if key=='q':
            break
        else:
          break

        q= ct.robot.Q(arm=arm)
        vx= map(lambda b:speed_gain*0.4*b,steps)+map(lambda c:speed_gain*1.0*c,[0.0,0.0,0.0])
        if ct.robot.DoF(arm=arm)>=6:
          dq= ToList(la.pinv(ct.robot.J(q,arm=arm))*MCVec(vx))
        else:  #e.g. Mikata Arm
              #We use weighted pseudo inverse of Jacobian.
              #W: weights on pose error.
          if Norm(vx[:3])>=Norm(vx[3:]):
            W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
          else:
            W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
          dq= ToList(la.pinv(W*ct.robot.J(q,arm=arm))*W*MCVec(vx))

        if not suppress_velctrl:
          velctrl[arm].Step(dq)
          rospy.loginfo(steps)
      else:
        pass

  finally:
    kbhit.Deactivate()
    for a in range(ct.robot.NumArms):
      velctrl[a].Finish()
    for a in range(ct.robot.NumArms):
      if is_dxlg[a]:
        if active_holding[arm]:
          ct.robot.EndEff(arm).StopHolding()
          active_holding[arm]= False
    for thread in filter(lambda th:th[:3]=='vs_', ct.thread_manager.thread_list):
      print 'Turn off:',thread
      ct.thread_manager.Stop(name=thread)

    ct.DelSub('/phantom/state')
    print 'Finished'