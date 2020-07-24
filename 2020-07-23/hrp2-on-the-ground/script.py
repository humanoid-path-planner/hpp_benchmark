#!/usr/bin/env python

# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#

from argparse import ArgumentParser

from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import ViewerFactory
from math import pi

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'
#Robot.urdfSuffix = ''
#Robot.srdfSuffix= ''

robot = Robot ('hrp2_14')
robot.setJointBounds ("root_joint", [-3, 3, -3, 3, 0, 1,-1,1,-1,1,-1,1,-1,1])
cl = robot.client

q0 = robot.getInitialConfig()

# Add constraints
ps = ProblemSolver (robot)
ps.addPartialCom ('hrp2_14', ['root_joint'])
robot.createSlidingStabilityConstraint ("balance/", 'hrp2_14', robot.leftAnkle,
                                        robot.rightAnkle,q0)
ps.addNumericalConstraints ("balance", ["balance/relative-com",
                                        "balance/relative-pose",
                                        "balance/pose-left-foot",])

# lock hands in closed position
lockedjointDict = robot.leftHandClosed ()
lockedJoints = list ()
for name, value in lockedjointDict.items ():
    ljName = "locked_" + name
    ps.createLockedJoint (ljName, name, value)
    lockedJoints.append (ljName)

lockedjointDict = robot.rightHandClosed ()
for name, value in lockedjointDict.items ():
    ljName = "locked_" + name
    ps.createLockedJoint (ljName, name, value)
    lockedJoints.append (ljName)

ps.addLockedJointConstraints ("locked-hands", lockedJoints)

q1 = [0.0, 0.0, 0.705, 0., 0., 0., 1.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q1)
if res [0]:
  q1proj = res [1]
else:
  raise RuntimeError ("Failed to apply constraint.")


q2 = [0.0, 0.0, 0.705, 0., 0., 0., 1., 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q2)
if res[0]:
  q2proj = res[1]
else:
  raise RuntimeError ("Failed to apply constraint.")


ps.selectPathValidation ("Progressive", 0.025)

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
for i in range (args.N):
    ps.client.problem.clearRoadmap ()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q1proj)
    ps.addGoalConfig (q2proj)
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
    totalTime += t2 - t1
    print((t2-t1))
    n = len (ps.client.problem.nodes ())
    totalNumberNodes += n
    print(("Number nodes: " + str(n)))

print(("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (args.N))))
print(("Average number nodes: " + str (totalNumberNodes/float (args.N))))

vf = ViewerFactory (ps)
