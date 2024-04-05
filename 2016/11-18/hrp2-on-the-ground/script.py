#/usr/bin/env python

from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient, Problem

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
robot.setJointBounds ("base_joint_xyz", (-3, 3, -3, 3, 0, 1))
cl = robot.client

q0 = robot.getInitialConfig ()

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle, "",
                                           Problem.SLIDING)

ps = ProblemSolver (robot)
ps.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])

# lock hands in closed position
lockedjoints = robot.leftHandClosed ()
for name, value in lockedjoints.iteritems ():
    ps.lockJoint (name, value)

lockedjoints = robot.rightHandClosed ()
for name, value in lockedjoints.iteritems ():
    ps.lockJoint (name, value)


q1 = [0.0, 0.0, 0.705, 0., 0., 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")


q2 = [0.0, 0.0, 0.705, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

ps.selectPathValidation ("Progressive", 0.025)

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
N = 20
for i in range (N):
    ps.client.problem.clearRoadmap ()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q1proj)
    ps.addGoalConfig (q2proj)
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
    totalTime += t2 - t1
    print (t2-t1)
    n = len (ps.client.problem.nodes ())
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (N)))
print ("Average number nodes: " + str (totalNumberNodes/float (N)))

from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)
r = vf.createViewer()
from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)
