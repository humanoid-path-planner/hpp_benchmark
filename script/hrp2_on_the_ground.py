#/usr/bin/env python

from argparse import ArgumentParser
from hpp_ros import ScenePublisher, PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient

# parse arguments {{{2
parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
args = parser.parse_args()
# 2}}}

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
robot.setTranslationBounds (-3, 3, -3, 3, 0, 1)
cl = robot.client

r = ScenePublisher (robot)
q0 = robot.getInitialConfig ()
r (q0)

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle)

ps = ProblemSolver (robot)
ps.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])

# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for name, value in lockedDofs.iteritems ():
    ps.lockDof (name, value, 0, 0)

lockedDofs = robot.rightHandClosed ()
for name, value in lockedDofs.iteritems ():
    ps.lockDof (name, value, 0, 0)


q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")


q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

ps.selectPathOptimizer ("None")
import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
success = 0
for i in range (args.N):
  ps.client.problem.clearRoadmap ()
  ps.resetGoalConfigs ()
  ps.setInitialConfig (q1proj)
  ps.addGoalConfig (q2proj)
  try:
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
  except:
    print ("Failed to plan path.")
  else:
    success += 1
    totalTime += t2 - t1
    print (t2-t1)
    n = len (ps.client.problem.nodes ())
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

if args.N != 0:
  print (f"Number of rounds: {args.N}")
  print (f"Number of successes: {success}")
  print (f"Success rate: {success/ args.N * 100}%")
  if success > 0:
    print (f"Average time per success: {totalTime.total_seconds()/success}")
    print (f"Average number nodes per success: {totalNumberNodes/success}")

pp = PathPlayer (cl, r)
pp (1)
