#!/usr/bin/env python

# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#

from argparse import ArgumentParser

from hpp.corbaserver.robot import HumanoidRobot as Robot
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import ViewerFactory
from math import pi
from hpp.corbaserver import Client
import sys
Client ().problem.resetProblem ()

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

Robot.urdfFilename = 'package://example-robot-data/robots/talos_data/robots/talos_full_v2.urdf'
Robot.srdfFilename = 'package://example-robot-data/robots/talos_data/srdf/talos.srdf'
Robot.rightAnkle = 'leg_right_6_joint'
Robot.leftAnkle = 'leg_left_6_joint'

robot = Robot ('pyrene', rootJointType = 'freeflyer')
robot.setJointBounds ("root_joint", [-3, 3, -3, 3, 0, 1,-1,1,-1,1,-1,1,-1,1])

q0 = [0, 0, 1.0192720229567027, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# Add constraints
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)
# Remove joint bound validation
ps.hppcorba.problem.clearConfigValidations()
ps.addConfigValidation("CollisionValidation")
ps.addPartialCom ('pyrene', ['root_joint'])
robot.createSlidingStabilityConstraint ("balance/", 'pyrene', robot.leftAnkle,
                                        robot.rightAnkle,q0)
ps.addNumericalConstraints ("balance", ["balance/relative-com",
                                        "balance/relative-pose",
                                        "balance/pose-left-foot",])

# lock hands in closed position
gripperJoints = filter(lambda x:x.startswith('gripper_'), robot.jointNames)
lockedJoints = list()
for j in gripperJoints:
  ljName = "locked_" + j
  value = q0[robot.rankInConfiguration[j]]
  ps.createLockedJoint(ljName, j, [value,])
  lockedJoints.append (ljName)

for j in ['torso_1_joint', 'torso_2_joint']:
  ljName = "locked_" + j
  value = q0[robot.rankInConfiguration[j]]
  ps.createLockedJoint(ljName, j, [value,])
  lockedJoints.append (ljName)

ps.addLockedJointConstraints ("locked-hands", lockedJoints)

ps.selectPathValidation ("Dichotomy", 0)
ps.selectPathProjector ("Progressive", 0.2)

q1 = [0.46186525119743765, 0.7691484390667176, 1.0, 0.044318662659833724, -0.0108631325758057, -0.0005624014939687202, 0.9989582234484302, 0.007182038754728065, -0.07804157609345573, -0.45119414082769843, 0.9175221606997885, -0.44402665063685365, -0.012200787668632173, 0.007200628661317587, -0.0788724231291963, -0.4956000845048934, 1.009916799073695, -0.49201388832345117, -0.011369733964913645, 0.0, 0.006761, 0.2408808670823526, 0.28558871367875255, 0.021347338765649856, -0.5979935578118766, -0.0014717027925961507, 0.006759032911476202, 0.08832103581416396, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0392843760345567, -0.10575191252250002, -0.05668798069441503, -1.7498341362736458, 0.0022744473854138473, 0.0015716871843022243, 0.07078184761729372, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00020052684970875304, 0.00019305414086825983]

q2 = [-0.03792613133823603, 0.24583989035169654, 1.0, 0.008581421388221004, 0.044373915255123464, -0.0006050481369481731, 0.9989779520933587, 0.0011692308149178052, -0.011583002652064677, -0.5522315456639073, 0.9525259684676938, -0.4890594525896807, -0.007366718494771048, 0.0011679806161439602, -0.01159704912053673, -0.5610095845869443, 0.9704046648090222, -0.49816012449736746, -0.007352616506901346, 0.0, 0.006761, 0.25575424894162485, 0.21391256924828497, 0.006460912367916318, -0.5673886888192637, -0.0007964566272850148, 0.0027266557203091918, 0.09323792816834059, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5020992312243198, -0.770681876188843, 2.42600766027, -1.8794064100743089, 0.0019251455338804122, 0.007445905986286772, 0.06939811636044525, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0013061717130372818, 4.856617592856522e-05]

res, q1proj,err = ps.applyConstraints(q1)
assert res
res, msg = robot.isConfigValid(q1proj)
assert res
res, q2proj,err = ps.applyConstraints(q2)
assert res
res, msg = robot.isConfigValid(q2proj)
assert res

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
  except Exception as e:
    print (f"Failed to plan path: {e}")
  else:
    success += 1
    totalTime += t2 - t1
    print((t2-t1))
    n = len (ps.client.problem.nodes ())
    totalNumberNodes += n
    print(("Number nodes: " + str(n)))
if args.N != 0:
  print ("#" * 20)
  print (f"Number of rounds: {args.N}")
  print (f"Number of successes: {success}")
  print (f"Success rate: {success/ args.N * 100}%")
  if success > 0:
    print (f"Average time per success: {totalTime.total_seconds()/success}")
    print (f"Average number nodes per success: {totalNumberNodes/success}")
