#!/usr/bin/env python
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#
# vim: foldmethod=marker foldlevel=2

from argparse import ArgumentParser

from hpp.corbaserver.manipulation import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Constraints, Rule, Client
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import Color, PathPlayer
from math import sqrt
from hpp.corbaserver import loadServerPlugin
loadServerPlugin ("corbaserver", "manipulation-corba.so")
Client ().problem.resetProblem ()

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

Robot.urdfFilename = "package://example-robot-data/robots/baxter_description/urdf/baxter.urdf"
Robot.srdfFilename = "package://example-robot-data/robots/baxter_description/srdf/baxter_manipulation.srdf"

# nbBoxes
K = 2
nBoxPerLine = 2
# grippers = [ "baxter/r_gripper",]
grippers = [ "baxter/r_gripper" ,  "baxter/l_gripper"]
# Box i will be at box goal[i] place at the end
#goal = [1, 2, 0]
goal = [1, 0]
# Load robot and object. {{{3
# Define classes for the objects {{{4
class Table (object):
  rootJointType = "anchor"
  packageName = 'hpp_tutorial'
  urdfFilename = "package://hpp_tutorial/urdf/table.urdf"
  srdfFilename = "package://hpp_tutorial/srdf/table.srdf"

class Box (object):
  rootJointType = "freeflyer"
  urdfFilename = "package://hpp_environments/urdf/baxter_benchmark/box.urdf"
  srdfFilename = "package://hpp_environments/srdf/baxter_benchmark/box.srdf"
  joint = "base_joint"
  handle = "handle"
# 4}}}
robot = Robot ('baxter-manip', 'baxter', rootJointType="anchor")
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)
#robot.setRootJointPosition ("baxter" , [-3.2,-3.9, 0.926, 1, 0, 0, 0])
robot.setRootJointPosition ("baxter" , [0, 0, 0.926, 0, 0, 0, 1])
vf.loadEnvironmentModel (Table, "table")
boxes = list()
for i in range(K):
  boxes.append ("box" + str(i))
  vf.loadObjectModel (Box, boxes[i])
  robot.setJointBounds (boxes[i]+ '/root_joint', [-1,0.5,-1,2,0.6,1.9,-1,1,-1,1,-1,1,-1,1])
def setBoxColors (gui):
  c = Color()
  for i in range(K):
    gui.setColor (boxes[i], c[i])
# 3}}}
# Define configurations. {{{3
q_init = robot.getCurrentConfig ()
rankB = list()
for i in range(K):
  rankB.append (robot.rankInConfiguration [boxes[i] + '/root_joint'])
bb = [0.7, 0.8, 0., 0.1]
c = sqrt (2) / 2
xstep = (bb[1] - bb[0]) / (nBoxPerLine - 1) if nBoxPerLine > 1 else (bb[1] - bb[0])
nbCols = int(K * 1. / nBoxPerLine + 0.5)
ystep = (bb[3] - bb[2]) / (nbCols - 1) if nbCols > 1 else (bb[3] - bb[2])
for i in range(K):
  iL = i % nBoxPerLine
  iC = (i - iL) / nBoxPerLine
  x = bb[0] + xstep * iL
  y = bb[2] + xstep * iC
  q_init [rankB[i]:rankB[i]+7] = [x, y, 0.746, 0, -c, 0, c]
q_goal = q_init [::]
for i in range(K):
  r  = rankB[i]
  rn = rankB[goal[i]]
  q_goal[r:r+7] = q_init[rn:rn+7]
# 3}}}
robot.client.basic.problem.resetRoadmap ()
ps.setErrorThreshold (1e-3)
ps.setMaxIterProjection (40)
ps.selectPathValidation ('Dichotomy', 0)
# ps.selectPathProjector ('Progressive', 0.2)
# ps.selectPathProjector ('Global', 0.2)
# Create constraints. {{{3
# Create passive DOF lists {{{4
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['baxterRightSide'] = list ()
jointNames['baxterLeftSide']  = list ()
for n in jointNames['all']:
  if n.startswith ("baxter"):
    if n.startswith ("baxter/left_"):
      jointNames['baxterLeftSide'].append (n)
    if n.startswith ("baxter/right_"):
      jointNames['baxterRightSide'].append (n)
# 4}}}
# Locks joints that are not used for this problem {{{4
lockFingers = ["r_gripper_l_finger",
               "r_gripper_r_finger",
               "l_gripper_l_finger",
               "l_gripper_r_finger",
        ]
for side in ["r", "l", ]:
  ps.createLockedJoint(side + "_gripper_l_finger", "baxter/" + side + "_gripper_l_finger_joint", [ 0.02,])
  ps.createLockedJoint(side + "_gripper_r_finger", "baxter/" + side + "_gripper_r_finger_joint", [-0.02,])
lockHead = ['head_pan',]
ps.createLockedJoint ('head_pan', 'baxter/head_pan',
    [q_init[robot.rankInConfiguration['baxter/head_pan']]])
for n in jointNames["baxterRightSide"]:
  ps.createLockedJoint (n, n, [0,])
for n in jointNames["baxterLeftSide"]:
  ps.createLockedJoint (n, n, [0,])
lockAll = lockFingers + lockHead
# 4}}}
# 3}}}
handlesPerObject = list ()
handles = list ()
objContactSurfaces  = list ()
for i in range(K):
  handlesPerObject.append ([boxes[i] + "/handle2"])
  handles.append (boxes[i] + "/handle2")
  objContactSurfaces .append ([boxes[i] + "/box_surface"])
# Build rules
rules = [Rule ([".*"], [".*"], True)]
# Get the built graph
cg = ConstraintGraph (robot, 'graph')
factory = ConstraintGraphFactory (cg)
factory.setGrippers (grippers)
factory.environmentContacts (['table/pancake_table_table_top'])
factory.setObjects (boxes, handlesPerObject, objContactSurfaces)
factory.setRules (rules)
factory.generate ()
cg.addConstraints (graph = True, constraints =\
                   Constraints (numConstraints = lockAll))
cg.initialize ()
res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_init)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')
q_init_proj = res [1]
res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_goal)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q_goal_proj = res [1]
ps.setInitialConfig (q_init_proj)
ps.addGoalConfig (q_goal_proj)
import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
success = 0
# Remove joint bound validation
ps.hppcorba.problem.clearConfigValidations()
ps.addConfigValidation("CollisionValidation")
# Set parameters for States Path Finder
ps.selectPathPlanner("StatesPathFinder")
ps.setParameter("StatesPathFinder/innerPlannerTimeOut", 0.0)
ps.setParameter("StatesPathFinder/innerPlannerMaxIterations", 100)
ps.setParameter("StatesPathFinder/nTriesUntilBacktrack", 3)

optimizers = ['Graph-PartialShortcut', 'Graph-RandomShortcut', 'PartialShortcut', 'RandomShortcut', 'SimpleShortcut']
iOpt = 0

for i in range (args.N):
  ps.clearPathOptimizers()
  ps.addPathOptimizer(optimizers[iOpt])
  iOpt += 1
  if iOpt == len(optimizers): iOpt=0
  ps.clearRoadmap ()
  ps.resetGoalConfigs ()
  ps.setInitialConfig (q_init_proj)
  ps.addGoalConfig (q_goal_proj)
  try:
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
  except Exception as e:
    print (f"Failed to plan path: {e}")
  else:
    success += 1
    totalTime += t2 - t1
    print (t2-t1)
    n = ps.numberNodes ()
    totalNumberNodes += n
    print ("Number nodes: " + str(n))
if args.N != 0:
  print ("#" * 20)
  print (f"Number of rounds: {args.N}")
  print (f"Number of successes: {success}")
  print (f"Success rate: {success/ args.N * 100}%")
  if success > 0:
    print (f"Average time per success: {totalTime.total_seconds()/success}")
    print (f"Average number nodes per success: {totalNumberNodes/success}")


if args.display:
  v = vf.createViewer ()
  pp = PathPlayer (v)
  if args.run:
    pp(0)
