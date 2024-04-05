# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.baxter import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import Color
from math import sqrt

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
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'table'
  urdfSuffix = ""
  srdfSuffix = ""

class Box (object):
  rootJointType = "freeflyer"
  packageName = 'hpp-baxter'
  meshPackageName = 'hpp-baxter'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "base_joint"
  handle = "handle"
# 4}}}

Robot.urdfSuffix = ""
robot = Robot ('baxter-manip', 'baxter')
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

#robot.setRootJointPosition ("baxter" , [-3.2,-3.9, 0.926, 1, 0, 0, 0])
robot.setRootJointPosition ("baxter" , [-0.8,0.8, 0.926, 1, 0, 0, 0])
vf.loadEnvironmentModel (Table, "table")
boxes = list()
for i in xrange(K):
  boxes.append ("box" + str(i))
  vf.loadObjectModel (Box, boxes[i])
  robot.setJointBounds (boxes[i]+ '/base_joint_xyz', [-1,0.5,-1,2,0.6,1.9])

def setBoxColors (gui):
  c = Color()
  for i in xrange(K):
    gui.setColor (boxes[i], c[i])
# 3}}}

# Define configurations. {{{3
q_init = robot.getCurrentConfig ()
rankB = list()
for i in xrange(K):
  rankB.append (robot.rankInConfiguration [boxes[i] + '/base_joint_xyz'])

bb = [-0.3, -0.4, 0.7, 0.9]
c = sqrt (2) / 2
xstep = (bb[1] - bb[0]) / (nBoxPerLine - 1) if nBoxPerLine > 1 else (bb[1] - bb[0])
nbCols = int(K * 1. / nBoxPerLine + 0.5)
ystep = (bb[3] - bb[2]) / (nbCols - 1) if nbCols > 1 else (bb[3] - bb[2])
for i in xrange(K):
  iL = i % nBoxPerLine
  iC = (i - iL) / nBoxPerLine
  x = bb[0] + xstep * iL
  y = bb[2] + xstep * iC
  q_init [rankB[i]:rankB[i]+7] = [x, y, 0.746, c, 0, -c, 0]

q_goal = q_init [::]
for i in xrange(K):
  r  = rankB[i]
  rn = rankB[goal[i]]
  q_goal[r:r+7] = q_init[rn:rn+7]
# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (20)
ps.selectPathValidation ('Discretized', 0.05)
ps.selectPathProjector ('Progressive', 0.2)
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

handles = list ()
shapes  = list ()
for i in xrange(K):
  handles.append ([boxes[i] + "/handle2"])
  shapes .append ([boxes[i] + "/box_surface"])

if K is 2:
    rules = [ Rule(grippers[0], handles[1][0], False),
              Rule(grippers[1], handles[0][0], False), ]
else:
    rules = []

# Build the constraint graph
cg = ConstraintGraph.buildGenericGraph(robot, "graph",
        grippers, boxes, handles, shapes,
        ['table/pancake_table_table_top'],
        rules)
cg.setConstraints (graph = True, lockDof = lockAll)

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
N = 20
for i in range (N):
    ps.clearRoadmap ()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q_init_proj)
    ps.addGoalConfig (q_goal_proj)
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
    totalTime += t2 - t1
    print (t2-t1)
    n = ps.numberNodes ()
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (N)))
print ("Average number nodes: " + str (totalNumberNodes/float(N)))

#r = vf.createViewer ()
#pp = PathPlayer (robot.client.basic, r)

