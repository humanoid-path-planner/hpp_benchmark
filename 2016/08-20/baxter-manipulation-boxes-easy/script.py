# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.baxter import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import Color
from math import sqrt

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

robot.setRootJointPosition ("baxter" , [-0.8,0.8, 0.926, 1, 0, 0, 0])
vf.loadEnvironmentModel (Table, "table")
boxes = list()
for i in xrange(2):
  boxes.append ("box" + str(i))
  vf.loadObjectModel (Box, boxes[i])
  robot.setJointBounds (boxes[i]+ '/base_joint_xyz', [-1,0.5,-1,2,0.6,1.9])

def setBoxColors (gui):
  c = Color()
  for i in xrange(2):
    gui.setColor (boxes[i], c[i])
# 3}}}

# Define configurations. {{{3
q_init = robot.getCurrentConfig ()

c = sqrt (2) / 2
rank = robot.rankInConfiguration ['box0/base_joint_xyz']
q_init [rank:rank+3] = [-0.3, 0.5, 0.76]
rank = robot.rankInConfiguration ['box0/base_joint_SO3']
q_init [rank:rank+4] = [c, 0, -c, 0]
rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q_init [rank:rank+3] = [-0.4, 0.5, 0.76]
rank = robot.rankInConfiguration ['box1/base_joint_SO3']
q_init [rank:rank+4] = [c, 0, -c, 0]

rank = robot.rankInConfiguration ['box0/base_joint_xyz']
q_goal = q_init[:]
q_goal[rank:rank+3] = [ -0.545, 0.268, 0.7460100959532432]
rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q_goal[rank:rank+3] = [ -0.445, 0.268, 0.7460100959532432]

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

lockAll = lockFingers + lockHead + jointNames["baxterLeftSide"]
# 4}}}

# 3}}}

# Build the constraint graph. {{{3

grippers = [ "baxter/r_gripper",]

handles = list ()
shapes  = list ()
for i in xrange(2):
  handles.append ([boxes[i] + "/handle2"])
  shapes .append ([boxes[i] + "/box_surface"])

rules = []

cg = ConstraintGraph.buildGenericGraph(robot, "graph",
        grippers, boxes, handles, shapes,
        ['table/pancake_table_table_top'],
        rules)
cg.setConstraints (graph = True, lockDof = lockAll)

# 3}}}

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
#setBoxColors(r.client.gui)
#pp = PathPlayer (robot.client.basic, r)

