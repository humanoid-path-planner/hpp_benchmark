# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.baxter import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto.manipulation import ViewerFactory
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

#robot.setRootJointPosition ("baxter" , [-3.2,-3.9, 0.926, 1, 0, 0, 0])
robot.setRootJointPosition ("baxter" , [-0.8,0.8, 0.926, 1, 0, 0, 0])
vf.loadEnvironmentModel (Table, "table")
vf.loadObjectModel (Box, "box1")
vf.loadObjectModel (Box, "box2")

robot.setJointBounds ('box1/base_joint_xyz', [-1,0.5,-1,2,0.6,1.9])
robot.setJointBounds ('box2/base_joint_xyz', [-1,0.5,-1,2,0.6,1.9])
# 3}}}

# Define configurations. {{{3
q_init = robot.getCurrentConfig ()
rankB1 = robot.rankInConfiguration ['box1/base_joint_xyz']
rankB2 = robot.rankInConfiguration ['box2/base_joint_xyz']

c = sqrt (2) / 2
rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q_init [rank:rank+3] = [-0.3, 0.8, 0.76]
rank = robot.rankInConfiguration ['box1/base_joint_SO3']
q_init [rank:rank+4] = [c, 0, -c, 0]
rank = robot.rankInConfiguration ['box2/base_joint_xyz']
q_init [rank:rank+3] = [-0.4, 0.8, 0.76]
rank = robot.rankInConfiguration ['box2/base_joint_SO3']
q_init [rank:rank+4] = [c, 0, -c, 0]

q_goal=q_init[:]
b1 = q_goal[rankB1:rankB1+7]
q_goal[rankB1:rankB1+7] = q_goal[rankB2:rankB2+7]
q_goal[rankB2:rankB2+7] = b1

# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (20)
ps.selectPathValidation ('Discretized', 0.05)
ps.selectPathProjector ('Progressive', 0.1)

# Create constraints. {{{3

# Create passive DOF lists {{{4
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['baxter'] = list ()
jointNames['baxterRightSide'] = list ()
jointNames['baxterLeftSide']  = list ()
jointNames['box1'] = list ()
jointNames['box2'] = list ()
for n in jointNames['all']:
  if n.startswith ("baxter"):
    jointNames['baxter'].append (n)
    if n.startswith ("baxter/left_"):
      jointNames['baxterLeftSide'].append (n)
    if n.startswith ("baxter/right_"):
      jointNames['baxterRightSide'].append (n)
  elif n.startswith ("box1"):
      jointNames["box1"].append (n)
  elif n.startswith ("box2"):
      jointNames["box2"].append (n)

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

# Create gaze constraints {{{4
ps.createPositionConstraint ("gaze1", "baxter/display_joint", "box1/base_joint_xyz", [0,0,0], [0,0,0], [1,0,0])
ps.createPositionConstraint ("gaze2", "baxter/display_joint", "box2/base_joint_xyz", [0,0,0], [0,0,0], [1,0,0])
# 4}}}

# 3}}}

# Build the constraint graph
cg = ConstraintGraph.buildGenericGraph (robot, "graph",
        [ "baxter/r_gripper" ,  "baxter/l_gripper"],
        [ "box1"             ,  "box2"],
        [["box1/handle2",]   , ["box2/handle2"]],
        [["box1/box_surface"], ["box2/box_surface"]],
        ['table/pancake_table_table_top'],
        [])

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

from hpp.corbaserver import Benchmark
import random, sys
b = Benchmark (robot.client.basic, robot, ps)
b.seedRange = random.sample (xrange (100000), 20)
# b.iterPerCase = 3

b.tryResumeAndDelete ()

try:
    results = b.do()
except:
    sys.exit(1)

b.writeResume (filename = "results.pickle")
