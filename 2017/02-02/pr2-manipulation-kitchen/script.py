# Import libraries and load robots. {{{1

# Import. {{{2
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory
from math import pi
# 2}}}

# Load PR2 and a box to be manipulated. {{{2
class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""

class Environment (object):
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'kitchen_area'
  urdfSuffix = ""
  srdfSuffix = ""

robot = Robot ('pr2-box', 'pr2')
ps = ProblemSolver (robot)
## ViewerFactory is a class that generates Viewer on the go. It means you can
## restart the server and regenerate a new windows.
## To generate a window:
## vf.createRealClient ()
vf = ViewerFactory (ps)

vf.loadObjectModel (Box, 'box')
vf.loadEnvironmentModel (Environment, "kitchen_area")

robot.setJointBounds ("pr2/root_joint" , [-5,-2,-5.2,-2.7,-2,2,-2,2] )
robot.setJointBounds ("box/root_joint", [-5.1,-2,-5.2,-2.7,0,1.5,-2,2,-2,2,-2,2,-2,2])
# 2}}}

# 2}}}

# 1}}}

# Initialization. {{{1

# Set parameters. {{{2
robot.client.basic.problem.resetRoadmap ()
ps.setErrorThreshold (1e-3)
ps.setMaxIterProjection (40)
# 2}}}

# Create lists of joint names - useful to define passive joints. {{{2
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['pr2'] = list ()
jointNames['allButPR2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("pr2"):
    jointNames['pr2'].append (n)
  if not n.startswith ("pr2/l_gripper"):
    jointNames['allButPR2LeftArm'].append (n)

ps.addPassiveDofs ('pr2', jointNames['pr2'])
# 2}}}

# Generate initial and goal configuration. {{{2
q_init = robot.getCurrentConfig ()
rank = robot.rankInConfiguration ['pr2/l_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_r_finger_joint']
q_init [rank] = 0.5
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['pr2/torso_lift_joint']
q_init [rank] = 0.2
rank = robot.rankInConfiguration ['box/root_joint']
q_init [rank:rank+7] = [-2.5, -4, 0.8,0,0,0,1]

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['pr2/l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['pr2/r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/r_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['box/root_joint']
q_goal [rank:rank+7] = [-4.8, -4.8, 0.9, 0, 0, 1, 0]
#rank = robot.rankInConfiguration ['box/base_joint_SO3']
#q_goal [rank:rank+4] = [0, 0, 0, 1]
# 2}}}

# Build rules
rules = [Rule (['pr2/l_gripper'], ['box/handle'], True),
         Rule ([""], [""], True)]

locklhand = ['l_l_finger','l_r_finger'];
ps.createLockedJoint ('l_l_finger', 'pr2/l_gripper_l_finger_joint', [0.5,])
ps.createLockedJoint ('l_r_finger', 'pr2/l_gripper_r_finger_joint', [0.5,])

grippers = ['pr2/l_gripper']
boxes = ['box']
handlesPerObject = [['box/handle']]
# envSurfaces = ['kitchen_area/pancake_table_table_top',
#                'kitchen_area/white_counter_top_sink']
# contactPerObject = [['box/box_surface']]
envSurfaces = []
contactPerObject = [[]]

ps.client.manipulation.graph.autoBuild ('graph',
        grippers, boxes, handlesPerObject, contactPerObject, envSurfaces, rules)

cg = ConstraintGraph (robot, 'graph', makeGraph = False)
cg.setConstraints (graph = True, lockDof = locklhand)
cg.setWeight ('pr2/l_gripper < box/handle | 0-0_ls', 0)

res, q_init, err = cg.applyNodeConstraints ('free', q_init)
if not res:
  raise RuntimeError ("Failed to project initial configuration")
res, q_goal, err = cg.applyNodeConstraints ('free', q_goal)
if not res:
  raise RuntimeError ("Failed to project initial configuration")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.setMaxIterPathPlanning (5000)

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
N = 20; success = 0
for i in range (N):
    ps.clearRoadmap ()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)
    try:
      t1 = dt.datetime.now ()
      ps.solve ()
      success += 1
      t2 = dt.datetime.now ()
      totalTime += t2 - t1
      print (t2-t1)
      n = ps.numberNodes ()
      totalNumberNodes += n
      print ("Number nodes: " + str(n))
    except:
      print ("Failed to plan path.")

print ("Number of successes: " + str (success))
print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (success)))
print ("Average number nodes: " + str (totalNumberNodes/float (success)))


# 1}}}

from hpp.gepetto import PathPlayer
v = vf.createViewer (); v (q_init)
pp = PathPlayer (robot.client.basic, v)

# vim: foldmethod=marker foldlevel=1
