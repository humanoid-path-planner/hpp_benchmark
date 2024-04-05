# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui
from math import sqrt

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

# Load robot and object. {{{3
robot = Robot ('pr2-box', 'pr2') # was anchor joint
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

vf.loadObjectModel (Box, 'box')
vf.loadEnvironmentModel (Environment, "kitchen_area")
robot.setJointBounds ("pr2/root_joint" , [-5,-2,-5.2,-2.7] )
robot.setJointBounds ("box/root_joint", [-5.1,-2,-5.2,-2.7,0,1.5])
# 3}}}

# Define configurations. {{{3
q_init = robot.getCurrentConfig ()
q_init[0:4] = [-3.2,-4,1,0] # FIX ME ! (see up )
rank = robot.rankInConfiguration ['pr2/r_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/r_gripper_r_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_r_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/torso_lift_joint']
q_init [rank] = 0.2
q_goal = q_init [::]

rank = robot.rankInConfiguration ['box/root_joint']
c = sqrt (2) / 2
#q_init [rank:rank+4] = [c, 0, c, 0]
q_init [rank:rank+7] = [-2.5, -3.6, 0.76,0,c,0,c]
#rank = robot.rankInConfiguration ['box/base_joint_SO3']


rank = robot.rankInConfiguration ['box/root_joint']
q_goal [rank:rank+7] = [-2.5, -4.4, 0.76,0,-c,0,c]
#rank = robot.rankInConfiguration ['box/base_joint_SO3']
#q_goal [rank:rank+4] = [c, 0, -c, 0]
del c

# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterProjection (40)
ps.selectPathProjector ('Progressive', 0.2)

# Create constraints. {{{3
from hpp.corbaserver.manipulation import ConstraintGraph
cg = ConstraintGraph (robot, 'graph')

robot.client.manipulation.problem.createPlacementConstraint \
    ('box_placement', ['box/box_surface',],
     ['kitchen_area/pancake_table_table_top',])

jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['pr2'] = list ()
jointNames['allButPR2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("pr2"):
    jointNames['pr2'].append (n)
  if not n.startswith ("pr2/l_gripper"):
    jointNames['allButPR2LeftArm'].append (n)

ps.addPassiveDofs ('pr2', jointNames ['pr2'])

cg.createGrasp ('l_grasp', 'pr2/l_gripper', 'box/handle')

cg.createGrasp ('r_grasp', 'pr2/r_gripper', 'box/handle2')

cg.createPreGrasp ('l_pregrasp', 'pr2/l_gripper', 'box/handle')
cg.createPreGrasp ('r_pregrasp', 'pr2/r_gripper', 'box/handle2')

lockbox = ps.lockFreeFlyerJoint ('box/root_joint', 'box_lock')

locklhand = ['l_l_finger','l_r_finger'];
ps.createLockedJoint ('l_l_finger', 'pr2/l_gripper_l_finger_joint', [0.5])
ps.createLockedJoint ('l_r_finger', 'pr2/l_gripper_r_finger_joint', [0.5])
lockPlanar = ps.lockPlanarJoint ("pr2/root_joint", "lockplanar", [-3.2,-4,1,0])

lockrhand = ['r_l_finger','r_r_finger'];
ps.createLockedJoint ('r_l_finger', 'pr2/r_gripper_l_finger_joint', [0.5])
ps.createLockedJoint ('r_r_finger', 'pr2/r_gripper_r_finger_joint', [0.5])

lockhands = lockrhand + locklhand

lockHeadAndTorso = ['head_pan', 'head_tilt', 'torso', 'laser'];
ps.createLockedJoint ('head_pan', 'pr2/head_pan_joint',
    [q_init[robot.rankInConfiguration['pr2/head_pan_joint']]])
ps.createLockedJoint ('head_tilt', 'pr2/head_tilt_joint',
    [q_init[robot.rankInConfiguration['pr2/head_tilt_joint']]])
ps.createLockedJoint ('torso', 'pr2/torso_lift_joint',
    [q_init[robot.rankInConfiguration['pr2/torso_lift_joint']]])
ps.createLockedJoint ('laser', 'pr2/laser_tilt_mount_joint',
    [q_init[robot.rankInConfiguration['pr2/laser_tilt_mount_joint']]])

lockAll = lockhands + lockHeadAndTorso + lockPlanar

# 3}}}

# Create the graph. {{{3

_ = dict ()
# Create a dictionnary to translate human readable names into LaTeX expressions.
# This goes well with option -tmath of dot2tex command line.
# {{{4
_["both"]="2 grasps"
_["left"]="Left grasp"
_["right"]="Right grasp"
_["free"]="No grasp"
_["move_both"] = "move both"
# _["r_grasp"]=""
# _["l_grasp"]=""
# _["b_r_grasp"]=""
# _["b_l_grasp"]=""
# _["b_r_ungrasp"]=""
# _["b_l_ungrasp"]=""
_["move_free"]="move_free"
_["r_keep_grasp"]="r_keep_grasp"
_["l_keep_grasp"]="l_keep_grasp"
# 4}}}
cg.setTextToTeXTranslation (_)

cg.createNode (['both', 'right', 'left', 'free'])

cg.setConstraints (node='free', numConstraints=['box_placement'])

# Right hand {{{4
cg.setConstraints (node='right', grasps = ['r_grasp',])

cg.createWaypointEdge ('free', 'right', 'r_grasp', 1, 10, True)

cg.setConstraints (edge='r_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='r_grasp_n0', pregrasps = ['r_pregrasp',])
cg.setConstraints (edge='r_grasp_e0', lockDof = lockbox)
# 4}}}

# Left hand {{{4
cg.setConstraints (node = 'left', grasps = ['l_grasp',])

cg.createWaypointEdge ('free', 'left', 'l_grasp', 1, 10, True, True)

cg.setConstraints (edge='l_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='l_grasp_n0', pregrasps = ['l_pregrasp',])
cg.setConstraints (edge='l_grasp_e0', lockDof = lockbox)
# 4}}}

# Both hands {{{4
cg.setConstraints (node='both', grasps = ['l_grasp', 'r_grasp'])

cg.createWaypointEdge ('both', 'right', 'b_l_ungrasp', 1, 1, False, True)

cg.createWaypointEdge ('right', 'both', 'b_l_grasp', 1, 10, True, True)

cg.setConstraints (node='b_l_ungrasp_n0', pregrasps = ['l_pregrasp',])
cg.setConstraints (edge='b_l_ungrasp_e0', lockDof = lockbox)
cg.setConstraints (edge='b_l_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='b_l_grasp_n0', pregrasps = ['l_pregrasp',])

cg.createWaypointEdge ('both', 'left', 'b_r_ungrasp', 1, 1, False, True)

cg.createWaypointEdge ('left', 'both', 'b_r_grasp', 1, 10, True, True)

cg.setConstraints (node='b_r_ungrasp_n0', pregrasps = ['r_pregrasp',])
cg.setConstraints (edge='b_r_ungrasp_e0', lockDof = lockbox)
cg.setConstraints (edge='b_r_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='b_r_grasp_n0', pregrasps = ['r_pregrasp',])

# 4}}}

# Loops {{{4
cg.createEdge ('free', 'free', 'move_free', 1, True)

cg.createEdge ('left', 'left', 'l_keep_grasp', 5, True)

cg.createEdge ('right', 'right', 'r_keep_grasp', 5, True)

cg.createEdge ('both', 'both', 'move_both', 1, True)

cg.setConstraints (edge='move_free', lockDof = lockbox)
# 4}}}

cg.setConstraints (graph = True, lockDof = lockAll)
# 3}}}

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_init)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')
q_init = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_goal)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q_goal = res [1]

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
N = 20
for i in range (N):
    ps.clearRoadmap ()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
    totalTime += t2 - t1
    print (t2-t1)
    n = ps.numberNodes ()
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (N)))
print ("Average number nodes: " + str (totalNumberNodes/float (N)))

#v = vf.createViewer ()
#pp = PathPlayer (robot.client.basic, v)

