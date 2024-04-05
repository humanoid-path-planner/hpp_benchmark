# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.baxter import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto import PathPlayer
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

q0 = q_init [::]
rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q_init [rank:rank+3] = [-0.3, 1.3, 0.76]
rank = robot.rankInConfiguration ['box1/base_joint_SO3']
c = sqrt (2) / 2
q_init [rank:rank+4] = [c, 0, c, 0]
rank = robot.rankInConfiguration ['box2/base_joint_xyz']
q_init [rank:rank+3] = [-0.3, 1.3, 0.76]
rank = robot.rankInConfiguration ['box2/base_joint_SO3']
c = sqrt (2) / 2
q_init [rank:rank+4] = [c, 0, c, 0]

rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q0 [rank:rank+3] = [-0.3, 0.5, 0.76]
rank = robot.rankInConfiguration ['box1/base_joint_SO3']
q0 [rank:rank+4] = [c, 0, -c, 0]
rank = robot.rankInConfiguration ['box2/base_joint_xyz']
q0 [rank:rank+3] = [-0.4, 0.5, 0.76]
rank = robot.rankInConfiguration ['box2/base_joint_SO3']
q0 [rank:rank+4] = [c, 0, -c, 0]

rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q1 = q_init[:]
q1[rank:rank+7] = [ -0.14391940018238783, 1.092383723903555, 0.7460100959532432, 0.44542854851747, 0.5489390643072047, 0.4452361192941091, -0.5495672023684386]
q2 = q0[:]
q2[rank:rank+3] = [ -0.545, 0.268, 0.7460100959532432]
rank = robot.rankInConfiguration ['box2/base_joint_xyz']
q1[rank:rank+7] = [ -0.14391940018238783, 1.092383723903555, 0.7460100959532432, 0.44542854851747, 0.5489390643072047, 0.4452361192941091, -0.5495672023684386]
q2[rank:rank+3] = [ -0.445, 0.268, 0.7460100959532432]

# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (20)
#ps.selectPathValidation ('Graph-Discretized', 0.05)
ps.selectPathProjector ('Progressive', 0.1)
# ps.selectPathProjector ('Global', 0.2)

# Create constraints. {{{3
from hpp.corbaserver.manipulation import ConstraintGraph
cg = ConstraintGraph (robot, 'graph')

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

ps.addPassiveDofs ('box1', jointNames ['box1'])
ps.addPassiveDofs ('box2', jointNames ['box2'])
ps.addPassiveDofs ('baxter', jointNames ['baxter'])
# 4}}}

# Create the grasps {{{4
cg.createGrasp ('grasp1', 'baxter/r_gripper', 'box1/handle2')
cg.createPreGrasp ('pregrasp1', 'baxter/r_gripper', 'box1/handle2')

cg.createGrasp ('grasp2', 'baxter/r_gripper', 'box2/handle2')
cg.createPreGrasp ('pregrasp2', 'baxter/r_gripper', 'box2/handle2')
# 4}}}

# Create placement constraints {{{4
robot.client.manipulation.problem.createPlacementConstraint (
  'box1_on_table', ['box1/box_surface',], ['table/pancake_table_table_top',])
robot.client.manipulation.problem.createPrePlacementConstraint (
  'box1_above_table', ['box1/box_surface',],
  ['table/pancake_table_table_top',], 0.05)
robot.client.manipulation.problem.createPlacementConstraint (
  'box2_on_table', ['box2/box_surface',], ['table/pancake_table_table_top',])
robot.client.manipulation.problem.createPrePlacementConstraint (
  'box2_above_table', ['box2/box_surface',],
  ['table/pancake_table_table_top',], 0.05)
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

lockHeadFingersAndLeftSide = lockFingers + lockHead + jointNames["baxterLeftSide"]
# 4}}}

# Create constraints corresponding to each object {{{4
lockBox1 = ps.lockFreeFlyerJoint ("box1/" + Box.joint, 'box1_lock')
lockBox2 = ps.lockFreeFlyerJoint ("box2/" + Box.joint, 'box2_lock')
# 4}}}

# Create gaze constraints {{{4
ps.createPositionConstraint ("gaze1", "baxter/display_joint", "box1/base_joint_xyz", [0,0,0], [0,0,0], [1,0,0])
ps.createPositionConstraint ("gaze2", "baxter/display_joint", "box2/base_joint_xyz", [0,0,0], [0,0,0], [1,0,0])
# 4}}}

# 3}}}

# Create the graph. {{{3

cg.createNode (['grasp2', 'grasp1', 'free'])

cg.setConstraints (node='free', numConstraints=['box1_on_table','box2_on_table'])
cg.setConstraints (node='grasp1', grasps = ['grasp1',])
cg.setConstraints (node='grasp2', grasps = ['grasp2',])

def genGrasp (iBox, lockBox, otherLockBox):
    iStr = str(iBox)
    grasp = 'grasp' + iStr
    edgeBaseName = grasp+'_edge'
    placement = 'box' + iStr + "_on_table"
    box_above_table = 'box' + iStr + "_above_table"

    # Free to right
    cg.createWaypointEdge ('free', grasp, edgeBaseName, nb=3, weight=1,
                           isInNode = 'free', automaticBuilder=True)

    cg.setConstraints (edge = edgeBaseName + '_e0',
                       lockDof = lockBox + otherLockBox)
    cg.graph.setContainingNode (cg.edges[edgeBaseName + "_e0"],
                                cg.nodes["free"])

    cg.setConstraints (node = edgeBaseName + '_n0',
                       pregrasps = ['pre' + grasp,],
                       numConstraints = [placement,])

    cg.setConstraints (edge = edgeBaseName + '_e1',
                       lockDof = lockBox + otherLockBox)

    cg.graph.setContainingNode (cg.edges[edgeBaseName + "_e1"],
                                cg.nodes["free"])

    cg.graph.setShort (cg.edges[edgeBaseName + "_e1"], True)

    cg.setConstraints (node=edgeBaseName + "_n1", grasps = [grasp],
                       numConstraints = [placement,])

    cg.setConstraints (edge=edgeBaseName + '_e2', lockDof = otherLockBox)
    cg.graph.setContainingNode (cg.edges[edgeBaseName + "_e2"], cg.nodes[grasp])
    cg.graph.setShort (cg.edges[edgeBaseName + "_e2"], True)

    cg.setConstraints (node=edgeBaseName + '_n2', grasps = [grasp],
                       numConstraints = [box_above_table,])

    cg.setConstraints (edge=edgeBaseName + '_e3', lockDof = otherLockBox)
    cg.graph.setContainingNode (cg.edges[edgeBaseName + "_e3"], cg.nodes[grasp])
    cg.graph.setShort (cg.edges[edgeBaseName + "_e3"], True)

    # Right to free
    ungraspEdge = 'ungrasp' + iStr
    cg.createWaypointEdge (grasp, 'free', ungraspEdge + '_e3', nb=3, weight=10,
                           isInNode = 'free', automaticBuilder=False)

    # Reuse previous waypoints
    cg.createEdge (grasp, edgeBaseName + '_n2', ungraspEdge + '_e0', -1, grasp)
    cg.createEdge (edgeBaseName + '_n2', edgeBaseName + '_n1',
                   ungraspEdge + '_e1', -1, grasp)
    cg.createLevelSetEdge (edgeBaseName + '_n2', edgeBaseName + '_n1',
                           ungraspEdge + '_ls_e1', -1)
    cg.createEdge (edgeBaseName + '_n1', edgeBaseName + '_n0',
                   ungraspEdge + '_e2', -1, 'free')
    cg.graph.setWaypoint (cg.edges[ungraspEdge + "_e3"], 0,
                          cg.edges[ungraspEdge + "_e0"],
                          cg.nodes[edgeBaseName + "_n2"])
    # cg.graph.setWaypoint (cg.edges[ungraspEdge + "_e3"], 1, cg.edges[ungraspEdge + "_e1"], cg.nodes[edgeBaseName + "_n1"])
    cg.graph.setWaypoint (cg.edges[ungraspEdge + "_e3"], 1,
                          cg.edges[ungraspEdge + "_ls_e1"],
                          cg.nodes[edgeBaseName + "_n1"])

    cg.graph.setWaypoint (cg.edges[ungraspEdge + "_e3"], 2,
                          cg.edges[ungraspEdge + "_e2"],
                          cg.nodes[edgeBaseName + "_n0"])

    cg.setConstraints (edge = ungraspEdge + '_e3',
                       lockDof = lockBox + otherLockBox)

    cg.setConstraints (edge=ungraspEdge + '_e2',
                       lockDof = lockBox + otherLockBox)

    cg.setContainingNode (ungraspEdge + "_e2", "free")
    cg.setShort (ungraspEdge + "_e2", True)

    cg.setConstraints (edge=ungraspEdge + '_e1', lockDof = otherLockBox)
    cg.setContainingNode (ungraspEdge + "_e1", grasp)
    cg.setShort (ungraspEdge + "_e1", True)

    cg.setConstraints (edge=ungraspEdge + '_ls_e1', lockDof = otherLockBox)
    cg.setContainingNode (ungraspEdge + "_ls_e1", grasp)
    cg.setShort (ungraspEdge + "_ls_e1", True)
    cg.setLevelSetFoliation (edge=ungraspEdge + '_ls_e1', condNC=[placement,],
                             paramLJ = lockBox)

    cg.setConstraints (edge=ungraspEdge + '_e0', lockDof = otherLockBox)
    cg.setContainingNode (ungraspEdge + "_e0", grasp)

genGrasp (1, lockBox1, lockBox2)
genGrasp (2, lockBox2, lockBox1)

cg.createEdge ('free', 'free', 'transit', 0, 'free')
cg.setConstraints (edge='transit', lockDof=lockBox1+lockBox2)
cg.createEdge ('grasp1', 'grasp1', 'transfer1', 0, 'grasp1')
cg.setConstraints (edge='transfer1', lockDof=lockBox2)
cg.createEdge ('grasp2', 'grasp2', 'transfer2', 0, 'grasp2')
cg.setConstraints (edge='transfer2', lockDof=lockBox1)

cg.setConstraints (graph = True, lockDof = lockHeadFingersAndLeftSide)
# 3}}}

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q1)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q1_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q0)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q0_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q2)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q2_proj = res [1]

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
for i in range (20):
    ps.clearRoadmap ()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q0_proj)
    ps.addGoalConfig (q2_proj)
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
    totalTime += t2 - t1
    print (t2-t1)
    n = ps.numberNodes ()
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/20.))
print ("Average number nodes: " + str (totalNumberNodes/20.))

# r = vf.createViewer()
# pp = PathPlayer (robot.client.basic, r)
