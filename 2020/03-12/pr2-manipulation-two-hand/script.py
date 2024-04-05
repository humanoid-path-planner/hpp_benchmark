#!/usr/bin/env python
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#
# vim: foldmethod=marker foldlevel=2
from argparse import ArgumentParser
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, Constraints, \
  ConstraintGraph, ConstraintGraphFactory, Rule, Client
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer
from math import sqrt
from hpp.corbaserver import loadServerPlugin

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

loadServerPlugin ("corbaserver", "manipulation-corba.so")
Client ().problem.resetProblem ()

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
grippers = ['pr2/l_gripper', 'pr2/r_gripper']
boxes = ['box']
handlesPerObject = [['box/handle', 'box/handle2'],]
objContactSurfaces = [['box/box_surface'],]
envSurfaces = ['kitchen_area/pancake_table_table_top',]

# Build rules
rules = [Rule ([""], [""], True)]

cg = ConstraintGraph (robot, 'graph')
factory = ConstraintGraphFactory (cg)
factory.setGrippers (grippers)
factory.environmentContacts (envSurfaces)
factory.setObjects (boxes, handlesPerObject, objContactSurfaces)
factory.setRules (rules)
factory.generate ()

cg.addConstraints (graph = True, constraints =\
                   Constraints (numConstraints = lockAll))
cg.initialize ()

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
for i in range (args.N):
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

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (args.N)))
print ("Average number nodes: " + str (totalNumberNodes/float (args.N)))

if args.display:
    v = vf.createViewer ()
    pp = PathPlayer (v, robot.client.basic)
    if args.run:
        pp(0)
