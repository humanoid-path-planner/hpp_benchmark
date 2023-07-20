#!/usr/bin/env python
#
#  Copyright (2017) CNRS
#
#  Author: Florent Lamiraux
#
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#

import datetime as dt
import re, os
from argparse import ArgumentParser
from math import pi, fabs
import hpp
from hpp.corbaserver.manipulation import ConstraintGraphFactory, Rule, \
  SecurityMargins
from setup import ConstraintGraph, Constraints, grippers, handlesPerObjects, nCylinder, nSphere, objects, ps, robot, shapesPerObject, vf
from hpp.gepetto import PathPlayer
from state_name import StateName
from visibility_prm import VisibilityPRM
import time, sys
from math import sqrt

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

# Remove joint bound validation
ps.hppcorba.problem.clearConfigValidations()
ps.addConfigValidation("CollisionValidation")

def cleanPaths (ps, solutions) :
  offset = 0
  for i, s in enumerate (solutions):
    for j in range (s - offset):
      ps.erasePath (i)
      offset += 1
    offset += 1
## Write log in a file
#
devel_dir = os.getenv ('DEVEL_DIR')
def makeRule (grasps):
  '''
  Build a rule that will generate a state where each specified gripper will
  grasp the specify handle.
  This rule is used in the construction of the constraint graph to generate the
  corresponding state
  '''
  _grippers = list ()
  _handles = list ()
  for (g,h) in grasps:
    _grippers.append (g)
    _handles.append (h)
  for g in grippers:
    if not g in _grippers:
      _grippers.append (g)
  _handles += (len (_grippers) - len (_handles)) * ['^$']
  return Rule (grippers = _grippers, handles = _handles, link = True)

# infinite norm between vectors
dC = lambda q1,q2: reduce (lambda x,y : x if fabs (y [0]- y [1]) < x \
                           else fabs (y [0]- y [1]), zip (q1, q2), 0)
if args.display:
  v = vf.createViewer ()
  pp = PathPlayer (v)
else:
  v = lambda x: None
## Initial configuration of manipulator arms
q0_r0 = [pi/6, -pi/2, pi/2, 0, 0, 0,]
q0_r1 = q0_r0 [::]
## Generate initial configurations of spheres
q0_spheres = list ()
i = 0
y = 0.04
while i < nSphere:
  q0_spheres.append ([- .1*(i/2), -.12 + y, 0.025, 0, 0, 0, 1])
  i+=1; y = -y
## Generate initial configurations of cylinders
q0_cylinders = list ()
i = 0
y = -0.04
while i < nCylinder:
  q0_cylinders.append ([0.45 + .1*(i/2), -.12 + y, 0.025, 0, 0, 0, 1])
  i+=1; y = -y
q0 = q0_r0 + q0_r1 + sum (q0_spheres, []) + sum (q0_cylinders, [])
if args.display:
  v (q0)
# List of nodes composing the assembly sequence
nodes = list ()
# List of rules that define all the nodes
rules = list ()
# List of grasps for each node. From any node to the next one, one grasp is
# added or removed
grasps = set ()
# list of nodes that are explored to cross each edge
exploreNodes = list ()
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
# grasp sphere0
grasps.add (('r0/gripper', 'sphere0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])
# grasp cylinder0
grasps.add (('r1/gripper', 'cylinder0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])
# assemble cylinder0 and sphere0
grasps.add (('cylinder0/magnet0', 'sphere0/magnet'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])
# release sphere0
grasps.remove (('r0/gripper', 'sphere0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-1])
# grasp sphere1
grasps.add (('r0/gripper', 'sphere1/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])
# assemble sphere1
grasps.add (('cylinder0/magnet1', 'sphere1/magnet'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])
# release sphere1
grasps.remove (('r0/gripper', 'sphere1/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])
# release cylinder0 : put assembly on the ground
grasps.remove (('r1/gripper', 'cylinder0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

ps.selectPathValidation ('Progressive', 0.02)

cg = ConstraintGraph (robot, 'assembly')
factory = ConstraintGraphFactory (cg)
factory.setGrippers (grippers)
#factory.environmentContacts (['table/pancake_table_table_top'])
factory.setObjects (objects, handlesPerObjects, shapesPerObject)
factory.setRules (rules)
factory.generate ()
sm = SecurityMargins(ps, factory, ["r0", "r1", "sphere0", "sphere1",
                                   "cylinder0", "cylinder1"])
sm.defaultMargin = 0.02
sm.apply()
cg.initialize ()

# Create a goal configuration with the construction set assembled.

ps.selectPathPlanner('M-RRT')
ps.setParameter("StatesPathFinder/innerPlannerTimeOut", 0.0)
ps.setParameter("StatesPathFinder/innerPlannerMaxIterations", 100)
ps.setParameter("StatesPathFinder/nTriesUntilBacktrack", 3)
ps.clearConfigValidations()

cg.initialize()

c = sqrt(2)/2
ps.setMaxIterPathPlanning(5000)

totalTime = dt.timedelta (0)
totalNumberNodes = 0
success = 0
for i in range (args.N):
    ps.clearRoadmap ()
    ps.setInitialConfig (q0)
    ps.setGoalConstraints(['cylinder0/magnet0 grasps sphere0/magnet',
                           'cylinder0/magnet1 grasps sphere1/magnet',
                           'place_cylinder0'])
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
