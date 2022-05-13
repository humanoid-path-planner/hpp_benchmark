#!/usr/bin/env python
#
#  Copyright (2017) CNRS
#
#  Author: Florent Lamiraux
#
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#

import re, os
from argparse import ArgumentParser
from math import pi, fabs
import hpp
from hpp.corbaserver.manipulation import ConstraintGraphFactory, Rule, \
  SecurityMargins
from setup import ConstraintGraph, Constraints, grippers, handlesPerObjects, nCylinder, nSphere, objects, ps, robot, shapesPerObject, vf
from hpp.gepetto import PathPlayer
from state_name import StateName
from math import sqrt
import typing as T
import re

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
parser.add_argument('--goalAsConstraints', action='store_true',
  help="Whether goal is defined as set of constraints. If unspecified, "
       "a goal configuration should be provided.")
parser.add_argument('--bigGraph', action='store_true',
  help="Whether constraint graph is generated with all the possible states. "
       "If unspecified, constraint graph only has a few states traversed by "
       "one safe path.")
args = parser.parse_args()

# Remove joint bound validation
ps.hppcorba.problem.clearConfigValidations()
ps.addConfigValidation("CollisionValidation")

allHandles = [handle for objHandles in handlesPerObjects for handle in objHandles]

def makeRule (grasps):
  '''
  Build a rule that will generate a state where each specified gripper will
  grasp the specified handle.
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

def forbidExcept (g: str, h: T.List[str]) -> T.List[Rule]:
  '''
  Build a list of rule that will forbid a state where a gripper matching the pattern
  grasp any handle **other than** the specified handles.
  This rule is used in the construction of the constraint graph to generate the
  corresponding state

  Args:
    g: regex pattern for grippers
    h: list of regex patterns for the only handles that apply for these grippers

  '''
  gRegex = re.compile(g)
  hRegex = [re.compile(handle) for handle in h]
  forbiddenList: T.List[Rule] = list()
  idForbidGrippers = [i for i in range(len(grippers))
      if gRegex.match(grippers[i])]
  # forbidden handles are those that don't match any of the specified patterns
  forbidHandles = [handle for handle in allHandles if not any([
      handlePattern.match(handle) for handlePattern in hRegex])]
  for id in idForbidGrippers:
    for handle in forbidHandles:
      _handles = [handle if i == id else '.*' for i in range(len(grippers))]
      forbiddenList.append(
        Rule (grippers = grippers, handles = _handles, link = False))
  return forbiddenList

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

if not args.bigGraph:
  # List of nodes composing the assembly sequence
  nodes = list ()
  # List of rules that define all the nodes
  rules = list ()
  # List of grasps for each node. From any node to the next one, one grasp is
  # added or removed
  grasps = set ()
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # grasp sphere0
  grasps.add (('r0/gripper', 'sphere0/handle'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # grasp cylinder0
  grasps.add (('r1/gripper', 'cylinder0/handle'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # assemble cylinder0 and sphere0
  grasps.add (('cylinder0/magnet0', 'sphere0/magnet'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # release sphere0
  grasps.remove (('r0/gripper', 'sphere0/handle'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # grasp sphere1
  grasps.add (('r0/gripper', 'sphere1/handle'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # assemble sphere1
  grasps.add (('cylinder0/magnet1', 'sphere1/magnet'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # release sphere1
  grasps.remove (('r0/gripper', 'sphere1/handle'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))
  # release cylinder0 : put assembly on the ground
  grasps.remove (('r1/gripper', 'cylinder0/handle'))
  nodes.append (StateName (grasps))
  rules.append (makeRule (grasps = grasps))

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
  ps.selectPathProjector ('Progressive', .05)
  ## Add a node to move robots in initial configurations
  nodes.append (nodes [-1])

else:
  # List of rules that define all the nodes
  rules = list ()

  ### Add rules to forbid grasps except for the specifed ones
  # r0/gripper can only grasp sphere handles
  rules.extend(forbidExcept('^r0/gripper$', ['^sphere\\d*/handle$']))
  # r1/gripper can only grasp cylinder handles
  rules.extend(forbidExcept('^r1/gripper$', ['^cylinder0*/handle$']))
  # magnets in cylinder0 can only grasp sphere magnets
  rules.extend(forbidExcept('^cylinder0/magnet\\d*$', ['^sphere\\d*/magnet$']))
  # cylinders other than cylinder0 are not considered in the graph
  rules.extend(forbidExcept('^cylinder[1-9][0-9]*.*$', ['^$']))

  # Accept all those that are not forbidden
  rules.append(Rule(grippers = grippers, handles=[".*"] * len(grippers), link = True))

  ps.selectPathValidation ('Progressive', 0.02)

  cg = ConstraintGraph (robot, 'assembly')
  factory = ConstraintGraphFactory (cg)
  factory.setGrippers (grippers)
  factory.setObjects (objects, handlesPerObjects, shapesPerObject)
  factory.setRules (rules)
  factory.generate ()
  sm = SecurityMargins(ps, factory, ["r0", "r1", "sphere0", "sphere1",
                                    "cylinder0", "cylinder1"])
  sm.defaultMargin = 0.02
  sm.apply()
  cg.initialize ()

# Create a goal configuration with the construction set assembled.

# Set parameters for States Path Finder
ps.selectPathPlanner("StatesPathFinder")
ps.setParameter("StatesPathFinder/innerPlannerTimeOut", 10.0)
ps.setParameter("StatesPathFinder/nTriesUntilBacktrack", 5)
#ps.selectPathValidation('NoValidation', 0)

cg.initialize()

c = sqrt(2)/2
ps.setInitialConfig(q0)

q_goal = None
goalConstraints = None
if not args.goalAsConstraints:
  ###### Use this to test the code with goal defined as a configuration
  # Note that this config is only for 2 cylinders and 2 spheres
  assert (nCylinder == 2 and nSphere == 2)
  # uncomment to test the case when initial and final state are different
  q_goal = q0_r0 + q0_r1 + [-0.06202136144745322, -0.15, 0.025, c, 0, -c, 0,
                            0.06202136144745322, -0.15, 0.025, c, 0,  c, 0,
                            0, -0.15, 0.025, 0, 0, 0, 1,
                            0.5, -0.08, 0.025, 0, 0, 0, 1]

  ## uncomment to test the case when initial and final state are the same
  # q_goal = [1.45] + q0[1:]

  ps.addGoalConfig(q_goal)

  ###### Goal defined as a configuration
else:
  ###### Use this to test the code with goal defined as a set of constraints
  # Create constraint for robot arms to go back to original configuration when done
  armConstraints: T.List[str] = []
  jointRanks: T.Dict[str, int] = robot.rankInConfiguration
  for joint in robot.getJointNames():
    if joint[:3] in ['r0/', 'r1/']:
      constraint = 'locked_' + joint
      ps.createLockedJoint(constraint, joint,
                          q0[jointRanks[joint]: jointRanks[joint] + robot.getJointConfigSize(joint)])
      armConstraints.append(constraint)

  ## Use this for the case when we need to place the finished product on table
  ## and move robot arms back to original pose
  ## there is only 1 goal state
  goalConstraints = ['cylinder0/magnet0 grasps sphere0/magnet',
                        'cylinder0/magnet1 grasps sphere1/magnet',
                        'place_cylinder0', *armConstraints]
  ## Use this for the case when we only need to assemble the product
  ## and move the robot arms back to original pose
  ## there are multiple potential goal states
  # goalConstraints = ['cylinder0/magnet0 grasps sphere0/magnet',
  #                     'cylinder0/magnet1 grasps sphere1/magnet',
  #                     *armConstraints]

  ###### Goal defined as a set of constraint

# Solve 20 times the problem
import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
success = 0
for i in range (args.N):
  # reset everything
  ps.clearRoadmap ()
  ps.resetGoalConfigs ()
  ps.resetGoalConstraints()

  ps.setInitialConfig (q0)
  # set the goal
  if not args.goalAsConstraints:
    ps.addGoalConfig (q_goal)
  else:
    ps.setGoalConstraints(goalConstraints)

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
