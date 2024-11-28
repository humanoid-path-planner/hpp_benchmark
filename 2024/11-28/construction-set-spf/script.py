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
from setup import ConstraintGraph, Constraints, grippers, handlesPerObjects, nCylinder, nSphere, objects, ps, robot, shapesPerObject, vf, EqualToZero
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

# Build set of possible grasps
possibleGrasps = {
  'r0/gripper': ['sphere0/handle', 'sphere1/handle', 'cylinder0/handle',
                 'cylinder1/handle'],
  'r1/gripper': ['sphere0/handle', 'sphere1/handle', 'cylinder0/handle',
                 'cylinder1/handle'],
  'cylinder0/magnet0':['sphere0/magnet'],
  'cylinder0/magnet1':['sphere1/magnet']
}

ps.selectPathValidation ('Progressive', 0.02)

cg = ConstraintGraph (robot, 'assembly')
factory = ConstraintGraphFactory (cg)
factory.setGrippers (grippers)
factory.setObjects (objects, handlesPerObjects, shapesPerObject)
factory.setPossibleGrasps(possibleGrasps)
factory.generate ()
sm = SecurityMargins(ps, factory, ["r0", "r1", "sphere0", "sphere1",
                                   "cylinder0", "cylinder1"])
sm.defaultMargin = 0.02
sm.apply()
cg.initialize ()
ps.selectPathProjector ('Progressive', .05)

# Create a goal configuration with the construction set assembled.

# Set parameters for States Path Finder
ps.selectPathPlanner("StatesPathFinder")
ps.setParameter("StatesPathFinder/innerPlannerTimeOut", 0.0)
ps.setParameter("StatesPathFinder/innerPlannerMaxIterations", 100)
ps.setParameter("StatesPathFinder/nTriesUntilBacktrack", 3)
#ps.selectPathValidation('NoValidation', 0)

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
                          q0[jointRanks[joint]: jointRanks[joint] + robot.getJointConfigSize(joint)],
                          [EqualToZero] * robot.getJointConfigSize(joint)
                          )
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

  ## Use this for the case when initial state is a potential goal state
  ## Only 1 joint of robot arm moves
  # constraint1 = "changed_locked_r0/shoulder_pan_joint"
  # ps.createLockedJoint(constraint1, "r0/shoulder_pan_joint", [1.45],
  #     [EqualToZero])
  # goalConstraints = [constraint1]

  ## Use this for the case when initial state is a potential goal state
  ## Robot arm needs to move sphere to a specified location
  ## NOT WORKING since placement complement for the sphere placement
  ## is not properly propagated to previous waypoints.
  # constraint1 = "changed_locked_r0/shoulder_pan_joint"
  # ps.createLockedJoint(constraint1, "r0/shoulder_pan_joint", [1.45],
  #     [EqualToZero])
  # constraint2 = "changed_locked_sphere0/root_joint"
  # ps.createLockedJoint(constraint2, "sphere0/root_joint",
  #     [q0_spheres[0][0] + 0.1]+q0_spheres[0][1:], [EqualToZero] * 6)
  # goalConstraints = [constraint1, constraint2]

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
