#!/usr/bin/env python
#
#  Copyright 2020 CNRS
#
#  Author: Florent Lamiraux
#
# Start hppcorbaserver before running this script
#

import os
from argparse import ArgumentParser
from math import pi, fabs
from hpp.corbaserver.manipulation import Client, ConstraintGraph, Rule, \
    ConstraintGraphFactory, ProblemSolver
from hpp.corbaserver.manipulation.ur5 import Robot
from hpp.gepetto.manipulation import ViewerFactory
from hpp.corbaserver import loadServerPlugin

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
args = parser.parse_args()

loadServerPlugin ("corbaserver", "manipulation-corba.so")
Client ().problem.resetProblem ()

Robot.urdfFilename = \
    "package://example-robot-data/robots/ur_description/urdf/ur3_gripper.urdf"
Robot.srdfFilename = \
    "package://example-robot-data/robots/ur_description/srdf/ur3_gripper.srdf"
dir = os.getenv('PWD')

class Sphere (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  urdfName = 'construction_set/sphere'
  urdfSuffix = ""
  srdfSuffix = ""

class Ground (object):
  rootJointType = 'anchor'
  packageName = 'hpp_environments'
  urdfName = 'construction_set/ground'
  urdfSuffix = ""
  srdfSuffix = ""

nSphere = 2

robot = Robot ('ur3-spheres', 'ur3')
ps = ProblemSolver (robot)
ps.setErrorThreshold (1e-4)
ps.setMaxIterProjection (40)

vf = ViewerFactory (ps)

# Change bounds of robots to increase workspace and avoid some collisions
robot.setJointBounds ('ur3/shoulder_pan_joint', [-pi, 4])
robot.setJointBounds ('ur3/shoulder_lift_joint', [-pi, 0])
robot.setJointBounds ('ur3/elbow_joint', [-2.6, 2.6])

vf.loadEnvironmentModel (Ground, 'ground')

objects = list ()
p = ps.client.basic.problem.getProblem()
r = p.robot()
for i in range (nSphere):
  vf.loadObjectModel (Sphere, 'sphere{0}'.format (i))
  robot.setJointBounds ('sphere{0}/root_joint'.format (i),
                        [-1.,1.,-1.,1.,-.1,1.,-1.0001, 1.0001,-1.0001, 1.0001,
                         -1.0001, 1.0001,-1.0001, 1.0001,])
  objects.append ('sphere{0}'.format (i))

## Gripper
#
grippers = ["ur3/gripper"]

## Handles
#
handlesPerObject  = [['sphere{0}/handle'.format (i)] for i in range (nSphere)]
contactsPerObject = [[] for i in range(nSphere)]
## Contact surfaces
shapesPerObject = [[] for o in objects]

## Constraints
#
for i in range (nSphere):
  # Change mask of sphere handle
  o = objects[i]
  h = r.getHandle(o + '/handle')
  h.setMask([True,True,True,False,True,True])

  placementName = "place_sphere{0}".format (i)
  ps.createTransformationConstraint (placementName, "",
                                     "sphere{0}/root_joint".format (i),
                                     [0, 0, 0.02, 0, 0, 0, 1],
                                     [False, False, True, True, True, False])
  ps.createTransformationConstraint (placementName + '/complement', "",
                                     "sphere{0}/root_joint".format (i),
                                     [0, 0, 0.02, 0, 0, 0, 1],
                                     [True, True, False, False, False, True])
  ps.setConstantRightHandSide(placementName + '/complement', False)

  preplacementName = "preplace_sphere{0}".format (i)
  ps.createTransformationConstraint (preplacementName, "",
                                     "sphere{0}/root_joint".format (i),
                                     [0, 0, 0.1, 0, 0, 0, 1],
                                     [False, False, True, True, True, False])

q_init = [pi/6, -pi/2, pi/2, 0, 0, 0,
          0.2, 0, 0.02, 0, 0, 0, 1,
          0.3, 0, 0.02, 0, 0, 0, 1,]

q_goal = [pi/6, -pi/2, pi/2, 0, 0, 0,
          0.3, 0, 0.02, 0, 0, 0, 1,
          0.2, 0, 0.02, 0, 0, 0, 1,]

lang = 'py'

if lang == 'cxx':
  rules = [Rule(grippers, [''], True),
           Rule(grippers, ['sphere0/handle'], True),
           Rule(grippers, ['sphere1/handle'], True)]
  cg = ConstraintGraph.buildGenericGraph (robot = robot, name = "manipulation",
                                          grippers = grippers,
                                          objects = objects,
                                          handlesPerObjects = handlesPerObject,
                                          shapesPerObjects = contactsPerObject,
                                          envNames = [],
                                          rules = rules)

if lang == 'py':
  cg = ConstraintGraph(robot,"manipulation")
  factory = ConstraintGraphFactory(cg)
  factory.setGrippers(grippers)
  factory.setObjects(objects, handlesPerObject, contactsPerObject)
  factory.generate()
  
cg.initialize()

# Run benchmark
#
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

if args.N!=0:
  print ("Average time: " +
         str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (args.N)))
  print ("Average number nodes: " + str (totalNumberNodes/float (args.N)))

