#!/usr/bin/env python
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#

from argparse import ArgumentParser
from hpp.corbaserver.pr2 import Robot
from hpp.gepetto import PathPlayer
from math import pi

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

robot = Robot ('pr2')
robot.setJointBounds ("root_joint", [-4, -3, -5, -3,-2,2,-2,2])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['torso_lift_joint']
q_init [rank] = 0.2

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['r_elbow_flex_joint']
q_goal [rank] = -0.5

vf.loadObstacleModel ("package://iai_maps/urdf/kitchen_area.urdf", "kitchen")

ps.selectPathValidation ("Progressive", 0.025)
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
    n = len (ps.client.problem.nodes ())
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (args.N)))
print ("Average number nodes: " + str (totalNumberNodes/float (args.N)))

if args.display:
    v = vf.createViewer(); v (q_init)
    pp = PathPlayer (v, robot.client)
    if args.run:
        pp(0)
