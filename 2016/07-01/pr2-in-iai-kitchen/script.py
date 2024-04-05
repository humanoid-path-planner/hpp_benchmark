from hpp.corbaserver.pr2 import Robot
from hpp.gepetto import PathPlayer

robot = Robot ('pr2')
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])

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

vf.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")

ps.selectPathValidation ("Dichotomy", 0)

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
for i in range (20):
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

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/20.))
print ("Average number nodes: " + str (totalNumberNodes/20.))

# r = vf.createViewer()
# pp = PathPlayer (robot.client, r)

