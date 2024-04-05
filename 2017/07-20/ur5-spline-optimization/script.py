from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import CORBA

robot = Robot ('ur5')
cl = robot.client
ps = ProblemSolver (robot)

# q = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 6 DoF#
q1 = [0, -1.57, 1.57, 0, 0, 0]; q2 = [0.2, -1.57, -1.8, 0, 0.8, 0]
q3 = [0, -1.57, 1.57, 3.267256451, 0, 0]

q_init = q1;
# q_goal = q2
q_goal = q3

from hpp.gepetto import ViewerFactory, PathPlayerGui
vf = ViewerFactory (ps)
# vf.loadObstacleModel ("ur_description","obstacles","obstacles")
vf.loadObstacleModel ("ur_description","table","table")
# vf.loadObstacleModel ("ur_description","wall","wall")
vf(q1)

ps.lockJoint("elbow_joint", [q1[2]])

ps.selectPathValidation("Progressive", .05)

ps.setParameter("SplineGradientBased/alphaInit", CORBA.Any(CORBA.TC_double, 0.3))
ps.setParameter("SplineGradientBased/alwaysStopAtFirst", CORBA.Any(CORBA.TC_boolean, True))
ps.setParameter("SplineGradientBased/linearizeAtEachStep", CORBA.Any(CORBA.TC_boolean, False))

print "Optimizer parameters are:"
print "alphaInit:", ps.getParameter("SplineGradientBased/alphaInit").value()
print "alwaysStopAtFirst:", ps.getParameter("SplineGradientBased/alwaysStopAtFirst").value()
print "linearizeAtEachStep:", ps.getParameter("SplineGradientBased/linearizeAtEachStep").value()
print ""

import datetime as dt
totalSolveTime = dt.timedelta (0)
totalOptimTime = dt.timedelta (0)
totalNumberNodes = 0
N = 20
for i in range (N):
    ps.clearPathOptimizers()
    ps.clearRoadmap ()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)

    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
    ps.addPathOptimizer ("SplineGradientBased_bezier3")
    ps.optimizePath (ps.numberPaths() - 1)
    t3 = dt.datetime.now ()

    totalSolveTime += t2 - t1
    totalOptimTime += t3 - t2
    print "Solve:", t2-t1
    print "Optim:", t3-t2
    n = len (ps.client.problem.nodes ())
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

print ("Average solve time: " + str ((totalSolveTime.seconds+1e-6*totalSolveTime.microseconds)/float (N)))
print ("Average optim time: " + str ((totalOptimTime.seconds+1e-6*totalOptimTime.microseconds)/float (N)))
print ("Average number nodes: " + str (totalNumberNodes/float (N)))
