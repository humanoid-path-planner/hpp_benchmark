from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2')
robot.setJointBounds ("base_joint_x", [-4, -3])
robot.setJointBounds ("base_joint_y", [-5, -3])

from hpp_ros import ScenePublisher
r = ScenePublisher (robot)

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

q_init = [-3.2, -4, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
r (q_init)

q_goal = [-3.2, -4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.5, 0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
r (q_goal)

ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area")

ps.selectPathOptimizer ("None")
import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
for i in range (20):
    ps.client.problem.clearRoadmap ()
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

from hpp_ros import PathPlayer
pp = PathPlayer (robot.client, r)

pp (0)
pp (1)
