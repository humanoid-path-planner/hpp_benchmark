from hpp.corbaserver.pr2 import Robot
robot = Robot ('pr2')
robot.setJointBounds ("base_joint_x", [-4, -3])
robot.setJointBounds ("base_joint_y", [-5, -3])

from hpp_ros import ScenePublisher
r = ScenePublisher (robot)

from argparse import ArgumentParser
from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

# parse arguments {{{2
parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
args = parser.parse_args()
# 2}}}

q_init = [-3.2, -4, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
r (q_init)

q_goal = [-3.2, -4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.5, 0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
r (q_goal)

ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "")

ps.selectPathOptimizer ("None")
import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
success = 0
for i in range (args.N):
  ps.client.problem.clearRoadmap ()
  ps.resetGoalConfigs ()
  ps.setInitialConfig (q_init)
  ps.addGoalConfig (q_goal)
  try:
    t1 = dt.datetime.now ()
    ps.solve ()
    t2 = dt.datetime.now ()
  except:
    print ("Failed to plan path.")
  else:
    success += 1
    totalTime += t2 - t1
    print (t2-t1)
    n = len (ps.client.problem.nodes ())
    totalNumberNodes += n
    print ("Number nodes: " + str(n))

if args.N != 0:
  print (f"Number of rounds: {args.N}")
  print (f"Number of successes: {success}")
  print (f"Success rate: {success/ args.N * 100}%")
  if success > 0:
    print (f"Average time per success: {totalTime.total_seconds()/success}")
    print (f"Average number nodes per success: {totalNumberNodes/success}")

from hpp_ros import PathPlayer
pp = PathPlayer (robot.client, r)

pp (0)
pp (1)
