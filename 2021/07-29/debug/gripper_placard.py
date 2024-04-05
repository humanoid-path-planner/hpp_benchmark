from argparse import ArgumentParser
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import ConstraintGraph, \
    ConstraintGraphFactory, Constraints, ProblemSolver, Robot, Rule, Client
from hpp.gepetto.manipulation import Viewer, ViewerFactory

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

loadServerPlugin ("corbaserver", "manipulation-corba.so")
Client ().problem.resetProblem ()

Robot.urdfFilename = "package://hpp_environments/urdf/tests/gripper.urdf"
Robot.srdfFilename = "package://hpp_environments/srdf/tests/gripper.srdf"

# Define class for the gripper
class Gripper (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  urdfFilename = 'package://hpp_environments/urdf/tests/gripper.urdf'
  srdfFilename = 'package://hpp_environments/srdf/tests/gripper.srdf'
  def __init__ (self, name, vf):
    self.name = name
    self.vf = vf
    self.joints = [name + '/root_joint']
    vf.loadObjectModel (self.__class__, name)
    self.rank = vf.robot.rankInConfiguration [name + '/root_joint']

class Placard (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  urdfName = 'placard'
  urdfSuffix = ''
  srdfSuffix = ''
  def __init__ (self, name, vf):
    self.name = name
    self.vf = vf
    self.joints = [name + '/root_joint']
    self.handles = dict ()
    self.handles ['low'] = name + '/low'
    self.handles ['high'] = name + '/high'
    vf.loadObjectModel (self.__class__, name)
    self.rank = vf.robot.rankInConfiguration [name + '/root_joint']

robot = Robot("2grippers-placard", "gripper1", "freeflyer")
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)

# Add the second gripper
gripper2 = Gripper('gripper2', vf)
# Add the placard
placard = Placard('placard', vf)

# Set joint bounds
for j in robot.jointNames:
  robot.setJointBounds(j, [-1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1])

# Create constraint graph
grippers = ['gripper1/gripper', 'gripper2/gripper']
handlesPerObjects = [['placard/low', 'placard/high'],]
# Each gripper has a dedicated handle
rules = [Rule (["gripper1/gripper","gripper2/gripper",],
               ["placard/low", ""], True),
         Rule (["gripper1/gripper","gripper2/gripper",],
               ["", "placard/high"], True),
         Rule (["gripper1/gripper","gripper2/gripper",],
               ["placard/low", "placard/high"], True),]

cg = ConstraintGraph (robot, "graph")
factory = ConstraintGraphFactory (cg)
factory.setGrippers (grippers)
factory.setObjects ([placard.name,], handlesPerObjects, [[],])
factory.setRules (rules)
factory.generate ()
cg.initialize()

# Initial configuration
q_init = (0.022656408215081747, 0.050114992988037593, -0.35000201226769789,
          0.69122759922768817, -0.14900491950810335, -0.14900319308743773,
          0.69123077801705113,
          0.010833173093688031, 0.053920938898609466, -0.19999812852307175,
          0.70363371025760868, -0.069999999775223559, -0.069999384241997753,
          0.70363320560240883,
          0,0,0,0,0,0,1)

q_goal = (0.022656408215081747, 0.050114992988037593, -0.35000201226769789,
          0.69122759922768817, -0.14900491950810335, -0.14900319308743773,
          0.69123077801705113,
          0.010833173093688031, 0.053920938898609466, -0.19999812852307175,
          0.70363371025760868, -0.069999999775223559, -0.069999384241997753,
          0.70363320560240883,
          0,0,0,0,0,1,0)

ps.selectPathProjector ("Progressive", .05)

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

