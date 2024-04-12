#
#

from __future__ import print_function
import time, CORBA, re, os, sys
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Constraints, Rule
from hpp.corbaserver.manipulation.romeo import Robot
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer
from hpp import Transform

Robot.srdfSuffix = '_moveit'
robot = Robot ('romeo-placard', 'romeo')

# Define classes for the objects {{{4
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

ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

ps.setErrorThreshold (1e-2)
ps.setMaxIterProjection (40)

robot.setJointBounds ('romeo/root_joint' , [-1,1,-1,1, 0, 2,-2.,2,-2.,2,-2.,2,
                                            -2.,2,])
placard = Placard ('placard', vf)

robot.setJointBounds (placard.name + '/root_joint', [-1,1,-1,1,0,1.5,-2.,2,
                                                     -2.,2,-2.,2,-2.,2,])
## Lock both hands
locklhand = list()
for j,v in robot.leftHandOpen.iteritems():
    locklhand.append ('romeo/' + j)
    if type(v) is float or type(v) is int:
        val = [v,]
    else:
        val = v;
    ps.createLockedJoint ('romeo/' + j, robot.displayName + '/' + j, val)


lockrhand = list()
for j,v in robot.rightHandOpen.iteritems():
    lockrhand.append ('romeo/' + j)
    if type(v) is float or type(v) is int:
        val = [v,]
    else:
        val = v;
    ps.createLockedJoint ('romeo/' + j, robot.displayName + '/' + j, val)
lockHands = lockrhand + locklhand

## Create static stability constraint
robot.leftAnkle  = robot.displayName + '/' + robot.leftAnkle
robot.rightAnkle = robot.displayName + '/' + robot.rightAnkle
ps.addPartialCom ('romeo', ['romeo/root_joint'])
q = robot.getInitialConfig ()
r = placard.rank
q [r:r+3] = [.4, 0, 1.2]
ps.createStaticStabilityConstraints ('balance-romeo', q, 'romeo',
                                     type = ProblemSolver.FIXED_ON_THE_GROUND)
commonConstraints = Constraints (numConstraints = ps.balanceConstraints (),
                                 lockedJoints = lockHands)

# build graph
rules = [Rule (["romeo/l_hand","romeo/r_hand",], ["placard/low", ""], True),
         Rule (["romeo/l_hand","romeo/r_hand",], ["", "placard/high"], True),
         Rule (["romeo/l_hand","romeo/r_hand",], ["placard/low", "placard/high"], True),
]

grippers = ['romeo/r_hand', 'romeo/l_hand']
handlesPerObjects = [placard.handles.values ()]

cg = ConstraintGraph.buildGenericGraph (robot, "graph",
                                        grippers,
                                        [placard.name,],
                                        handlesPerObjects,
                                        [[],],
                                        [],
                                        rules)

# cg = ConstraintGraph (robot, "graph")
# factory = ConstraintGraphFactory (cg)
# factory.setGrippers (grippers)
# factory.setObjects ([placard.name,], handlesPerObjects, [[],])
# factory.setRules (rules)
# factory.generate ()
  
cg.addConstraints (graph = True, constraints = commonConstraints)
cg.initialize ()

# Define initial and final configurations
q_goal = [-0.003429678026293006, 7.761615492429529e-05, 0.8333148411182841, -0.08000440760954532, 0.06905332841243099, -0.09070086400314036, 0.9902546570793265, 0.2097693637044623, 0.19739743868699455, -0.6079135018296973, 0.8508704420155889, -0.39897628829947995, -0.05274298289004072, 0.20772797293264825, 0.1846394290733244, -0.49824886682709824, 0.5042013065348324, -0.16158420369261683, -0.039828502509861335, -0.3827070014985058, -0.24118425356319423, 1.0157846623463191, 0.5637424355124602, -1.3378817283780955, -1.3151786907256797, -0.392409481224193, 0.11332560818107676, 1.06, 1.06, 1.06, 1.06, 1.06, 1.06, 1.0, 1.06, 1.06, -1.06, 1.06, 1.06, 0.35936687035487364, -0.32595302056157444, -0.33115291290191723, 0.20387672048126043, 0.9007626913161502, -0.39038645767349395, 0.31725226129015516, 1.5475253831101246, -0.0104572058777634, 0.32681856374063933, 0.24476959944940427, 1.06, 1.06, 1.06, 1.06, 1.06, 1.06, 1.0, 1.06, 1.06, -1.06, 1.06, 1.06, 0.412075621240969, 0.020809907186176854, 1.056724788359247, 0.0, 0.0, 0.0, 1.0]
q_init = q_goal [::]
q_init [r+3:r+7] = [0, 0, 1, 0]

n = 'romeo/l_hand grasps placard/low'
res, q_init, err = cg.applyNodeConstraints (n, q_init)
if not res: raise RuntimeError ("Failed to project initial configuration.")
res, q_goal, err = cg.applyNodeConstraints (n, q_goal)
if not res: raise RuntimeError ("Failed to project initial configuration.")

ps.selectPathProjector ("Progressive", .05)

import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
N = 20
for i in range (N):
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

print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (N)))
print ("Average number nodes: " + str (totalNumberNodes/float (N)))

# v = vf.createViewer ()
# v (q)
