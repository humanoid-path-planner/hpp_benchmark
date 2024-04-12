#!/usr/bin/env python
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#


from argparse import ArgumentParser
import time, re, os, sys
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Constraints, Rule, Client
from hpp.corbaserver.manipulation.romeo import Robot
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.benchmark import PathChecker
import sys

parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

loadServerPlugin ("corbaserver", "manipulation-corba.so")
Client ().problem.resetProblem ()

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

dimensionRegex = re.compile ("dimension ([0-9]*)")
reducedDimRegex = re.compile ("reduced dimension ([0-9]*)")

def getConstraintDimension (reduced = False):
    cstr = ps.client.basic.problem.displayConstraints()
    m = (reducedDimRegex if reduced else dimensionRegex).search (cstr)
    return int(m.group(1))

def createGraspConstraint(gripperName, handleName):
    name = "Relative transformation " + gripperName + "/" + handleName
    gjn, gpos = robot.getGripperPositionInJoint(gripperName)
    hjn, hpos = robot.getHandlePositionInJoint(handleName)
    ps.createTransformationConstraint (name, gjn, hjn, (Transform(gpos) * Transform(hpos).inverse()).toTuple(), [True] * 6)
    return name

def benchConstraints (constraints, lockDofs):
    ps.client.basic.problem.resetConstraints()
    ps.resetConstraints ()
    ps.addNumericalConstraints ("test", constraints)
    ps.addLockedJointConstraints ("test", lockDofs)
    res = [None] * N
    q   = [None] * N
    err = [None] * N
    start = time.time()
    for i in range(N):
        res[i], q[i], err[i] = ps.applyConstraints(qs[i])
    duration = time.time() - start
    # return res,q,err,duration
    return float(res.count(True)) / N,duration / N

ps = ProblemSolver (robot)
# Remove joint bound validation
ps.hppcorba.problem.clearConfigValidations()
ps.addConfigValidation("CollisionValidation")
vf = ViewerFactory (ps)

ps.setErrorThreshold (1e-4)
ps.setMaxIterProjection (40)

robot.setJointBounds ('romeo/root_joint' , [-1,1,-1,1, 0, 2,-2.,2,-2.,2,-2.,2,
                                            -2.,2,])
placard = Placard ('placard', vf)

robot.setJointBounds (placard.name + '/root_joint', [-1,1,-1,1,0,1.5,-2.,2,
                                                     -2.,2,-2.,2,-2.,2,])
## Lock both hands
locklhand = list()
for j,v in robot.leftHandOpen.items():
    locklhand.append ('romeo/' + j)
    if type(v) is float or type(v) is int:
        val = [v,]
    else:
        val = v;
    ps.createLockedJoint ('romeo/' + j, robot.robotNames [0] + '/' + j, val)


lockrhand = list()
for j,v in robot.rightHandOpen.items():
    lockrhand.append ('romeo/' + j)
    if type(v) is float or type(v) is int:
        val = [v,]
    else:
        val = v;
    ps.createLockedJoint ('romeo/' + j, robot.robotNames [0] + '/' + j, val)
lockHands = lockrhand + locklhand

## Create static stability constraint
robot.leftAnkle  = robot.robotNames [0] + '/' + robot.leftAnkle
robot.rightAnkle = robot.robotNames [0] + '/' + robot.rightAnkle
ps.addPartialCom ('romeo', ['romeo/root_joint'])
q = robot.getInitialConfig ()
r = placard.rank
q [r:r+3] = [.4, 0, 1.2]
ps.addPartialCom ("romeo", ["romeo/root_joint"])
robot.createStaticStabilityConstraint ('balance/', 'romeo', robot.leftAnkle,
                                       robot.rightAnkle, q)
balanceConstraints = ['balance/pose-left-foot',
                      'balance/pose-right-foot',
                      'balance/relative-com',]
commonConstraints = Constraints (numConstraints = balanceConstraints + \
                                 lockHands)

# build graph
rules = [Rule (["romeo/l_hand","romeo/r_hand",], ["placard/low", ""], True),
         Rule (["romeo/l_hand","romeo/r_hand",], ["", "placard/high"], True),
         Rule (["romeo/l_hand","romeo/r_hand",], ["placard/low", "placard/high"], True),
]

grippers = ['romeo/r_hand', 'romeo/l_hand']
handlesPerObjects = [list(placard.handles.values ())]

lang = 'py'

if lang == 'cxx':
  cg = ConstraintGraph.buildGenericGraph (robot, "graph",
                                          grippers,
                                          [placard.name,],
                                          handlesPerObjects,
                                          [[],],
                                          [], rules)

if lang == 'py':
  cg = ConstraintGraph (robot, "graph")
  factory = ConstraintGraphFactory (cg)
  factory.setGrippers (grippers)
  factory.setObjects ([placard.name,], handlesPerObjects, [[],])
  factory.setRules (rules)
  factory.generate ()

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

#check if there are collisions, discontinuities or wrong configurations in each path
p = PathChecker(ps, q_init, q_goal)
dtime = 0.001
p.continuityThreshold = 50 
for i in range (ps.numberPaths()):
  print("\n---Path {}---".format(i))
  p.check_path(i, dtime)

if args.display:
    v = vf.createViewer ()
    v (q)
    pp = PathPlayer(v)
    if args.run:
        pp(0)

def toVector (s):
  return list(map (float, [x for x in s.split (" ") if x != ""]))

N=10000
qs = [None] * N
for i in range(N):
    qs[i] = robot.shootRandomConfig()

implicitGraspConstraints = dict()
explicitGraspConstraints = dict()
for handles in handlesPerObjects:
    for h in handles:
        for g in grippers:
            n = createGraspConstraint (g,h)
            ne = g + " grasps " + h
            cg.createGrasp (ne, g, h)
            implicitGraspConstraints[(g,h)] = n
            explicitGraspConstraints[(g,h)] = ne + "/hold"

# List of grasps for each node. From any node to the next one, one grasp is
# added or removed
grasps = set ()
# list of set of grasps which are benchmarked
benchGrasps = list()

# grasp high
grasps.add (('romeo/r_hand', placard.handles['high']))
benchGrasps.append (grasps.copy())

# grasp low
grasps.add (('romeo/l_hand', placard.handles['low']))
benchGrasps.append (grasps.copy())

# assemble cylinder0 and sphere0
grasps.remove (('romeo/r_hand', placard.handles['high']))
benchGrasps.append (grasps.copy())

iResults = dict()
eResults = dict()
for g in benchGrasps:
    constraints = balanceConstraints + [ implicitGraspConstraints[c] for c in g ]
    iResults[tuple (g)] = benchConstraints(constraints, lockHands)

    constraints = balanceConstraints + [ explicitGraspConstraints[c] for c in g ]
    eResults[tuple (g)] = benchConstraints(constraints, lockHands)

name = dict ()
name [(('romeo/l_hand', 'placard/low'), ('romeo/r_hand', 'placard/high'))] =\
  "both hands"
name [(('romeo/r_hand', 'placard/high'),)] = "right hand"
name [(('romeo/l_hand', 'placard/low'),)] = "left hand"

for k, v in iResults.items ():
  print ("Average success rate for implicit {0}: {1}".format (name [k], v [0]))
  print ("Average time for implicit {0}: {1}".format (name [k], v [1]))
for k, v in eResults.items ():
  print ("Average success rate for explicit {0}: {1}".format (name [k], v [0]))
  print ("Average time for explicit {0}: {1}".format (name [k], v [1]))