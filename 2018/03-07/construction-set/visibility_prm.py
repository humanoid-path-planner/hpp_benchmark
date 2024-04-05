#
#  Copyright (2017) CNRS
#
#  Author: Florent Lamiraux
#

from state_name import StateName

class Edge (object):
    '''
    Store an edge as a triple (n0, n1, pathId)

    Input
      - n0: initial node,
      - n1 : goal node,
      - pathId: id of a path going from "n0" to "n1".
    '''
    def __init__ (self, n0, n1, pathId):
        self.n0 = n0
        self.n1 = n1
        self.pathId = pathId

class VisibilityPRM (object):
    '''
    Run visibility PRM in a State of a manipulation constraint graph in order
    to connect two configurations
    '''

    @staticmethod
    def getState (cg, nodeName) :
        for n in cg.nodes:
            if StateName (n) == nodeName : return n
        raise KeyError (nodeName)

    maxIter = 100

    def __init__ (self, cg, ps, q_init, q_goal, state, loopTransition,
                  logStream):
        '''
        Constructor

        Input
          - cg: the constraint graph that contains the state and edge,
          - ps: manipulation ProblemSolver instance,
          - q_init, q_goal: the initial and goal configurations. Both should lie
            in the input state,
          - state: state of the constraint graph in which the roadmap is built,
          - loopTransition: the transition linking the state with itsef,
          - logStream: output stream to write log
        '''
        self.cg = cg
        self.ps = ps
        self.state = self.getState (cg = cg, nodeName = state)
        self.loopTransition = loopTransition
        res, err = cg.getConfigErrorForNode (self.state, q_init)
        if not res:
            raise RuntimeError ("q_init = " + str (q_init) + " does not satisfy"
                                + " constraints of state " + self.state)
        self.q_init = q_init
        self.q_goal = q_goal
        self.ps.addConfigToRoadmap (q_init)
        self.ps.addConfigToRoadmap (q_goal)
        res, pid, msg = self.ps.directPath (q_init, q_goal, True)
        if res:
            self.ps.addEdgeToRoadmap (q_init, q_goal, pid, True)
            self.solved = True
        else:
            self.solved = False
            self.isSolved ()
        self.logStream = logStream
        # build list of nodes from current roadmap that lie in the right state
        nodes = filter (lambda q : self.cg.getConfigErrorForEdgeLeaf \
                        (self.loopTransition, self.q_init, q) [0],
                        self.ps.nodes ())
        nodes.append (self.q_init)
        nodes.append (self.q_goal)
        self.nodes = set (map (tuple, nodes))

    def writeLog (self, s):
        if self.logStream: self.logStream.write (s)

    def isSolved (self):
        '''
        Compute whether problem is solved
        '''
        if self.solved == True: return True
        for i in range (self.ps.numberConnectedComponents ()):
            nodes = self.ps.nodesConnectedComponent (i)
            if self.q_init in nodes and self.q_goal in nodes:
                self.solved = True
                return self.solved
        self.solved = False
        return self.solved

    def solve (self):
        '''
        Solve path planning problem by running a Visibility PRM algorithm
        '''
        self.nIter = 0
        while not self.isSolved ():
            self.oneStep ()
            self.nIter += 1
            self.writeLog ("nIter = " + str (self.nIter))
            if self.nIter > self.maxIter :
                self.writeLog ("Maximal number of iterations reached")
                self.writeLog (' between\n')
                self.writeLog ('q_init = {0} and\n'.format (self.q_init))
                self.writeLog ('q_goal = {0}\n'.format (self.q_goal))
                raise RuntimeError ("Maximal number of iterations reached.")
        self.writeLog ('successfully solved path planning between\n')
        self.writeLog ('q_init = {0} and\n'.format (self.q_init))
        self.writeLog ('q_goal = {0}\n'.format (self.q_goal))

    def oneStep (self):
        '''
        Run one step of Visibility PRM
        '''
        # Generate a random configuration
        for i in range (50):
            q = self.ps.robot.shootRandomConfig ()
            res, q_rand, err = self.cg.generateTargetConfig \
                               (self.loopTransition, self.q_init, q)
            if not res: continue
            res, msg = self.ps.robot.isConfigValid (q_rand)
            if res: break
        if i == 49:
            self.writeLog \
                ("Failed to generate a random valid configuration.")
            raise RuntimeError \
                ("Failed to generate a random valid configuration.")
        # try to connect to other connected components
        edges = list ()
        for i in range (self.ps.numberConnectedComponents ()):
            edge = self.connect (q_rand, i)
            if edge:
                edges.append (edge)
        if len (edges) != 1:
            self.ps.addConfigToRoadmap (q_rand)
            self.nodes.add (tuple (q_rand))
            #self.writeLog ('Added q_rand = {0} in roadmap and connected it with\n'.format (q_rand))
            for e in edges:
                self.ps.addEdgeToRoadmap (e.n0, e.n1, e.pathId, True)
                #self.writeLog ('n1 = {0}\n'.format (e.n1))
                #self.writeLog ('pathId = {0}\n'.format (e.pathId))

    def connect (self, config, i):
        '''
        Try to connect random configuration to connected component of roadmap

          Input
            - config: random configuration in search state,
            - i: id of the connected component of the roadmap
        '''
        # Consider only nodes in the state that are reachable from q_init
        nodes = self.nodes & set (map (tuple,
                                       self.ps.nodesConnectedComponent (i)))
        # sort by increasing distance to input configuration
        nodesAndDistances = list ()
        for n in nodes:
            res, pid, msg = self.ps.directPath (config, n, False)
            if not res:
                d = float('+inf')
            else:
                d = self.ps.pathLength (pid)
            nodesAndDistances.append ((n, d))
        nodesAndDistances.sort (key = lambda x : x [1])
        # Try to connect input configuration to each node in order of increasing
        # distance.
        for n, d in nodesAndDistances:
            if d == float('+inf'):
                res = False; pid = -1;
            else:
                res, pid, msg = self.ps.directPath (config, n, True)
            if res:
                return Edge (n0 = config, n1 = n, pathId = pid)
        return None
