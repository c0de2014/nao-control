#!/usr/bin/env python
#########################################################################
# Reinforcement Learning with PGPE on the CCRL ODE Environment
#
# The CCRL robot is a body structure with 2x 7 DoF Arms.
# Complex grasping tasks can be learned with this environment.
#
# Control/Actions:
# The agent can control all 14 DOF of the robot arms plus the 2 hands.
#
# A wide variety of sensors are available for observation and reward:
# - 16 angles of joints
# - 16 angle velocitys of joints
# - Number of hand parts that have contact to target object
# - collision with table
# - distance of hand to target
# - angle of hand to horizontal and vertical plane
#
# Task available are:
# - Grasp Task, agent has to get hold of the object with avoiding collision with table
#
# Requirements: pylab (for plotting only). If not available, comment the
# last 3 lines out
# Author: Frank Sehnke, sehnke@in.tum.de
#########################################################################
__author__ = "Frank Sehnke"
__version__ = '$Id$'


######
# from grabbingTask.. some algos
from pybrain.rl.agents import LearningAgent
from pybrain.rl.learners import Reinforce
from pybrain.rl.learners.valuebased.sarsa import SARSA
from pybrain.rl.learners.valuebased.q import Q
from pybrain.rl.learners.valuebased.qlambda import QLambda
from pybrain.rl.learners.directsearch import gpomdp
from pybrain.rl.learners.directsearch.enac import ENAC
from pybrain.rl.learners.directsearch.policygradient import PolicyGradientLearner

from pybrain.rl.explorers.continuous.sde import StateDependentExplorer

from pybrain.optimization import SimpleSPSA
from pybrain.optimization import *
from pybrain.optimization import RandomSearch

from pybrain.rl.learners.valuebased import NFQ, ActionValueNetwork
from pybrain.rl.explorers import BoltzmannExplorer

######


#from pybrain.tools.example_tools import ExTools
#from pybrain.rl.environments.ode import CCRLEnvironment

#from pybrain.rl.environments.ode.tasks import CCRLGlasTask
from pybrain.structure.modules import TanhLayer,SoftmaxLayer, SigmoidLayer
from pybrain.tools.shortcuts import buildNetwork
from pybrain.rl.agents import OptimizationAgent
from pybrain.optimization import PGPE
from pybrain.rl.experiments import EpisodicExperiment

from grabbingEnvironment import grabbingEnvironment
from grabbingTask import grabbingTask

from pybrain.rl.agents import LearningAgent
from pybrain.rl.learners import Reinforce
import math
from test2Dagent import world
import myRL

class grabbingTestNew():
    hiddenUnits = 4
    batch=10 #number of samples per learning step
    prnts=1 #number of learning steps after results are printed
    epis=2000/batch/prnts #number of roleouts
    numbExp=10 #number of experiments
    #et = ExTools(batch, prnts) #tool for printing and plotting

    testcount = 1

    env = None

    nao = None

    task = None

    testagent = None

    world2d = None

    def __init__(self,nao):
        self.nao = nao
        

        self.world2d = world()
        self.env = grabbingEnvironment(self.nao,self.world2d)
        self.task = grabbingTask(self.env,self.world2d)

        #module = ActionValueNetwork(3, 3)
        #module = NeuronLayer(40)

        #agent = LearningAgent(net, SARSA())
        #learner = PolicyGradientLearner()
        #learner._setExplorer(StateDependentExplorer(3,3))
        #learner._setModule(module)
        #agent = LearningAgent(module, learner)
        #agent = LearningAgent(net, ENAC())
        #agent = LearningAgent(net, Reinforce())

        #learner = NFQ()
        #learner.explorer.epsilon = 0.4
        #agent = LearningAgent(net, learner)

        #testagent = OptimizationAgent(net,None,env)
        #agent = LearningAgent(module, Q())
        #agent = LearningAgent(module, QLambda())

        #agent = OptimizationAgent(net, SimpleSPSA(storeAllEvaluations = True, verbose = True))


        #agent = OptimizationAgent(net, RandomSearch(storeAllEvaluations = True, verbose = True))

        # create controller network
        
        # create agent with controller and learner (and its options)

        self.net = buildNetwork(len(self.task.getObservation()),self.env.indim, outclass=SigmoidLayer)#outclass=TanhLayer  ,,SoftmaxLayer)# , recurrent=False) #, hiddenUnits
        #self.agent = OptimizationAgent(self.net, SimpleSPSA(storeAllEvaluations = True, verbose = True))
        #self.agent = OptimizationAgent(self.net, RandomSearch(storeAllEvaluations = True, verbose = True))
        #self.agent = OptimizationAgent(self.net, StochasticHillClimber(storeAllEvaluations = True, verbose = True))
        
        self.agent = OptimizationAgent(self.net, PGPE(storeAllEvaluations = True,minimize=False,verbose=True, epsilon = 2.0))#, sigmaLearningRate = 0.3, learningRate = 0.2))

        #self.agent = LearningAgent(self.net, SARSA(alpha=0.5, gamma=0.98999999999999999))
        #self.agent = LearningAgent(self.net, QLambda(alpha=0.5, gamma=0.98999999999999999, qlambda=0.9))
        #self.agent = LearningAgent(self.net, Reinforce())

        self.testagent = None
        #et.agent = agent
        # create the experiment
        self.experiment = EpisodicExperiment(self.task, self.agent)

    mylearner = myRL.myRL()





    def run(self,count):

#        self.mylearner.learn((0,0),(1,0),0.00000001)
#        self.mylearner.learn((0,0),(0,-1),0.00000001)
#        self.mylearner.learn((0,0),(-1.0,0),0.00000001)
#        self.mylearner.learn((0,0),(0,1.0),0.00000001)
#
#
#        #self.mylearner.learn((0,0),(0,-1.0),0.00000001)
#
#
#        for i in range (0,100000):
#            print "learn"
#            for i in range (0,5):
#                self.mylearner.doLearning(self.nao, streuung = 2.0,frequency = 0.75,rangeext = 5.0, count = 40)
#                self.mylearner.minimize(500)
#            print "test"
#            self.mylearner.doTestRun(self.nao, 40)


        batch=1 #number of samples per learning step
        prnts=1 #number of learning steps after results are printed
        epis=2/batch/prnts #number of roleouts
        tests = 50
        counter = 0
        minruns = 2

        for runs in range(count):


            counter += 1
            # create environment
            #Options: XML-Model, Bool(OpenGL), Bool(Realtime simu. while client is connected), ServerIP(default:localhost), Port(default:21560)
            #if env != None: env.closeSocket()

            # create task

            print "start learning"
            self.experiment.agent = self.agent
            #Do the experiment
            for updates in range(epis):
                for i in range(prnts):
                    self.env.reset()
#                    self.agent.reset()

                    self.experiment.doEpisodes(batch)
                    self.experiment.doEpisodes(batch)


#                    self.agent.learn()
                    
            #toTest = self.experiment.optimizer._bestFound()

            #print "best net:",toTest
            
            print "start testing"
            #self.experiment.agent = self.testagent
            if counter >= minruns:
                self.env.reset()

                for j in range(0,self.testcount):
                    #self.task.getReward()
                    rewsum = 0
                    self.env.reset()
                    self.world2d.reset()
    #                agent.reset()

                    self.world2d.drawtest = True
                    # Evaluate the current learned policy for numTestingEps episodes
                    rewards = []
                    #testingJointProbs = []
                    for dummy in range(self.testcount):
                        print "\t\tTESTING EP",dummy
                        self.agent.newEpisode()
                        # Execute the agent in the environment without learning for one episode.
                        # This uses the current set of parameters
                        r = self.agent.learner._BlackBoxOptimizer__evaluator(self.agent.learner.wrappingEvaluable)
                        print "\t\t\treward testing episode",dummy,r
                        rewards.append(r)
                        #testingJointProbs.append(task.objects[0].jointProb)
                    # average of all rewards earned by the current policy running numTestingEps episodes
                    total_rewards=0
                    for r in rewards:
                        total_rewards += r
                    total_rewards /= len(rewards)

                    print "\n\taverage testing reward batch",total_rewards
                    print "\n"
                    self.world2d.drawtest = False


#                    print "my Testing"
#                    for i in range (0,tests):
#                        self.task.getObservation()
#                        #self.task.performAction(self.agent.getAction())
#                        #rewsum += self.task.getReward()
#
#
#    #                    self.world2d.test = True
#    #                    self.experiment.doEpisodes(batch)
#    #                    self.world2d.test = False
#
#                        action = self.experiment.optimizer._bestFound()[0].activate(self.task.getObservation())
#                        self.env.performAction(action)
#                        self.task.getObservation()
#                        rewc = self.task.getReward()
#                        rewsum += rewc
#                        thickness = rewc**2
#                        self.world2d.doAction(action,int(round(thickness)),test = True)
#                        #self.world2d.doAction(action, width, test)




#                    print "done testing ",j+1,":",rewsum
                    #et.printResults((agent.learner._allEvaluations)[-50:-1], runs, updates)
            #et.addExps()
        #et.showExps()
        #To view what the simulation is doing at the moment, go to pybrain/rl/environments/ode/ and start viewer.py (python-openGL musst be installed, see PyBrain documentation)
