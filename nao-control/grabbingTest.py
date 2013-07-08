

import naoControl
import generateTraining

#from generateTraining import runDeltaMovements
import time,math

from grabbingEnvironment import grabbingEnvironment
from grabbingTask import grabbingTask


from pybrain.tools.shortcuts import buildNetwork
from pybrain.structure.modules.tanhlayer import TanhLayer
from pybrain.structure.modules.softmax import SoftmaxLayer
from pybrain.structure.modules.neuronlayer import NeuronLayer

#from pybrain.optimization import PGPE
from grabbingPGPE import grabbingPGPE

from pybrain.rl.agents import OptimizationAgent
from pybrain.rl.experiments import EpisodicExperiment

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
from pybrain.optimization import HillClimber
from pybrain.optimization import RandomSearch

from pybrain.rl.learners.valuebased import NFQ, ActionValueNetwork
from pybrain.rl.explorers import BoltzmannExplorer
#from matplotlib import pyplot as plt
#from pybrain.rl.environments.cartpole import CartPoleEnvironment, DiscreteBalanceTask#, CartPoleRenderer

import pickle, math
from pickle import PicklingError

from pybrain.datasets import SupervisedDataSet
from numpy import array, arange, meshgrid, pi, zeros, mean
#from test2Dagent import world
#from myLearner import myLearner

from pyx import *




def run(nao,pad):





    # ################################
    # choose bottom cam, so nao can see object when standing next to it
    nao.camera.selectCam(1)
    
    env = grabbingEnvironment(nao)
    #env.connect(nao)

    task = grabbingTask(env)

    net = buildNetwork(len(task.getObservation()),8, env.indim, bias = True, recurrent=True)
    print env.indim
    #net = ActionValueNetwork(5,4)
    #, outclass=TanhLayer)
    #, hiddenclass=TanhLayer, outclass=TanhLayer

    # not correct right now..
    # TODO: train into RL Modules, dataset needs to be merged with exploration data
    #generateTraining.generateTraining().runDeltaMovements(nao,net,env,pad)


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

    testagent = OptimizationAgent(net,None,env)
    #agent = LearningAgent(module, Q())
    #agent = LearningAgent(module, QLambda())
    learner = grabbingPGPE(storeAllEvaluations = True, verbose = True, epsilon = 1.0, deltamax =5.0, sigmaLearningRate = 0.1, learningRate = 0.2)
    agent = OptimizationAgent(net, learner,env)
    #agent = OptimizationAgent(net, SimpleSPSA(storeAllEvaluations = True, verbose = True))
    #agent = OptimizationAgent(net, HillClimber(storeAllEvaluations = True, verbose = True))

    #agent = OptimizationAgent(net, RandomSearch(storeAllEvaluations = True, verbose = True))
    
    experiment = EpisodicExperiment(task, agent)
    # only for optimizationAgent
    #experiment.doOptimization = True

    # only for simulator!
    nao.fractionMaxSpeed = 1.0



    print "#env"
    print "  sensors:", env.outdim
    print "  actions:", env.indim
    print "  discreteStates:", env.discreteStates
    print "  discreteActions:", env.discreteActions
    
    print
    print "#task"
    print "  sensor_limits:", task.sensor_limits
    print "  actor_limits:", task.actor_limits
    print "  epilen: ", task.epiLen
    print "#EpisodicTask"
    print "  discount:", task.discount
    print "  batchsize:", task.batchSize
    

    print
    print "#PGPE"
    print "  exploration type:", grabbingPGPE().exploration
    print "  LearningRate:", grabbingPGPE().learningRate
    print "  sigmaLearningRate:", grabbingPGPE().sigmaLearningRate
    print "  epsilon:", grabbingPGPE().epsilon
    print "  wDecay:", grabbingPGPE().wDecay
    print "  momentum:", grabbingPGPE().momentum
    print "  rprop:", grabbingPGPE().rprop



#    # switch this to True if you want to see the cart balancing the pole (slower)
#    render = False
#
#    plt.ion()
#
#    env = CartPoleEnvironment()
#    if render:
#        renderer = CartPoleRenderer()
#        env.setRenderer(renderer)
#        renderer.start()
#
#    module = ActionValueNetwork(4, 3)
#
#    task = DiscreteBalanceTask(env, 100)
#    learner = NFQ()
#    learner.explorer.epsilon = 0.4
#
#    agent = LearningAgent(module, learner)
#    testagent = LearningAgent(module, None)
#    experiment = EpisodicExperiment(task, agent)
#
#    performance = []
#
#    if not render:
#        pf_fig = plt.figure()

    count = 0
    while(True):
            # one learning step after one episode of world-interaction
        count += 1
        print "learning #",count
        experiment.agent = agent
        experiment.doOptimization = True
        erg = experiment.doEpisodes(1)
        print erg
        #experiment.doOptimization = False
        #print "agent learn"
        #agent.learner.learn(1)

        if count > 8:
        # test performance (these real-world experiences are not used for training)
#        if render:
#            env.delay = True
            #experiment.agent = testagent
            print "testing"
            experiment.doOptimization = False

            erg = experiment.doEpisodes(1)
            summe = 0
            #print erg
#            for x in erg:
#                summe = sum(x)
#            print summe
        #r = mean([sum(x) for x in experiment.doEpisodes(5)])
#        env.delay = False
#            testagent.reset()
        

#        performance.append(r)
#        if not render:
#            plotPerformance(performance, pf_fig)

#        print "reward avg", r
#        print "explorer epsilon", learner.explorer.epsilon
#        print "num episodes", agent.history.getNumSequences()
#        print "update step", len(performance)




#    #for updates in range(5000000)
#    updates = 0
#    episodes = 10
#    while True:
#        updates += 1
#        #raw_input("next episode")
#        print "lerne episode:",updates
#        experiment.doEpisodes(episodes)
##        print "lernen beendet, starte testlauf"
#        env.reset()
##        if updates > 0:
##            experiment.doInteractions(20)
##            rewsum = 0
##            rewlist = []
##            for i in range (0,100):
##                rew = task.performAction(net.activate(task.getObservation()))
##
##                task.getObservation()
##                rewlist.append(rew)
##                rewsum += rew
#                #print "  testlauf: ",updates,"aktion: ",i+1," reward: ",rew
##            print "-> summe = ",rewsum, " avg: ",rewsum / 100.0
#            #print "episodes:",updates," rewsum: ",rewsum," testrewards:",rewlist
##            #x = "episode:" + updates + " testrewards:" + rewlist
##            #o.write(x)
##            for i in range(0,len(rewlist)):
##                x = (updates % 20) - 10
##                y = i - 10
##                z = rewlist[i]
##                #g.plot((x,y,z),x=1, y=2, z=3)
#
#            #g.doplot()
#
#
#
#        #print "-------------------------------------------------------------------"

    print "finished grabbingTest"