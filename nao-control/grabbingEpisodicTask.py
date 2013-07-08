__author__ = 'Tom Schaul, tom@idsia.ch'

from scipy import power

from pybrain.utilities import abstractMethod
from pybrain.rl.environments.task import Task
from pybrain.rl.agents.agent import Agent
from pybrain.structure.modules.module import Module
from pybrain.rl.environments.fitnessevaluator import FitnessEvaluator
from pybrain.rl.experiments.episodic import EpisodicExperiment


class grabbingEpisodicTask(Task, FitnessEvaluator):
    """ A task that consists of independent episodes. """

    # tracking cumulative reward
    cumreward = 0

    # tracking the number of samples
    samples = 0

    #: Discount factor
    #  None to disable
    discount = 0.99

    batchSize = 1

    myminr = None
    
    mymaxr = None
    
    mycount = 0

    def reset(self):
        """ Re-initialize the environment """
        # Note: if a task needs to be reset at the start, the subclass constructor
        # should take care of that.
        self.env.reset()
        self.cumreward = 0
        self.samples = 0

    def isFinished(self):
        """ Is the current episode over? """
        abstractMethod()

    def performAction(self, action):
        """ Execute one action. """
        Task.performAction(self, action)
        self.addReward()
        self.samples += 1

    def addReward(self):
        """ A filtered mapping towards performAction of the underlying environment. """
        # by default, the cumulative reward is just the sum over the episode
        print "my addreward"
        if self.discount:
            #original
            #self.cumreward += power(self.discount, self.samples) * self.getReward()
            #exp = self.epiLen - self.samples
            #print "exp:", exp
            exp = self.samples
            self.cumreward += power(self.discount, (exp)) * self.getReward()
            
            #x = self.getReward()
            #print x
            #self.cumreward *= self.discount
            #self.cumreward += x
            #print self.cumreward
        else:
            self.cumreward += self.getReward()
            

    def getTotalReward(self):
        """ Return the accumulated reward since the start of the episode """
        
        self.mycount += 1
        if self.myminr == None: self.myminr = self.cumreward
        if self.mymaxr == None: self.mymaxr = self.myminr
        if (self.cumreward < self.myminr): self.myminr = self.cumreward
        if (self.cumreward > self.mymaxr): self.mymaxr = self.cumreward
        
        print "########### gesamt reward:", self.cumreward, "   [min:", self.myminr, "   max:", self.mymaxr, "]   Anzahl Versuche:",self.mycount
        return self.cumreward

    def f(self, x):
        """ An episodic task can be used as an evaluation function of a module that produces actions
        from observations, or as an evaluator of an agent. """
        r = 0.
        min = None
        max = None
        for _ in range(self.batchSize):
            if isinstance(x, Module):
                x.reset()
                self.reset()
                while not self.isFinished():
                    action = x.activate(self.getObservation())
                    #print "generating new action", action
                    self.performAction(action)
            elif isinstance(x, Agent):
                EpisodicExperiment(self, x).doEpisodes()
            else:
                raise ValueError(self.__class__.__name__+' cannot evaluate the fitness of '+str(type(x)))
            
            rew = self.getTotalReward()
            
            if min == None or rew < min: min = rew
            if max == None or rew > max: max = rew
            r += rew
            
        #print (r / float(self.batchSize)),"/",min,"/", max,"(avg,min,max)"
        #self.env.world.reset()
        return r / float(self.batchSize)
