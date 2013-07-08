__author__ = 'Frank Sehnke, sehnke@in.tum.de, Tom Schaul'

from scipy import ones, random

from pybrain.auxiliary import GradientDescent
from grabbingFiniteDifferences import FiniteDifferences

import time

class grabbingPGPE(FiniteDifferences):
    """ Policy Gradients with Parameter Exploration (ICANN 2008)."""
    
    #:exploration type
    exploration = "local"
    #: specific settings for sigma updates
    learningRate = 0.2
    #: specific settings for sigma updates
    sigmaLearningRate = 0.1
    #: Initial value of sigmas
    epsilon = 2.0
    #:lasso weight decay (0 to deactivate)
    wDecay = 0.0
    #:momentum term (0 to deactivate)
    momentum = 0.0
    #:rprop decent (False to deactivate)
    rprop = False

# fd parameter
    deltamax = 1.0
    
    def _additionalInit(self):
        if self.sigmaLearningRate is None:
            self.sigmaLearningRate = self.learningRate    
        self.gdSig = GradientDescent()
        self.gdSig.alpha = self.sigmaLearningRate
        self.gdSig.rprop = self.rprop
        
        self.sigList = ones(self.numParameters) * self.epsilon #Stores the list of standard deviations (sigmas)
        #random init of sigmas
        #self.sigList = ones(self.numParameters) *random.uniform(0.6,2.0) * self.epsilon #Stores the list of standard deviations (sigmas)
        
        self.gdSig.init(self.sigList)
        self.baseline = None
        
    def perturbation(self):
        """ Generate a difference vector with the given standard deviations """
        #print self.sigList
        #print "_*_*_*_*_*_*_*_"
        #raw_input("Press Enter to continue")
        #time.sleep(3)

        
    
        return random.normal(0., self.sigList)

          
            
            
    def _learnStep(self):
        """ calculates the gradient and executes a step in the direction
            of the gradient, scaled with a learning rate alpha. """
        deltas = self.perturbation()
        #print deltas
        #reward of positive and negative perturbations
        #print "pos pert"
        reward1 = self._oneEvaluation(self.current + deltas)        
        #print "neg pert"
        reward2 = self._oneEvaluation(self.current - deltas)

        self.mreward = (reward1 + reward2) / 2.                
        if self.baseline is None: 
            # first learning step
            self.baseline = self.mreward
            fakt = 0.
            fakt2 = 0.          
        else: 
            #calc the gradients
            if reward1 != reward2:
                #gradient estimate alla SPSA but with likelihood gradient and normalization
                fakt = (reward1 - reward2) / (2. * self.bestEvaluation - reward1 - reward2) 
            else: 
                fakt=0.
            #normalized sigma gradient with moving average baseline
            norm = (self.bestEvaluation-self.baseline)
            if norm != 0.0:
                fakt2=(self.mreward-self.baseline)/(self.bestEvaluation-self.baseline)
            else:
                fakt2 = 0.0
            
        #update baseline        
        self.baseline = 0.9 * self.baseline + 0.1 * self.mreward             
        
        #print "old values:", self.current
        # update parameters and sigmas
        self.current = self.gd(fakt * deltas - self.current * self.sigList * self.wDecay)

        #print "new values:", self.current

        if fakt2 > 0.: #for sigma adaption alg. follows only positive gradients
            #print "positiver gradient:",fakt2
            if self.exploration == "global":         
                #apply sigma update globally        
                self.sigList = self.gdSig(fakt2 * ((deltas ** 2).sum() - (self.sigList ** 2).sum())
                                          / (self.sigList * float(self.numParameters)))
                #print "sigList (global expl) veraendert:", self.sigList
            elif self.exploration == "local":
                #apply sigma update locally
                self.sigList = self.gdSig(fakt2 * (deltas * deltas - self.sigList * self.sigList) / self.sigList) 
                #print "sigList (local expl) veraendert (laenge:",len(self.sigList),"):", self.sigList
            elif self.exploration == "cma":
                #I have to think about that - needs also an option in perturbation
                raise NotImplementedError()
            else:
                raise NotImplementedError(str(self.exploration) + " not a known exploration parameter setting.")
