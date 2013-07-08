
# originally by:
__author__ = 'Tom Schaul, tom@idsia.ch'

from scipy import clip

from pybrain.utilities import abstractMethod
from pybrain.rl.environments import EpisodicTask
#from grabbingEpisodicTask import grabbingEpisodicTask
import math, time
from scipy import tanh, zeros, array, random, sqrt, asarray

class grabbingTask(EpisodicTask):
    """ A task is associating a purpose with an environment. It decides how to evaluate the
    observations, potentially returning reinforcement rewards or fitness values.
    Furthermore it is a filter for what should be visible to the agent.
    Also, it can potentially act as a filter on how actions are transmitted to the environment. """

    epiLen = 50
    #: Discount factor
    #  None to disable
    discount = None

    batchSize = 1

    lastdist = None
    dist = None

    success = False

    world2d = None

    def __init__(self, environment,world):
        self.world2d = world
        """ All tasks are coupled to an environment. """
        EpisodicTask.__init__(self,environment)
        self.env = environment
        self.reward_history = []
        # limits for scaling of sensors and actors (None=disabled)
        #self.actor_limits = [(-1.0,1.0),(-1.0,1.0),(-1.0,1.0),(-1.0,1.0)]
        #self.actor_limits = [(-1.0,1.0),(-1.0,1.0),(-1.0,1.0)]
        #self.actor_limits = [(-1.0,1.0),(-1.0,1.0),(-1.0,1.0),(-1.0,1.0),(-1.0,1.0)]

        self.actor_limits = None
        #self.actor_limits = [(0.0,1.0),(0.0,1.0)]#,(-0.3,0.3),(-0.3,0.3),(-1.0,1.0)]
        
        #self.actor_limits = [(-2.0856,2.0856),(-0.6720+ 0.6720,0.5149 + 0.6720)]

        #self.actor_limits = [(-1.64933,-0.00872)]
        
        #self.actor_limits = [(-2.0856,2.0856),(-0.6720,0.5149)]
        #self.sensor_limits = [(-1.64933,-0.00872),(-2.0856,2.0856),(-2.0856,2.0856),(0.00872,1.562),(0,640),(0,480),(0,300),(-2.0856,2.0856),(-0.6720,0.5149)] # joints, X,Y,radius, head
        #self.sensor_limits = [(0.0,640.0),(0.0,480.0),(-2.0856,2.0856),(-0.6720,0.5149),(0.0,300.0)] # joints, X,Y,radius, head

        #self.sensor_limits = [(-2.0856,2.0856),(-0.6720,0.5149)]#,(0.0,500.0),(-1.0,1.0),(-1.0,1.0)]
        self.sensor_limits = [(0.0,160.0),(0.0,120.0),(0.0,80.0)]#,(-2.0856,2.0856),(-0.6720,0.5149)]

        #self.sensor_limits = [(0.0,640.0),(0.0,480.0),(0.0,300.0)] # joints, X,Y,radius, head


        #self.sensor_limits = [(-1.64933,-0.00872),(-2.0856,2.0856),(-2.0856,2.0856)]
        #self.sensor_limits = [(0,500),(0,400)]
        self.clipping = True
        self.count = 0 #timestep counter

        self.oldAction = zeros(environment.indim, float)

        # needed for distance difference at first run
        self.getObservation()
#    shoulderRollMin = -1.64933
#    shoulderRollMax = -0.00872
#    shoulderPitchMin = -2.0856
#    shoulderPitchMax = 2.0856
#    elbowYawMin = -2.0856
#    elbowYawMax = 2.0856
#    elbowRollMin = 0.00872
#    elbowRollMax = 1.562
#    headYawMin = -2.0857
#    headYawMax = 2.0857
#    headPitchMin = -0.6720
#    headPitchMax = 0.5149
        

    def setScaling(self, sensor_limits, actor_limits):
        """ Expects scaling lists of 2-tuples - e.g. [(-3.14, 3.14), (0, 1), (-0.001, 0.001)] -
            one tuple per parameter, giving min and max for that parameter. The functions
            normalize and denormalize scale the parameters between -1 and 1 and vice versa.
            To disable this feature, use 'None'. """
        self.sensor_limits = sensor_limits
        self.actor_limits = actor_limits

    

    def performAction(self, action):
        #print "perform action",action
        """ A filtered mapping towards performAction of the underlying environment. """
        #print "vor:",action
#        print "oldaction:",self.oldAction
#        print "actor_limits",self.actor_limits
        #self.getReward()
        self.oldAction = action
        if self.actor_limits:
            action = self.denormalize(action)
            #print "nach:",action

#        if self.lastCamDist < 0.1:
#            print "self.lastCamDist",self.lastCamDist
##            print "..would turn now"
#            action[0] = 0
#            action[1] = 0
##            if self.lastHeadDist < 0.1:
##                print "self.lastHeadDist",self.lastHeadDist
##                print "move straight"
##                action[0] = 0
##                action[1] = 0
##                action[3] = 0
##            else:
#            print "turn"
#            action[0] = 0
#            action[1] = 0
##            action[2] = 0
#        else:
#            print "move head"
#            action[2] = 0
##            action[3] = 0

        #print "action:",action
        #print "     step: ", self.count, " action: ",action, " sensors of last action: ",self.env.sensors, self.env.observation

        a = [0.0,0.0]
        a[0] = (action[0]*2.0)-1.0
        a[1] = (action[1]*2.0)-1.0

        #a[0] = action[0]
        #a[1] = action[1] - 0.6720

        a[0] = a[0]
        a[1] = a[1]# - 0.6720

        #print "env a :",a
        self.success = self.env.performAction(a)
        #time.sleep(2)
        self.addReward()
        #print self.reward
        thickness = self.reward**2
        #print self.reward, thickness
        self.world2d.doAction(a,round(thickness,0))
        self.samples += 1
        #return self.getReward()

    reward = 0
        
    sensors = None

    def getObservation(self):
#        print "##new Observation!"
        """ A filtered mapping to getSample of the underlying environment. """
        self.sensors = self.env.getSensors()
        #print "orig sens:",self.sensors



        if self.sensor_limits:
            self.sensors = self.normalize(self.sensors)
            #print "normalized sensors:",self.sensors

        
        self.lastdist= self.dist
        
        self.dist = math.sqrt((self.sensors[0])**2 + (self.sensors[1])**2 )

        if self.lastdist == None:
            self.lastdist = 0

#        print self.sensors
#        print self.oldAction
#        print self.dist


        list = []

        for i in range(0,len(self.sensors)):
            list.append(self.sensors[i])
        list.append(self.lastdist)
        for a in self.oldAction:
            list.append(a)
#        print list
        return list



    lastCamDist = None
    lastHeadDist = None

    def getReward(self):
        """ Compute and return the current reward (i.e. corresponding to the last action performed) """
        
        # sim code
        #sensors = self.env.nao.sensors.getRightArmValues()
        
        #sensors = self.getObservation()
        #dist = math.sqrt((self.sensors[0])**2)
        #print self.sensors
        #dist = math.sqrt((sensors[0])**2 + (sensors[1])**2 )
        reward =0
        #print "dist",self.dist
        #print "sensors:",self.sensors

        dist1 = math.sqrt((-0.1-self.sensors[0])**2 + (0.2-self.sensors[1])**2)
        dist2 = math.sqrt((0.1-self.sensors[0])**2 + (0.2-self.sensors[1])**2)
        dist3 = math.sqrt((-0.2-self.sensors[0])**2 + (0.2-self.sensors[1])**2)
        dist4 = math.sqrt((0.2-self.sensors[0])**2 + (0.2-self.sensors[1])**2)
        #disty = abs(-0.2-self.sensors[1])

        if (self.sensors[0]==-1 and self.sensors[1]==-1):#and self.sensors[2]==-1): # and sensors[2]==0) or (sensors[2] <  -0.9):
            #print "ball lost, reward -> -1000000"
            reward = -0
#            return reward
        elif (dist1 < 0.1) or (dist2 < 0.1) or (dist3 < 0.1) or (dist4 < 0.1):#and self.sensors[2]==-1): # and sensors[2]==0) or (sensors[2] <  -0.9):
            #print "ball lost, reward -> -1000000"
            reward = 0
#        elif dist < 0.1:
#            reward = 1000
#        else:
#            reward = -dist*1000.0
            #print reward
        elif self.dist < 0.1:
            reward = 2
        elif self.dist < 0.2:
            reward = 1
        #elif self.dist < 0.3:
        #    reward = 0.01
        #elif self.dist < 0.5:
        #    reward = 0.001
            #print reward
        else:
            
            #reward = (self.lastdist - self.dist)*1000
            #reward = (1.0 / self.dist)/10.0
            reward = 0
            #print reward, "= 1.0 / ",dist
        

        #print "     reward #",self.count,":",reward
        self.reward = reward
        return reward


    def normalize(self, sensors):
        """ The function scales the parameters to be between -1 and 1. e.g. [(-pi, pi), (0, 1), (-0.001, 0.001)] """
        #print self.sensor_limits
        #print
        #print sensors
        
        assert(len(self.sensor_limits) == len(sensors))
        result = []
        for l, s in zip(self.sensor_limits, sensors):
            if not l:
                result.append(s)
            else:
                result.append((s - l[0]) / (l[1] - l[0]) * 2 - 1.0)
        if self.clipping:
            clip(result, -1, 1)
        return asarray(result)

    def denormalize(self, actors):
        """ The function scales the parameters from -1 and 1 to the given interval (min, max) for each actor. """
        #print len(self.actor_limits),self.actor_limits
        #print len(actors),actors

        assert(len(self.actor_limits) == len(actors))
        result = []
        for l, a in zip(self.actor_limits, actors):
            if not l:
                result.append(a)
            else:
                r = (a + 1.0) / 2 * (l[1] - l[0]) + l[0]
                if self.clipping:
                    r = clip(r, l[0], l[1])
                result.append(r)
        #print actors,"->",result
        return result

    @property
    def indim(self):
        return self.env.indim 

    @property
    def outdim(self):
        return self.env.outdim

    def isFinished(self):
        #returns true if episode timesteps has reached episode length and resets the task
        
        if self.count >= self.epiLen:
            self.count = 0
            self.reward_history.append(self.getTotalReward())
            self.env.currentStep = 0
            self.env.reset()
            self.oldAction = zeros(self.env.indim, float)
            self.getObservation()
            self.getObservation()
            return True
        else:
            self.count += 1
            self.env.currentStep = self.count
            return False