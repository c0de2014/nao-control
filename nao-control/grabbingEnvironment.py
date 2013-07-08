# originally by:
__author__ = 'Tom Schaul, tom@idsia.ch'

#from pybrain.utilities import abstractMethod
import naoControl
import time
import os
import ballInteraction
from test2Dagent import world 

import random

#from test2Dagent import world
from pybrain.rl.environments.environment import Environment

#import win32api, win32con

class grabbingEnvironment(Environment):
    """ The general interface for whatever we would like to model, learn about,
        predict, or simply interact in. We can perform actions, and access
        (partial) observations.
    """

    # nao object.. needed to get sensors and perform actions, filled bay connect method!
    nao = None

    # the number of action values the environment accepts
    indim = 2

    # the number of sensor values the environment produces
    outdim = 5#8  ## sens + oldaction + dist

    # discrete state space
    discreteStates = False

    # discrete action space
    discreteActions = False

    # number of possible actions for discrete action space
    numActions = 3
    currentStep = 0
    
    world2d = None

    #world = world()

    def __init__(self,in_nao,world):
        #self.startWebots()
        self.nao = in_nao
        self.world2d = world

        self.reset()


    def getSensors(self):
        """ the currently visible state of the world (the observation may be
            stochastic - repeated calls returning different values)

            :rtype: by default, this is assumed to be a numpy array of doubles
            :note: This function is abstract and has to be implemented.
        """

        #self.sensors = self.nao.sensors.getRightArmValues()


        ball = self.nao.camera.getBallPos()


        #head = self.nao.sensors.getHeadValues()
        #return ([self.sensors[0],self.sensors[1],self.sensors[2],self.sensors[3]])
        #return ([self.sensors[0],self.sensors[1],self.sensors[2]])

        #return self.world.getSensors()
        #return self.sensors[0],self.sensors[1],self.sensors[2],self.sensors[3],ball[0],ball[1],ball[2],head[0],head[1]
        #return ball[0],ball[1],head[0]
        #return [head[0],head[1]]
        return [ball[0],ball[1],ball[2]]#,head[0],head[1]]


    def performAction(self, action):
        a = [0.0,0.0]
        a[0] = action[0]/4.0
        a[1] = action[1]/4.0

        #print "performaction",action, a
        """ perform an action on the world that changes it's internal state (maybe
            stochastically).
            :key action: an action that should be executed in the Environment.
            :type action: by default, this is assumed to be a numpy array of doubles
            :note: This function is abstract and has to be implemented.
        """
        #print "envaction: ",action
        #self.armSuccess = self.nao.moveArm([action[0],action[1],action[2],action[3]])

        #self.nao.moveHead([action[0],action[1]])
        #self.nao.turnHeadToBall()
        #self.nao.turnToBall()
        #self.nao.moveToBall(dist=55,angle=0.5)

        #print "--action before",action
        self.nao.moveHead(a)
        
        #print "--action after",action
        time.sleep(0.05)
        #raw_input("check reward -> enter")

        #self.nao.moveStraight(action[0])
        #self.nao.turn(action[2]/10.0)

        #if (action[4] > 0.0):
        #    self.nao.grab()

        #self.armSuccess = self.nao.moveArm([action[0],action[1],action[2],0.0])
        #time.sleep(0.5)
        #print "out:",action
        #return self.world.doAction(action)



    def reset(self):
        #print "reset environment"
        """ Most environments will implement this optional method that allows for
            reinitialization.
        """

        # create random starting point
        x = random.uniform(-0.5,0.5)
        y = random.uniform(-0.4,-0.2)

        # override random values to have fixed starting point or comment the the 2 lines above
        x = 0.0
        y = -0.3

        self.nao.moveHeadTo([x,y])
        ###-2.0856,2.0856),(-0.6720+ 0.6720,0.5149 + 0.6720)

        #self.nao.moveHeadTo([0.0,-0.3])
        self.world2d.reset(point = (int(750-(x*2*75)),int((400-(y+0.3)*10*75))))
        time.sleep(2)


        #while (self.nao.moveArmTo([-0.05,1.2,0.7,0.3]) < 0):
        #    #print "resetting"
        #    x = 0

        #self.world.reset()




    # only needed for resetting the webots simulator..
    # TODO: better if click goes to reset button, instead of restarting webots!
    #    ..not used for real nao! this needs to be called by self.reset()
    def startWebots(self):
        print "Now I'm killing all running webots"
        result=os.system("taskkill /im webots.exe /f")
        # process result
        if result == 0:
            print "All webots should be death now..."
        else:
            print "Error executing taskkill command !!!"    

        print "start webots"
        #os.system("start notepad.exe")
        os.system("start webots.exe")

        print" waiting 5s.."
        time.sleep(1)    
        print" waiting 4s.."
        time.sleep(1)    
        print" waiting 3s.."
        time.sleep(1)    
        print" waiting 2s.."
        time.sleep(1)    
        print" waiting 1s.."
        time.sleep(1)    
        print "no more waiting, try to click trial message"

        x = 1170
        y = 650

        win32api.SetCursorPos((x,y))
        win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN,x,y,0,0)
        win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP,x,y,0,0)

        time.sleep(3)