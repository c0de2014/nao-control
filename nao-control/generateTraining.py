
import naoControl
import padInput
import ballInteraction
import time
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
import random

from pybrain.structure.modules.module import Module

import pickle


class generateTraining():

    nao = None
    net = None
    pad = None
    env = None
    ds = None
    observations = []
    actions = []

    filename = "network.data"
    filenameds = "dataset.data"


    def runDirectMovements(self,in_nao,in_net, in_env, in_pad):
        

        
        self.nao = in_nao
        self.net = in_net
        self.env = in_env
        self.pad = in_pad
        #self.pad.connect()
        
        load = raw_input("Netz aus Datei laden?")
        
        if load == 'j':
            fileObject = open(self.filename,'r')
            self.net = pickle.load(fileObject)
            fileObject.close()        
        
        
        
        next = "j"
        
        while (raw_input("weiteren trainingsatz aufzeichnen? (n/N zum Beenden)") != 'n'):
            print "--->",next
            #raw_input()
            
            #print "grabbed?", self.nao.sensors.grabbedSomething()

            # TODO: maybe we want to start from random positions everytime.. think about that..
            print "   resetting environment"
            
            randomV = [0,0,0,0]
            
            for i in range (0,len(randomV)):
                randomV[i] = (random.random()*2.0)-1.0
            
            self.nao.openHand()
            
            #self.nao.moveArmTo(randomV)
            self.nao.moveArmTo([-0.3,1.4,0.0,0.0])
        
            raw_input("   Press Enter to begin!")
            
            self.ball = ballInteraction.getBallAngles(self.nao)
        
            newObservation = [self.ball[0],self.ball[1],self.ball[2]]
            print self.ball
            
            while (self.nao.sensors.grabbedSomething() == False):
                #print "move further"
               
                #print "   reading PAD input"
                
                input = self.pad.getAll()
                #print "     ",input
                
                armactions = [input[0][0],input[0][1],input[0][2],input[0][3]]
                
                # do arm action
                #print "   moving arm: ",armactions
                self.nao.moveArm(armactions)
                
#                grabaction = 0.0
                if (input[3][2] == 1):
#                    grabaction = 1.0
                    # do grab action
                    self.nao.grab()

                #print "   waiting for next iteration"
                time.sleep(0.05)

                
                
            newAction = self.nao.sensors.getRightArmValues()
            if ((newAction[0] > 0.0) or (newAction[1] > 0.0) or (newAction[2] > 0.0) or (newAction[3] > 0.0)):
                self.observations.append(newObservation)    
                self.actions.append(newAction)
                

                
        
        
        # reset environment to release arm motors
        self.env.reset()

        if len(self.actions) <= 3:
            print "not enough data"
        else:

            for i in range (0,len(self.actions)):
                print self.observations[i], self.actions[i]



            print "building dataset for NN Training"
            # TODO: first try with simple backprop trainer.. maybe change that!

            ds = SupervisedDataSet(9, 5)
            for i in range (0,len(self.actions)):
                ds.addSample(self.observations[i], self.actions[i])

            print "   got ", len(ds), " samples"

            print "now training network"
            trainer = BackpropTrainer(self.net, ds, verbose = True)

            trainer.trainUntilConvergence(maxEpochs = 200, continueEpochs = 10, validationProportion = 0.5)

            print "Finally finished training"


            save= raw_input("Netz in Datei speichern?")

            if save == 'j':
                print "   save net to file"
                fileObject = open(self.filename,'w')
                pickle.dump(self.net,fileObject)
                fileObject.close()
        
        raw_input("press enter to continue to exploration mode")

        
        
        

        
        
        
        
        
        
        
        
        
        
        
    def runDeltaMovements(self,in_nao,in_ds,in_net, in_env, in_pad,count=100):
        self.nao = in_nao
        self.net = in_net
        self.env = in_env
        self.pad = in_pad
        self.ds = in_ds
        #self.pad.connect()
        
        
        load = raw_input("Datensaetze aus Datei laden?")
        
        if load == 'j':
            #fileObject = open(self.filename,'r')
            #self.net = pickle.load(fileObject)
            #fileObject.close()        
            fileObjectds = open(self.filenameds,'r')
            self.ds = pickle.load(fileObjectds)
            fileObjectds.close()        
            print len(self.ds)," samples loaded from file"
 
            
        
        #next = "j"
        
        while (raw_input("weiteren trainingsatz aufzeichnen? (n/N zum Beenden)") != 'n' ):
            
            #print "--->",next
            #raw_input()
            
            #print "grabbed?", self.nao.sensors.grabbedSomething()

            # TODO: maybe we want to start from random positions everytime.. think about that..
            print "   resetting environment"
            
            randomV = [0,0,0,0]
            
            for i in range (0,len(randomV)):
                randomV[i] = (random.random()*2.0)-1.0
            
            self.nao.openHand()
            self.nao.moveArmTo(randomV)
            #self.nao.moveArmTo([-0.3,1.4,0.0,0.0])
        
            raw_input("   Press Enter to begin!")
            
            print "gettin Ball Angles"
            ball = self.nao.camera.getBallPos()
            head = self.nao.sensors.getHeadValues()
            #ball = ballInteraction.getBallAngles(self.nao)
            i = 0
            input = self.pad.getAll()
            while (self.nao.sensors.grabbedSomething() == False and input[3][0]==0):
                i += 1
                print "get next values"

                
                print "   checking if grabbed something"
                grabbed = 0.0
                if (self.nao.sensors.grabbedSomething() == True):
                    grabbed = 1.0
                
                #ball = ballInteraction.getBallAngles(self.nao)
                
                print "   reading armvalues"
                
                arm = self.nao.sensors.getRightArmValues()
                
                print "      armsensors: ", arm
                
                # with ball
                #newObservation = [arm[0],arm[1],arm[2],arm[3],ball[0],ball[1],ball[2], grabbed]
                
                # without ball
                newObservation = [arm[0],arm[1],arm[2],arm[3], ball[0],ball[1],ball[2],head[0],head[1]]
                
                
                print "   reading PAD input"
                
                input = self.pad.getAll()
                print "     ",input
                
                armactions = [input[0][0],input[0][1],input[0][2],input[0][3]]
                self.nao.sideStep(input[1])

                # do arm action
                print "   moving arm: ",armactions
                self.nao.moveArm(armactions)
                
                grabaction = 0.0
                if (input[3][2] == 1):
                    grabaction = 1.0
                    # do grab action
                    self.nao.grab()
                    
                print "   grabbing? ", grabaction > 0.5
                
                    
                newAction = [armactions[0], armactions[1],armactions[2], armactions[3]]
                
                # only add to list if not all actions are 0!
                
                if ((newAction[0] > 0.0) or (newAction[1] > 0.0) or (newAction[2] > 0.0) or (newAction[3] > 0.0)):
                    self.observations.append(newObservation)    
                    self.actions.append(newAction)
                
                print "   waiting for next iteration"
                time.sleep(0.05)

                
            
        # no more training, so release arm and continue    
        self.nao.moveArmTo([-0.3,1.4,0.0,0.0])

        for i in range (0,len(self.actions)):
            print self.observations[i], self.actions[i]


# TODO: Check for more than samples!


        print "building dataset for NN Training"
        # TODO: first try with simple backprop trainer.. maybe change that!
        if self.ds == None:
            self.ds = SupervisedDataSet(9, 4)
        for i in range (0,len(self.actions)):
            self.ds.addSample(self.observations[i], self.actions[i])

        print "   got ", len(self.ds), " samples"

        save= raw_input("Dataset in Datei speichern?")

        if save == 'j':
            print "   save dataset to file"
            #fileObject = open(self.filename,'w')
            #pickle.dump(self.net,fileObject)
            #fileObject.close()                
            fileObjectds = open(self.filenameds,'w')
            pickle.dump(self.ds,fileObjectds)
            fileObjectds.close() 


        print "now training network"
        trainer = BackpropTrainer(self.net, self.ds, verbose = True)

        trainer.trainUntilConvergence(maxEpochs = 2000, continueEpochs = 5, validationProportion = 0.7)
        #trainer.trainUntilConvergence(continueEpochs = 5, validationProportion = 0.7)

        print "Finally finished training"


        
        
        # reset environment to release arm motors
        #self.env.reset()
        #self.nao.moveArmTo([-0.3,1.4,0.0,0.0])
        randomV = [0,0,0,0]

        for i in range (0,len(randomV)):
            randomV[i] = (random.random()*4.0)-2.0

        self.nao.openHand()
        self.nao.moveArmTo(randomV)
            
        
        
        #raw_input("press enter to continue to exploration mode")
        


        # test the net

        while True:
            randomV = [0,0,0,0]

            raw_input("Next Run --> Enter")
    
            ball = self.nao.camera.getBallPos()
            head = self.nao.sensors.getHeadValues()
            for i in range (0,len(randomV)):
                randomV[i] = (random.random()*2.0)-1.0


            for i in range (0,1000):
                #self.nao.openHand()
                self.nao.moveArmTo(randomV)


                
                arm = self.nao.sensors.getRightArmValues()
                

                newObservation = [arm[0],arm[1],arm[2],arm[3], ball[0],ball[1],ball[2], head[0],head[1]]
                self.env.performAction(self.net.activate(newObservation))

                #time.sleep(0.1)