from naoqi import ALProxy
import time

#import cv
#import numpy
#import Image
#import ImageDraw


class naoSensors():
    
    motionproxy = None
    headnames = ['HeadYaw', 'HeadPitch']
    armnames = ['RShoulderRoll', 'RShoulderPitch', 'RElbowYaw', 'RElbowRoll']
    redballproxy = None
    memproxy = None
    openHand = True
    nao = None

    
    def connect(self,nao):
        self.nao = nao
        self.motionproxy = nao.motionproxy
        # exception needed needed to be able to use this code with webots, which does not support redballdetection
        # TODO: add class for own balldetection, that will be loaded when using naoqi redballdetection fails
        #       .. use opencv, see old localization module
        #       ! IMPORTANT, methods of second implementation MUST be the same as in redballdetection, so this needs no changes at other places!
        try:
            self.redballproxy = ALProxy("ALRedBallDetection", nao.ip, nao.port)
            self.memproxy = ALProxy("ALMemory",nao.ip,nao.port)
            # this might not be the best way to start the naoqi redballdetection
            # .. after subscribing the nao head processor searches and calculates the
            # .. ballposition over and over again, <- and very likely will get hot
            self.redballproxy.subscribe("detector")
        except RuntimeError,e:
            print e

        
        
    def getRightArmValues(self):
        return self.motionproxy.getAngles(self.armnames,True)
      
        
        
        
    # wrapper to call the chosen function to detect Ball position
    # returns [vertical angle, horizontal angle, distance]
    #  .. method = 0 -> naoqi.redballdetection
    #  .. method = 1 -> opencv.HoughCircles
    #  .. method = 2 -> other implementation
    # method 0 has a delay of 0.5 second (500ms), since the naoqi method
    # should be called with a delay of at least 60ms
    def getBallPosition(self,method=0):
        # delay to avoid multiple calls from one button click
        time.sleep(0.05)
        data = None
        # naoqi method
        if (method == 0):
            # delay to make sure nao has new values
            # ..(head processor might get hot with lower values)
            time.sleep (0.45)
            data = self.memproxy.getData("redBallDetected")
            # data[0] contains time stamp
            # data[1] contains ball position
            # data[2] contains camera info

            if (data != None):
                return [data[1][0],data[1][1],data[1][2]]
            else:
                return [0,0,0]
        # basic alternative method (only for optimized scenery! ..ONLY 1 red ball/object!)
        elif (method == 1):
            return self.nao.camera.getBallPos()







        
    def getHeadValues(self):
        return self.motionproxy.getAngles(self.headnames,True)


    # return True if Hand is open by some degrees
    # .. empty Hand -> about 0.012, sometimes more, if hand movement is not 100% finished or nao tried to close twice
    # .. bigger than 0.4 -> something grabbed = True
    def grabbedSomething(self):
        hand = "RHand"
        handsensor = self.motionproxy.getAngles(hand,True)
        grabbed = False
        #print handsensor[0]
        if ((handsensor[0] > 0.4) and (self.openHand == False)):
            #print "grabbed something"
            grabbed = True
        else:
            #print "grabbed nothing"
            grabbed = False
        return grabbed
    
    
    # TODO: other sensors:
    # - robot pose
    # - ultraschall
    # - force sensors (feet)
    # - touch sensors (legs & hands)
    # - maybe microphones
    # - etc..
    
    