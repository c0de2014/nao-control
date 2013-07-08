from naoqi import ALProxy
import naoSensors
import naoCamera
import time
import math

class naoControl():
    # Variables
    leftArmEnable  = False
    rightArmEnable  = True
    
    # TODO: bring mappedInput.delay, naoControl.fractionMaxSpeed and Sensitivities together
    #       .. these values influence each other and therefore should be calculated/influence
    #       .. eachother.. bad values cause stuttering movements or "tearing"
    
    # control options
    # higher speed -> faster movements
    # good value for manual control is 0.3 .. with both sensitivities at 30
    fractionMaxSpeed  = 0.3
    
    # Parameter to detect failed moves, due to resistance
    #maxAllowedJointDifference = 0.3 
    maxAllowedJointDifference = 0.2
    
    # higher sensitivity -> higher accuracy
    movesensitivity = 30
    headsensitivity = 30
    
    stiffness = 0.0
    
    # Proxy Options
    #ip = "192.168.0.15"
    ip = "192.168.0.102"
    port = 9559
    motionproxy = None
    behaviorproxy = None
    # this object is available to read nao sensors
    #  .. e.g. call from mappedInput:   nao.sensors.<sensormethod>
    sensors = None
    
    # this object contains Methods to access Nao's Cameras
    camera = None

    # constraints for joint 
    # TODO: could be read with "motionproxy.getLimits()"
    shoulderRollMin = -1.64933
    shoulderRollMax = -0.00872
    shoulderPitchMin = -2.0856
    shoulderPitchMax = 2.0856
    elbowYawMin = -2.0856
    elbowYawMax = 2.0856
    elbowRollMin = 0.00872
    elbowRollMax = 1.562
    headYawMin = -2.0857
    headYawMax = 2.0857
    headPitchMin = -0.6720
    headPitchMax = 0.5149
    
    # 0 - sitting
    # 1 - "walking" / after having walked
    # 2 - fully standing
    robotStanding = 0
    
    # joint names need for motionproxy
    headnames = ['HeadYaw', 'HeadPitch']
    armnames = ['RShoulderRoll', 'RShoulderPitch', 'RElbowYaw', 'RElbowRoll']
    # values to remember the last pose
    armvalues = [0.0,0.0,0.0,0.0]
    headvalues = [0.0,0.0]
   
    
    
    # method to change ip and port
    #  .. ip: STRING
    #  .. port: INT
    def setAddress(self,in_ip,in_port):
        self.ip = in_ip
        self.port = in_port



    # method to create proxies and initialize this object
    # TODO: get nao pose from naoqi and decide start conditions
    #       .. now we need to "stand up" to get stiff joints even if we start
    #       .. with nao already standing..
    def connect(self):
        # create Proxy to give motion commands
        self.motionproxy = ALProxy("ALMotion",self.ip,self.port)
        # create Proxy to call behaviors
        self.behaviorproxy = ALProxy("ALBehaviorManager", self.ip,self.port)        
        # create sensor object and share the motion proxy with it (connect(self))
        self.sensors = naoSensors.naoSensors()
        # give nao object to sensors, since it uses the same motionproxy
        self.sensors.connect(self)
        
        self.camera = naoCamera.naoCamera()
        self.camera.setAddress(self.ip,self.port)
        self.camera.connect()
        
        # read sensors, so that robot can start with current pose
        #  .. second parameter chooses if the value is read from the sensor or the last command
        # THOSE VALUES SHOULD COME FROM sensor object!
        #  .. change when implementation is done
        # SHOULD be OK now (changed to what it should be)
        self.headvalues = self.sensors.getHeadValues()
        #motionproxy.getAngles(self.headnames,True)
        self.armvalues = self.sensors.getRightArmValues()
        #self.motionproxy.getAngles(self.armnames,True)

        # set wich arm is free for balancing during walking
        self.setActiveArm("left")
        ## set Head to initial position
        #self.setHeadAngles(0,0)
        # TODO: better if this was "somewhere else"
        #self.motionproxy.stiffnessInterpolation("Body",1.0,0.5)
        self.motionproxy.stiffnessInterpolation("Head",1.0,0.5)
        print "!! press stand-up button !!"
        print "!!  If robot was already standing at start, just press stand-up button (default = left stick button) once to gain arm control"



    def setSimulatorSettings(self):
        self.maxAllowedJointDifference = 3
        self.fractionMaxSpeed = 0.3

    # method to choose the controllable arm
    #  .."left" or "right"
    # TODO: add mirrored armmovements, otherwise this function won't work correctly
    # CURRENTLY THIS METHOD DOESNT WORK FOR CHANGING THE REAL ACTIVE ARM
    #  .. armvalues need to be mirrored, checked and implemented for moveArm()
    #  .. and openHand() and closeHand()
    def setActiveArm(self,armstring):
        if (armstring == "left"):
            self.leftArmEnable = True
            self.rightArmEnable = False
        else:
            self.leftArmEnable = False
            self.rightArmEnable = True            
        self.motionproxy.setWalkArmsEnable(self.leftArmEnable, self.rightArmEnable)

        
        
    # method to set stiffness of whole robot body
    #  .. stiffnessvalue: FLOAT
    #  .. time: FLOAT
    def setStiffness(self,stiffnessvalue, time=0.5):
        # only allow to raise stiffness if nao is standing to avoid damaging motors by pushing against nao legs or body
        if ((self.robotStanding == 2) and (stiffnessvalue == 1.0)):
            self.stiffness = stiffnessvalue
            self.motionproxy.stiffnessInterpolation("Body",stiffnessvalue,time)
        # only allow lowering stiffness if sitting, to prevent nao from falling
        elif ((self.robotStanding == 0) and (stiffnessvalue == 0.0)):
            self.stiffness = stiffnessvalue
            self.motionproxy.stiffnessInterpolation("Body",stiffnessvalue,time)   
        #allow Head always to be moved
        self.motionproxy.stiffnessInterpolation("Head",1.0,time)


    # TODO: Include choregraphe behavior files & upload, if not available on nao
            
            
    # method to stand up fully, with legs in 'stable' position
    # .. needed after walking
    def stand(self):
        try:
            print "stand called"
            self.behaviorproxy.runBehavior("beineStrecken1")
            # set robotStanding to "fully standing"
            self.robotStanding = 2
            # allow moving arms
            self.setStiffness(1.0)
        except RuntimeError,e:
            print "could not run behavior! (beineStrecken1)"         



    # method to make robot sit
    #  .. this method requires that this behavior is available on the robot!!
    def sitDown(self):
        try:
            print "sitDown called"
            self.behaviorproxy.runBehavior("sitzen1")
            # set robotStanding to "sitting"
            self.robotStanding = 0
            # disable stiffness
            self.setStiffness(0.0)
        except RuntimeError,e:
            print "could not run behavior! (sitzen1)"    

                        

    # method to make robot standUp
    #  .. this method requires that this behavior is available on the robot!!
    def standUp(self):
        try:
            print "standUp called"
            self.behaviorproxy.runBehavior("aufstehen1")
            # set robotStanding to "walking"
            self.robotStanding = 1
            # fully standup <- this should not be neccessary, maybe skip
            self.stand()
        except RuntimeError,e:
            print "could not run behavior! (aufstehen1)"    



    # method to simply change from sitting to standing or vice versa
    def changePose(self):
        # if standing -> sitDown
        if (self.robotStanding == 2):
            self.sitDown()
        # else fully standUp -> beineStrecken1
        else:
            self.standUp()

    def switchCam(self,camID):
        self.videoproxy.setParam(kCameraSelectID,camID)
    
    
    # method to do a sidestep
    #  .. stepwidth: neg -> left
    #  ..            pos -> right
    #  .. value: -1.0..1.0
    #   1.0 -> 4cm
    #   0.5 -> 2cm
    #   0.25 -> 1cm
    #   0.125 -> 5mm
    #   0.0625 -> 2,5mm
    def sideStep(self,stepwidth):
        if (abs(stepwidth) > 0.0):
            # check for constraints, max stepwidth
            if (stepwidth < -1.0):
                stepwidth = -1.0
            elif (stepwidth > 1.0):
                stepwidth = 1.0
            # scale to nao stepsize, -0.4..0.4
            Y = (stepwidth / 100.0) * 4.0
            # no need to step forward or backward
            X = 0.0
            # no need to change direction
            Theta = 0.0
            if (stepwidth != 0.0):
                # step left
                if (stepwidth < 0.0):
                    firstleg = 'RLeg'
                    secondleg = 'LLeg'
                # step right
                elif (stepwidth > 0.0):
                    firstleg = 'LLeg'
                    secondleg = 'RLeg'
                # move first leg
                self.motionproxy.stepTo(firstleg, X, Y, Theta)
                # move second leg
                self.motionproxy.stepTo(secondleg, X, Y, Theta)
            # set robotStanding to "walking"
            self.robotStanding = 1



    # method to do move forward or backward
    #  .. stepwidth: neg -> backward
    #  ..            pos -> forward
    #  .. value: -1.0..1.0
    def moveStraight(self,stepwidth):
        maxstep = 0.03
        if (abs(stepwidth) > 0.0):
            # scale stepwidth from -1..1 to max of 0.03, otherwise nao could stumble
            stepwidth = (stepwidth / 100.0) * 3
            # check for constraints, max stepwidth
            if (stepwidth < -maxstep):
                stepwidth = -maxstep
            elif (stepwidth > maxstep):
                stepwidth = maxstep
            # no turning or sidestepping
            Y = 0.0
            Theta = 0
            # move both legs
            self.motionproxy.stepTo('LLeg', stepwidth, Y, Theta)
            #move second leg only half way, so nao stands with feet next to eachother
            self.motionproxy.stepTo('RLeg', stepwidth/2.0, Y, Theta)
            # set robotStanding to "walking"
            self.robotStanding = 1




    # method to turn left or right
    # TODO: Input is not scaled, since we want to hand over 'NAO-Angles' directly
    #  .. degree: neg -> left
    #  ..         pos -> right
    #  .. from exception: "theta must be in range [+0.349066 <-> -0.349066]"
    #  .. max turn is scaled to 0.5 (whatever this means in degrees) <- CURRENTLY NOT SCALED!
    #  .. input should be radians ( -3.14 .. 3.14 ) but is only working for -0.34 to 0.34
    # naoqi method takes radians (-pi .. +pi)
    # DONE: check again with documentation and nao user forum 
    # -> theta / degree must be less than maxStepX ( -1.0 .. 1.0 )
    def turn(self,degree):
        #print "turn: ", degree
        leftmax = -0.349066
        rightmax = 0.349066 
        if (abs(degree) > 0.0):
            # check for constraints
            if (degree < leftmax):
                degree = leftmax
            elif (degree > rightmax):
                degree = rightmax
            # 2 steps -> each one turns half way
            degree = degree / 2.0
            if (degree < 0.0):
                firstleg = 'LLeg'
                secondleg = 'RLeg'
            else:
                firstleg = 'RLeg'
                secondleg = 'LLeg'
            # small first step backwards ("inner circle")
            # -> without "step" forward or backward its way more stable!
            #X = -0.015
            X = 0.0
            # no sidestep
            Y = 0.0
            # send command
            self.motionproxy.stepTo(firstleg, X, Y, -degree)
            # bigger second step forward ("outer circle")
            # -> without "step" forward or backward its way more stable!
            #X = 0.015
            X = 0.0
            # again no sidestep
            Y = 0.0
            # send command
            self.motionproxy.stepTo(secondleg, X, Y, -degree)
            # set robotStandig to "walking"
            self.robotStanding = 1





    # method to move arm joints stepwise
    #  .. 4 input values: FLOAT  -1.0..1.0
    # TODO: should check for values bigger 0.0, otherwise avoid calling setAngles
    # TODO: maybe set success to False if cutting movement down to constraints.. .. think about that
    def moveArm(self,values):
        #print "called moveArm"
        #print values
        # allows to give more negative reward if movement was unsuccessful 
        # ..only one per joint possible, < and > at the same time should not be possible
        
        # binary coded:
        #  0 - none
        #  1 - joint 0
        #  2 - joint 1
        #  3 - joint 0 + 1
        #  4 - joint 2
        #  5 - joint 2 + 0
        #  6 - joint 2 + 1
        #  7 - joint 2 + 1 + 0
        #  8 - joint 3
        #  9 - joint 3 + 0
        # 10 - joint 3 + 1
        # 11 - joint 3 + 1 + 0
        # 12 - joint 3 + 2
        # 13 - joint 3 + 2 + 0
        # 14 - joint 3 + 2 + 1
        # 15 - joint 3 + 2 + 1 + 0
        # 16 - movement error
        # ...
        
        success = 0        

        # check that 4 input values are given
        if ((len(values)==4) and (self.stiffness != 0.0)):
            # adjust input to sensitivity
            for i in range (0,len(values)):            
                values[i] = values[i] / float(self.movesensitivity)
                if (values[i] > 1.0):
                    values[i] = 1.0
                elif (values[i] < -1.0):
                    values[i]= -1.0
                # calculate new armvalues
                self.armvalues[i] = self.armvalues[i] + values[i]
            #print self.armvalues
            # Check for valid maximum joint angles
            if (self.armvalues[0] < self.shoulderRollMin):
                self.armvalues[0] = self.shoulderRollMin
                success -= 1
                #print "constraint 0 erreicht -> bad move!!"
            if (self.armvalues[0] > self.shoulderRollMax):
                self.armvalues[0] = self.shoulderRollMax
                success -= 1
                #print "constraint 0 erreicht -> bad move!!"
            if (self.armvalues[1] < self.shoulderPitchMin):
                self.armvalues[1] = self.shoulderPitchMin
                success -= 2
                #print "constraint 1 erreicht -> bad move!!"
            if (self.armvalues[1] > self.shoulderPitchMax):
                self.armvalues[1] = self.shoulderPitchMax
                success -= 2
                #print "constraint 1 erreicht -> bad move!!"
            if (self.armvalues[2] < self.elbowYawMin):
                self.armvalues[2] = self.elbowYawMin
                success -= 4
                #print "constraint 2 erreicht -> bad move!!"
            if (self.armvalues[2] > self.elbowYawMax):
                self.armvalues[2] = self.elbowYawMax
                success -= 4
                #print "constraint 2 erreicht -> bad move!!"
            if (self.armvalues[3] < self.elbowRollMin):
                self.armvalues[3] = self.elbowRollMin
                success -= 8
                #print "constraint 3 erreicht -> bad move!!"
            if (self.armvalues[3] > self.elbowRollMax):
                self.armvalues[3] = self.elbowRollMax
                success -= 8
                #print "constraint 3 erreicht -> bad move!!"
                
            
            
            # send command to nao
            #print "arm:", self.armvalues #debug for getting initial armvalues for RL1 reset
            self.motionproxy.setAngles(self.armnames, self.armvalues, self.fractionMaxSpeed)
            
            # wait for movement to finish, before checking success.. BAD, causes stuttering
            
            
            
            # check for successful movement
            #print "checking move success"
            feedback = self.sensors.getRightArmValues()
            

            
            for i in range (0,len(feedback)):
                diff = abs(self.armvalues[i] - feedback[i])
                
                if ( diff > self.maxAllowedJointDifference):
                    #print "                 arm difference (joint ",i," ):", diff
                    self.armvalues[i] = feedback[i]
                    # set armvalues to position that sensors report, so this should be reachable
                    self.motionproxy.setAngles(self.armnames, self.armvalues, self.fractionMaxSpeed)
                    success -= 16
                    
                    #time.sleep(2)
                #else:
                    #print "armmove successful (joint ",i," ):", diff
            return success


    # method to move arm joints directly
    #  .. 4 input values: FLOAT  according to joint constraints
    def moveArmTo(self,values):
        #print "called moveArm"
        
        # allows to give more negative reward if movement was unsuccessful 
        success = 0                
        
        
        # check that 4 input values are given
        if ((len(values)==4) and (self.stiffness != 0.0)):
            # adjust input to sensitivity
            for i in range (0,len(values)):            
                self.armvalues[i] = values[i]

            # Check for valid maximum joint angles
            if (self.armvalues[0] < self.shoulderRollMin):
                self.armvalues[0] = self.shoulderRollMin
                success -= 1
                print "constraint 0 erreicht  <<<  -> bad move!!"
            if (self.armvalues[0] > self.shoulderRollMax):
                self.armvalues[0] = self.shoulderRollMax
                success -= 1
                print "constraint 0 erreicht >>> -> bad move!!"
            if (self.armvalues[1] < self.shoulderPitchMin):
                self.armvalues[1] = self.shoulderPitchMin
                success -= 2
                print "constraint 1 erreicht -> bad move!!"
            if (self.armvalues[1] > self.shoulderPitchMax):
                self.armvalues[1] = self.shoulderPitchMax
                success -= 2
                print "constraint 1 erreicht -> bad move!!"
            if (self.armvalues[2] < self.elbowYawMin):
                self.armvalues[2] = self.elbowYawMin
                success -= 4
                print "constraint 2 erreicht -> bad move!!"
            if (self.armvalues[2] > self.elbowYawMax):
                self.armvalues[2] = self.elbowYawMax
                success -= 4
                print "constraint 2 erreicht -> bad move!!"
            if (self.armvalues[3] < self.elbowRollMin):
                self.armvalues[3] = self.elbowRollMin
                success -= 8
                print "constraint 3 erreicht -> bad move!!"
            if (self.armvalues[3] > self.elbowRollMax):
                self.armvalues[3] = self.elbowRollMax
                success -= 8
                print "constraint 3 erreicht -> bad move!!"
                
            # send command to nao
            #print self.armvalues #debug for getting initial armvalues for RL1 reset
            self.motionproxy.setAngles(self.armnames, self.armvalues, self.fractionMaxSpeed)

            # wait for movement to finish, before checking success.. BAD, causes stuttering
            #print "waiting for arm to finish movement"
            #time.sleep(0.1)


            
            # check for successful movement
            #print "checking move success"
            feedback = self.sensors.getRightArmValues()
            

            for i in range (0,len(feedback)):
                diff = abs(self.armvalues[i] - feedback[i])
                if ( diff > self.maxAllowedJointDifference):
                    #print "                 arm difference (joint ",i," ):", diff
                    #self.armvalues[i] = feedback[i]
                    # set armvalues to position that sensors report, so this should be reachable
                    #self.motionproxy.setAngles(self.armnames, self.armvalues, self.fractionMaxSpeed)
                    success -= 16
                    #time.sleep(2)
                #else:
                    #print "armmove successful (joint ",i," ):", diff
            

            return success

    # experimental method to move arm in XYZ space
    #  .. no orientation control
    #  .. movements inaccurate
    #  .. movements don't check for constraints, e.g. trying to move through the rest of the body
    # DOESNT update armvalues, right now
    # .. use only after applying any needed changes to this and the rest of the code!
    def moveArmXYZ(self,values):
        if (len(values)==3):
            for i in range (0,len(values)):            
                values[i] = values[i] / self.movesensitivity    
            # send command to nao
            # Example showing how to move forward (2cm) "LArm". 
            effectorName     = "RArm" 
            space            = 2 # SPACE_NAO 
            positionChange   = [-values[0], -values[1], values[2], 0, 0, 0.0] 
            axisMask         = 7 
            motionproxy.changePosition(effectorName, space, positionChange, self.fractionMaxSpeed, axisMask)
        
        

    # method to move head directly to given angles
    #  .. 2 values: FLOAT min&max -> variable definitions above
    #  .. speed: FLOAT  speed the move should take
    def setHeadAngles(self,values, speed = 0.3):
        # check that 2 input values are given
        if (len(values)==2):
            self.headvalues = [values[0],values[1]]
            #check for constraints
            if (self.headvalues[0]<self.headYawMin):
                self.headvalues[0]=self.headYawMin
            if (self.headvalues[0]>self.headYawMax):
                self.headvalues[0]=self.headYawMax
            if (self.headvalues[1]<self.headPitchMin):
                self.headvalues[1]=self.headPitchMin
            if (self.headvalues[1]>self.headPitchMax):
                self.headvalues[1]=self.headPitchMax
            self.motionproxy.setAngles(self.headnames, self.headvalues, speed)

    def moveHeadTo(self,values, speed = 0.3):
        self.setHeadAngles(values,speed)


    def turnHeadToBall(self):
        print "turnHeadToBall"
        ball = self.camera.getBallPos()
        ball[0]=(ball[0]-320) / 320.0
        ball[1]=(ball[1]-240) / 240.0
        dist = math.sqrt((0.0-ball[0])**2  + (0.0-ball[1])**2)
        print "  dist:",dist
        while dist > 0.1:
            ball = self.camera.getBallPos()
            ball[0]=(ball[0]-320) / 320.0
            ball[1]=(ball[1]-240) / 240.0
            dist = math.sqrt((0.0-ball[0])**2  + (0.0-ball[1])**2)
            print "  dist:",dist
            
            print "camdist still too far",ball
            self.moveHead([-ball[0]*10,ball[1]*10])
        print "Done!",ball

    def turnToBall(self,dest=0.0):
        print "turnToBall"
        self.turnHeadToBall();

        angle = dest + self.sensors.getHeadValues()[0]
        print "angle:",angle
        while abs(angle) > 0.1:
            print "angle still not good"
            angle = dest + self.sensors.getHeadValues()[0]
            print "angle:",angle
            self.turn(-angle*1.0)
            self.turnHeadToBall();
        print "should face ball", angle

        
    def moveToBall(self,dist=50,angle=0.0):
        print "moveToBall"
        self.turnToBall(angle)
        if dist - self.camera.getBallPos()[2] == 0:
            newdist = 0
        else:
            newdist = float(dist - float(self.camera.getBallPos()[2])) / dist

        print "newdist:",newdist
        while abs(newdist) > 0.01:

            newdist = float(dist - float(self.camera.getBallPos()[2])) / dist
            print "move",newdist*20
            i = 0
            max = 5
            while int(abs(newdist > 0.01)) and i < max:    #in range(0,int(abs(newdist))):
                i += 1
                print "move",newdist*20
                self.moveStraight(newdist*20)
                newdist =float(dist - float(self.camera.getBallPos()[2])) / dist
                #self.turnToBall(0.0)
            self.turnToBall(angle)
            
        self.turnToBall(angle)

        print "should have arrived"
    
    # method to move head stepwise
    #  .. 2 values: FLOAT  -1.0..1.0
    def moveHead(self,values_in):
        values =[]
        for i in range(0,len(values_in)):
            values.append(values_in[i])
            
        #print "movehead:",values
        # check that 2 input values are given
        if (len(values)==2):
            for i in range (0,len(values)):
                # check for input limits
                if (values[i] > 1.0):
                    values[i] = 1.0
                elif (values[i] < -1.0):
                    values[i]= -1.0   
                # scale with sensitivity
                values[i] = values[i] / float(self.headsensitivity)
                # calculate new headvalues
                self.headvalues[i] = self.headvalues[i] + values[i]
            # check for constraints
            if (self.headvalues[0]<self.headYawMin):
                self.headvalues[0]=self.headYawMin
            if (self.headvalues[0]>self.headYawMax):
                self.headvalues[0]=self.headYawMax
            if (self.headvalues[1]<self.headPitchMin):
                self.headvalues[1]=self.headPitchMin
            if (self.headvalues[1]>self.headPitchMax):
                self.headvalues[1]=self.headPitchMax
            # send command to nao
            self.motionproxy.setAngles(self.headnames, self.headvalues, self.fractionMaxSpeed)
            
    # method to try to grab, keep hand closed if success, else reopen hand
    def grab(self):
        self.closeHand()
        time.sleep(1)
        if (self.sensors.grabbedSomething() == False):
            self.openHand()
        #print "grabbing success?", self.sensors.grabbedSomething()

    # method to open hand
    #  .. CURRENTLY ONLY RIGHT HAND 
    def openHand(self):
        self.motionproxy.openHand('RHand')
        self.sensors.openHand = True
    
    
    # method to close hand
    #  .. CURRENTLY ONLY RIGHT HAND 
    # -> returns True if something was grabbed
    #    .. can be checked manually by calling sensors.grabbedSomething()
    def closeHand(self):
        self.motionproxy.closeHand('RHand')
        #self.motionproxy.setAngles(["RHand"],[0.5],1.0)
        self.sensors.openHand = False
        #time.sleep(1)
        #print self.motionproxy.getSummary()    
            
            
            

        
        