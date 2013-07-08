import naoControl
import padInput
import threading
import time
# this is imported right after pressing button to start learning process
#import grabbingTestNew




# creates padInput and naoControl Objects
# can get pointers to other modules like ballDetection or learning and map input to them
# if neccessary sets proxy adresses
#
# runs the input loop and calls corresponding methods for input made
class mappedInput(threading.Thread):
    
    # polling intervall
    # TODO: add delays after buttons to avoid multiple calls
    delay = 0.05
    pad = None
    nao = None
    
    # @mobileLab, real nao
    ip = "192.168.0.102"
    # @home, simulator
    #ip = "192.168.0.15
    port = 9559
    
    
    def setAddress(self,in_ip,in_port):
        self.ip = in_ip
        self.port = in_port
    
    def connect(self):
        try:
            # connect gamepad
            self.pad = padInput.padInput()
            self.pad.connect()
        except RuntimeError,e:
            print "could not connect gamepad"
            print e
            exit(0)

        try:
            # connect nao
            self.nao = naoControl.naoControl()
            self.nao.setAddress(self.ip,self.port)
            self.nao.connect()
        except RuntimeError,e:
            print "could not connect to nao"
            print e
            exit(0)
        
    def run(self):
        # endless loop for input
        while (True):
            # get all values
            values = self.pad.getAll()
            
            sticks = values[0]
            trigger = values[1]
            hat = values[2]
            buttons = values[3]
            
            # map arm
            self.nao.moveArm(sticks)
            # map sidestep
            self.nao.sideStep(trigger)
            # map head
            self.nao.moveHead(hat)
            
            
            # map buttons
            # A - move forward
            if (buttons[0] == 1):
               self.nao.moveStraight(1)
            # B - move backward
            if (buttons[1] == 1):
                self.nao.moveStraight(-1)
            # X - close Hand
            if (buttons[2] == 1):
                #self.nao.closeHand()
                self.nao.grab()
            # Y - open Hand
            if (buttons[3] == 1):
                self.nao.openHand()
            # LB - turn left
            if (buttons[4] == 1):
                self.nao.turn(-0.3)
            # RB - turn right
            if (buttons[5] == 1):
                self.nao.turn(0.3)
            # back Button - use NaoQi ball detection
            if (buttons[6] == 1):
                # test RL1
                #raw_input(" Enter um testRL1 zu starten")

                # this import opens a pygame window, that is not needed if not testing learning..
                # (.. dirty code)
                import grabbingTestNew
                grabbingTestNew.grabbingTestNew(self.nao).run(5000)

                # only available on real nao (for sim use method = 1), values of both methods are NOT yet normalized!
                #print self.nao.sensors.getBallPosition(method = 0)

            # start Button - use own ball detection"
            if (buttons[7] == 1):

                #self.nao.camera.selectCam(1)
                #ballInteraction.moveToBall(self.nao)

                # use own ball detection
                print self.nao.sensors.getBallPosition(method = 1)
                
            
            # LStick - sit / stand
            if (buttons[8] == 1):
                self.nao.changePose()
            # RStick - print armvalues and switch camera (2 functions, since too few buttons available)
            if (buttons[9] == 1):
                self.nao.camera.switchCam()
                #print self.nao.sensors.getRightArmValues()
            
            
            
            # wait to keep cpu time low
            time.sleep(self.delay)
        
        