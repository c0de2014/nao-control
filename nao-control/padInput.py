import pygame


# this fits xBox360 wireless Pad connected to USB via USB receiver for PC
class padInput():
    
    # ID 0 for the first available joystick.
    joystickID = 0
    #Joystick object
    stick = None

    # Joystick deadzones
    stickdeadzone = 0.3
    triggerdeadzone = 0.01
    allValues = None
    
    
    
    def connect(self):
        pygame.joystick.init()
        pygame.display.init()
        # check for availability
        if (pygame.joystick.get_count() >= self.joystickID+1):
            # create joystick object
            self.stick = pygame.joystick.Joystick(self.joystickID)
            # init joystick object
            self.stick.init()
            print "Joystick connected:", self.stick.get_name()
        else:
            print "Joystick not available"




    # get all Values
    # polls for new values
    # returns [sticks,trigger,hat,buttons]
    def getAll(self):
        # get new values from driver
        self.poll()
        self.allValues = [self.getSticks(),self.getTrigger(),self.getHat(),self.getButtons()]
        return self.allValues
    


        
    # need to make the driveer poll next values
    def poll(self):
        pygame.event.pump()





    # returns 4-tupel of FLOATS -1.0 .. 1.0
    def getSticks(self):
        # invert first axis
        # axis[2] is the trigger, so we take 0,1,3,4
        # values should already be between -1.0 and 1.0
        values = [-self.stick.get_axis(0) , self.stick.get_axis(1), self.stick.get_axis(4), self.stick.get_axis(3)]
        # filter joystick noise -> deadzone
        for i in range (0,len(values)):
            if (abs(values[i]) < self.stickdeadzone):
                values[i] = 0
        return values

                
                
                
                
    # returns FLOAT -1.0 .. 1.0
    def getTrigger(self):
        value = self.stick.get_axis(2)
        if (abs(value) < self.triggerdeadzone):
            value = 0.0
        return value

            
            
            
    
    # returns 2-tupel INT -1 / 0 / 1
    def getHat(self):
        # not much to do here, since driver returns the desired 2-tupel
        values = self.stick.get_hat(0)
        return [-values[0],values[1]]
        

        
        
        
    # returns 10-tupel INT 0 / 1
    def getButtons(self):
        # 10-tupel for buttons
        buttons = [0,0,0,0,0,0,0,0,0,0]
        # xBox360 Pad has 10 usable Buttons
        # 0 - A      1 - B   2 - X     3 - Y
        # 4 - LB     5 - RB  6 - back  7 - start
        # 8 - LStick 9 - RStick
        for i in range(0,10):
            buttons[i] = self.stick.get_button(i)
        return buttons
        