import pygame, sys, time, random
from pygame.locals import *
# set up pygame
import os, math


class world():
    #os.environ['SDL_VIDEO_CENTERED'] = '1'
    #os.environ['SDL_VIDEO_WINDOW_POS'] = str(400) + "," + str(250)
    os.environ['SDL_VIDEO_WINDOW_POS'] = str(0) + "," + str(0)

    pygame.init()

    x = 1500
    y = 800

    # set up the window
    windowSurface = pygame.display.set_mode((x, y),0, 32)#pygame.RESIZABLE|pygame.NOFRAME, 32)
    pygame.display.set_caption('Hello world!')

    count = 0

    # set up the colors
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 50, 50)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    COLOR = (200,10,10)
    # set up fonts
    #basicFont = pygame.font.SysFont(None, 48)
    # set up the text
    #text = basicFont.render('Hello world!', True, WHITE, BLUE)
    #textRect = text.get_rect()
    #textRect.centerx = windowSurface.get_rect().centerx
    #textRect.centery = windowSurface.get_rect().centery

    # draw the white background onto the surface
    
    windowSurface.fill(BLACK)
    pygame.draw.rect(windowSurface, (0, 255, 0), (745, 395 , 10,10 ),2)
    pygame.display.update()
    ## draw a green polygon onto the surface
    #pygame.draw.polygon(windowSurface, GREEN, ((146, 0), (291, 106), (236, 277), (56, 277), (0, 106)))
    # draw some blue lines onto the surface
    #pygame.draw.line(windowSurface, BLUE, (60, 60), (120, 60), 4)
    #pygame.draw.line(windowSurface, BLUE, (120, 60), (60, 120))
    #pygame.draw.line(windowSurface, BLUE, (60, 120), (120, 120), 4)


    currentX = x/2
    currentY = x/2
    COLORLIST = None



    def reset(self,randomres = False, full = False, point = (750,400)):

        self.COLORLIST = [self.COLOR[0],self.COLOR[1],self.COLOR[2]]

        #if self.COLORLIST[1] < 105:
        #    self.C2 = +25
        #    if self.COLORLIST[0] < 105:
        #if self.COLORLIST[1] >= 150:
        #    self.C2 = -25
        #self.COLORLIST[1] += self.C2

        for i in range(0,3):
            self.COLORLIST[i] = int(round(random.uniform(1,25),0))*10+5
            

        self.COLOR = (self.COLORLIST[0],self.COLORLIST[1],self.COLORLIST[2])

        if randomres:
            self.currentX = random.uniform(5,1495)
            self.currentY = random.uniform(5,795)
        else:
            #self.currentX = self.x/2
            #self.currentY = self.y/2
            self.currentX = point[0]
            self.currentY = point[1]

        
        # never true -> keep all
        if self.count < -150:
            full = True

        if full == True:

            self.windowSurface.fill(self.BLACK)
            self.count = 0
        #print self.currentX
        #print self.currentY


        pygame.draw.rect(self.windowSurface, (0, 255, 0), (745, 395 , 10,10 ),2)
        pygame.display.update()

#        background = pygame.Surface(self.windowSurface.get_size())
#        background = background.convert()
#        background.fill((0, 0, 0))
#        pygame.draw.rect(background, (0, 255, 0), (745, 395 , 10,10 ),2)
#        self.windowSurface.blit(background, (0, 0))
#        #allsprites.draw(screen)
#        pygame.display.flip()

    C2 = +1
    C3 = +1
    
    test = None

    drawtest = False

    def doAction(self,action, width, test = False):
        if self.drawtest == False:
            self.test = test
        else:
            self.test = self.drawtest
        #print "action:",action
        self.COLORLIST = [self.COLOR[0],self.COLOR[1],self.COLOR[2]]


#        if self.COLORLIST[2] < 50:
#            self.C3 = +1
#        if self.COLORLIST[2] == 255:
#            self.C3 = -1
#        self.COLORLIST[2] += self.C3
#        elif self.COLORLIST[2] == 255 and self.COLORLIST[1] < 255:
#            self.COLORLIST[1] += C2
#            self.C3 = -1
#            self.COLORLIST[2] += C3
#        elif self.COLORLIST[2] == 255 and self.COLORLIST[1] == 255:
#            self.COLORLIST[1] = 10
#            self.COLORLIST[2] = 10

#        for i in range(0,3):
#            #self.COLORLIST[i] = int(round(random.uniform(1,25),0))*10+5
#            self.COLORLIST [i] = int(round(width*50))+50
#            #print self.COLORLIST[i]
        x=25
        if self.test == True:
            self.COLORLIST[0] = int(round(width*x))
            self.COLORLIST[1] = 0#int(round(width*x))
            self.COLORLIST[2] = 255 - int(round(width*x))
        else:
            self.COLORLIST[2] = 10#128 - (int(round(width*x))/2)
            self.COLORLIST[1] = 30 + (round(width*(x+1)))#0#int(round(width*x))
            self.COLORLIST[0] = 10#int(round(width*x))/2


        if self.COLORLIST [2] > 255:
            diff = self.COLORLIST [2] - 255
            #print "color:",self.COLORLIST, "diff:", diff
            if test == False:
                self.COLORLIST [2] = 255
            self.COLORLIST [2] = 255
            
        if self.COLORLIST [0] > 255:
            self.COLORLIST [0] = 255
        if self.COLORLIST [1] > 255:
            self.COLORLIST [1] = 255


        if self.COLORLIST[0]>255:
            #print "self.COLOR[0]>255"
            self.COLORLIST[0] = 255
        if self.COLORLIST[1]>255:
            #print "self.COLOR[1]>255"
            self.COLORLIST[1] = 255
        if self.COLORLIST[2]>255:
            #print "self.COLOR[2]>255"
            self.COLORLIST[2] = 255


        self.COLOR = (self.COLORLIST[0],self.COLORLIST[1],self.COLORLIST[2])


        
        a = [0.0,0.0]

        if action[0] < -1.0:
            a[0] = -1.0

        elif action[0] > 1.0:
            a[0] = 1.0
        else:
            a[0] = action[0]
        if action[1] < -1.0:
            a[1] = -1.0
        elif action[1] > 1.0:
            a[1] = 1.0
        else:
            a[1] = action[1]
        #print "->",(a[0],-a[1])

        action = (a[0],-a[1])
        #print "  ",action
        success = True
        if self.currentX + action[0] < 0:
            a[0] = 0
            success = False
        if self.currentX + action[0] > 1500:
            a[0] = 0
            success = False
        if self.currentY + action[1] < 0:
            a[1] = 0
            success = False
        if self.currentY + action[1] > 800:
            a[1] = 0
            success = False
        action = (a[0],-a[1])
        #print "in:", action[0],action[1]
        #print int(round(action[0],0)),int(round(action[1],0))


        self.oldx = self.currentX
        self.oldy = self.currentY

        #self.currentX = self.currentX + int(round(action[0]*7.0/100.0*100*3.3,0))
        #self.currentY = self.currentY + int(round(action[1]*3.75/100.0*100*3.3,0))
        self.currentX = self.currentX + round(action[0]*5,0)
        self.currentY = self.currentY + round(action[1]*5,0)

        #print self.currentX
        #print self.currentY
        if self.test == True:
            
            self.drawAction((round(self.currentX,0)),(round(self.currentY,0)),math.sqrt(width)*2+1,self.test)
        else:
            self.drawAction((round(self.currentX,0)),(round(self.currentY,0)),1,self.test)

        
        pygame.display.update()

    def getSensors(self):
        #print "sensors:",self.currentX,self.currentY
        return [self.currentX,self.currentY]

    def drawAction(self,x,y,width=1, test = False):
        #print "before action:",self.currentX,self.currentX
        #width=1
        if width > 0:
            if test == test:
                pygame.draw.line(self.windowSurface, self.RED, (self.oldx, self.oldy), (self.currentX, self.currentY),width+4)
                pygame.display.update()
                time.sleep(0.01)


            #print self.COLOR

            pygame.draw.line(self.windowSurface, self.BLACK, (self.oldx, self.oldy), (self.currentX, self.currentY),width+4)
            pygame.draw.line(self.windowSurface, self.COLOR, (self.oldx, self.oldy), (self.currentX, self.currentY),width+3)
            pygame.display.update()
        #self.currentX = self.currentX + x
        #self.currentY = self.currentY + y
        #print "after action:",self.currentX,self.currentX

