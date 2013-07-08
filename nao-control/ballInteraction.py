# module to move nao close to the ball
# here it is done by "dumb" methods.. maybe later everything could/should be done by ML-Module

import naoControl
import time



# TODO: 1 - check for distance:
#           high: turn to ball, and move a few steps to ball, then recheck distance -> (1)
#           low: near enough, get good position ( ball a bit right of nao ) -> (2)
#       2 - check angle to ball:
#           good angle: try to grab
#           bad angle: correct by sidestepping until angle is good -> (2)

# TODO: finish method to position nao next to ball
# TODO add counter for 
def moveToBall(nao):
    angles = turnToBall(nao)
    dist = angles[2]
    
    # value is experimental right now - near enough
    if (dist > 0.237):
        print
        print "Good distance reached!",dist
        print "now move a little to the left, to be able to grab with right hand.."
        nao.sideStep(1.0)
        nao.sideStep(0.5)
        print 
    elif (dist > 0.15):    
        print "not near enough.. moving a little closer (6cm)",dist
        nao.moveStraight(1.0)
        moveToBall(nao)        
    else:
        # move some cm and try again
        print "not near enough.. moving a little closer (6cm)",dist
        nao.moveStraight(1.0)
        nao.moveStraight(1.0)
        moveToBall(nao)
    



def getBallAngles(nao):
    ball = nao.sensors.getBallPosition()
    head = nao.sensors.getHeadValues()

    while ((abs(ball[0]) > 0.02) or (abs(ball[1]) > 0.05)):
        #print "    ",ball
        x = head[0] + ball[0]
        y = head[1] + ball[1]

        # turn head to ball and read values
        nao.setHeadAngles([x,y],0.05)
        time.sleep(1)
        
        head = nao.sensors.getHeadValues()
        ball = nao.sensors.getBallPosition()

    #print "ball @ angles: ", head, "   ballangles relative to head: ",ball
    #print "Distance: ",ball[2]
    
    
    # get 10 values to calculate average, since radius is not stable
    max = 1
    avgradius = 0.0
    for i in range (0,max):
        #print "getRadius:",i+1
        avgradius = avgradius + nao.sensors.getBallPosition()[2]
    avgradius = avgradius / float(max)

    
    #invert head[0] to match turning method
    head[0] = - head[0]
    
    #roundedBallRadius = ball[2] * 100  
    #roundedBallRadius = int(roundedBallRadius) / 100.0
    
    roundedBallRadius = round(avgradius,3)
    roundedBallRadius = avgradius
    
    return [head[0],head[1],roundedBallRadius]
    



# returns "distance" or "radius" -- third value of naoqi redBallDetection!
# TODO: implement invert and scale of "distance"/radius value
# distance is actually a radius or something alike, in fact decreases the farer away the ball is
def turnToBall(nao):
    angles = getBallAngles(nao)
    # distance check
    dist= angles[2]
    
    
    # TODO: check if this works without scaling anything, especially sidestepping probably could be more effective
    # if far away or big angle -> turn, else sidestep
    if ((dist < 0.05) or (abs(angles[0]) > 0.2)):
        # use turn method
        print "turn:",angles[0]
        
        # TODO: turning only accepts -0.34 .. 0.34  so this is max turning degree that will happen.. no problem, since rest will remain, but yet no good style here!
        nao.turn(angles[0])
    else:
        #use sidestep method
        print "sideStep:",angles[0]
        nao.sideStep(-angles[0])
        
    angles = getBallAngles(nao)
    print "result:",angles[0]
    # recursion if angle does not fit yet
    if (abs(angles[0]) > 0.1):
        print "-> trying again"
        return turnToBall(nao)
    else:
        print
        print "Direction SHOULD be OK now"
        return angles

        
    

        