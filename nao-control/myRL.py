#from grabbingTask import reward
import test2Dagent
import math, random, time
import SAR
# To change this template, choose Tools | Templates
# and open the template in the editor.

__author__="David"
__date__ ="$10.08.2011 01:35:00$"

class myRL():
    actions = 2
    sensors = 2

    world = test2Dagent.world()


    spur = []
    spur.append(SAR.SAR().get())

    def size(self):
        return len(self.spur)

    def dist(self,x,y):
        dist = 0.0
        for i in range(0,len(x)):
            dist += (x[i]-y[i])**2
        dist = math.sqrt(dist)
        return dist



    def nearest2Entries(self,state,rangeext = 2.0):
        best = None
        bestDist = None
        secondbest = None
        secondbestDist = None

        self.sortSpur(state)
#
#        print "sorted spur?"
#        for i in range(0,len(self.spur)):
#            print "  dist: ",self.dist(state,self.spur[i].state)
#
#
#        for entry in self.spur:
#
#
#            if entry.new == False:
#                #print state, entry.state
#                dist = self.dist(state,entry.state)
#                #print dist
#
#                #
#                if best != None:
#                    print "......"
#                    print bestDist, best.state, best.reward
#                    print dist, entry.state, entry.reward
#                    print "......"
#                if bestDist == None  or  (dist <= bestDist and entry.reward >= best.reward):# and entry.reward > best.reward) :
#                    if best != None  and entry.reward > best.reward:
#                        print "  took better neighbour",entry.action,entry.reward,dist
#                    secondbest = best
#                    secondbestDist = bestDist
#                    bestDist = dist
#                    best = entry
#
#        if best == None:
#            best = SAR.SAR(state=state).get()
#        if secondbest == None:
#            secondbest = SAR.SAR(state=state).get()
        #print best.state, secondbest.state

        best = None
        second = None

        for entry in self.spur:
            if best == None or (self.dist(state,entry.state) < self.dist(state,best.state)*rangeext and entry.reward > best.reward):
#                if second != None:
#                    print "xyxyxx",entry.reward
                second = best
                best = entry

        

        if second == None:

            return best, best
        else:
            #print "best,second", best.state,best.reward,second.state, second.reward
            return best, second
        #return best,secondbest

    def getAction(self,state, streuung = 5.0, frequency = 1.0,rangeext = 2.0):
        #print self.nearest2Entries(state)
        nearest,second = self.nearest2Entries(state,rangeext)
        #print "nearest:",nearest.action
        #print "second: ",second.action

        newaction = []

        if random.uniform(0,1.0) <= frequency:
            pert = random.uniform(-streuung,streuung)
        else:
            pert = 0

        for i in range (0,len(nearest.action)):
            if pert != 0:
                pert = random.uniform(-streuung,streuung)
            #print "obs:",nearest.state,nearest.action,nearest.reward
            #print "obs:",second.state,second.action,second.reward
            x =  (nearest.action[i]*(nearest.reward/float(second.reward)) + second.action[i]*(second.reward/nearest.reward))/2.0
            fact = ((nearest.reward/float(second.reward) + second.reward/nearest.reward)/2.0)

            
            #print "-->" ,(x / fact),pert

            x = (x / fact)+pert
            if x > 1.0: x = 1.0
            elif x < -1.0: x = -1.0

            #if (nearest.action[i]>0 and x <0 or nearest.action[i]<0 and x >0) or (nearest.action[i]>0 and x <0 or nearest.action[i]<0 and x >0):
                #print "switched direction"



            #print x, nearest.reward,float(second.reward),(nearest.reward/float(second.reward))
            #print "newaction:", x
            newaction.append(x)


        #print newaction

        estimatedreward = ( nearest.reward + second.reward ) / 2.0
        #print  "newaction:",newaction, "estimated reward:",estimatedreward
        return newaction, estimatedreward

    rewsum = 0
    traincount = 0

    def doLearning(self,nao,streuung = 1.0, frequency = 1.0, rangeext = 2.0,count=1):
        self.reset(nao)
        samples = []
        self.rewsum = 0
        self.traincount = 0
        for i in range(0,int(count)):
            self.traincount += 1
            state = self.getObservation(nao)
            action = self.getAction(state, streuung, frequency,rangeext)
            reward = self.performAction(action[0], nao, test = False)
            #print "new reward:", reward
            samples.append((state,action[0],None))
            self.rewsum += reward

        sum = 0
        avg = 0
        if len(self.spur) > 0:
            for i in range (len(self.spur)):
                sum+=self.spur[i].reward
            avg = float(sum) / float(len(self.spur))

        for sample in samples:
            self.learn(sample[0],sample[1],self.rewsum/float(count),force=True)


        if self.rewsum/float(count) > avg:
            print "learned ",len(samples)+1, " samples with average reward of ",self.rewsum/float(count), " average known samples reward: ",avg
        else:
            print "nothing better learned"

    def doTestRun(self,nao,count = 1):
        self.reset(nao)
        for i in range(0,count):
            state = self.getObservation(nao)
            action = self.getAction(state, streuung = 0.0)
            rew = self.performAction(action[0],nao,test=True)
            #print rew

    def learn(self,state,action,reward,force=False):
        sum = 0
        new = []
        avg =0
        if len(self.spur) > 0:
            for i in range (len(self.spur)):
                sum+=self.spur[i].reward
            avg = float(sum) / float(len(self.spur))
        if len(self.spur) == 0 or reward >= avg or force == True:
            self.spur.append(SAR.SAR(state=state,action=action,reward=reward).get())
            #print "reward ueber average",avg,reward,len(self.spur), "action:",action
        #else:
            #print "    - reward unter average",avg,reward,len(self.spur)

    ball = None
    head = None
    def getObservation(self,nao):
        self.ball = nao.camera.getBallPos()
        self.head = nao.sensors.getHeadValues()
        return self.ball[0],self.ball[1]#,self.ball[2],self.head[0],self.head[1]
        

    def performAction(self,action,nao, test = True):
        #self.getObservation(nao)
        #print "perform:",self.action
        x = (action[0]/3.0,action[1]/3.0)
        nao.moveHead(x)
        rew = self.getReward(nao)
        #print "AAAAAA",action, rew, len(action)

        #print  rew, action[0],action[1]
        print self.rewsum/self.traincount
        self.world.doAction((action[0],action[1]), (self.rewsum/self.traincount)*10, test)
        #self.world.doAction((10,10), 2, test)
        #print "x perf",x
        #print "action perf",self.action,action
        time.sleep(0.05)
        return rew
        
    def reset(self,nao):
        pert = 0#random.uniform(-0.1,0.1)
        nao.moveHeadTo([0.0+pert,-0.3+pert])
        for entry in self.spur:
            entry.new = False
        self.world.reset()
        time.sleep(3)

    lastdist = None
    def getReward(self,nao):
        self.getObservation(nao)
        dist = math.sqrt((float((self.ball[0]/80.0)-1)**2 + (((self.ball[1]/60.0)-1)**2)))
        if self.lastdist == None:
            self.lastdist = dist

        #print "dist:",dist, self.lastdist
        rew = 0
        if self.ball[0]== 0 and self.ball[1]==0:
            rew += 0
        elif dist <= 0.1:
            rew += 10
        else:

            rew = (1.0/dist) #self.lastdist - dist#
        self.lastdist = dist
        #print "dist",dist,"reward:",rew, "ball",self.ball[0],self.ball[1]
        return rew

    def minimize(self,max = 1000):
        print "total samples:",len(self.spur)
        if len(self.spur)> max:
            sum = 0
            new = []
            
            for i in range (len(self.spur)):
                sum+=self.spur[i].reward
            avg = float(sum) / len(self.spur)
            print "  average reward before:",avg
            for i in range (len(self.spur)):
                if self.spur[i].reward >= avg:
                    new.append(self.spur[i])
            print "  remaining:", len(new)
            self.spur = new
            for i in range (len(self.spur)):
                sum+=self.spur[i].reward
            avg = float(sum) / len(self.spur)
            print "  average reward after:",avg
    action = None

    def sortSpur(self,state):
        sortedSpur = []
        while len(self.spur) > 0:
            bestdist = None
            best = None
            for entry in self.spur:
                dist = self.dist(state, entry.state)
                if bestdist == None or dist < bestdist:
                    best = entry
                    bestdist = dist
            sortedSpur.append(best)
            self.spur.remove(best)


        self.spur = sortedSpur


    def run(self,nao,count = 10,streuung = 0.0,learning = False):
        self.reset(nao)
        self.getObservation(nao)
        self.getReward(nao)
        for i in range(0,count):
            state = self.getObservation(nao)

            #self.getReward()
            self.action,estimated = self.getAction(state,streuung)
            #print "0->",self.action
            x = self.performAction(self.action,nao)
            #time.sleep(2)
            #print "x:", x
            #print "1->",self.action
            self.getObservation(nao)
            #print "2->",self.action
            reward = self.getReward(nao)
            #print "3->",self.action

            if learning == True:
                if reward > 0:
                    print "->",self.action
                    self.learn(state,self.action,reward)
                else:
                    print "invert action"
                    for i in range(0,len(self.action)):
                        self.action[i] = -self.action[i]
                    state = self.getObservation(nao)
                    reward = self.getReward(nao)
                    self.performAction(self.action,nao)
    #                self.getReward(nao)
    #                self.performAction(self.action,nao)
                    self.getObservation(nao)
                    reward = self.getReward(nao)
                    if reward > 0:
                        print "->",self.action
                        self.learn(state,self.action,reward)
            print reward >= estimated,"action:",self.action, "estimated:",estimated,"reward:",reward
        self.minimize()
        
        #self.run(nao)
