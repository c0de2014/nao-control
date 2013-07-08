import time
from naoqi import ALProxy

import cv
#import numpy
from numpy import *
import Image
#import ImageDraw

#from pybrain.auxiliary import kmeans

import math
from PIL import ImageChops


class naoCamera():
    
    ip = "192.168.0.102"
    port = 9559
    
    videoproxy = None
    # GVM
    eyes = None
    # value to choose cameraparameter (needed to change the cam, camID selects the camera)
    kCameraSelectID = 18    
    # this selects the camera
    camID = 0    
    
    # Parameters
    # needed for convertion of naoqi image to PIL Image, see old localization Module
    pilcolorspace = "RGB"
    # Resolution requested. { 0 = kQQVGA, 1 = kQVGA, 2 = kVGA }
    gvmresolution = 0
    # Colorspace requested. { 0 = kYuv, 9 = kYUV422, 10 = kYUV, 11 = kRGB, 12 = kHSY, 13 = kBGR }
    gvmcolorspace = 11
    # Fps (frames per second) requested. { 5, 10, 15, 30 }
    gvmfps = 30
    # cameraID
    
    
    def setAddress(self,in_ip,in_port):
        self.ip = in_ip
        self.port = in_port
        
    def connect(self):
        self.videoproxy = ALProxy("ALVideoDevice", self.ip, self.port)
        # disconnect any previously connected GVM, nao only accepts 8 of them
        # .. if this throws an error if no GVM called eyes is connected.. try&catch this command!
        self.videoproxy.unsubscribe("eyes")
        self.eyes = self.videoproxy.subscribe("eyes", self.gvmresolution, self.gvmcolorspace, self.gvmfps)
        
    def selectCam(self,ID):
        self.camID = ID
        self.videoproxy.setParam(self.kCameraSelectID,self.camID)
        
    def switchCam(self):
        time.sleep(0.05)
        self.camID = (self.camID + 1) % 2
        self.videoproxy.setParam(self.kCameraSelectID,self.camID)
        
    def disconnect(self):
        self.videoproxy.unsubscribe("eyes")
        self.videoproxy = None


    size = None

    def getPILImage(self):
        if self.videoproxy != None and self.eyes != None:
            naoImage = self.videoproxy.getImageRemote(self.eyes)

            imageWidth=naoImage[0]
            imageHeight=naoImage[1]
            imageByteArray=naoImage[6]
            self.size = (imageWidth,imageHeight)
            
            #convert Nao Image to PIL Image
            im = Image.fromstring('RGB',self.size,imageByteArray)

            #Debug Code
            #im.save("PIL_Image.jpg")


            return im


    def getCVImage(self,im):
        # BGR2RGB Flip Method 1
        #data = numpy.asarray(im)
        #im = Image.fromarray(numpy.roll(data, 1, axis=-1))

        # BGR2RGB Flip Method 2
        #r, g, b = im.split()
        #print b
        #print g
        #print r
        #imempty = Image.new('RGB', size, (0,0,0))
        #re, ge, be = imempty.split()
        #im = Image.merge("RGB", (b, g, r))

        # Analysis shows:
        #  OpenCV interprets incoming Image as BGR

        # convert PIL Image to OpenCV Image
        cv_im = cv.CreateImageHeader(im.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_im, im.tostring())

        # IMPORTANT! ohterwise OpenCV reads blue as red and vice versa
        #cv.CvtColor(cv_im, cv_im, cv.CV_BGR2RGB)

        # Debug Code
        #cv.SaveImage("cv_im.jpg", cv_im)

        # IMPORTANT! ohterwise OpenCV reads blue as red and vice versa
        cv.CvtColor(cv_im, cv_im, cv.CV_BGR2RGB)


        return cv_im


    def redFilter(self,cv_im, color = "red"):

        # generate HSV Image
        hsv_frame = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 3)
        thresholded = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 1)

        thresholded2 = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 1)
        # check colorspace!!!!

        # SOLUTION:
        # Hue: 0..179
        # Saturation: 0..255
        # Brightness: 0..255

        #  USE THESE NOW FOR TESTING THRESH2

        if (color == "blue"):
            # blue
            hsv_min = cv.Scalar(100, 0, 0, 0)
            hsv_max = cv.Scalar(130, 255 , 255, 255)
            hsv_min2 = cv.Scalar(0, 0, 0, 0)
            hsv_max2 = cv.Scalar(0, 0 , 0, 0)

        if (color == "green"):
            # green
            hsv_min = cv.Scalar(40, 0, 0, 0)
            hsv_max = cv.Scalar(80, 255 , 255, 255)
            hsv_min2 = cv.Scalar(0, 0, 0, 0)
            hsv_max2 = cv.Scalar(0, 0 , 0, 0)

        if (color == "red"):
            # [0] is color (hsv-angle) 0..179
            # [1] is saturation 0..255
            # [2] is brightness 0..255
            # hopefully red
            # WORKS! =)
            hsv_min = cv.Scalar(0, 40, 40, 0)
            hsv_max = cv.Scalar(15, 255 , 255, 255)
            hsv_min2 = cv.Scalar(164, 40, 40, 0)
            hsv_max2 = cv.Scalar(179, 255 , 255, 255)

        #if (color == "custom"):
            #get values from gui

        #hsv_min2 = cv.Scalar(0, 300, 0, 0)
        #hsv_max2 = cv.Scalar(359, 359, 359, 0)


        # convert to HSV for color matching
        # as hue wraps around, we need to match it in 2 parts and OR together
        cv.CvtColor(cv_im, hsv_frame, cv.CV_BGR2HSV)
        cv.InRangeS(hsv_frame, hsv_min, hsv_max, thresholded)
        cv.InRangeS(hsv_frame, hsv_min2, hsv_max2, thresholded2)

        # Debug Code
        #cv.SaveImage(filteredfilename+str(run)+".jpg", thresholded)
        #cv.SaveImage(filteredfilename+"_2_"+str(run)+".jpg", thresholded2)
        #cv.SaveImage(hsvfilename+str(run)+".jpg", hsv_frame)

        #cv.Not(thresholded, thresholded)
        cv.Or(thresholded, thresholded2, thresholded)

        # pre-smoothing improves Hough detector
        for i in range(0,5):
            cv.Smooth(thresholded, thresholded, cv.CV_GAUSSIAN, 5, 5)



        # Sharpening TEST
        #cv.AddWeighted(thresholded, 10, thresholded2, -10, 0, thresholded);
        #cv.AddWeighted(thresholded, 255, thresholded, 0, 0, thresholded);

        #cv.Smooth(thresholded, thresholded, cv.CV_GAUSSIAN, 3, 3)

        #cv.Smooth(thresholded, thresholded, cv.CV_GAUSSIAN, 3, 3)



        cv.AddWeighted(thresholded, 255, thresholded, 0, 0, thresholded);

        return thresholded


    def houghCircles(self, input):

        thresholded = input
        #cv_im = input[1]

        storage = cv.CreateMat(1, 100, cv.CV_32FC3)


        circles = cv.HoughCircles(thresholded, storage, cv.CV_HOUGH_GRADIENT, 1, 10, 50, 50, 10,1000 )
        #circles = cv.HoughCircles(thresholded, storage, cv.CV_HOUGH_GRADIENT, 2, thresholded.height/4, 100, 100, 10,maxRadius )

        #print "Found Circles (run:"+str(run)+"):", str(storage.cols)

        #find largest circle
        maxFoundRadius = 0
        x = 0
        y = 0
        found = False
        for i in range(storage.cols):
            circle = cv.Get1D(storage,i)
            if circle[2] > maxFoundRadius:
                found = True
                maxFoundRadius = circle[2]
                x = circle[0]
                y = circle[1]

        if found:
            #print "ball detected at position:",x, ",", y, " with radius:", maxFoundRadius
            # Draw Circles and save Image
            #for n in range(0,storage.cols):
            for n in range(0,1):
                p=cv.Get1D(storage,n)
                ptStr=(cv.Round(p[0]),cv.Round(p[1]),cv.Round(p[2]))
                #print ptStr
                pt=(cv.Round(p[0]),cv.Round(p[1]))
                cv.Circle(thresholded,pt,cv.Round(p[2]),cv.CV_RGB(128,128,128),1)
                cv.Circle(cv_im,pt,cv.Round(p[2]),cv.CV_RGB(128,128,255),1)

        # Debug Code
        #cv.SaveImage("markedfilename+str(run).jpg", cv_im)
        #cv.SaveImage("markedfilename+str(run)filtered.jpg", thresholded)

        

        # return biggest Circle found in naoImage (None -> (0,0,0)
        #return (x,y,maxFoundRadius),cv_im
        if found:
            return cv.Get1D(storage,0), cv_im
        else:
            return (-10000,-10000,0),cv_im

    cvBackground = None

    def getBackground(self):
        self.cvBackground = self.getPILImage()

    def getBackgroundFiltered(self):
        if self.cvBackground != None:
            newImage = self.getPILImage()

            newImage = ImageChops.difference(newImage,self.cvBackground)




            #print self.houghCircles(redImage)
            return newImage
            #cv.SaveImage("filteredBackground.jpg", newImage)

    def PosByAvg(self,cv_im):
        # convert PIL Image to OpenCV Image
        #cv_im = cv.CreateImageHeader(im.size, cv.IPL_DEPTH_8U, 3)
        #cv.SetData(cv_im, im.tostring())

        # IMPORTANT! ohterwise OpenCV reads blue as red and vice versa
        #cv.CvtColor(cv_im, cv_im, cv.CV_BGR2RGB)

        start = time.ctime()

        count = 0
        sumX = 0
        sumY = 0
        sumDist = 0

        list = []

        for line in range(0, self.size[1]):
            for pixel in range(0,self.size[0]):
                if (cv.Get2D(cv_im, line, pixel)[0]  == 255) :
                    sumX += pixel
                    sumY += line
                    count += 1

                    #add pixel to list
                    list.append((pixel,line))



        avgR = 0
        avgX = 0
        avgY = 0

        counter = []
        for i in range (1,self.size[0]+1):
            counter.append(0)

        if count > 0:
            avgX = sumX / count
            avgY = sumY / count
            


            for line in range(0, self.size[1]):
                for pixel in range(0,self.size[0]):
                    if (cv.Get2D(cv_im, line, pixel)[0]  == 255) :
                        dist = math.sqrt((avgX - pixel)**2 + (avgY - line)**2)
                        sumDist += dist
                        counter[int(dist)] += 1



            avgR = int((sumDist / float(count))*1.1)

            r = 0
            counts = 0
            for i in range (0,len(counter)):
                if counter[i] > counts:
                    r = i
                    counts = counter[i]

            avgR = r

            cv.Circle(cv_im,(avgX,avgY),avgR,cv.CV_RGB(128,128,128),1)

            #cv.SaveImage("avgCircle.jpg", cv_im)

        #print "avg:", avgX,avgY,avgR

        #kmeansres = kmeans.kinit(numpy.array(list),1)

        #print "1->", kmeansres

        #totalkmeans = kmeans.kmeans2(numpy.array(list),1)

        #print " ->", totalkmeans

        #raw_input("--> Enter")

        end = time.ctime()
        #print "time:",end,start
        return avgX,avgY,avgR
        


    def getBallPos(self,debug = False):
        #if self.cvBackground == None:
        #    raw_input("Hintergrund aufnehmen -> Enter")
        #    self.getBackground()

        #raw_input("Objekt aufnehmen -> Enter")
        #redImage = self.redFilter(self.getCVImage(self.getBackgroundFiltered()))

        pilim = self.getPILImage()

        cvim = self.getCVImage(pilim)

        redImage = self.redFilter(cvim)

        
        if debug == True:

            cv.SaveImage("unFiltered.jpg", cvim)
            cv.SaveImage("redFiltered.jpg", redImage)


        #cv_im, size, color)





        #filteredImage = newImage - self.cvBackground

        #newImage.save("filteredBackground.jpg")

        #cv_new = self.getCVImage(newImage)


        erg = self.PosByAvg(redImage)

        cv.Circle(cvim,(erg[0],erg[1]),erg[2],cv.CV_RGB(128,128,128),1)

        if debug == True:

            cv.SaveImage("unFiltered_marked.jpg", cvim)


        #cv.SaveImage("avgCircle_cv.jpg", cvim)
        #cv.SaveImage("avgCircle_red.jpg", redImage)



        #print erg
        return [erg[0],erg[1],erg[2]]


    def testBackground(self):
        raw_input("Hintergrund aufnehmen-> Enter")
        self.getBackground()
        raw_input("Object platzieren -> Enter")
        self.getBackgroundFiltered()


    # TODO: implement getImage and other things, see old code in:
    #         manualControl, localization, distanceEstimation, naoqiBallDetection
    