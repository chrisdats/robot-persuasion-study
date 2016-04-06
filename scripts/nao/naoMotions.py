import os
import sys
import random
import time

import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBehavior 

#____________________________________________________________

class Gesture:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.stiffness = 1.0

        self.frame = None
        self.speechDevice = None
        self.motion = None
        self.posture = None
        self.led = None
        self.connectNao()
    #initialize all nao devices____________________________________________________________
    def connectNao(self):
        #FRAME MANAGER FOR CALLING BEHAVIORS
        try:
            self.frame  = ALProxy("ALFrameManager", self.host, self.port)
        except Exception, e:
            print "Error when creating frame manager device proxy:"+str(e)
            exit(1)
        #POSTURE MANAGER#
        try:
            self.posture = postureProxy = ALProxy("ALRobotPosture", self.host, self.port)
        except Exception, e:
            print "Error creating posture proxy"+str(e)
            exit(1)

        #MOTION DEVICE FOR MOVEMENTS
        try:
            self.motion = ALProxy("ALMotion", self.host, self.port)
        except Exception, e:
            print "Error when creating motion device proxy:"+str(e)
            exit(1)

        #MAKE NAO STIFF (OTHERWISE IT WON'T MOVE)
        self.motion.stiffnessInterpolation("Body",self.stiffness,1.0)

        #MOTION DEVICE FOR MOVEMENTS
        try:
            self.led = ALProxy("ALLeds", self.host, self.port)
        except Exception, e:
            print "Error when creating led proxy:"+str(e)
            exit(1)

        #CONNECT TO A SPEECH PROXY
        try:
            self.speechDevice = ALProxy("ALTextToSpeech", self.host, self.port)
        except Exception, e:
            print "Error when creating speech device proxy:"+str(e)
            exit(1)

    #SAY A SENTENCE___________________________________________________________________________________
    def genSpeech(self, sentence):
        try:
            self.speechDevice.post.say(sentence)
        except Exception, e:
            print "Error when saying a sentence: "+str(e)

    def goodbye(self):
        self.genSpeech("finished")
        time.sleep(5)
        self.posture.goToPosture("SitRelax", 1.0)

    #____________________________________________________________

    def demo(self):
        self.posture.goToPosture("Stand", 1.0)
        self.led.fadeListRGB("FaceLeds",[0x00FFFFFF],[0.1])

        self.genSpeech("Hello! My name is Nao.")     
        #self.send_command("wave.xar")
        self.posture.goToPosture("Stand", 1.0)

        self.genSpeech("I am excited to play rock paper scissors with you. Let me demonstrate the gestures that I can make")
        time.sleep(8)
        
        angleTopShoulder = 0.2
        angleBotShoulder = 0.6

        angleTopElbow = 1.4
        angleBotElbow = 0.4
        wristYaw = 0.3

        self.motion.setAngles("RShoulderPitch",angleBotShoulder,0.1)
        self.motion.setAngles("RElbowRoll",angleBotElbow,0.3)
        time.sleep(0.8)

        #INITIALIZE POSITION OF THE HAND
        self.genSpeech("Let me show you how I do rock")     

    # RELEASE THE JOINTS SO IT WON'T COMPLAIN
    def releaseNao(self):
        try:
            self.posture.goToPosture("SitRelax", 1.0)
            self.motion.stiffnessInterpolation("Body",0.0,self.stiffness)
        except Exception, e:
            print "Error when sitting down nao and making nao unstiff: "+str(e)
#____________________________________________________________
