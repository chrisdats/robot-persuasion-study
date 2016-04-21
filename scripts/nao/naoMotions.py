# naoMotions.py
# Contains Nao behaviors relevant to persuasion study
# Christopher Datsikas Apr 2016
# Adapted from Henny Admoni, Thomas Weng, and Aditi Ramachandran

import os
import sys
import random
import time
import motion

import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBehavior 

#____________________________________________________________

class Robot:
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
    def genSpeech(self, sentence, blocking=False):
        try:
            if blocking == False:
                self.speechDevice.post.say(sentence) # non-blocking
            else:
                self.speechDevice.say(sentence)
        except Exception, e:
            print "Error when saying a sentence: "+str(e)


    def moveEffectorToPosition(self, target, effector, fractionMaxSpeed=0.5):
            '''
            Moves the specified end effector to the target position at 0.5 speed.

            Input:
                target    6D position in world coordinates, i.e., 
                            (x, y, z, wx, wy, wz) where the first three
                            entries are x, y, z points in meters, and the 
                            last three are rotation coordinates in radians

                effector    end effector to move, can be "Head", "LArm", "RArm", "Torso"

                fractionMaxSpeed fraction of max speed to move
            Output: none

            Action: moves the desired end effector to the specified point

            '''
            # Set movement paramters
            space = motion.FRAME_ROBOT
            useSensor = False
            axisMask = 63 # control both position and rotation

            self.motion.post.setPosition(effector, space, target, 
                                         fractionMaxSpeed, axisMask)


    def moveEffectorsToPositions(self, targetList, 
                                effectorList, fractionMaxSpeedList=[0.5]):
        '''
        Moves the effectors specified in effectorList to their respective
        targets specified in targetList.
        eg call: r.moveEffectorsToPositions([armTargetPos,headTargetPos], [arm,"Head"], [armSpeed,headSpeed])
        '''

        # If the default fractionMaxSpeed argument is used, make that list
        # as long as the other lists
        if len(fractionMaxSpeedList) == 1:
            fractionMaxSpeedList = fractionMaxSpeedList * len(targetList)
         
        for target, effector, speed in zip(targetList, effectorList, fractionMaxSpeedList):
            self.moveEffectorToPosition(target, effector, speed)


    def getEffectorPosition(self, effector):
        '''
        Returns the 6D position vector of the current position of the chosen effector.

        Input:
            effector    Name of the effector, such as "Head", "LArm", "RArm"
 
        Output:
            currentPos  The current position of that effector, as a 6D vector
 
        Action: none
        '''

        space = motion.FRAME_ROBOT
        useSensor = False

        currentPos = self.motion.getPosition(effector, space, useSensor)
        return currentPos


    # this bit of code makes the robot shake its head
    def shake(self):
        self.motion.setStiffnesses("Head", 1.0)

        #shake head
        self.motion.setAngles("HeadPitch", 0, 0.05)
        #self.motion.setAngles("HeadYaw", 0, 0.05)
        #time.sleep(0.5)
        self.motion.setAngles("HeadYaw", -0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", 0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", -0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", 0.5, 0.3)
        time.sleep(0.5)
        self.motion.setAngles("HeadYaw", 0, 0.3)
        time.sleep(1)


    # this bit of code makes the robot nod its head
    def nod(self):
        self.motion.setStiffnesses("Head", 1.0)
        time.sleep(0.5)

        #move head
        #self.motion.setAngles("HeadPitch", 0, 0.5)
        #time.sleep(0.5)
        self.motion.setAngles("HeadPitch", 0.15, 0.25)
        time.sleep(0.5)
        self.motion.setAngles("HeadPitch", -0.15, 0.25)
        time.sleep(0.5)
        self.motion.setAngles("HeadPitch", 0.15, 0.25)
        #time.sleep(0.5)

    # this bit of code makes the robot wave
    def wave(self):
        #self.posture.goToPosture("Sit", 0.5)
        #self.motion.closeHand("RHand")
        #self.motion.closeHand("LHand")
        #time.sleep(0.5)

        self.motion.setAngles("RShoulderPitch",-1.0, 0.15)
        self.motion.setAngles("RShoulderRoll", -1.2, 0.15)
        self.motion.setAngles("RElbowRoll", 1.0, 0.1)
        self.motion.setAngles("RElbowYaw", 0.5, 0.1)
        self.motion.setAngles("RWristYaw", 0, 0.1)
        self.motion.openHand("RHand")

        time.sleep(0.7)
        #self.motion.setAngles("RShoulderRoll", -1.2, 0.5)
        #self.motion.setAngles("RHand", Open, 1.0)
        #self.motion.setAngles("RElbowRoll",angleBotElbow,0.3)

        #wave the hand 3 times, by moving the elbow
        self.motion.setAngles("RElbowRoll", 1.5, 0.5)
        time.sleep(0.5)
        self.motion.setAngles("RElbowRoll", 0.5, 0.5)
        time.sleep(0.5)
        self.motion.setAngles("RElbowRoll", 1.5, 0.5)
        time.sleep(0.5)
        self.motion.setAngles("RElbowRoll", 0.5, 0.5)
        time.sleep(0.5)
        self.motion.setAngles("RElbowRoll", 1.5, 0.5)
        time.sleep(0.5)
        self.motion.setAngles("RElbowRoll", 0.5, 0.5)
        time.sleep(0.5)
        self.motion.setAngles("RElbowRoll", 1.0, 0.5)
        #time.sleep(0.5)
        #self.genSpeech("hello there!")

        time.sleep(1)

        self.motion.closeHand("RHand")
        #self.posture.goToPosture("Sit", 0.5)
        #self.motion.setAngles("HeadPitch", 0.3, 0.15)




    # RELEASE THE JOINTS SO IT WON'T COMPLAIN
    def releaseNao(self):
        try:
            self.posture.goToPosture("Crouch", 0.6)
            time.sleep(1)
            self.motion.stiffnessInterpolation("Body",0.0,self.stiffness)
        except Exception, e:
            print "Error when sitting down nao and making nao unstiff: "+str(e)
#____________________________________________________________
