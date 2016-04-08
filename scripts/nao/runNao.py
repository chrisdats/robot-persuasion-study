# runNao.py
# main Nao file, calls upon naoMotions.py and computeParticipant.py
# Christopher Datsikas Apr 2016
# Adapted from Thomas Weng

import os
import sys
import random
import time
import collections
import rospy

import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBehavior    

from naoMotions import *
from participantData import ComputeParticipant

#http://doc.aldebaran.com/1-14/family/robots/joints_robot.html
# Nao Coodinates (x,y,z,wx,wy,wz) in meters
armTargetPosItem = [-0.14040428400039673, -0.3072628080844879, 0.34422609210014343, 1.5012134313583374, 0.3255554735660553, -0.9553275108337402]
headTargetPosItem =  [-0.16018611192703247, -0.11574727296829224, 0.4552813768386841, 0.044422950595617294, 0.503412663936615, -1.3181275129318237]
headTargetPosParticipant = [-0.16018611192703247, -0.11574727296829224, 0.4552813768386841, 0.04351760447025299, -0.4655193090438843, -1.3529722690582275]
headPitchAngleItem = 0.35  #-0.672 to +0.514
headPitchAngleParticipant = -0.5

armSpeed = 0.8
headSpeed = 0.5
postureSpeed = 0.3


# Get sensing
POINT_APERTURE = 0.4 # radians
LOOK_APERTURE = 3.0
TOUCH_DISTANCE = 0.1 # meters
max_people = 6
num_objects = 4

# items
itemList = ["chocolate", "chocolate", "chocolate"]


#Get the Nao's IP
ipAdd = None
try:
    ipFile = open("ip.txt")
    ipAdd = ipFile.readline().replace("\n","").replace("\r","")
except Exception as e:
    print "Could not open file ip.txt"
    ipAdd = raw_input("Please write Nao's IP address... ") 

#Try to connect to it
goNao = None
try:
    # Robot class is initialized as goNao
    goNao = Robot(ipAdd, 9559)
except Exception as e:
    print "Could not find nao. Check that your ip is correct (ip.txt)"
    sys.exit()

class Demo:
    def __init__(self, goNao):
        self.goNao = goNao
        self.participant = ComputeParticipant(max_people, num_objects, POINT_APERTURE, LOOK_APERTURE, TOUCH_DISTANCE)
       # self.postureProxy = postureProxy
        self.timeout = False

    def timeout_callback(self, event):
        self.timeout = True

    # THIS RUNS THE EXPERIMENT
    def run(self):
        #self.goNao.posture.goToPosture("Stand", 1.0)
        #time.sleep(5)
        #self.goNao.genSpeech("Hello! My name is Nao.")
        #self.goNao.genSpeech("Today we are going to be playing an economic game")
        #time.sleep(3)

        self.demonstrateMotions()

        self.trial(random.randint(0,len(itemList)-1))

        self.goNao.genSpeech("Thank you for your help. Bye now!")
        self.goNao.releaseNao()
    # END OF EXPERIMENT

    def trial(self, trialNumber):
        """ imports speech data from file
         robot reads (tts) speech to participant
         and performs behavior according to condition
        """
        print trialNumber
        # imports speech data for a particular trial
        fileName = "itemScripts/"+ itemList[trialNumber] + ".txt"
        try:
            f = open(fileName, 'r')
            text = f.readline()
            f.close()
        except Exception as e:
            print "Could not open" + fileName
 
        self.goNao.posture.goToPosture("Stand", 0.6)
        self.goNao.genSpeech(text)
        self.goNao.moveEffectorToPosition(armTargetPosItem, "RArm", armSpeed)
        time.sleep(2)
        self.goNao.posture.goToPosture("Stand", 0.6)
        time.sleep(2)
        # begin recording participant data
        #self.participant.monitor(60)

    def demonstrateMotions(self):

        # Intro
        self.goNao.genSpeech("Hello! My name is Nao.")
        time.sleep(3)
        self.goNao.genSpeech("Let me demonstrate my movements to you.")

        # Standing
        self.goNao.posture.goToPosture("Stand", 0.6)
        
        time.sleep(2)
        self.goNao.genSpeech("Now I am standing")
        time.sleep(2)

        # Pointing at the item
        self.goNao.genSpeech("Now I am pointing at the item")
        self.goNao.moveEffectorToPosition(armTargetPosItem, "RArm", armSpeed)
        time.sleep(3)
        self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Looking at the item
        self.goNao.genSpeech("Now I am looking at the item")
        self.goNao.motion.setAngles("HeadPitch", headPitchAngleItem, 0.25)
        #self.goNao.moveEffectorToPosition(headTargetPosItem, "Head", headSpeed)
        print "Head:"
        print self.goNao.getEffectorPosition("Head")
        time.sleep(3)
        self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Looking and pointing at the item
        self.goNao.genSpeech("Now I am looking and pointing at the item simultaneously")
        self.goNao.moveEffectorsToPositions([armTargetPosItem, headTargetPosItem], ["RArm","Head"], [armSpeed, headSpeed])
        time.sleep(3)
        self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Looking at participant
        self.goNao.genSpeech("Now I am looking at the participant")
        self.goNao.motion.setAngles("HeadPitch", headPitchAngleParticipant, 0.15)
        time.sleep(3)
        self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Nodding
        self.goNao.genSpeech("Now I am nodding")
        self.goNao.nod()
        self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Shaking my head
        self.goNao.genSpeech("Now I am shaking my head")
        self.goNao.shake()
        self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Sit
        self.goNao.genSpeech("Now I am going to sit")
        self.goNao.posture.goToPosture("SitRelax", 0.6)
        time.sleep(3)

demo = Demo(goNao)
demo.run()

#______________________________________________________________________________

## Gesture only
#r.moveEffectorToPosition(armTargetPos, "LArm", armSpeed)


## Gaze only
#r.moveEffectorToPosition(headTargetPos, "Head", headSpeed)


## Verbal only
#r.speak(targetSpeech)

## 5. GAZE + GESTURE
#r.moveEffectorsToPositions([armTargetPos,headTargetPos], ["LArm","Head"], [armSpeed, headSpeed])

## 6. GAZE + VERBAL
#r.speak(targetSpeech)
#r.moveEffectorToPosition(headTargetPos, "Head", headSpeed)

## 8. GESTURE + VERBAL
#r.speak(targetSpeech)
#r.moveEffectorToPosition(armTargetPos, "LArm", armSpeed)


## 11. GAZE + GESTURE + VERBAL
#r.moveEffectorsToPositions([armTargetPos,headTargetPos], ["LArm","Head"], [armSpeed, headSpeed])
#r.speak(targetSpeech)
