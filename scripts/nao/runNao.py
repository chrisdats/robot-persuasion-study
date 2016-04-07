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


# participantPos 

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

    def run(self):
        self.goNao.posture.goToPosture("Stand", 1.0)
        time.sleep(5)
        self.goNao.genSpeech("Hello! My name is Nao.")
        self.goNao.genSpeech("Today we are going to be playing an economic game")
        time.sleep(3)

        self.trial(random.randint(1,len(itemList)))

        self.goNao.genSpeech("Thank you for your help. Bye now!")
        self.goNao.releaseNao()

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
            data = f.readlines()
            f.close()
        except Exception as e:
            print "Could not open" + fileName
 
        # begin recording participant data
        self.participant.monitor(60)

    def task(self, gesture, index):
        # ex call: self.task("look at", BLUE)
        self.goNao.genSpeech("Can you " + gesture + " the " + colors[index] + " block?")
        time.sleep(2)
        if self.participant.run(gesture, index, 6): # checks if person did the task correctly
            self.goNao.genSpeech(good[random.randint(0,len(good)-1)])
            time.sleep(0.5)
        else: #if person failed to complete the task
            self.goNao.genSpeech(bad[random.randint(0,len(bad)-1)])
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
