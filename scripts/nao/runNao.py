# runNao.py
# main Nao file, calls upon naoMotions.py and video_demo.py
# Christopher Datsikas
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

from video_demo import ComputeAttention

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
    # Gesture class is initialized as goNao
    goNao = Gesture(ipAdd, 9559)
except Exception as e:
    print "Could not find nao. Check that your ip is correct (ip.txt)"
    sys.exit()

#Set postureProxy
try:
    postureProxy = ALProxy("ALRobotPosture", ipAdd, 9559)
except Exception, e:
    print "Could not create proxy to ALRobotPosture"
    print "Error was: ", e

# Get sensing
POINT_APERTURE = 0.7 # radians
LOOK_APERTURE = 3.0
TOUCH_DISTANCE = 0.1 # meters
max_people = 6
num_objects = 4

# colors
BLUE = 0
RED = 1 
GREEN = 2
YELLOW = 3
colors = ["blue", "red", "green", "yellow"]

# animacy strings
good = ["good job!", "great!", "great job.", "awesome"]
bad = ["Time is up. I did not see you do what I asked.", "I don't think you did what I asked.", "Oops, time is up."]

class Demo:
    def __init__(self, goNao, postureProxy):
        self.goNao = goNao
        self.attention = ComputeAttention(max_people, num_objects, POINT_APERTURE, LOOK_APERTURE, TOUCH_DISTANCE)
        self.postureProxy = postureProxy
        self.timeout = False

    def timeout_callback(self, event):
        self.timeout = True

    def run(self):
        self.goNao.genSpeech("Hello! My name is Nao.")
        time.sleep(2)
        self.postureProxy.goToPosture("Stand", 1.0)
        self.goNao.genSpeech("I need your help identifying blocks.")
        time.sleep(3)

        self.task("point to", YELLOW)
        self.task("point to", GREEN)
        self.task("point to", RED)
        self.task("point to", BLUE)
        self.task("point to", BLUE)

        self.task("look at", GREEN)
        self.task("look at", YELLOW)
        self.task("look at", BLUE)
        self.task("look at", GREEN)
        self.task("look at", RED)

        self.goNao.genSpeech("Thank you for your help. Bye now!")
        self.goNao.releaseNao()

    def task(self, gesture, index):
        self.goNao.genSpeech("Can you " + gesture + " the " + colors[index] + " block?")
        time.sleep(2)
        if self.attention.run(gesture, index, 6): # checks if person did the task correctly
            self.goNao.genSpeech(good[random.randint(0,len(good)-1)])
            time.sleep(0.5)
        else: #if person failed to complete the task
            self.goNao.genSpeech(bad[random.randint(0,len(bad)-1)])
            time.sleep(3)

demo = Demo(goNao, postureProxy)
demo.run()

#______________________________________________________________________________
