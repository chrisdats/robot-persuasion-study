# runNao.py
# main Nao file, calls upon naoMotions.py and computeParticipant.py
# Christopher Datsikas Apr 2016
# Adapted from Thomas Weng, Henny Admoni

import os
import sys
import random
import time
import logging
import collections
import rospy
import threading
import time
import Queue
import re
from threading import Thread

import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy
from naoqi import ALBehavior    

from naoMotions import *
from participantData import ComputeParticipant

#http://doc.aldebaran.com/1-14/family/robots/joints_robot.html
# Nao Coodinates (x,y,z,wx,wy,wz) in meters
# These coordinates are for crouch position
RArmTargetItem = [0.1596788913011551, -0.25627022981643677, 0.27666962146759033, 1.0591833591461182, 0.19004228711128235, -0.548386812210083]
RArmTargetParticipant = [0.1977016031742096, -0.12935318052768707, 0.36685818433761597, 1.4542509317398071, -0.1924528181552887, 0.08652161061763763]
RArmTargetOpen = [0.15322326123714447, -0.09104180335998535, 0.3085833787918091, 1.522914171218872, -0.25367528200149536, 0.706211268901825]
RArmTargetDefault = [0.10287497192621231, -0.031998250633478165, 0.16709882020950317, -1.329634189605713, 0.5766682028770447, -0.9145272374153137]
LArmTargetDefault = [0.10287497192621231, -0.031998250633478165, 0.16709882020950317, -1.329634189605713, 0.5766682028770447, -0.9145272374153137]

headPitchAngleItem = 0.24  #-0.672 to +0.514
headYawAngleItem = -0.65
headPitchAngleParticipant = -0.22
headYawAngleParticipant = 0.0
headPitchAngleUp = 0.670
headYawAngleUp = 0.21
headPitchAngleDefault = -0.14730596542358398

armSpeed = 0.8
headSpeed = 0.5
postureSpeed = 0.3

# Get sensing
POINT_APERTURE = 0.4 # radians
LOOK_APERTURE = 1.1 # radians
TOUCH_DISTANCE = 0.1 # meters
max_people = 6
num_objects = 1

# items
itemList = ["chocolate"]

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


lockedOut = False
exitFlag = False

class Demo:
    def __init__(self, goNao):
        self.goNao = goNao
        self.participant = ComputeParticipant(max_people, num_objects, POINT_APERTURE, LOOK_APERTURE, TOUCH_DISTANCE)
       # self.postureProxy = postureProxy
        self.timeout = False
        self.rate = rospy.Rate(10) # 5hz, or 5 per second


    def timeout_callback(self, event):
        self.timeout = True

    # THIS RUNS THE EXPERIMENT #################################
    def run(self):
        # Introduces nao
        
        #self.goNao.posture.goToPosture("Sit", 0.8)
        time.sleep(2)
        self.goNao.genSpeech("Hello! My name is Nao.")
        self.goNao.genSpeech("I have a few item that I would like to show to you today.")
        time.sleep(3)

        #self.demonstrateMotions()
        #self.participant.monitor(90)

        # Runs all the trials
        random.shuffle(itemList)
        for item in itemList:
            self.trial(item)

        # Concludes the experiment
        time.sleep(2)
        self.goNao.genSpeech("Thank you for your help. We are now finished.")
        self.goNao.genSpeech("Please call over the experimenter and make sure to fill out the exit survey. Bye now!")
        #self.goNao.releaseNao()
    # END OF EXPERIMENT #########################################

    def trial(self, trialName):
        """ imports speech data from file
         robot reads (tts) speech to participant
         and performs behavior according to condition
        """
        print trialName
        # imports speech + gesture data for a particular trial
        script_filename = "itemScripts/"+ trialName + ".txt"
        #self.goNao.posture.goToPosture("Sit", 0.6) #blocking
       
        self.goNao.genSpeech("Please retreive the item in folder " + str(trialName))
        self.goNao.genSpeech("Please place the item in the center of the red rectangle.")
        self.goNao.genSpeech("Then, type the name of the item")
        print "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
        user_input=raw_input("Type name of the object and hit enter.")
        print "Please return to the black x"
        self.goNao.genSpeech("Great, now stand on the black x and I will begin.")
        time.sleep(5)

        # create a logger to record all the robot commands that occured
        # for this particular participant
        FORMAT = '%(asctime)-15s [%(levelname)s] (%(threadName)-10s) %(message)s'
        logging.basicConfig(level=logging.DEBUG, format=FORMAT)   
        log_filename = participantNumber + trialName + ".txt"       
        file_handler = logging.FileHandler(log_filename)
        file_handler.setFormatter(logging.Formatter(FORMAT))
        logging.getLogger().addHandler(file_handler)
        start_time = time.time()

        exitFlag = False
        t1 = Thread(target=self.readScript, args=(script_filename, ))
        t2 = Thread(target=self.monitorParticipant, args=(100, ))

        t1.start()
        t2.start()
        t1.join()
        t2.join()

        self.goNao.genSpeech("Thanks for listening")
        self.goNao.genSpeech("Please head to the computer and record your willingness to pay in the google form.")
        self.goNao.genSpeech("When you are done please hit enter in the controller window to continue the experiment.")
        print "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
        user_input=raw_input("Please hit enter here when you are done filling out the google form.")
        print "Great!"

    def monitorParticipant(self, time_limit):
        gazeTargetHistory = []
        start = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            # exits the function if we have timed out
            if rospy.get_rostime().secs - start >= time_limit:
                print "Time Out - End of monitor Participant"
                return False

            # monitors exit flag from other thread
            if exitFlag == True:
                print "Exit Flag True - End of monitor Participant"
                return False

            # get person id of the person in frame
            person_id = None
            for i in range(0, self.participant.skeletons.max_people):
                if list(self.participant.skeletons.array[i].head) != [0.0,0.0,0.0]:
                    person_id = i
            if person_id == None:
                rospy.loginfo("No one is detected in the frame.")
                time.sleep(1)
                continue        #why do you need continue here?

            # Get eye gaze target and add it to the history of 3 most recent gaze targets
            currentGazeTarget = self.participant.eye_gaze_target(person_id)
            rospy.loginfo("%s %s\n", "Current Gaze Target: ", currentGazeTarget)
            if len(gazeTargetHistory) >= 3:
                gazeTargetHistory.pop(0) 
            gazeTargetHistory.append(currentGazeTarget)

            if all(gazeTarget == "item" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking at item"
                elapsed_time = time.time() - start_time
                logging.info("<look item>" + " " + str(elapsed_time))
                lockedOut= True
                self.lookAtItem()
                time.sleep(1.5)
                self.lookAtParticipant()
                lockedOut = False
            elif all(gazeTarget == "up" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking up"
                elapsed_time = time.time() - start_time
                logging.info("<look up>" + " " + str(elapsed_time))
                lockedOut= True
                self.goNao.motion.setAngles("HeadYaw", -0.0, 0.15)
                self.goNao.motion.setAngles("HeadPitch", -0.4, 0.15)
                time.sleep(1.5)
                self.goNao.motion.setAngles("HeadPitch", headPitchAngleParticipant, 0.15)
                lockedOut = False
            elif all(gazeTarget == "right" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking right"
                elapsed_time = time.time() - start_time
                logging.info("<look right>" + " " + str(elapsed_time))
                lockedOut= True
                self.goNao.motion.setAngles("HeadYaw", 0.25, 0.15)
                time.sleep(1.5)
                self.goNao.motion.setAngles("HeadYaw", 0.0, 0.15)
                lockedOut = False
                previousGazeTarget = currentGazeTarget
            
            self.rate.sleep()

    def readScript(self, script_filename):
        """
        Parse script and send speech commands to robot.

        Parses the interaction script. Sends speech commands to Nao with appropriate
        timings as determined by script. Broadcasts object references as ScriptObjectRef
        ROS messages at appropriate times as determined by script.

        Arguments: none

        Returns: none (but causes robot to speak)
        """
        rospy.loginfo("Beginning to read script " + script_filename)

        try:
            self.script = open(script_filename, 'r')
        except Exception as e:
             print "Could not open" + script_filename

        alphanumeric = re.compile('[\W_]+')

        for line in self.script:
            utterance = ''
            reference = ''
            timing = '' # a timing command
            inreference = False # is this char part of the reference?
            intiming = False # is this char part of a timing cue?
            # Parse any action commands
            for char in line:
                # Allow for comments
                if char == '#':
                    break # go to next line
                # Find object references (bracketed with "<" and ">")
                if char == "<" and not intiming:
                    inreference = True
                    continue
                if char == "[" and not inreference:
                    intiming = True
                    continue
                if inreference:
                    # Read until the next '>'
                    if not char == '>': 
                        reference = reference + char
                    else:
                        # Speak the utterance to this point
                        elapsed_time = time.time() - start_time
                        logging.info(utterance + " " + str(elapsed_time))
                        self.goNao.genSpeech(utterance, True) # blocking
                        utterance = ''
                        print str(reference)
                        # Send the reference message
                        if lockedOut == False:
                            elapsed_time = time.time() - start_time
                            logging.info("<" + reference + "> " + str(elapsed_time))
                            self.process_cmd(reference)
                        
                        # Reset to be out of reference state
                        reference = ''
                        inreference = False
                elif intiming:
                    # A timing command affects the timing of the next utterance
                    # Read until the next ']'
                    if not char == ']':
                        timing = timing + char
                    else:
                        #self.performTimingCommand(timing)
                        
                        # Reset to be out of timing state
                        timing = ''
                        intiming = False
                # Extract utterances
                else:
                    utterance = utterance + char

            # Speak what's left of the utterance
            elapsed_time = time.time() - start_time
            logging.info(utterance + " " + str(elapsed_time))
            self.goNao.genSpeech(utterance, True)
            time.sleep(0.5)

        # exit the thread when the script is done
        exitFlag = True
        print("Exit Flag set to True")

    def process_cmd(self, ref):
        # splits each reference into its two words <word0 word1>
        words = ref.split()

        if words[0] == "pointandlook":
            print "I am in point and look"
            self.pointAndLookAtItem()
        elif words[0] == "point" and words[1] == "item":
            self.pointAtItem()
        elif words[0] == "point" and words[1] == "return":
            self.pointReturn()
        elif words[0] == "look" and words[1] == "participant":
            self.lookAtParticipant()
        elif words[0] == "look" and words[1] == "item":
            self.lookAtItem()

    def testMotion(self):
        # Pointing at the item
        #self.goNao.motion.setStiffnesses("Head", 0.2)
        #time.sleep(1)
        print self.goNao.motion.getStiffnesses("Body") , '\n'
        time.sleep(1)
        self.goNao.posture.goToPosture("Sit", 0.6)
        time.sleep(1)
        print self.goNao.motion.getStiffnesses("Body")
        time.sleep(1)
        self.goNao.motion.setAngles("HeadPitch", -0.20, 0.25)
        time.sleep(1)
        self.goNao.motion.stiffnessInterpolation("Body",0.0,1.0)
        time.sleep(1)
        print self.goNao.motion.getStiffnesses("Body")
        #self.goNao.genSpeech("Now I am pointing at the item")
        #self.goNao.motion.setAngles("HeadPitch", headPitchAngleItem, 0.25)
        #time.sleep(3)

    def demonstrateMotions(self):

        # Intro
        self.goNao.genSpeech("Hello! My name is Nao.")
        time.sleep(3)
        self.goNao.genSpeech("Let me demonstrate my movements to you.")

        # Standing
        #self.goNao.posture.goToPosture("Stand", 0.6) #blocking
        #time.sleep(2)
        #self.goNao.genSpeech("Now I am standing")
        #time.sleep(2)

        # Pointing at the item
        self.goNao.genSpeech("Now I am pointing at the item")
        self.pointAtItem()
        time.sleep(2)
        self.pointReturn()
        time.sleep(2)

        # Looking at the item
        self.goNao.genSpeech("Now I am looking at the item")
        self.lookAtItem()
        time.sleep(3)
        #self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(2)

        # Looking and pointing at the item simultaneously
        self.goNao.genSpeech("Now I am looking and pointing at the item simultaneously")
        self.pointAndLookAtItem()
        time.sleep(2)
        self.lookReturn()
        self.pointReturn()
        time.sleep(2)

        # Looking at participant
        self.goNao.genSpeech("Now I am looking at the participant")
        self.lookAtParticipant()
        time.sleep(3)
        #self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Nodding
        self.goNao.genSpeech("Now I am nodding")
        self.goNao.nod()
        #self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)
        # make more pronounced

        # Shaking my head
        self.goNao.genSpeech("Now I am shaking my head")
        self.goNao.shake()
        #self.goNao.posture.goToPosture("Stand", postureSpeed)
        time.sleep(3)

        # Sit
        #self.goNao.genSpeech("Now I am going to Sit")
        #self.goNao.posture.goToPosture("Sit", 0.6)
        #time.sleep(3)
    

    #def lookReturn(self):
    #    self.goNao.motion.setAngles("HeadPitch", headPitchAngleDefault, 0.15)

    def pointReturn(self):
        self.goNao.moveEffectorToPosition(RArmTargetDefault,"RArm", 0.3)

    def pointAtItem(self):
        self.goNao.moveEffectorToPosition(RArmTargetItem,"RArm", 0.3)

    def lookAtItem(self):
        self.goNao.motion.setAngles("HeadPitch", headPitchAngleItem, 0.25)
        self.goNao.motion.setAngles("HeadYaw", headYawAngleItem, 0.25)

    def pointAndLookAtItem(self):
        self.goNao.moveEffectorToPosition(armTargetPosItem,"RArm", 0.8)
        self.goNao.motion.setAngles("HeadPitch", headPitchAngleItem, 0.25)
        self.goNao.motion.setAngles("HeadYaw", headYawAngleItem, 0.25)

    def lookAtParticipant(self):
        self.goNao.motion.setAngles("HeadPitch", headPitchAngleParticipant, 0.25)
        self.goNao.motion.setAngles("HeadYaw", headYawAngleParticipant, 0.25)

    #def pointAtParticipant(self):

    #def idleGesture(self):

demo = Demo(goNao)
demo.testMotion()
