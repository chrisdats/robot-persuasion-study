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
# These coordinates are for the sit position Nao3
RArmTargetItem = [0.08865346759557724, -0.22455188632011414, 0.21142783761024475, 1.56281316280365, -0.0054118018597364426, -0.5769132971763611]
RArmTargetParticipant = [0.10347125679254532, -0.09411738067865372, 0.28878143429756165, 1.0531508922576904, -0.5454456806182861, 0.24398885667324066]
RArmTargetParticipant2 = [0.08484018594026566, -0.04775525629520416, 0.2966286242008209, 0.9779533743858337, -0.6467088460922241, 0.5922301411628723]
RArmTargetLift = [0.08617198467254639, -0.05527321994304657, 0.22831135988235474, 0.11964680254459381, -0.3332363963127136, 1.0237292051315308]
LArmTargetLift = [0.07553913444280624, 0.03513256087899208, 0.24100753664970398, -0.46564310789108276, -0.5226525664329529, -0.8622301816940308]
RArmTargetDefault = [0.08699753880500793, -0.04760340601205826, 0.1661260426044464, 0.6560360193252563, -0.16207846999168396, 0.8656076788902283]
LArmTargetDefault = [0.0664924681186676, 0.037688370794057846, 0.16274607181549072, -0.5783596634864807, -0.02439243346452713, -0.8660477995872498]
RArmTargetCapo = [0.04031921923160553, -0.022381991147994995, 0.30415746569633484, -0.06435168534517288, -0.9264166951179504, 1.6822551488876343]

headPitchAngleItem = 0.35  #-0.672 to +0.514
headYawAngleItem = -0.60

headPitchAngleParticipant = -0.023
headYawAngleParticipant = 0.0

headPitchAngleUp = -0.52
headYawAngleUp = 0.10

headPitchAngleDefault = -0.063
headYawAngleDefault = 0.0

armSpeed = 0.8
headSpeed = 0.15
postureSpeed = 0.6

# Get sensing
POINT_APERTURE = 0.4 # radians
LOOK_APERTURE = 1.25 # radians
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

class Demo:
    def __init__(self, goNao):
        self.goNao = goNao
        self.participant = ComputeParticipant(max_people, num_objects, POINT_APERTURE, LOOK_APERTURE, TOUCH_DISTANCE)
       # self.postureProxy = postureProxy
        self.timeout = False
        self.rate = rospy.Rate(25) # 5hz, or 5 per second
        self.participantNumber = "P10"
        self.condition = "contingent"
        self.exitFlag = False

    def timeout_callback(self, event):
        self.timeout = True

    # THIS RUNS THE EXPERIMENT ##########################################
    def run(self):
        # Introduces naoexitFlag
        
        self.goNao.posture.goToPosture("Sit", 0.8)
        time.sleep(2)
        self.goNao.genSpeech("Hello! My name is Nao.")
        self.goNao.genSpeech("I have a few item that I would like to show to you today.")
        time.sleep(3)

        #self.participant.monitor(60)

        # Runs all the trials
        #TODO: create tuple to shuffle
        random.shuffle(itemList)
        for item in itemList:
            self.trial(item)

        # Concludes the experiment
        time.sleep(2)
        self.goNao.genSpeech("Thank you for your help. We are now finished.")
        self.goNao.genSpeech("Please call over the experimenter and make sure to fill out the exit survey. Bye now!")
        self.goNao.releaseNao()
    # END OF EXPERIMENT ################################################

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
        self.goNao.genSpeech("Then, type the name of the item into the computer")
        print "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
        print "First, please place the item in the center of the red rectangle" 
        user_input_item_name=raw_input("Type name of the item and hit enter. ")
        print "Thanks! Please return to the black x"
        self.goNao.genSpeech("Great, now stand on the black x and I will begin.")
        time.sleep(5)

        # create a logger to record all the robot commands that occured
        # for this particular participant
        FORMAT = '%(asctime)-15s [%(levelname)s] (%(threadName)-10s) %(message)s'
        logging.basicConfig(level=logging.DEBUG, format=FORMAT)   
        log_filename = self.participantNumber + trialName + ".txt"       
        file_handler = logging.FileHandler(log_filename)
        file_handler.setFormatter(logging.Formatter(FORMAT))
        logging.getLogger().addHandler(file_handler)
        start_time = time.time()

        
        self.exitFlag = False

        # second thread is executed only when in contingent condition
        # TODO: fix this
        t1 = Thread(target=self.readScript, args=(script_filename, start_time, ))
        if self.condition == "contingent":
            t2 = Thread(target=self.monitorParticipant, args=(130, start_time, ))

        t1.start()
        if self.condition == "contingent":
            t2.start()

        t1.join()
        if self.condition == "contingent":
            t2.join()

        time.sleep(2)
        self.goNao.genSpeech("Thanks for listening")
        self.goNao.genSpeech("Please head to the computer and record your willingness to pay")
        print "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
        print "You just heard about the " , user_input_item_name
        user_input_WTP=raw_input("Please enter your willingness to pay (0.00-5.00) ")
        print "Great!"

    def monitorParticipant(self, time_limit, start_time):
        gazeTargetHistory = []  # create empty array to store past values
        start = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            # exits the function if we have timed out
            if rospy.get_rostime().secs - start >= time_limit:
                print "Time Out - End of monitor Participant"
                return False

            print self.exitFlag
            # monitors exit flag from other thread
            if self.exitFlag == True:
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
            # this is done to avoid false positives
            currentGazeTarget = self.participant.eye_gaze_target(person_id)
            rospy.loginfo("%s %s\n", "Current Gaze Target: ", currentGazeTarget)
            if len(gazeTargetHistory) >= 3:
                gazeTargetHistory.pop(0) 
            gazeTargetHistory.append(currentGazeTarget)

            ### Socially Contingent Rules
            # If the participant is looking at the item, robot looks at item, then returns gaze to participant
            if all(gazeTarget == "item" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking at item"
                elapsed_time = time.time() - start_time
                logging.info("<look item>" + " " + str(elapsed_time))
                lockedOut= True # locks out other thread until motion is complete
                self.lookAtItem()
                time.sleep(1.5)
                self.lookAtParticipant()
                lockedOut = False
            # if participant looks up, robot looks up, then returns gaze to participant
            elif all(gazeTarget == "up" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking up"
                elapsed_time = time.time() - start_time
                logging.info("<look up>" + " " + str(elapsed_time))
                lockedOut= True # locks out other thread until motion is complete
                self.lookUp()
                time.sleep(1.5)
                self.lookAtParticipant()
                lockedOut = False
            # if participant looks right, robot looks right, then returns gaze to participant
            elif all(gazeTarget == "right" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking right"
                elapsed_time = time.time() - start_time
                logging.info("<look right>" + " " + str(elapsed_time))
                lockedOut= True # locks out other thread until motion is complete
                self.goNao.motion.setAngles("HeadYaw", 0.25, 0.15)
                time.sleep(1.5)
                self.goNao.motion.setAngles("HeadYaw", 0.0, 0.15)
                lockedOut = False
            
            self.rate.sleep()

    def readScript(self, script_filename, start_time):
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
                        # delay for the amount indicated in the txt script [timing]
                        time.sleep(timing)
                        
                        # Reset to be out of timing state
                        timing = ''
                        intiming = False
                # Extract utterances
                else:
                    utterance = utterance + char

            # Speak what's left of the utterance #TODO: skip empty lines
            elapsed_time = time.time() - start_time
            logging.info(utterance.rstrip('\n') + " " + str(elapsed_time))
            self.goNao.genSpeech(utterance, True)
            time.sleep(0.5)

        # exit the thread when the script is done
        self.exitFlag = True
        print("Exit Flag set to True")

    def process_cmd(self, ref):
        # splits each reference into its two words <word0 word1>
        words = ref.split()

        if words[0] == "pointandlook":
            self.pointAtItem()
            self.lookAtItem()
        elif words[0] == "point" and words[1] == "item":
            self.pointAtItem()
        elif words[0] == "point" and words[1] == "return":
            self.pointReturn()
        elif words[0] == "look" and words[1] == "participant":
            self.lookAtParticipant()
        elif words[0] == "look" and words[1] == "item":
            self.lookAtItem()
        elif words[0] == "point" and words[1] == "open":
            self.pointLift("both")
        elif words[0] == "point" and words[1] == "lift":
            self.pointLift("right")
        elif words[0] == "point" and words[1] == "head":
            self.pointAtCapo()
        elif words[0] == "point" and words[1] == "participant":
            self.pointAtParticipant()
        else:
            print "command not found"

    def testMotion(self):
        # Pointing at the item
        #self.goNao.motion.setStiffnesses("Head", 0.2)
        #time.sleep(1)
        #print self.goNao.motion.getStiffnesses("Body") , '\n'
        self.goNao.posture.goToPosture("Sit", 0.6)
        time.sleep(1)

        self.pointAtCapo()
        time.sleep(2)
        self.pointReturn()
        time.sleep(2)

        time.sleep(3)
        self.goNao.releaseNao()
        time.sleep(1)
        print self.goNao.motion.getStiffnesses("Body")


    def demonstrateMotions(self):

        # Intro
        self.goNao.genSpeech("Hello! My name is Nao.")
        time.sleep(3)
        self.goNao.genSpeech("Let me demonstrate my movements to you.")

        # Sitting
        self.goNao.posture.goToPosture("Sit", 0.6) #blocking
        time.sleep(2)
        self.goNao.genSpeech("Now I am Sititng")
        time.sleep(2)

        # Pointing at the item
        self.goNao.genSpeech("Now I am pointing at the item")
        self.pointAtItem()
        time.sleep(2)
        self.pointReturn()
        time.sleep(2)

        # Looking at the item
        self.goNao.genSpeech("Now I am looking at the item")
        self.lookAtItem()
        time.sleep(2)
        self.lookReturn()
        time.sleep(2)

        # Looking and pointing at the item simultaneously
        self.goNao.genSpeech("Now I am looking and pointing at the item simultaneously")
        self.pointAtItem()
        self.lookAtItem()
        time.sleep(2)
        self.pointReturn()
        self.lookReturn()
        time.sleep(2)

        # Looking at participant
        self.goNao.genSpeech("Now I am looking at the participant")
        self.lookAtParticipant()
        time.sleep(2)
        self.lookReturn()
        time.sleep(2)

        # Pointing at participant
        self.goNao.genSpeech("Now I am pointing at the participant")
        self.pointAtParticipant()
        time.sleep(2)
        self.pointReturn()
        time.sleep(2)

        # Looking Up
        self.goNao.genSpeech("Now I am looking up")
        self.lookUp()
        time.sleep(2)
        self.lookReturn()
        time.sleep(2)

        # Open Gesture
        self.goNao.genSpeech("Open Gesture")
        self.goNao.genSpeech("Now I am lifting both arms")
        self.pointLift("both")
        time.sleep(2)
        self.pointReturn()
        time.sleep(2)

        # Point at its own head
        self.goNao.genSpeech("Now I am pointing at my own head.")
        self.pointAtCapo()
        time.sleep(2)
        self.pointReturn()
        time.sleep(2)

        # Nodding
        #self.goNao.genSpeech("Now I am nodding")
        #self.goNao.nod()
        #self.goNao.posture.goToPosture("Sit", postureSpeed)
        #time.sleep(2)
        # make more pronounced

        # Shaking my head
        #self.goNao.genSpeech("Now I am shaking my head")
        #self.goNao.shake()
        #self.goNao.posture.goToPosture("Sit", postureSpeed)
        #time.sleep(2)

        # End interaction: sit and set stiffness to zero
        self.goNao.releaseNao()
    

    # Gaze Commands
    def pointReturn(self):
        self.goNao.moveEffectorToPosition(RArmTargetDefault,"RArm", armSpeed)
        self.goNao.moveEffectorToPosition(LArmTargetDefault,"LArm", armSpeed)

    def pointAtItem(self):
        self.goNao.moveEffectorToPosition(RArmTargetItem,"RArm", armSpeed)

    def pointAtParticipant(self):
        self.goNao.moveEffectorToPosition(RArmTargetParticipant,"RArm", armSpeed)

    def pointLift(self, whichArms):
        if whichArms == "left":
            self.goNao.moveEffectorToPosition(LArmTargetLift,"LArm", armSpeed)
        elif whichArms == "right":
            self.goNao.moveEffectorToPosition(RArmTargetLift,"RArm", armSpeed)
        elif whichArms == "both":
            self.goNao.moveEffectorToPosition(RArmTargetLift,"RArm", armSpeed)
            self.goNao.moveEffectorToPosition(LArmTargetLift,"LArm", armSpeed)

    def pointAtCapo(self):
        self.goNao.moveEffectorToPosition(RArmTargetCapo,"RArm", armSpeed)


    # Gesture Commands
    def lookReturn(self):
        names = ["HeadPitch", "HeadYaw"]
        angles = [headPitchAngleDefault, headYawAngleDefault]
        fractionMaxSpeed = headSpeed
        self.goNao.motion.setAngles(names, angles, fractionMaxSpeed)

    def lookAtItem(self):
        names = ["HeadPitch", "HeadYaw"]
        angles = [headPitchAngleItem, headYawAngleItem]
        fractionMaxSpeed = headSpeed
        self.goNao.motion.setAngles(names, angles, fractionMaxSpeed)

    def lookAtParticipant(self):
        names = ["HeadPitch", "HeadYaw"]
        angles = [headPitchAngleParticipant, headYawAngleParticipant]
        fractionMaxSpeed = headSpeed
        self.goNao.motion.setAngles(names, angles, fractionMaxSpeed)

    def lookUp(self):
        names = ["HeadPitch", "HeadYaw"]
        angles = [headPitchAngleUp, headYawAngleUp]
        fractionMaxSpeed = headSpeed
        self.goNao.motion.setAngles(names, angles, fractionMaxSpeed)  

    #def pointAtParticipant(self):

    #def idleGesture(self):

demo = Demo(goNao)
demo.run()