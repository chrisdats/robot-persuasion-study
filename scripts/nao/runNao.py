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

# for official "sit" posture
#RArmTargetItem = [0.08865346759557724, -0.22455188632011414, 0.21142783761024475, 1.56281316280365, -0.0054118018597364426, -0.5769132971763611]
#RArmTargetParticipant = [0.10347125679254532, -0.09411738067865372, 0.28878143429756165, 1.0531508922576904, -0.5454456806182861, 0.24398885667324066]
#RArmTargetParticipant2 = [0.08484018594026566, -0.04775525629520416, 0.2966286242008209, 0.9779533743858337, -0.6467088460922241, 0.5922301411628723]
#RArmTargetLift = [0.08617198467254639, -0.05527321994304657, 0.22831135988235474, 0.11964680254459381, -0.3332363963127136, 1.0237292051315308]
#LArmTargetLift = [0.07553913444280624, 0.03513256087899208, 0.24100753664970398, -0.46564310789108276, -0.5226525664329529, -0.8622301816940308]
#RArmTargetDefault = [0.08699753880500793, -0.04760340601205826, 0.1661260426044464, 0.6560360193252563, -0.16207846999168396, 0.8656076788902283]
#LArmTargetDefault = [0.0664924681186676, 0.037688370794057846, 0.16274607181549072, -0.5783596634864807, -0.02439243346452713, -0.8660477995872498]
#RArmTargetCapo = [0.04031921923160553, -0.022381991147994995, 0.30415746569633484, -0.06435168534517288, -0.9264166951179504, 1.6822551488876343]
#RArmTargetUp = [0.009432021528482437, -0.08175484091043472, 0.371983140707016, -0.8779908418655396, -1.0446586608886719, 1.6765592098236084]

# for custom posture to get rid of the knee situation
# RArmTargetItem = [0.27113649249076843, -0.1597001850605011, 0.24354188144207, -0.4636474847793579, 0.5175201892852783, -0.815000057220459]
# RArmTargetParticipant = [0.35123905539512634, 0.03591252863407135, 0.2890858054161072, 0.1437036097049713, 0.1828632652759552, 0.505771279335022]
# RArmTargetLift = [0.32420217990875244, 0.017824910581111908, 0.2558828294277191, 0.3244765102863312, 0.32813310623168945, 0.6213055849075317]
# LArmTargetLift = [0.31409716606140137, 0.10803753137588501, 0.21965578198432922, -0.692030668258667, 0.004523070063441992, -0.5290369391441345]
# RArmTargetDefault = [0.22943827509880066, -0.01901804655790329, 0.16480287909507751, 0.46975523233413696, 1.012012004852295, 0.7680749297142029]
# LArmTargetDefault = [0.21610277891159058, 0.07438623905181885, 0.13450753688812256, -1.0386477708816528, 0.481380432844162, -0.9133720397949219]
# RArmTargetCapo = [0.2779078781604767, 0.08055191487073898, 0.3424759805202484, 0.4772016704082489, -0.4996189475059509, 1.2079194784164429]
# RArmTargetUp = [0.32541969418525696, 0.06272923201322556, 0.417009174823761, 0.6178413033485413, -0.7007210850715637, 0.7312569618225098]

# for "in chair" custom position
# RArmTargetItem = [0.1787424385547638, -0.13102713227272034, 0.20228078961372375, 0.8831148147583008, 0.48292243480682373, -0.07802501320838928]
# RArmTargetParticipant = [0.26121291518211365, -0.02853187546133995, 0.3383389711380005, 0.2403903752565384, -0.20407554507255554, 0.3436628580093384]
# RArmTargetLift = [0.24034619331359863, -0.0266873799264431, 0.28032568097114563, 0.41311052441596985, -0.10404398292303085, 0.4189245402812958]
# LArmTargetLift = [0.232427716255188, 0.08839939534664154, 0.26824894547462463, -0.20066113770008087, -0.05390549823641777, -0.5235291719436646]
# RArmTargetDefault = [0.1262950748205185, 0.0066351620480418205, 0.20487400889396667, 0.7301231622695923, 0.1812235713005066, 0.8010936975479126]
# LArmTargetDefault = [0.07123622298240662, 0.10694202780723572, 0.1942804455757141, -0.25017789006233215, 0.3708241581916809, 0.030952192842960358]
# RArmTargetCapo = [0.1892612874507904, 0.008467502892017365, 0.37676820158958435, 0.3789498209953308, -0.9486193060874939, 0.8554478883743286]
# RArmTargetUp = [0.1916750967502594, -0.045830000191926956, 0.4810193181037903, 0.2855706214904785, -1.1820805072784424, 0.5192687511444092]

#RArmTargetDefault = [0.0673317015171051, -0.056636203080415726, 0.20075754821300507, 0.43358147144317627, 0.3018096685409546, 0.2926729917526245]
#RArmTargetItem = [0.04852019250392914, -0.2269805371761322, 0.2249896377325058, 0.08219961076974869, 0.33574655652046204, -0.560173749923706]

# Frame_TORSO "in chair"
RArmTargetDefault = [0.14, -0.06, -0.05, 0.4327031672000885, 0.6230831742286682, 0.46302056312561035]
RArmTargetItem = [0.15759922564029694, -0.2066766321659088, -0.005902979522943497, 0.5399747490882874, 0.4663883447647095, -0.5464719533920288]
RArmTargetParticipant = [0.2067900449037552, -0.03441447764635086, 0.08661369979381561, 0.7018489837646484, -0.15505152940750122, 0.3424641489982605]
RArmTargetUp = [0.14909350872039795, -0.054054729640483856, 0.21166686713695526, 0.2237071692943573, -1.1330797672271729, 0.6848310232162476]
RArmTargetCapo = [0.11982980370521545, -0.0139005146920681, 0.164911150932312, -0.1447601467370987, -1.0350098609924316, 1.319083571434021]
LArmTargetDefault = [0.14, 0.06, -0.05, -0.3635063171386719, 0.6917455792427063, -0.42913269996643066]
LArmTargetLift = [0.19350610673427582, 0.04604508727788925, 0.039861708879470825, -0.15831591188907623, 0.022452719509601593, -0.5135396122932434]
RArmTargetLift = [0.1880233883857727, -0.05803588777780533, 0.0408395379781723, 0.335469126701355, -0.10646793991327286, 0.4467528462409973]

headPitchAngleItem = 0.32  #-0.672 to +0.514
headYawAngleItem = -0.61

headPitchAngleParticipant = 0.05
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
#folderLetterList = ["A", "B", "C", "D", "E", "F"]
folderLetterList = ["A"]

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
        self.participantNumber = "PB"
        self.condition = "contingent" # scripted, contingent, or random
        self.exitFlag = False

        # if a directory for the participant logs does not exist, create it
        #directory = "/nao/" + self.participantNumber
        #print directory
        #if not os.path.exists(directory):
        #    os.makedirs(directory)


    def timeout_callback(self, event):
        self.timeout = True

    # THIS RUNS THE EXPERIMENT ##########################################
    def run(self):
        time.sleep(2)
        self.goNao.genSpeech("Hello! My name is Nao.")
        self.goNao.genSpeech("I have a few items that I would like to show to you today.")
        time.sleep(3)

        # used to see what the kinect spits out about participant data
        #time_out = 60
        #self.participant.monitor(time_out)

        # Runs all the trials in randomized order
        random.shuffle(folderLetterList)
        for folderLetter in folderLetterList:
            self.trial(folderLetter)

        # Concludes the experiment
        time.sleep(2)
        self.goNao.genSpeech("Thank you for your help. We are now finished.")
        self.goNao.genSpeech("Please head to the computer and fill out the exit survey. When you are done, please call over the experimenter. Bye now!")
        self.lookReturn()
        self.pointReturn()
        self.goNao.releaseNao()
    # END OF EXPERIMENT ################################################

    def trial(self, trialName):
        """ imports speech data from file
         robot reads (tts) speech to participant
         and performs behavior according to condition
        """
        # imports speech + gesture data for a particular trial
        script_filename = "itemScripts/"+ trialName + ".txt"
        time.sleep(1)
       
        # instructions for participant to go and retreive item
        self.goNao.genSpeech("We are going to begin a round.")
        self.goNao.genSpeech("Please retreive the item in folder " + str(trialName))
        self.goNao.genSpeech("Please place the item in the center of the red rectangle.")
        #self.goNao.genSpeech("Please leave the yellow folder under the blackboard.")
        self.goNao.genSpeech("Then, type the name of the item into the computer")
        print "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
        print "First, please retrieve the item in folder " + str(trialName) + " and place it in the center of the red rectangle." 
        user_input_item_name=raw_input("Type name of the item and hit enter. ")
        if user_input_item_name == "exit":
            return
        print "Thanks! Please return to the black x"
        self.goNao.genSpeech("Great, now stand on the black x and I will begin.")
        time.sleep(10)

        # create two loggers to record all the robot commands that occured
        # for this particular participant, as well as participant data 
        FORMAT = '%(asctime)-15s [%(levelname)s] (%(threadName)-10s) %(message)s'
        log_filename1 =  self.participantNumber + "/" + self.participantNumber + trialName + "full.txt" 
        logger1 = logging.getLogger('logger1')
        logging.basicConfig(level=logging.DEBUG, format=FORMAT)        
        file_handler = logging.FileHandler(log_filename1)
        file_handler.setFormatter(logging.Formatter(FORMAT))
        logger1.addHandler(file_handler)

        log_filename2 =  self.participantNumber + "/" + self.participantNumber + trialName + "stream.txt"  
        logger2 = logging.getLogger('logger2')
        logging.basicConfig(level=logging.DEBUG, format=FORMAT)        
        file_handler = logging.FileHandler(log_filename2)
        file_handler.setFormatter(logging.Formatter(FORMAT))
        logger2.addHandler(file_handler)

        FORMAT3 = '%(message)s'
        log_filename3 =  self.participantNumber + "/" + self.participantNumber + trialName + "overrides.txt" 
        logger3 = logging.getLogger('logger3')
        logging.basicConfig(level=logging.DEBUG, format=FORMAT3)        
        file_handler = logging.FileHandler(log_filename3)
        file_handler.setFormatter(logging.Formatter(FORMAT3))
        logger3.addHandler(file_handler)

        # start time for the logs
        start_time = time.time()

        self.exitFlag = False

        # second thread is executed only when in contingent condition
        # TODO: fix 
        t1 = Thread(target=self.readScript, args=(script_filename, start_time, ))
        if self.condition == "contingent":
            t2 = Thread(target=self.monitorParticipant, args=(130, start_time, ))

        t1.start()
        if self.condition == "contingent":
            t2.start()

        t1.join()
        if self.condition == "contingent":
            t2.join()

        # Script is completed
        time.sleep(2)
        self.goNao.genSpeech("Thanks for listening")
        self.goNao.genSpeech("Please head to the computer and record your willingness to pay")
        
        # participant records their willingness to pay and their data is saved
        print "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
        print "You just heard about the" , user_input_item_name
        user_input_WTP=raw_input("Please enter your willingness to pay (0.00-5.00) ")
        with open("results.txt", "a") as myfile:
            myfile.write(self.participantNumber+ "," + self.condition + "," + trialName + "," + user_input_WTP + "," + user_input_item_name + '\n')

        self.goNao.genSpeech("Great! This is the end of a round. Please put the item back into the folder")
        print "Great! This is the end of a round. Please put the item back into the folder"
        time.sleep(5)

    def monitorParticipant(self, time_limit, start_time):
        gazeTargetHistory = []  # create empty array to store past values
        logger1 = logging.getLogger("logger1")  # full 
        logger2 = logging.getLogger("logger2")  # stream
        logger3 = logging.getLogger("logger3")  # cmds
        start = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            # exits the function if we have timed out
            if rospy.get_rostime().secs - start >= time_limit:
                rospy.loginfo("Time Out - End of monitor Participant")
                return False       # kills the thread

            # monitors exit flag from other thread
            if self.exitFlag == True:
                rospy.loginfo("Exit Flag True - End of monitor Participant")
                return False        # kills the thread

            # get person id of the person in frame
            person_id = None
            for i in range(0, self.participant.skeletons.max_people):
                if list(self.participant.skeletons.array[i].head) != [0.0,0.0,0.0]:
                    person_id = i
            if person_id == None:
                rospy.loginfo("No one is detected in the frame.")
                logger2.info("No one is detected in the frame.")
                time.sleep(1)
                continue        #why do you need continue here?

            # Get eye gaze target and add it to the history of 3 most recent gaze targets
            # this is done to avoid false positives
            currentGazeTarget = self.participant.eye_gaze_target(person_id)
            happy = self.participant.is_happy(person_id)
            logger2.info("%s %s %s %s", "Current Gaze Target: ", currentGazeTarget, "Is happy", happy)
            if len(gazeTargetHistory) >= 3:
                gazeTargetHistory.pop(0) 
            gazeTargetHistory.append(currentGazeTarget)

            ### Socially Contingent Rules
            # If the participant is looking at the item, robot looks at item, then returns gaze to participant
            if all(gazeTarget == "item" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking at item"
                lockedOut= True # locks out other thread until motion is complete
                elapsed_time = time.time() - start_time
                logger1.info("<look item>" + " " + str(elapsed_time) + " contingentCmd")
                logger3.info("<look item>" + "," + str(elapsed_time))
                self.lookAtItem()
                time.sleep(1.5)
                elapsed_time = time.time() - start_time
                logger1.info("<look participant>" + " " + str(elapsed_time) + " contingentCmd")
                logger3.info("<look participant>" + "," + str(elapsed_time))
                self.lookAtParticipant()
                lockedOut = False
            # if participant looks up, robot looks up, then returns gaze to participant
            elif all(gazeTarget == "up" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking up"
                lockedOut= True # locks out other thread until motion is complete
                elapsed_time = time.time() - start_time
                logger1.info("<look up>" + " " + str(elapsed_time) + " contingentCmd")
                logger3.info("<look up>" + "," + str(elapsed_time))
                self.lookUp()
                time.sleep(1.5)
                elapsed_time = time.time() - start_time
                logger1.info("<look participant>" + " " + str(elapsed_time) + " contingentCmd")
                logger3.info("<look participant>" + "," + str(elapsed_time))
                self.lookAtParticipant()
                lockedOut = False
            # if participant looks right, robot looks right, then returns gaze to participant
            elif all(gazeTarget == "right" for gazeTarget in gazeTargetHistory):
                print "Contingent by looking right"
                lockedOut= True # locks out other thread until motion is complete
                elapsed_time = time.time() - start_time
                logger1.info("<look right>" + " " + str(elapsed_time) + " contingentCmd")
                logger3.info("<look item>" + "," + str(elapsed_time))
                self.goNao.motion.setAngles("HeadYaw", 0.25, 0.15)
                time.sleep(1.5)
                elapsed_time = time.time() - start_time
                logger1.info("<look participant>" + " " + str(elapsed_time) + " contingentCmd")
                logger3.info("<look item>" + "," + str(elapsed_time))
                self.lookAtParticipant()
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
        logger1 = logging.getLogger("logger1")  # return reference to logger object
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
                        # if utterance is not all whitespace, speak it
                        if any(char.isalpha() for char in utterance):
                            elapsed_time = time.time() - start_time
                            logger1.info(utterance + " " + str(elapsed_time))
                            self.goNao.genSpeech(utterance, True) # blocking
                            utterance = ''
                        print str(reference)
                        # Send the reference message
                        if lockedOut == False:
                            elapsed_time = time.time() - start_time
                            logger1.info("<" + reference + "> " + str(elapsed_time))
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

            # If what's left of the utterance is not whitespace, speak it
            if any(char.isalpha() for char in utterance):
                elapsed_time = time.time() - start_time
                logger1.info(utterance.rstrip('\n') + " " + str(elapsed_time))
                self.goNao.genSpeech(utterance, True)
            time.sleep(0.5) # 0.5 sec delay between each line

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
        elif words[0] == "point" and words[1] == "up":
            self.pointUp()
        else:
            print "command not found"

    def testMotion(self):
        """ 
        Short script to check NAO is OK
        """
        # Pointing at the item
        #self.goNao.motion.setStiffnesses("Head", 0.2)
        time.sleep(1)
        print self.goNao.motion.getStiffnesses("Body") , '\n'
        #self.goNao.posture.goToPosture("Sit", 0.6)
        #time.sleep(1)

        self.pointAtItem()
        time.sleep(2)
        self.pointReturn()
        time.sleep(2)

        #time.sleep(3)
        #self.goNao.releaseNao()
        time.sleep(3)
        self.goNao.motion.stiffnessInterpolation("Body", 0.0, 1.0)
        print self.goNao.motion.getStiffnesses("Body")


    def demonstrateMotions(self):
        """
        Demonstrates all the motions that are available to nao during this interaction
        """

        # Intro
        self.goNao.genSpeech("Hello! My name is Nao.")
        time.sleep(3)
        self.goNao.genSpeech("Let me demonstrate my movements to you.")
        time.sleep(2)

        # Sitting
        #self.goNao.posture.goToPosture("Sit", 0.6) #blocking
        #time.sleep(2)
        #self.goNao.genSpeech("Now I am Sititng")
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

        # Point Up
        self.goNao.genSpeech("Now I am pointing up.")
        self.pointUp()
        time.sleep(3)
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
        self.lookReturn()
        self.pointReturn()
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

    def pointUp(self):
        self.goNao.moveEffectorToPosition(RArmTargetUp,"RArm", armSpeed)

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