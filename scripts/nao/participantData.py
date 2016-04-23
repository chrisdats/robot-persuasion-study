#!/usr/bin/env python

# participantData.py
# Christopher Datsikas Apr 2016
# Adapted from Thomas Weng
# contains software that subscribes to parsed Kinect data
# and interprets that information

import math
from numpy import *
import rospy
import time
from std_msgs.msg import String
from kinect2_pointing_recognition.msg import SkeletonInfo, ObjectsInfo, FaceInfo, SpeechInfo

refreshRate = 25 # hz

def dot_prod(a, b):
	return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def magn(a):
	return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

class ComputeParticipant():
	def __init__(self, max_people, num_objects, point_aperture, look_aperture, touch_distance):
		self.node_name = 'compute_attention'
		rospy.init_node(self.node_name)

		self.point_aptr = point_aperture
		self.look_aptr = look_aperture
		self.touch_dist = touch_distance

		self.skeletons = Skeletons(max_people)
		self.faces = Faces(max_people)
		self.objects = Objects(num_objects)
		self.speech = Speech()

		self.rate = rospy.Rate(refreshRate) # 5hz, or 5 per second

		self.head_target = None
		self.leftarm_target = None
		self.rightarm_target = None

	def leftarm_pointing(self, obj):
		if self.leftarm_target == obj:
			return True
		return False

	def rightarm_pointing(self, obj):
		if self.rightarm_target == obj:
			return True
		return False

	def head_pointing(self, obj):
		if self.head_target == obj:
			return True
		return False

	def face_vs_obj_angle(self, person_id, object_id):
		face_orientation = self.faces.face_orientation_vec(person_id)
		head_to_obj = self.head_to_obj_vec(person_id, object_id)

		dotprod = dot_prod(face_orientation, head_to_obj)
		if dotprod != 0.0:
			cosine = dotprod / (magn(face_orientation) * magn(head_to_obj))
			angle = math.acos(cosine)
			return angle

	def head_to_obj_vec(self, person_id, object_id):
		if list(self.skeletons.array[person_id].head) == [0.0, 0.0, 0.0]:
			return [0.0, 0.0, 0.0]
		return self.objects.array[object_id].pos - array(self.skeletons.array[person_id].head)

	def point_vs_obj_angles(self, person_id, object_id):
		head_to_handtips = self.skeletons.head_to_handtip_vecs(person_id)
		handtips_to_obj = self.handtip_to_obj_vecs(person_id, object_id)

		left_point = head_to_handtips[0]
		left_handtip_to_obj = handtips_to_obj[0]
		left_dotprod = dot_prod(left_point, left_handtip_to_obj)
		left_point_vs_obj_angle = None

		if left_dotprod != 0.0:
			left_cosine = left_dotprod / (magn(left_point) * magn(left_handtip_to_obj))
			left_point_vs_obj_angle = math.acos(left_cosine)

		right_point = head_to_handtips[1]
		right_handtip_to_obj = handtips_to_obj[1]
		right_dotprod = dot_prod(right_point, right_handtip_to_obj)
		right_point_vs_obj_angle = None

		if right_dotprod != 0.0:
			right_cosine = right_dotprod / (magn(right_point) * magn(right_handtip_to_obj))
			right_point_vs_obj_angle = math.acos(right_cosine)

		return [left_point_vs_obj_angle, right_point_vs_obj_angle]

	def handtip_to_obj_vecs(self, person_id, object_id):
		if list(self.skeletons.array[person_id].handTipLeft) == [0.0, 0.0, 0.0]:
			return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
		left = self.objects.array[object_id].pos - array(self.skeletons.array[person_id].handTipLeft)
		right = self.objects.array[object_id].pos - array(self.skeletons.array[person_id].handTipRight)
		return [left, right]

	def is_touching(self, person_id, object_id):
		handtips_to_obj = self.handtip_to_obj_vecs(person_id, object_id)
		if not any(handtips_to_obj[0]) and not any(handtips_to_obj[1]):
			return [None, None]
		return [magn(handtips_to_obj[0]), magn(handtips_to_obj[1])]

	def is_happy(self, person_id):
	 	rospy.loginfo("%s \t%s", "Happy?", self.faces.array[person_id].happy)
	 	return self.faces.array[person_id].happy

	def is_engaged(self, person_id):
	 	rospy.loginfo("%s \t%s", "Engaged?", self.faces.array[person_id].engaged)

	def eye_gaze_target(self, person_id):
		looking_angle = self.face_vs_obj_angle(person_id, 0)
		print str(looking_angle) + " " + str(self.look_aptr)

		atItem = looking_angle is not None and looking_angle < self.look_aptr / 2

		pitch = self.faces.face_pitch_value(person_id)
		yaw = self.faces.face_yaw_value(person_id)

		if  pitch > 0.38:
			gazeTarget = "up"
		elif self.faces.array[person_id].looking_away == False:
			gazeTarget = "kinect/nao"
		elif atItem == True:
			gazeTarget = "item"
		elif yaw < -0.45:
			gazeTarget = "right"
		else:
			gazeTarget = "unknown"

		return gazeTarget

	def monitor(self,time_limit):
		# Header
		rospy.loginfo("%s \t%s \t%s \t%s \t%s \t%s",
		 	"Happy?",
		 	"Engaged?",
		 	"Looking Away",
		 	"Eye Gaze Target",
		 	"Head Center Pos",
		 	"Item Pos")
		start = rospy.get_rostime().secs
		while not rospy.is_shutdown():
			if rospy.get_rostime().secs - start >= time_limit:
				return False


			# get person id of the person in frame
			person_id = None
			for i in range(0, self.skeletons.max_people):
				if list(self.skeletons.array[i].head) != [0.0,0.0,0.0]:
					person_id = i
			if person_id == None:
				rospy.loginfo("No one is detected in the frame.")
				time.sleep(1)
				continue		#why do you need continue here?

			object_id = 0
			rospy.loginfo("%s \t%s", "face vs obj angle", self.face_vs_obj_angle(person_id, 0))
			rospy.loginfo("%s \t%s \t\t%s \t\t%s \t%s \t%s",
			 	self.faces.array[person_id].happy,
			 	self.faces.array[person_id].engaged,
			 	self.faces.array[person_id].looking_away,
			 	self.eye_gaze_target(person_id),
			 	self.faces.array[person_id].nose_pos,
			 	self.objects.array[object_id].pos)

			self.rate.sleep()

	def eye_testing(self, time_limit):
		start = rospy.get_rostime().secs
		while not rospy.is_shutdown():
			if rospy.get_rostime().secs - start >= time_limit:
				return False


			# get person id of the person in frame
			person_id = None
			for i in range(0, self.skeletons.max_people):
				if list(self.skeletons.array[i].head) != [0.0,0.0,0.0]:
					person_id = i
			if person_id == None:
				rospy.loginfo("No one is detected in the frame.")
				time.sleep(1)
				continue		#why do you need continue here?

			pitch = self.faces.face_pitch_value(person_id)
			yaw = self.faces.face_yaw_value(person_id)

			object_id = 0
			print "[x,y,z] Head Coord" + str(self.skeletons.array[person_id].head)
			print "[x,y,z] Object Coord", str(self.objects.array[object_id].pos)
			print " head to obj vec", str(self.head_to_obj_vec(person_id, 0))
			print " "

			print "Pitch (degrees)" + str(self.faces.array[person_id].pitch)
			print "Yaw (degrees)" + str(self.faces.array[person_id].yaw)
			print " "

			print "Pitch (rad)" + str(pitch)
			print "Yaw (rad)" + str(yaw)
			print "face orientation vec" + str(self.faces.face_orientation_vec(person_id))
			print " "

			print "face vs obj angle" + str(self.face_vs_obj_angle(person_id, 0))
			print " "
			print "----------------------------------------"
			print " "


	def run(self, gesture, target, time_limit):
		start = rospy.get_rostime().secs
		while not rospy.is_shutdown():
			if rospy.get_rostime().secs - start >= time_limit:
				return False 

			# get person id of the person in frame
			person_id = None
			for i in range(0, self.skeletons.max_people):
				if list(self.skeletons.array[i].head) != [0.0,0.0,0.0]:
					person_id = i
			if person_id == None:
				rospy.loginfo("No one is detected in the frame.")
				time.sleep(1)
				continue		#why do you need continue here?

			self.is_happy(person_id)
			self.is_engaged(person_id)

			# Boolean and Radians of Head vs. Object
			rospy.loginfo("%s", "Booleans and Radians of Head vs. Object")
			smallest_angle = 1000
			target_object = None
			for j in range(0, self.objects.num_objects):
				looking_angle = self.face_vs_obj_angle(person_id, j)
				threshold = looking_angle is not None and looking_angle < self.look_aptr / 2
				if threshold and looking_angle < smallest_angle:
					smallest_angle = looking_angle
					target_object = j
				rospy.loginfo("%s \t%s \t%s", j, threshold, looking_angle)
			self.head_target = target_object
			rospy.loginfo("Looking at %s\n", self.head_target)

			if gesture == "look at":
				if self.head_target == target:
					return True
			elif gesture == "point to":
				if self.rightarm_target == target or self.leftarm_target == target:
					return True
			elif gesture == "look at and point to":
				if self.head_target == target and (self.rightarm_target == target or self.leftarm_target == target):
					return True

			rospy.loginfo("-----------")
			self.rate.sleep()
		return False

class Skeletons():
	def __init__(self, max_people):
		self.max_people = max_people
		self.array = [SkeletonInfo()] * max_people
		rospy.Subscriber('/skeleton_info', SkeletonInfo, self.skeleton_callback)

	def skeleton_callback(self, msg):
		self.array[int(msg.person_id)] = msg
	
	def head_to_handtip_vecs(self, person_id):
		if list(self.array[person_id].head) == [0.0, 0.0, 0.0]:
			return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
		left = array(self.array[person_id].handTipLeft) - self.array[person_id].head
		right = array(self.array[person_id].handTipRight) - self.array[person_id].head
		return [left, right]

class Faces():
	def __init__(self, max_people):
		self.max_people = max_people
		self.array = [FaceInfo()] * max_people
		rospy.Subscriber('/face_info', FaceInfo, self.face_callback)

	def face_callback(self, msg):
		self.array[int(msg.person_id)] = msg

	def face_pitch_value(self, person_id):
		if self.array[person_id].pitch == -1.0:
			return 0.0
		pitch_rad = math.radians(self.array[person_id].pitch)
		return pitch_rad

	def face_yaw_value(self, person_id):
		if self.array[person_id].yaw == -1.0:
			return 0.0
		yaw_rad = math.radians(self.array[person_id].yaw)
		return yaw_rad

	def face_orientation_vec(self, person_id):
		if self.array[person_id].yaw == -1.0 and self.array[person_id].pitch == -1.0: 
			return [0.0, 0.0, 0.0]
		yaw_rad = math.radians(self.array[person_id].yaw)
		pitch_rad = 1.1*math.radians(self.array[person_id].pitch)  # manually put in multiplier
		return [-math.sin(yaw_rad)*math.cos(pitch_rad), math.sin(pitch_rad), -math.cos(yaw_rad)*math.cos(pitch_rad)]

class Objects():
	def __init__(self, num_objects):
		self.num_objects = num_objects
		self.array = [ObjectsInfo()] * num_objects
		rospy.Subscriber('/objects_info', ObjectsInfo, self.objects_callback)

	def objects_callback(self, msg):
		self.array[int(msg.object_id)] = msg

class Speech():
	def __init__(self):
		self.words = None
		self.timestamp = rospy.get_rostime()
		rospy.Subscriber('speech_info', SpeechInfo, self.speech_callback)

	def speech_callback(self, msg):
		self.words = msg.speech
		self.timestamp = rospy.get_rostime()

#comPart = ComputeParticipant(6, 1, 0.3, 0.95, 0.1)
#comPart.monitor(50)