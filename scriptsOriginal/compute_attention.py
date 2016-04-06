#!/usr/bin/env python

import math
from numpy import *
import rospy
from std_msgs.msg import String
from kinect2_pointing_recognition.msg import SkeletonInfo, ObjectsInfo, FaceInfo, SpeechInfo

POINT_APERTURE = 0.7 # radians
LOOK_APERTURE = 3.0
TOUCH_DISTANCE = 0.1 # meters

def dot_prod(a, b):
	return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def magn(a):
	return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

class ComputeAttention():
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

		self.rate = rospy.Rate(5) # 5hz, or 5 per second

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

	def run(self):
		while not rospy.is_shutdown():
			rospy.loginfo("Object Positions [x, y, z]")
			for i in range(0, self.objects.num_objects):
				rospy.loginfo("\t%s\t%s", i, self.objects.array[i].pos)

			rospy.loginfo("Boolean and Radians of Head vs. Object (Object 0, Object 1)")
			for i in range(0, self.skeletons.max_people):
				looking_angles = [float] * self.objects.num_objects
				for j in range(0, self.objects.num_objects):
					looking_angles[j] = self.face_vs_obj_angle(i, j)
				looking_threshold = [x is not None and x < self.look_aptr / 2 for x in looking_angles]
				rospy.loginfo("\t%s\t%s\t%s", i, looking_threshold, looking_angles)

			rospy.loginfo("Boolean and Radians of Point vs. Object (Object 0 [L,R], Object 1 [L,R])")
			for i in range(0, self.skeletons.max_people):
				pointing_angles = [float] * self.objects.num_objects
				pointing_threshold = [None] * self.objects.num_objects
				for j in range(0, self.objects.num_objects):
					pointing_angles[j] = self.point_vs_obj_angles(i, j)
					pointing_threshold[j] = [angle is not None and angle < self.point_aptr / 2 for angle in pointing_angles[j]]
				rospy.loginfo("\t%s\t%s\t%s", i, pointing_threshold, pointing_angles)

			rospy.loginfo("Touching an Object (Object 0 [L,R], Object 1 [L,R])")
			touching_threshold = [None] * self.objects.num_objects
			for i in range(0, self.skeletons.max_people):
				point_distance = [None] * self.objects.num_objects
				touching_threshold = [None] * self.objects.num_objects
				for j in range(0, self.objects.num_objects):
					point_distance[j] = self.is_touching(i, j)
					touching_threshold[j] = [dist is not None and dist < self.touch_dist for dist in point_distance[j]]
				rospy.loginfo("\t%s\t%s\t%s", i, touching_threshold, point_distance)

			rospy.loginfo("Last Utterance")
			rospy.loginfo("\t%s\tHeard %s seconds ago", self.speech.words, rospy.get_rostime().secs - self.speech.timestamp.secs)

			rospy.loginfo("-------")
			self.rate.sleep()

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

	def face_orientation_vec(self, person_id):
		if self.array[person_id].yaw == -1.0 and self.array[person_id].pitch == -1.0: 
			return [0.0, 0.0, 0.0]
		yaw_rad = math.radians(self.array[person_id].yaw)
		pitch_rad = math.radians(self.array[person_id].pitch)
		return [math.sin(yaw_rad), math.sin(pitch_rad), math.cos(yaw_rad)]

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
	

if __name__ == '__main__':
	max_people = 6
	num_objects = 4
	compute_attention = ComputeAttention(max_people, num_objects, POINT_APERTURE, LOOK_APERTURE, TOUCH_DISTANCE)
	try:
		compute_attention.run()
	except rospy.ROSInterruptException:
		pass
