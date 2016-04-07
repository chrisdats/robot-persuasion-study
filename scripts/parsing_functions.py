import rospy
from kinect2_pointing_recognition.msg import AudioLoc, FaceInfo, SkeletonInfo, ObjectsInfo, SpeechInfo


DEBUG = 1

def debug(msg):
	if DEBUG:
		print(msg)



def parse_audio_loc(data_string):
	"""
	Publishes an AudioLoc message, which contains 2 float32's, one that specifies the angle from the sensor (in degrees)
	and one that specifies the confidence in that measurement (from 0.00 - 1.00)
	"""
	outgoing = AudioLoc()

	outgoing.angle = float(data_string[0])
	outgoing.confidence = float(data_string[1])
    
	return outgoing


def parse_face_info(data_string):
	"""
	Publishes a FaceInfo message, which contains facial expression and feature data. 

	"""
	# Array of messages to be sent, in case there are multiple faces sensed.
	faceinfo_msg_array = []

	# The number of faces is represented as the first element of "data"
	num_people = 6

	# The number of fields in the faceinfo msg, which need to be extracted
	# for each person
	debug("DATASTRING")
	debug(data_string)
	debug("Fields per")
	fields_per_person = (len(data_string)) / num_people
	debug(fields_per_person) 

	# Parse out info for each person
	for i in range(0, num_people):
		person_id = i
		index_shift = (i * fields_per_person)

		# Create message that will be sent out
	 	outgoing = FaceInfo()
	 	outgoing.person_id = str(i)

		#if boolean val == '1', then True
		#if boolean val == '0', then False
		#if boolean val == '', then Unknown 

		# Currently setting all Unknowns to False
	 	outgoing.happy = (data_string[index_shift+1] == '1')
	 	outgoing.engaged = (data_string[index_shift+2] == '1')
	 	outgoing.wear_glasses = (data_string[index_shift+3] == '1')
	 	outgoing.left_eye_closed = (data_string[index_shift+4] == '1')
	 	outgoing.right_eye_closed = (data_string[index_shift+5] == '1')
	 	outgoing.mouth_open = (data_string[index_shift+6] == '1')
	 	outgoing.mouth_moved = (data_string[index_shift+7] == '1')
	 	outgoing.looking_away = (data_string[index_shift+8] == '1')

	 	# Face position info
	 	outgoing.yaw = float(data_string[index_shift+9])
	 	outgoing.pitch = float(data_string[index_shift+10])
	 	outgoing.roll = float(data_string[index_shift+11])

	 	# Points in the face
	 	outgoing.left_eye_pos = [float(x) for x in data_string[index_shift+12].split(',')]
	 	outgoing.right_eye_pos = [float(x) for x in data_string[index_shift+13].split(',')]
	 	outgoing.nose_pos = [float(x) for x in data_string[index_shift+14].split(',')]
	 	outgoing.left_mouth_corner = [float(x) for x in data_string[index_shift+15].split(',')]
	 	outgoing.right_mouth_corner = [float(x) for x in data_string[index_shift+16].split(',')]

		faceinfo_msg_array.append(outgoing)

	return faceinfo_msg_array

def parse_skeleton_info(data_string):
	"""
	Publishes an SkeletonInfo message, which contains 3D skeleton points 

	"""
	num_people = 6
	fields_per_person = len(data_string) / num_people
	debug("fields per person is " + str(fields_per_person))
	skel_info_msg_array = []

	for i in range(0,num_people):
		outgoing = SkeletonInfo()
		index_shift = i * fields_per_person

		outgoing.person_id = str(i)

		outgoing.spineBase = [float(x) for x in data_string[index_shift].split(',')]
		outgoing.spineMid = [float(x) for x in data_string[index_shift + 1].split(',')]
		outgoing.neck = [float(x) for x in data_string[index_shift + 2].split(',')]
		outgoing.head = [float(x) for x in data_string[index_shift + 3].split(',')]
		outgoing.shoulderLeft = [float(x) for x in data_string[index_shift + 4].split(',')]
		outgoing.elbowLeft = [float(x) for x in data_string[index_shift + 5].split(',')]
		outgoing.wristLeft = [float(x) for x in data_string[index_shift + 6].split(',')]
		outgoing.handLeft = [float(x) for x in data_string[index_shift + 7].split(',')]
		outgoing.shoulderRight = [float(x) for x in data_string[index_shift + 8].split(',')]
		outgoing.elbowRight = [float(x) for x in data_string[index_shift + 9].split(',')]
		outgoing.wristRight = [float(x) for x in data_string[index_shift + 10].split(',')]
		outgoing.handRight = [float(x) for x in data_string[index_shift + 11].split(',')]
		outgoing.hipLeft = [float(x) for x in data_string[index_shift + 12].split(',')]
		outgoing.kneeLeft = [float(x) for x in data_string[index_shift + 13].split(',')]
		outgoing.ankleLeft = [float(x) for x in data_string[index_shift + 14].split(',')]
		outgoing.footLeft = [float(x) for x in data_string[index_shift + 15].split(',')]
		outgoing.hipRight = [float(x) for x in data_string[index_shift + 16].split(',')]
		outgoing.kneeRight = [float(x) for x in data_string[index_shift + 17].split(',')]
		outgoing.ankleRight = [float(x) for x in data_string[index_shift + 18].split(',')]
		outgoing.footRight = [float(x) for x in data_string[index_shift + 19].split(',')]
		outgoing.spineShoulder = [float(x) for x in data_string[index_shift + 20].split(',')]
		outgoing.handTipLeft = [float(x) for x in data_string[index_shift + 21].split(',')]
		outgoing.thumbLeft = [float(x) for x in data_string[index_shift + 22].split(',')]
		outgoing.handTipRight = [float(x) for x in data_string[index_shift + 23].split(',')]
		outgoing.thumbRight = [float(x) for x in data_string[index_shift + 24].split(',')]

		skel_info_msg_array.append(outgoing)

	return skel_info_msg_array

def parse_objects_info(data_string):

	"""
	Publishes an ObjectsInfo message, which contains 3D object coordinates

	"""
	num_objects = 1
	fields_per_object = 1
	obj_info_msg_array = []

	for i in range(0, num_objects):
		outgoing = ObjectsInfo()
		index_shift = i * fields_per_object
		outgoing.object_id = str(i)
		outgoing.pos = [float(x) for x in data_string[index_shift].split(',')]
		if outgoing.pos != [0.0,0.0,0.0]:
			obj_info_msg_array.append(outgoing)

	return obj_info_msg_array

def parse_speech_info(data_string):
	"""
	Publishes a SpeechInfo message, which contains speech as text.

	"""
	speech_info_msg_array = []
	outgoing = SpeechInfo()
	outgoing.speech = str(data_string[0])
	speech_info_msg_array.append(outgoing)

	return speech_info_msg_array


def parse_XXX(data_string):

	"""
	Publishes an XXX message, which contains XXXXX, 

	"""

	outgoing = ROSMsgType()

	outgoing.field1 = float(data_string[3])
	outgoing.field2 = float(data_string[4])

	return outgoing
