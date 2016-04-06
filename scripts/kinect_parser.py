#!/usr/bin/env python


"""
Author: Sam Spaulding (7/21/14)

This node listens for incoming messages from Windows kinect topics (published via ROSbridge)
parses the incoming data to identify what kind of data (e.g. audio location, skeleton data, etc.)
and invokes the appropriate function to parse and repubish that information 
"""

import rospy
import json
from pprint import pprint
from std_msgs.msg import String
from kinect2_pointing_recognition.msg import AudioLoc, FaceInfo, SkeletonInfo, ObjectsInfo, SpeechInfo
from parsing_functions import parse_audio_loc, parse_face_info, parse_skeleton_info, parse_objects_info, parse_speech_info

DEBUG = 1

HEADER_LENGTH = 3
"""
the first [HEADER_LENGTH] fields of split string contain important metadata about the message
followed by the actual message data, which can be variable length.
"""


def debug(msg):
    if DEBUG:
        print msg


"""
WHEN YOU ADD NEW CAPABILITIES TO THIS NODE, DOCUMENT WHAT YOU HAVE ADDED HERE.


Subscribes to: /windows_rosbridge
Publishes on: /audio_loc
              /face_info
              /skeleton_info
              /objects_info
              /speech_info

"""

class KinectParser:
    def __init__(self):
        self.node_name = 'kinect2_pointing_recognition'
        rospy.init_node(self.node_name)

        # Subscribe to the rosbridge to listen to Windows messages
        # When new messages are received, self.kinect_parse_callback is invoked
        self.sub = rospy.Subscriber('/windows_rosbridge', String, self.kinect_parse_callback)
        
        # Publishers
        ##### Add new publishers for topics here #####
        size = 100
        self.pub_audioloc = rospy.Publisher('/audio_loc', AudioLoc, queue_size = size)
        self.pub_faceinfo = rospy.Publisher('/face_info', FaceInfo, queue_size = size)
        self.pub_skelinfo = rospy.Publisher('/skeleton_info', SkeletonInfo, queue_size = size)
        self.pub_objinfo = rospy.Publisher('/objects_info', ObjectsInfo, queue_size = size)
        self.pub_speechinfo = rospy.Publisher('/speech_info', SpeechInfo, queue_size = size)

    def kinect_parse_callback(self, win_kinect_msg):
      # parse as json, identify id and key

      sendStr = ""
      debug("here is the uncut message")
      debug(win_kinect_msg)

      str_msg = str(win_kinect_msg)
      debug("here is the str converted")
      debug(str_msg)

      #audio_loc_data = json.loads(msgJSON)
      split_string = str(win_kinect_msg.data).split(":")

      out_topic = str(split_string[0])
      win_timestamp = split_string[1]
      origin = split_string[2]

      if out_topic == "audio_loc":
        msg = parse_audio_loc(split_string[HEADER_LENGTH:])
        self.pub_audioloc.publish(msg)
      elif out_topic == "face_info":
        msg_array = parse_face_info(split_string[HEADER_LENGTH:])
        for msg in msg_array:
          self.pub_faceinfo.publish(msg)
      elif out_topic == "skeleton_info":
        msg_array = parse_skeleton_info(split_string[HEADER_LENGTH:])
        for msg in msg_array:
          self.pub_skelinfo.publish(msg)
      elif out_topic == "objects_info":
        msg_array = parse_objects_info(split_string[HEADER_LENGTH:])
        for msg in msg_array:
          self.pub_objinfo.publish(msg)
      elif out_topic == "speech_info":
        msg_array = parse_speech_info(split_string[HEADER_LENGTH:])
        for msg in msg_array:
          self.pub_speechinfo.publish(msg)
      else:
        rospy.logerr("Unrecognized topic from Windows: " + out_topic)


    def run(self):
        rospy.spin()
    


if __name__ == '__main__':
    parseNode = KinectParser()
    parseNode.run()

