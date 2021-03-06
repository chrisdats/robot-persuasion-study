
robot-persuasion-study
Code for robot persuasion study in the social robotics lab (Scaz Lab) 

Adapted from kinect2-pointing-recognition
by Thomas Weng
https://github.com/thomasweng15/kinect2_pointing_recognition

Equipment
Kinect v2
Windows Machine
Linux Machine
Nao Robot

Add the following lines to .bashrc (found in home directory)
source /opt/ros/indigo/setup.bash
export PYTHONPATH=${PYTHONPATH}:~/Documents/pynaoqi-python-2.7-naoqi-1.14-linux64
source catkin_ws/devel/setup.bash

So $PYTHONPATH should include python library in ros/indigo plus naoqi


To Run
1. roslaunch on Linux machine: roslaunch kinect2_pointing_recognition kinect_listen_and_parse.launch
2.  press play on visual studio on Windows machine
3. Monitor with commands such as: rostopic list, rostopic echo /skeleton_info


FOR WINDOWS
Contains:
 - C#  files for publishing data via rosbridge over websocket (rosbridge.cs)
 - Associated .dll's and libraries
 - PointingRecognition Solution, the solution with the main Kinect sensing code
 - RGB Thresholder Solution, for object color segmentation



FOR ROS
Contains:
 - A node, kinect_parser.py, that transforms incoming JSON msgs from rosbridge into useful ROS messages.
 - code for the video demo


General workflow for extending the capabilities of this package:

- Write C# code in Visual Studio that interfaces directly with the Kinect hardware. Publish relevant data via rosbridge over topic /windows_rosbridge
- Within 'kinect_parser.py' 
- Create a new XXX.msg
- 
- add a function to 'parsing_functions.py' that parses the messages, and returns an appropriate ROS msg to be published on topic /[TOPIC_NAME] (e.g. /audio_loc)
- Note: The kinect parser expects data in the format of colon seperated values. The first 3 fields are predefined:
 
[out_topic] - the outgoing ROS topic on which this message data will be published (e.g. /audio_loc)
[windows_timestamp] - a windows generated timestamp which can be used to measure ROSbridge network latency
[origin] - the name of the kinect from which this message was generated

After those three, any additional values are feature specific! For the Audio Location example, the JSON messages  have the format 

[out_topic]:[windows_timestamp]:[origin]:[angle]:[confidence]

and might look like "/audio_loc:0.0:kinect_1:45:.97"
