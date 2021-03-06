#!/usr/bin/env python
from __future__ import print_function

import sys
import math
import time
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from nav_msgs.msg import OccupancyGrid

from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Both detectors can be used to find faces in images
from detectors.haar_detector import HaarDetector
from detectors.dlib_detector import HogDetector, CnnDetector

class FaceFinder(object):
    def __init__(self):
        # Initialize node, anonymous means that we can run same node multiple times
        rospy.init_node('face_detector', anonymous=True)

        # Read parameters from our face_finder.launch file
        self.display_camera_window = rospy.get_param('~display_camera_window', False)
        self.haar_cascade_data_file_path = rospy.get_param('~haar_cascade_data_file_path', '')
        self.cnn_face_detector_data_file_path = rospy.get_param('~cnn_face_detector_data_file_path', '')
        self.rotate_image = rospy.get_param('~rotate_image', False)
        
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # We can use dlib, haar, or hog detector here
        self.detector_id = rospy.get_param('~use_detector')
        if self.detector_id == 1:
            self.face_detector = CnnDetector(self.cnn_face_detector_data_file_path)
        elif self.detector_id == 2:
            self.face_detector = HaarDetector(self.haar_cascade_data_file_path)
        elif self.detector_id == 3:
            self.face_detector = HogDetector()

        # Subscriber for new camera images (the video_stream_opencv publishes to different topic)
        self.image_subscriber = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)

        # How much we should downscale image before trying to find faces
        # For example - factor 4 means that that the new image width will be width / 4
        self.downscale_factor = rospy.get_param('~downscale_factor', 4)

        # Should the image be converted to graycale before processing
        # true -> grayscale, false -> rgb
        self.convert_to_grayscale = rospy.get_param('~black_and_white', True)

        # Time when last image was received
        self.last_image_received = -1

        # Parameters for evaluating the detector
        self.frame_id = 0
        self.no_of_frames = rospy.get_param('~total_frames', 0) # number of frames in a video
        self.total_detected_faces_in_video = 0 # counts all detected faces
        self.more_than_one_detection_in_frame = 0 # counts how many times more than one face was detected (false positives)
        self.frames_with_detections = 0 #counts how many times at least one face was detected
        self.video_id = rospy.get_param('~video_source', "video01.mp4")

        self.frames = []

        # How many seconds we should wait for new image until we can
        # assume that the video has finished
        self.wait_no_image = 1.0

        # Create a new rate object that we will use to check whether no image
        # was received for some time. Set it to 60Hz and create a new loop
        # that will check whether we should end this script.
        self.rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            # Check if we should exit, call the exit function and shutdown this node
            if self.last_image_received >= 0 and (time.time() - self.last_image_received) >= self.wait_no_image:
                self.finish()
                rospy.signal_shutdown("No frame received for {} s".format(self.wait_no_image))
                sys.exit(1)

            self.rate.sleep()
    
    def finish(self):
        rospy.loginfo("No image received for {} seconds. Ending face detector.".format(self.wait_no_image))
        rospy.loginfo("Total number of frames: {}".format(self.frame_id))
        rospy.loginfo("Number of detected faces: {}".format(self.total_detected_faces_in_video))

        # Construct filename from video id and detector type
        detector_types = ["cnn", "haar", "hog"]
        filename = "{}_{}_{}_{}.txt".format(detector_types[self.detector_id - 1], self.video_id.replace('.mp4', ''), self.downscale_factor, self.convert_to_grayscale)

        # Open file for writing
        with open("/home/niki/Desktop/ROS/src/homework1/videos/{}".format(filename), "w") as output_file:
            # First, write out all parameters to file
            output_file.write("{}\n".format(self.video_id))
            output_file.write("{}\n".format(self.detector_id))
            output_file.write("{}\n".format(self.downscale_factor))
            output_file.write("{}\n".format(self.convert_to_grayscale))
            output_file.write("{}\n".format(self.no_of_frames))
            output_file.write("{}\n".format(self.frame_id)) # processed frames
            output_file.write("{}\n".format(self.total_detected_faces_in_video))
            output_file.write("{}\n".format(self.frames_with_detections)) # TP
            output_file.write("{}\n".format(self.more_than_one_detection_in_frame)) # FP
            for frame in self.frames:
                output_file.write(str([f.to_list() for f in frame]))
                output_file.write('\n')

    def process_face(self, image, face):
        # Get coordinates of the rectangle around the face
        x1 = face.left()
        x2 = face.right()
        y1 = face.top()
        y2 = face.bottom()

        # Extract region containing face
        face_region = image[y1:y2,x1:x2]

        # Mark the face on our image
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 3, 8, 0)
    
    def preprocess_image(self, image):
        # Get image width and height to calculate new size
        height, width, channels = image.shape
        new_width, new_height = (int(width / self.downscale_factor), int(height / self.downscale_factor))

        # Resize image to be 1/4 the original size, so the face detector is faster
        image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

        # If the rotate_image argument is set to true, rotate image 90 deg clockwise
        if self.rotate_image:
            image = cv2.transpose(image)
        
        # Convert the image to rgb for faster image processing?
        if self.convert_to_grayscale:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        return image
    
    def image_callback(self, rgb_image_message):
        # This function will be called when new camera rgb image is received
        # rospy.loginfo('New image frame received, id {}'.format(self.frame_id))
        self.frame_id += 1

        # Set the last received image time to current time
        self.last_image_received = time.time()

        rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        rgb_image = self.preprocess_image(rgb_image)

        face_rectangles = self.face_detector.find_faces(rgb_image)
        # rospy.loginfo('Found {} faces'.format(len(face_rectangles)))
        self.total_detected_faces_in_video += len(face_rectangles)
        if len(face_rectangles) > 1:
            self.more_than_one_detection_in_frame += 1
        if len(face_rectangles) > 0:
            self.frames_with_detections += 1
        # rospy.loginfo('Total detected faces: {}'.format(self.total_detected_faces_in_video))
        self.frames.append(face_rectangles)
        for face_rectangle in face_rectangles:
            self.process_face(rgb_image, face_rectangle)
        
        if self.display_camera_window:
            cv2.imshow("Image", rgb_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    face_finder = FaceFinder()
    rospy.loginfo('Face detector started')