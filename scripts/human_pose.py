#!/usr/bin/env python3

"""Human_pose.py script to get all 33 pose data from a single 2D images

    Descriptions:
        Image source: the single image can be get from three different ways
                        a specific path "my_image.jpg", a ROS topic in this case from the usb-cam
                        or a video file.
        Main module: for this porpose this script is using mediapipe module from google
        Main function:   get all the 33 pose from a 2D image and public the data through a ROS topic 
"""
# Import modules from ROS resources

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException
from geometry_msgs.msg import Point
from reemc_pose_imitation.msg import point_pose, human_pose

# import modules from others resource

import mediapipe as mp
import time
from os import system
import cv2 as cv
from copy import deepcopy
import numpy as np

# Parameters and variables

try:
    # verify if parameters are available
    rospy.get_param_names()
except ROSException:
    print("[WARNING] can not get the parameters")

PACKAGE_NAME = "/reemc_pose_imitation"
NODE_NAME = "human_pose_points"
IMG_MODE = rospy.get_param("image_source/mode", default=0)
TOPIC_S1_IMG = PACKAGE_NAME+"/webcam/image_raw"
TOPIC_S2_IMG = PACKAGE_NAME+"/activar_movimientos"
if int(IMG_MODE) == 1:
    TOPIC_S1_IMG = rospy.get_param(PACKAGE_NAME+"/image_source/video_path",default="/videofile/image")
if int(IMG_MODE) == 2:
    TOPIC_S1_IMG = rospy.get_param(PACKAGE_NAME+"/image_source/usb_cam",default="/webcam/image_raw")
TOPIC_P1_POSE = PACKAGE_NAME+"/pose_human"
IMG_PATH = '/home/kmedrano101/catkin_ws/src/reemc_pose_imitation/img/'
IMG_NAME = rospy.get_param(PACKAGE_NAME+"/image_source/name", default='javi1')
BRIDGE = CvBridge()

# Clase HumanPose


class HumanPose:
    """HumanPose object"""

    def __init__(self):
        self._subTopicImageSourceName = None
        self._subTopicActionName = None
        self._pubTopicStatusNodeName = None
        self._pubTopicPoseNodeName = None
        self.cvFrame = []
        self.pubTopicDataNode = None
        self.dataReceivedTopic1 = False
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose()
        self.readyCapturePose = False
        self.imgName = ''
        self.lastImage = []
        self.statusAction = False
        self.poses_data = human_pose()

    """Properties"""

    @property
    def subTopicImageSourceName(self):
        """The subTopicImageSourceName property."""
        return self._subTopicImageSourceName

    @subTopicImageSourceName.setter
    def subTopicImageSourceName(self, value):
        if value:
            self._subTopicImageSourceName = value
        else:
            rospy.loginfo("Invalid Name of topic ImageSource")

    @property
    def subTopicActionName(self):
        """The subTopicActionName property."""
        return self._subTopicActionName

    @subTopicActionName.setter
    def subTopicActionName(self, value):
        if value:
            self._subTopicActionName = value
        else:
            rospy.loginfo("Invalid Name of topic ImageSource")
    @property
    def pubTopicPoseNodeName(self):
        """The pubTopicDataNodeName property."""
        return self._pubTopicPoseNodeName

    @pubTopicPoseNodeName.setter
    def pubTopicPoseNodeName(self, value):
        if value:
            self._pubTopicPoseNodeName = value
        else:
            rospy.loginfo("Invalid Name of topic Data")

    """ Methods and Actions"""

    def find_pose(self,img, draw=True) -> None:
        """Function to get all the 33 poses with the mediapipe module

        Args:
            img (cv_img): image
            draw (bool, optional): to draw the image and be able to see the poses. Defaults to True.
        """
        imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    self.cvFrame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)

    def send_positions(self,img) -> None:
        """ Save date into a ROS message types and send them through a ROS topic

        Args:
            img (cv_img): image
        """
        # Define ROS messages
        dataPoint = Point()
        dataPose = point_pose()
        contador = 0
        # Save data into the ROS messages and validate the visibility of some poses
        pointsValidate = [0,11,12,13,14,15,16] # Poses to validate
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                dataPoint.x = lm.x
                dataPoint.y = lm.y
                dataPoint.z = lm.z
                dataPose.id_pos = id
                dataPose.position = dataPoint
                dataPose.visibility = lm.visibility
                # Validate poses to make sure the other scritp works good
                if id in pointsValidate:
                    if lm.visibility > 0.5:
                        contador += 1
                self.poses_data.points_pose.append(deepcopy(dataPose))
            if contador >= 7:
                self.readyCapturePose = True
            else:
                self.readyCapturePose = False
        # Create a window to show up the image with the draw poses on the image
        if len(img) > 0:
            cv.namedWindow("IMAGE_FROM_HUMAN_POSE", cv.WINDOW_NORMAL)
            cv.moveWindow("IMAGE_FROM_HUMAN_POSE", 1280, 0)
            cv.imshow("IMAGE_FROM_HUMAN_POSE", img)
            cv.resizeWindow("IMAGE_FROM_HUMAN_POSE", 640, 480)
            cv.waitKey(1)
        # Save the image to a specific path, statusAction is a flag that is activated
        # by a topic from the web interface
        if(self.statusAction):
                cv.imwrite(IMG_PATH+'img_poses.jpg',self.cvFrame)
                self.statusAction = False
                rospy.loginfo("Save img!")
        # Save the current image shape to the param server
        self.set_param_values()

    def image_source_callback(self, msg) -> None:
        self.dataReceivedTopic1 = True
        try:
            self.cvFrame = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"[ERROR] {e}")

    def action_callback(self, msg) -> None:
        self.statusAction = msg.data
        rospy.loginfo(self.statusAction)

    def start_subscribers(self) -> None:
        """To start the subscribers, if the image source is from a path image topic will not be subscribed"""
        rospy.Subscriber(self.subTopicActionName,Bool,self.action_callback)
        if int(IMG_MODE) > 0:
            rospy.Subscriber(self.subTopicImageSourceName,Image, self.image_source_callback)

    def start_publishers(self) -> None:
        """To start the publishers""" 
        self.pubTopicDataNode = rospy.Publisher(
            self._pubTopicPoseNodeName, human_pose, queue_size=10)

    def get_param_values(self) -> None:
        """Get the name image param value"""
        self.imgName = rospy.get_param("/reemc_pose_imitation/image_source/name", default='javi1.jpg')

    def set_param_values(self) -> None:
        """Set parameters to the server"""
        h,w,_ = self.cvFrame.shape
        rospy.set_param("/reemc_pose_imitation/image_source/weight",w)
        rospy.set_param("/reemc_pose_imitation/image_source/height",h)

def main():
    # Clean up the terminal
    system('clear')  
    # Write the main top title with the name of the node                                   
    print("#"*70)                                           
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    # Wait for one second, just to make sure roscore is running first
    time.sleep(1)
    # Start the node
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    # Create the object humanpose
    humanpose = HumanPose()
    # Set the names of the topics
    humanpose.subTopicImageSourceName = TOPIC_S1_IMG
    humanpose.subTopicActionName = TOPIC_S2_IMG
    humanpose.pubTopicPoseNodeName = TOPIC_P1_POSE
    # Start publishers and subscribers
    humanpose.start_subscribers()
    humanpose.start_publishers()
    # Auxiliary variables to show up info just one time
    INFO1 = True
    INFO2 = True
    # Verify if the image source is from a specific path
    if int(IMG_MODE) == 0:
        humanpose.dataReceivedTopic1 = True

    # Main loop that is running while roscore is up
    while not rospy.is_shutdown():
        humanpose.get_param_values()
        # Get the image if the source is from a path
        if int(IMG_MODE) == 0:
                path_img = IMG_PATH+humanpose.imgName
                humanpose.cvFrame = cv.imread(path_img)
        if not humanpose.dataReceivedTopic1 and INFO1:
            print("[WARNING] Data not available, waiting to receive data...")
            INFO1 = False
        # Make sure there is an image loaded
        elif np.array(humanpose.cvFrame).size:
            # Run the main functions in order to get the data and publish them through a ROS topic
            humanpose.find_pose(humanpose.cvFrame)
            humanpose.send_positions(humanpose.cvFrame)
            if humanpose.readyCapturePose:
                if INFO2:
                    rospy.loginfo("Node is running...")
                    INFO2 = False 
                # Publish the data poses 
                humanpose.pubTopicDataNode.publish(humanpose.poses_data)
    rospy.spin()

# Run as a main program
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass