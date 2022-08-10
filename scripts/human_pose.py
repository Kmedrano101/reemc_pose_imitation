#!/usr/bin/env python3

# Importar modulos
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException
from geometry_msgs.msg import Point
from reemc_pose_imitation.msg import point_pose, human_pose
import mediapipe as mp
import time
from os import system
import cv2 as cv
from copy import deepcopy
import numpy as np

# Parametros y variables

PACKAGE_NAME = "/reemc_pose_imitation"
NODE_NAME = "human_pose_points"
TOPIC_S1_CAMERA = "/usb_cam/image_raw"
TOPIC_P1_POSE = "pose_human"
BRIDGE = CvBridge()

# Clase HumanPose


class HumanPose:
    """HumanPose object"""

    def __init__(self):
        self._subTopicFlagName = None
        self._subTopicImageSourceName = None
        self._pubTopicStatusNodeName = None
        self._pubTopicPoseNodeName = None
        self.cvFrame = []
        self.pubTopicDataNode = None
        self.dataReceivedTopic1 = False
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose()
        self.readyCapturePose = False

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

    def find_pose(self, draw=True) -> None:
        imgRGB = cv.cvtColor(self.cvFrame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    self.cvFrame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)

    def find_position(self, draw=True) -> None:
        lmAux = human_pose()
        dataPoint = Point()
        dataPose = point_pose()
        contador = 0
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                dataPoint.x = lm.x
                dataPoint.y = lm.y
                dataPoint.z = lm.z
                dataPose.id_pos = id
                dataPose.position = dataPoint
                dataPose.visibility = lm.visibility
                if id == 0 or id == 15 or id == 16 or id == 11 or id == 12:
                    if lm.visibility > 0.5:
                        contador += 1
                lmAux.points_pose.append(deepcopy(dataPose))
            if contador >= 3:
                self.readyCapturePose = True
            else:
                self.readyCapturePose = False
        if len(self.cvFrame) > 0:
            cv.namedWindow("IMAGE_FROM_HUMAN_POSE", cv.WINDOW_NORMAL)
            cv.moveWindow("IMAGE_FROM_HUMAN_POSE", 1280, 0)
            cv.imshow("IMAGE_FROM_HUMAN_POSE", self.cvFrame)
            cv.resizeWindow("IMAGE_FROM_HUMAN_POSE", 640, 480)
            cv.waitKey(1)
        self.pubTopicDataNode.publish(lmAux)

    def image_source_callback(self, msg) -> None:
        self.dataReceivedTopic1 = True
        try:
            self.cvFrame = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"[ERROR] {e}")

    def flag_callback(self, msg) -> None:
        self.dataReceivedTopic2 = True
        self.statusFlag = msg

    def start_subscribers(self) -> None:
        rospy.Subscriber(self._subTopicImageSourceName,
                        Image, self.image_source_callback)

    def start_publishers(self) -> None:
        self.pubTopicDataNode = rospy.Publisher(
            self._pubTopicPoseNodeName, human_pose, queue_size=10)


def main():
    system('clear')
    print("#"*70)
    print(f"\t\t* TEST MODE *\t NODE: {NODE_NAME}")
    print("#"*70)
    time.sleep(1)
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"NODO {NODE_NAME} INICIADO.")
    """Inicializar el objeto HumanPose"""
    objNode = HumanPose()
    objNode.subTopicImageSourceName = TOPIC_S1_CAMERA
    objNode.pubTopicPoseNodeName = TOPIC_P1_POSE
    objNode.start_subscribers()
    objNode.start_publishers()
    INFO1 = True
    INFO2 = True
    while not rospy.is_shutdown():
        if not objNode.dataReceivedTopic1 and INFO2:
            print("[WARNING] Datos no recibidos")
            INFO2 = False
        elif np.array(objNode.cvFrame).size:
            objNode.find_pose()
            objNode.find_position(draw=False)
            if objNode.readyCapturePose and INFO1:
                rospy.loginfo("READY TO CAPTURE DATA")
                INFO1 = False
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass