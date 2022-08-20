#!/usr/bin/env python3

# Importar modulos
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
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
try:
    rospy.get_param_names()
except ROSException:
    print("[WARNING] No se puede obtener los nombres de parametros")

PACKAGE_NAME = "/reemc_pose_imitation"
NODE_NAME = "human_pose_points"
IMG_MODE = rospy.get_param("image_source/mode", default=0)
TOPIC_S1_IMG = '/usb_cam/image_raw'
TOPIC_S2_IMG = '/activar_movimientos'
if int(IMG_MODE) == 1:
    TOPIC_S1_IMG = rospy.get_param("image_source/video_path",default="/videofile/image")
if int(IMG_MODE) == 2:
    TOPIC_S1_IMG = rospy.get_param("image_source/usb_cam",default="/usb_cam/image_raw")
TOPIC_P1_POSE = "pose_human"
IMG_PATH = '/home/kmedrano101/catkin_ws/src/reemc_pose_imitation/img/' # Cambiar dir otrher machine user name change
IMG_NAME = rospy.get_param("image_source/name", default='javi1')
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
        imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    self.cvFrame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)

    def find_position(self,img, draw=True) -> None:
        lmAux = human_pose()
        dataPoint = Point()
        dataPose = point_pose()
        contador = 0
        # Puntos de pose para validar, en caso de faltar un punto en la imagen
        # no se procedera a realizar las funciones de movimiento
        pointsValidate = [0,11,12,13,14,15,16]
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                dataPoint.x = lm.x
                dataPoint.y = lm.y
                dataPoint.z = lm.z
                dataPose.id_pos = id
                dataPose.position = dataPoint
                dataPose.visibility = lm.visibility
                if id in pointsValidate:
                    if lm.visibility > 0.5:
                        contador += 1
                lmAux.points_pose.append(deepcopy(dataPose))
            if contador >= 7:
                self.readyCapturePose = True
            else:
                self.readyCapturePose = False
        if len(img) > 0:
            cv.namedWindow("IMAGE_FROM_HUMAN_POSE", cv.WINDOW_NORMAL)
            cv.moveWindow("IMAGE_FROM_HUMAN_POSE", 1280, 0)
            cv.imshow("IMAGE_FROM_HUMAN_POSE", img)
            cv.resizeWindow("IMAGE_FROM_HUMAN_POSE", 640, 480)
            cv.waitKey(1)
        self.pubTopicDataNode.publish(lmAux)
        if(self.statusAction):
                cv.imwrite(IMG_PATH+'img_poses.jpg',self.cvFrame)
                self.statusAction = False
                rospy.loginfo("Save img!")
        self.set_param_values()
        #print("shape of img",img.shape)

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
        rospy.Subscriber(self.subTopicActionName,Bool,self.action_callback)
        if int(IMG_MODE) > 0:
            rospy.Subscriber(self.subTopicImageSourceName,Image, self.image_source_callback)

    def start_publishers(self) -> None:
        self.pubTopicDataNode = rospy.Publisher(
            self._pubTopicPoseNodeName, human_pose, queue_size=10)

    def get_param_values(self) -> None:
        # Obtener todos los paramatros 
        self.imgName = rospy.get_param("image_source/name", default='javi1')

    def set_param_values(self) -> None:
        h,w,_ = self.cvFrame.shape
        rospy.set_param("/image_source/weight",w)
        rospy.set_param("/image_source/height",h)

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
    objNode.subTopicImageSourceName = TOPIC_S1_IMG
    objNode.subTopicActionName = TOPIC_S2_IMG
    objNode.pubTopicPoseNodeName = TOPIC_P1_POSE
    objNode.start_subscribers()
    objNode.start_publishers()
    INFO1 = True
    INFO2 = True
    # Modo de recuso de imagen video-stream o imagen
    if int(IMG_MODE) == 0:
        objNode.dataReceivedTopic1 = True
    
    ### MAKE FUNCTION FOR VALIDATIONS ALL DATA NEEDED
    while not rospy.is_shutdown():
        objNode.get_param_values()
        if int(IMG_MODE) == 0:
                path_img = IMG_PATH+objNode.imgName
                objNode.cvFrame = cv.imread(path_img)
        if not objNode.dataReceivedTopic1 and INFO2:
            print("[WARNING] Datos no recibidos, esperando datos...")
            INFO2 = False
        elif np.array(objNode.cvFrame).size:
            objNode.find_pose(objNode.cvFrame)
            objNode.find_position(objNode.cvFrame,draw=False)
            if objNode.readyCapturePose and INFO1:
                rospy.loginfo("READY TO CAPTURE DATA")
                INFO1 = False
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass